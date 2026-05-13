"""Flask + Socket.IO server for the Solar Panel Cleaning Robot.

Glue layer wiring:

* :class:`GPSReader`         → :class:`Estimator`
* :class:`RobotBridge`       → :class:`Estimator` + :class:`SafetyMonitor`
* :class:`Navigator`         (consumes both, drives the Pico)
* :class:`MissionStore`      (boundary / waypoints persistence)

REST and Socket.IO are intentionally thin — all logic lives in the
modules above.

Run:
    python3 -m mission_planner.app
    python3 -m mission_planner.app --no-gps --no-robot   # UI dev mode
"""

from __future__ import annotations

import argparse
import logging
import sys
import threading
import time
from dataclasses import replace

from flask import Flask, jsonify, render_template, request
from flask_socketio import SocketIO, emit

from .boundary_capture import BoundaryCapture
from .config import Config, load_config
from .geo import LatLon
from .gps_reader import GPSReader
from .mission_store import MissionStore
from .navigation import Navigator, NavMode
from .path_planner import PlanRequest, plan_boundary_route, plan_coverage
from .radio_bridge import RadioBridge
from .robot_bridge import RobotBridge
from .safety import SafetyMonitor
from .sensor_fusion import Estimator

log = logging.getLogger(__name__)
BOUNDARY_ROUTE_GEOFENCE_MARGIN_M = 0.0


# ─── App factory ──────────────────────────────────────────────────────────────


def create_app(cfg: Config, *, with_gps: bool = True,
               with_robot: bool = True) -> tuple[Flask, SocketIO, Navigator]:
    app_version = "2.2.1"
    app = Flask(__name__)
    app.config["SECRET_KEY"] = "solar-bot-mission-planner"
    sio = SocketIO(
        app,
        cors_allowed_origins="*",
        async_mode="threading",
        ping_timeout=60,
        ping_interval=cfg.server.socketio_ping,
        logger=False,
        engineio_logger=False,
    )

    store = MissionStore()
    safety = SafetyMonitor(cfg.safety)
    estimator = Estimator(cfg.navigation)

    gps = (
        GPSReader(port=cfg.hardware.gps_port, baud=cfg.hardware.gps_baud)
        if with_gps else None
    )
    radio = RadioBridge(
        port=cfg.hardware.radio_port,
        baud=cfg.hardware.radio_baud,
        steer_channel=cfg.hardware.radio_steer_channel,
        throttle_channel=cfg.hardware.radio_throttle_channel,
        steer_invert=cfg.hardware.radio_steer_invert,
        throttle_invert=cfg.hardware.radio_throttle_invert,
        switch_high_us=cfg.hardware.radio_switch_high_us,
    )
    robot = (
        RobotBridge(
            transport=cfg.hardware.pico_transport,
            port=cfg.hardware.pico_port,
            baud=cfg.hardware.pico_baud,
            udp_host=cfg.hardware.pico_udp_host,
            udp_port=cfg.hardware.pico_udp_port,
            battery_full_mv=cfg.robot.battery_full_mv,
            battery_empty_mv=cfg.robot.battery_empty_mv,
        )
        if with_robot else None
    )

    def _emit(event: str, payload: dict) -> None:
        try:
            sio.emit(event, payload)
        except Exception:  # pragma: no cover
            log.exception("socket emit %s failed", event)

    nav = Navigator(cfg, robot, estimator, safety, on_event=_emit)
    if not with_gps:
        nav.set_mode(NavMode.NON_GPS)

    if gps is not None:
        gps.start()
    radio.start()
    if robot is not None:
        robot.start()
    nav.start()
    last_manual_until = 0.0
    capture = BoundaryCapture()
    planner_settings = {
        "cleaning_width_m": cfg.robot.cleaning_width_m,
        "overlap_pct": cfg.planner.overlap_pct,
        "sweep_angle_deg": cfg.planner.sweep_angle_deg,
        "return_to_home": cfg.planner.return_to_home,
        "path_mode": "coverage",
    }
    last_plan_stats: dict[str, float | int] = {}

    def _plan_payload(plan) -> dict:
        return {
            "path_mode": planner_settings["path_mode"],
            "rows": plan.rows,
            "distance_m": round(plan.distance_m, 1),
            "area_m2": round(plan.area_m2, 1),
            "sweep_angle_deg": round(plan.sweep_angle_deg, 1),
            "effective_spacing_m": round(plan.effective_spacing_m, 3),
            "estimated_time_s": round(plan.estimated_time_s),
        }

    def _sync_payload(*, saved_filename: str | None = None) -> dict:
        mission = nav.snapshot()["mission"]
        boundary = capture.to_dict()["points"] or mission["boundary"]
        return {
            "boundary": boundary,
            "waypoints": mission["waypoints"],
            "mission": mission,
            "capture": capture.to_dict(),
            "plan_stats": dict(last_plan_stats),
            "planner_settings": dict(planner_settings),
            "saved_filename": saved_filename,
        }

    def _emit_sync(*, saved_filename: str | None = None) -> None:
        _emit("planner_sync", _sync_payload(saved_filename=saved_filename))

    def _update_planner_settings(body: dict) -> None:
        planner_settings["cleaning_width_m"] = float(body.get(
            "cleaning_width_m",
            planner_settings["cleaning_width_m"],
        ))
        planner_settings["overlap_pct"] = float(body.get(
            "overlap_pct",
            planner_settings["overlap_pct"],
        ))
        sweep = body.get("sweep_angle_deg", planner_settings["sweep_angle_deg"])
        planner_settings["sweep_angle_deg"] = (
            None if sweep in (None, "", "auto") else float(sweep)
        )
        planner_settings["return_to_home"] = bool(body.get(
            "return_to_home",
            planner_settings["return_to_home"],
        ))
        planner_settings["path_mode"] = (
            "boundary"
            if body.get("path_mode", planner_settings["path_mode"]) == "boundary"
            else "coverage"
        )

    def _current_pose_point() -> LatLon | None:
        pose = estimator.pose()
        if pose.has_position:
            return LatLon(pose.lat, pose.lon)
        if gps is not None:
            fix = gps.get_fix()
            if fix.has_fix:
                return LatLon(fix.lat, fix.lon)
        return None

    def _record_boundary_point(source: str) -> tuple[bool, str]:
        mission = nav.snapshot()["mission"]
        mission_state = mission["state"]
        if mission_state == "running":
            return False, "Pause the mission before saving new boundary points"
        if capture.planned or mission["wp_total"]:
            nav.clear_mission()
            last_plan_stats.clear()
        point = _current_pose_point()
        if point is None:
            return False, "No live robot position yet"
        try:
            count = capture.add_point(point, source=source)
        except ValueError as exc:
            return False, str(exc)
        _emit("boundary_point_saved", {
            "source": source,
            "count": count,
            "lat": round(point.lat, 8),
            "lon": round(point.lon, 8),
        })
        _emit_sync()
        return True, f"Saved point {count}"

    def _set_capture_boundary(
        boundary: list[LatLon],
        *,
        source: str,
        planned: bool,
    ) -> None:
        capture.clear()
        capture.points = list(boundary)
        capture.recording = not planned and bool(boundary)
        capture.planned = planned and bool(boundary)
        capture.last_source = source if boundary else "none"
        capture.updated_at = time.time()

    def _mission_geofence_buffer_m() -> float | None:
        if planner_settings["path_mode"] == "boundary":
            return BOUNDARY_ROUTE_GEOFENCE_MARGIN_M
        return None

    def _build_plan(boundary: list[LatLon]):
        if planner_settings["path_mode"] == "boundary":
            return plan_boundary_route(
                boundary,
                return_to_home=bool(planner_settings["return_to_home"]),
                robot_speed_ms=float(
                    cfg.robot.max_speed_cms / 100.0
                    * max(nav.snapshot()["target_speed_pct"], 1) / 100.0
                ),
            )
        edge_buffer_m = (
            cfg.safety.geofence_buffer_m + 0.05
            if nav.mode is NavMode.NON_GPS
            else 0.0
        )
        return plan_coverage(PlanRequest(
            boundary=tuple(boundary),
            cleaning_width_m=float(planner_settings["cleaning_width_m"]),
            overlap_pct=float(planner_settings["overlap_pct"]),
            sweep_angle_deg=planner_settings["sweep_angle_deg"],
            return_to_home=bool(planner_settings["return_to_home"]),
            robot_speed_ms=float(
                cfg.robot.max_speed_cms / 100.0
                * max(nav.snapshot()["target_speed_pct"], 1) / 100.0
            ),
            edge_buffer_m=edge_buffer_m,
        ))

    def _generate_plan_from_capture(source: str) -> tuple[bool, str]:
        nonlocal last_plan_stats
        if capture.point_count < cfg.planner.min_polygon_points:
            return False, (
                f"Need at least {cfg.planner.min_polygon_points} saved points"
            )
        boundary = list(capture.points)
        plan = _build_plan(boundary)
        if not plan.waypoints:
            return False, "Path generation failed for the saved boundary"
        last_plan_stats = _plan_payload(plan)
        name = f"drive_boundary_{int(time.time())}"
        nav.set_mission(
            name,
            boundary,
            list(plan.waypoints),
            path_mode=str(planner_settings["path_mode"]),
            geofence_buffer_m=_mission_geofence_buffer_m(),
        )
        capture.mark_planned(source=source)
        payload = {
            "name": name,
            "boundary": [{"lat": p.lat, "lon": p.lon} for p in boundary],
            "waypoints": [
                {"seq": i, "lat": w.lat, "lon": w.lon}
                for i, w in enumerate(plan.waypoints)
            ],
            "params": dict(planner_settings),
            "stats": _plan_payload(plan),
        }
        saved_filename = store.save(name, payload)
        _emit("plan_generated", {
            "source": source,
            "rows": plan.rows,
            "distance_m": round(plan.distance_m, 1),
            "path_mode": planner_settings["path_mode"],
        })
        _emit_sync(saved_filename=saved_filename)
        if planner_settings["path_mode"] == "boundary":
            return True, f"Saved boundary route with {plan.rows} segments"
        return True, f"Generated {plan.rows} lanes from {capture.point_count} points"

    def _start_or_resume_mission() -> tuple[bool, str]:
        mission = nav.snapshot()["mission"]
        state = mission["state"]
        if state == "paused":
            nav.resume_mission()
            return True, "Mission resumed"
        if mission["waypoints"]:
            if nav.start_mission():
                return True, "Mission started"
        return False, "No planned path yet"

    def _plan_or_start(source: str) -> tuple[bool, str]:
        if capture.recording:
            return _generate_plan_from_capture(source)
        return _start_or_resume_mission()

    def _reset_capture_and_mission(source: str) -> tuple[bool, str]:
        nonlocal last_plan_stats
        mission = nav.snapshot()["mission"]
        capture.clear()
        last_plan_stats = {}
        if mission["state"] != "idle" or mission["wp_total"]:
            nav.abort_mission()
        nav.clear_mission()
        _emit("boundary_reset", {"source": source})
        _emit_sync()
        return True, "Boundary and mission reset"

    # ── Background bridges: hardware threads ↔ estimator/safety ──────────

    def _gps_pump() -> None:
        while True:
            time.sleep(0.2)
            if gps is None:
                continue
            fix = gps.get_fix()
            if fix.has_fix:
                estimator.update_gps(fix.lat, fix.lon, fix.heading,
                                     fix.speed_ms, has_fix=True)
                safety.mark_gps_fix()
            sio.emit("gps", fix.to_dict())

    def _robot_pump() -> None:
        while True:
            time.sleep(0.1)
            if robot is None:
                continue
            status = robot.get_status()
            if status.connected:
                safety.mark_status_received()
            estimator.update_imu(status.heading, status.speed_cms)
            sio.emit("robot", status.to_dict())

    def _radio_pump() -> None:
        nonlocal last_manual_until
        radio_driving = False
        prev_mark = False
        prev_plan = False
        prev_abort = False
        while True:
            time.sleep(0.05)
            status = radio.get_status()
            sio.emit("radio", status.to_dict())
            mark_now = status.switch_high(cfg.hardware.radio_mark_channel)
            plan_now = status.switch_high(cfg.hardware.radio_plan_channel)
            abort_now = status.switch_high(cfg.hardware.radio_abort_channel)
            if mark_now and not prev_mark:
                ok, msg = _record_boundary_point("radio")
                _emit("ui_message", {
                    "level": "good" if ok else "bad",
                    "text": msg,
                })
            if plan_now and not prev_plan:
                ok, msg = _plan_or_start("radio")
                _emit("ui_message", {
                    "level": "good" if ok else "bad",
                    "text": msg,
                })
            if not plan_now and prev_plan and nav.snapshot()["mission"]["state"] == "running":
                nav.pause_mission()
            if abort_now and not prev_abort:
                ok, msg = _reset_capture_and_mission("radio")
                _emit("ui_message", {
                    "level": "good" if ok else "bad",
                    "text": msg,
                })
            prev_mark, prev_plan, prev_abort = mark_now, plan_now, abort_now
            if robot is None:
                continue
            mission_state = nav.snapshot()["mission"]["state"]
            mission_running = mission_state == "running"
            manual_hold = time.time() < last_manual_until
            can_drive = (
                status.control_active
                and not mission_running
                and not manual_hold
            )
            if can_drive:
                robot.move(status.steer_pct, status.throttle_pct)
                radio_driving = True
            elif radio_driving:
                robot.stop_robot()
                radio_driving = False

    def _nav_state_pump() -> None:
        while True:
            time.sleep(0.5)
            sio.emit("mission_state", nav.snapshot()["mission"])

    def _pose_pump() -> None:
        while True:
            time.sleep(0.2)
            sio.emit("pose", estimator.pose().to_dict())

    threading.Thread(target=_gps_pump, daemon=True, name="gps_pump").start()
    threading.Thread(target=_robot_pump, daemon=True, name="robot_pump").start()
    threading.Thread(target=_radio_pump, daemon=True, name="radio_pump").start()
    threading.Thread(target=_nav_state_pump, daemon=True,
                     name="nav_pump").start()
    threading.Thread(target=_pose_pump, daemon=True, name="pose_pump").start()

    # ── Routes ────────────────────────────────────────────────────────────

    @app.route("/")
    def index():
        return render_template(
            "index.html",
            version=app_version,
            static_version=str(int(time.time())),
        )

    @app.route("/api/health")
    def api_health():
        return jsonify({
            "ok": True,
            "version": app_version,
            "gps_enabled": gps is not None,
            "gps_connected": bool(gps and gps.connected),
            "gps_has_data": bool(gps and gps.last_sentence_t),
            "gps_has_fix": bool(gps and gps.get_fix().has_fix),
            "gps_port": cfg.hardware.gps_port,
            "gps_actual_port": gps.current_port if gps else "",
            "robot_enabled": robot is not None,
        })

    # Mission CRUD ─────────────────────────────────────────────────────────

    @app.route("/api/missions")
    def api_missions_list():
        return jsonify([m.to_dict() for m in store.list()])

    @app.route("/api/missions/<filename>", methods=["GET"])
    def api_mission_load(filename: str):
        nonlocal last_plan_stats
        data = store.load(filename)
        if data is None:
            return jsonify({"error": "not found"}), 404
        try:
            _update_planner_settings(data.get("params", {}))
        except (TypeError, ValueError):
            pass
        boundary = [LatLon(p["lat"], p["lon"]) for p in data.get("boundary", [])]
        waypoints = [LatLon(p["lat"], p["lon"])
                     for p in data.get("waypoints", [])]
        _set_capture_boundary(boundary, source="mission", planned=bool(waypoints))
        nav.set_mission(
            data.get("name", filename),
            boundary,
            waypoints,
            path_mode=str(planner_settings["path_mode"]),
            geofence_buffer_m=_mission_geofence_buffer_m(),
        )
        last_plan_stats = dict(data.get("stats", {}))
        _emit_sync()
        return jsonify(data)

    @app.route("/api/missions/<filename>", methods=["DELETE"])
    def api_mission_delete(filename: str):
        return jsonify({"ok": store.delete(filename)})

    @app.route("/api/missions", methods=["POST"])
    def api_mission_save():
        body = request.get_json(force=True) or {}
        name = (body.get("name") or "").strip() or f"mission_{int(time.time())}"
        snap = nav.snapshot()["mission"]
        params = dict(planner_settings)
        params.update(body.get("params", {}))
        payload = {
            "name": name,
            "boundary": snap["boundary"],
            "waypoints": snap["waypoints"],
            "params": params,
            "stats": body.get("stats", {}),
        }
        filename = store.save(name, payload)
        return jsonify({"ok": True, "filename": filename})

    @app.route("/api/planner/settings", methods=["POST"])
    def api_planner_settings():
        body = request.get_json(force=True) or {}
        try:
            _update_planner_settings(body)
        except (TypeError, ValueError) as exc:
            return jsonify({"error": f"bad planner settings: {exc}"}), 400
        return jsonify({"ok": True, "settings": dict(planner_settings)})

    # Boundary + path generation ───────────────────────────────────────────

    @app.route("/api/plan", methods=["POST"])
    def api_plan():
        nonlocal last_plan_stats
        body = request.get_json(force=True) or {}
        boundary = body.get("boundary") or []
        if len(boundary) < cfg.planner.min_polygon_points:
            return jsonify({"error": "Boundary needs ≥ 3 points"}), 400
        try:
            _update_planner_settings(body)
        except (TypeError, ValueError) as exc:
            return jsonify({"error": f"bad params: {exc}"}), 400
        boundary_ll = [LatLon(p["lat"], p["lon"]) for p in boundary]
        plan = _build_plan(boundary_ll)
        last_plan_stats = _plan_payload(plan)
        nav.set_mission(
            name=body.get("name", ""),
            boundary=boundary_ll,
            waypoints=list(plan.waypoints),
            path_mode=str(planner_settings["path_mode"]),
            geofence_buffer_m=_mission_geofence_buffer_m(),
        )
        _set_capture_boundary(boundary_ll, source="map", planned=True)
        _emit_sync()
        return jsonify({"ok": True, "plan": plan.to_dict()})

    @app.route("/api/boundary/mark", methods=["POST"])
    def api_boundary_mark():
        ok, msg = _record_boundary_point("web")
        code = 200 if ok else 400
        return jsonify({"ok": ok, "message": msg}), code

    @app.route("/api/boundary/plan_or_start", methods=["POST"])
    def api_boundary_plan_or_start():
        ok, msg = _plan_or_start("web")
        code = 200 if ok else 400
        return jsonify({"ok": ok, "message": msg}), code

    @app.route("/api/boundary/reset", methods=["POST"])
    def api_boundary_reset():
        ok, msg = _reset_capture_and_mission("web")
        return jsonify({"ok": ok, "message": msg})

    # Mission control ──────────────────────────────────────────────────────

    @app.route("/api/mode", methods=["POST"])
    def api_mode():
        body = request.get_json(force=True) or {}
        try:
            nav.set_mode(NavMode(body.get("mode", "gps")))
            return jsonify({"ok": True, "mode": nav.mode.value})
        except ValueError:
            return jsonify({"error": "mode must be 'gps' or 'non_gps'"}), 400

    @app.route("/api/speed", methods=["POST"])
    def api_speed():
        body = request.get_json(force=True) or {}
        nav.set_speed_pct(int(body.get("pct", cfg.navigation.target_speed_pct)))
        return jsonify({"ok": True})

    @app.route("/api/run/start", methods=["POST"])
    def api_run_start():
        if nav.start_mission():
            return jsonify({"ok": True})
        return jsonify({"error": "no waypoints"}), 400

    @app.route("/api/run/pause", methods=["POST"])
    def api_run_pause():
        nav.pause_mission()
        return jsonify({"ok": True})

    @app.route("/api/run/resume", methods=["POST"])
    def api_run_resume():
        nav.resume_mission()
        return jsonify({"ok": True})

    @app.route("/api/run/abort", methods=["POST"])
    def api_run_abort():
        nav.abort_mission()
        return jsonify({"ok": True})

    @app.route("/api/estop", methods=["POST"])
    def api_estop():
        body = request.get_json(silent=True) or {}
        if body.get("clear"):
            safety.clear_estop()
        else:
            safety.trigger_estop("user")
            nav.abort_mission()
        return jsonify({"ok": True, "estop": safety.estop_active})

    # Manual drive ─────────────────────────────────────────────────────────

    @app.route("/api/manual", methods=["POST"])
    def api_manual():
        nonlocal last_manual_until
        body = request.get_json(force=True) or {}
        if robot is not None:
            robot.move(int(body.get("steer", 0)), int(body.get("throttle", 0)))
        last_manual_until = time.time() + 0.75
        return jsonify({"ok": True})

    @app.route("/api/manual/stop", methods=["POST"])
    def api_manual_stop():
        nonlocal last_manual_until
        if robot is not None:
            robot.stop_robot()
        last_manual_until = time.time() + 0.25
        return jsonify({"ok": True})

    # Snapshot ─────────────────────────────────────────────────────────────

    @app.route("/api/state")
    def api_state():
        snap = nav.snapshot()
        return jsonify({
            "mission": snap["mission"],
            "mode": snap["mode"],
            "target_speed_pct": snap["target_speed_pct"],
            "pose": estimator.pose().to_dict(),
            "robot": (robot.get_status().to_dict() if robot else {}),
            "radio": radio.get_status().to_dict(),
            "gps": ({
                **gps.get_fix().to_dict(),
                "connected": gps.connected,
                "port": cfg.hardware.gps_port,
                "actual_port": gps.current_port,
                "has_data": bool(gps.last_sentence_t),
                "last_sentence_t": gps.last_sentence_t,
            } if gps else {}),
            "capture": capture.to_dict(),
            "planner_settings": dict(planner_settings),
            "plan_stats": dict(last_plan_stats),
            "estop": safety.estop_active,
            "config": {
                "cleaning_width_m": cfg.robot.cleaning_width_m,
                "overlap_pct": cfg.planner.overlap_pct,
                "row_spacing_m": cfg.planner.row_spacing_m,
                "geofence_buffer_m": cfg.safety.geofence_buffer_m,
            },
        })

    # Socket.IO ────────────────────────────────────────────────────────────

    @sio.on("connect")
    def on_connect():
        snap = nav.snapshot()
        emit("hello", {
            "version": app_version,
            "mission": snap["mission"],
            "capture": capture.to_dict(),
        })

    return app, sio, nav


# ─── Entry point ──────────────────────────────────────────────────────────────


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Solar Panel Cleaning Robot — server")
    p.add_argument("--config", default=None, help="Path to config.yaml")
    p.add_argument("--no-gps", action="store_true", help="Disable GPS reader")
    p.add_argument("--no-robot", action="store_true", help="Disable Pico link")
    p.add_argument(
        "--ignore-battery",
        action="store_true",
        help="Disable battery safety checks for indoor testing",
    )
    p.add_argument("--host", default=None, help="HTTP bind host")
    p.add_argument("--port", type=int, default=None, help="HTTP port")
    args = p.parse_args(argv)

    cfg = load_config(args.config)
    if args.ignore_battery:
        cfg = replace(
            cfg,
            safety=replace(cfg.safety, ignore_battery_for_testing=True),
        )
    logging.basicConfig(
        level=cfg.server.log_level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )
    if args.ignore_battery:
        log.warning("Battery safety checks disabled for indoor testing")

    app, sio, _ = create_app(cfg, with_gps=not args.no_gps,
                             with_robot=not args.no_robot)

    host = args.host or cfg.server.host
    port = args.port or cfg.server.port
    log.info("Solar Bot mission planner v2.2.1 — http://%s:%d",
             "localhost" if host == "0.0.0.0" else host, port)
    sio.run(app, host=host, port=port,
            debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
