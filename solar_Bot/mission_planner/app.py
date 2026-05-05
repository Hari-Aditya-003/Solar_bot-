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
from pathlib import Path

from flask import Flask, jsonify, render_template, request
from flask_socketio import SocketIO, emit

from .config import Config, load_config
from .geo import LatLon
from .gps_reader import GPSReader
from .mission_store import MissionStore
from .navigation import NavMode, Navigator
from .path_planner import PlanRequest, plan_coverage
from .robot_bridge import RobotBridge
from .safety import SafetyMonitor
from .sensor_fusion import Estimator

log = logging.getLogger(__name__)


# ─── App factory ──────────────────────────────────────────────────────────────


def create_app(cfg: Config, *, with_gps: bool = True,
               with_robot: bool = True) -> tuple[Flask, SocketIO, Navigator]:
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

    if gps is not None:
        gps.start()
    if robot is not None:
        robot.start()
    nav.start()

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
    threading.Thread(target=_nav_state_pump, daemon=True,
                     name="nav_pump").start()
    threading.Thread(target=_pose_pump, daemon=True, name="pose_pump").start()

    # ── Routes ────────────────────────────────────────────────────────────

    @app.route("/")
    def index():
        return render_template("index.html", version="2.0.0")

    @app.route("/api/health")
    def api_health():
        return jsonify({
            "ok": True,
            "version": "2.0.0",
            "gps_enabled": gps is not None,
            "robot_enabled": robot is not None,
        })

    # Mission CRUD ─────────────────────────────────────────────────────────

    @app.route("/api/missions")
    def api_missions_list():
        return jsonify([m.to_dict() for m in store.list()])

    @app.route("/api/missions/<filename>", methods=["GET"])
    def api_mission_load(filename: str):
        data = store.load(filename)
        if data is None:
            return jsonify({"error": "not found"}), 404
        boundary = [LatLon(p["lat"], p["lon"]) for p in data.get("boundary", [])]
        waypoints = [LatLon(p["lat"], p["lon"])
                     for p in data.get("waypoints", [])]
        nav.set_mission(data.get("name", filename), boundary, waypoints)
        return jsonify(data)

    @app.route("/api/missions/<filename>", methods=["DELETE"])
    def api_mission_delete(filename: str):
        return jsonify({"ok": store.delete(filename)})

    @app.route("/api/missions", methods=["POST"])
    def api_mission_save():
        body = request.get_json(force=True) or {}
        name = (body.get("name") or "").strip() or f"mission_{int(time.time())}"
        snap = nav.snapshot()["mission"]
        payload = {
            "name": name,
            "boundary": snap["boundary"],
            "waypoints": snap["waypoints"],
            "params": body.get("params", {}),
            "stats": body.get("stats", {}),
        }
        filename = store.save(name, payload)
        return jsonify({"ok": True, "filename": filename})

    # Boundary + path generation ───────────────────────────────────────────

    @app.route("/api/plan", methods=["POST"])
    def api_plan():
        body = request.get_json(force=True) or {}
        boundary = body.get("boundary") or []
        if len(boundary) < cfg.planner.min_polygon_points:
            return jsonify({"error": "Boundary needs ≥ 3 points"}), 400
        try:
            req = PlanRequest(
                boundary=tuple(LatLon(p["lat"], p["lon"]) for p in boundary),
                cleaning_width_m=float(body.get(
                    "cleaning_width_m", cfg.robot.cleaning_width_m)),
                overlap_pct=float(body.get(
                    "overlap_pct", cfg.planner.overlap_pct)),
                sweep_angle_deg=(
                    None if body.get("sweep_angle_deg") in (None, "", "auto")
                    else float(body["sweep_angle_deg"])
                ),
                return_to_home=bool(body.get(
                    "return_to_home", cfg.planner.return_to_home)),
                robot_speed_ms=float(body.get(
                    "robot_speed_ms", cfg.robot.max_speed_cms / 100.0 * 0.5)),
            )
        except (TypeError, ValueError) as exc:
            return jsonify({"error": f"bad params: {exc}"}), 400

        plan = plan_coverage(req)
        nav.set_mission(
            name=body.get("name", ""),
            boundary=[LatLon(p["lat"], p["lon"]) for p in boundary],
            waypoints=list(plan.waypoints),
        )
        return jsonify({"ok": True, "plan": plan.to_dict()})

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
        body = request.get_json(force=True) or {}
        if robot is not None:
            robot.move(int(body.get("steer", 0)), int(body.get("throttle", 0)))
        return jsonify({"ok": True})

    @app.route("/api/manual/stop", methods=["POST"])
    def api_manual_stop():
        if robot is not None:
            robot.stop_robot()
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
            "gps": (gps.get_fix().to_dict() if gps else {}),
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
        emit("hello", {"version": "2.0.0", "mission": snap["mission"]})

    return app, sio, nav


# ─── Entry point ──────────────────────────────────────────────────────────────


def main(argv: list[str] | None = None) -> int:
    p = argparse.ArgumentParser(description="Solar Panel Cleaning Robot — server")
    p.add_argument("--config", default=None, help="Path to config.yaml")
    p.add_argument("--no-gps", action="store_true", help="Disable GPS reader")
    p.add_argument("--no-robot", action="store_true", help="Disable Pico link")
    p.add_argument("--host", default=None, help="HTTP bind host")
    p.add_argument("--port", type=int, default=None, help="HTTP port")
    args = p.parse_args(argv)

    cfg = load_config(args.config)
    logging.basicConfig(
        level=cfg.server.log_level,
        format="%(asctime)s [%(levelname)s] %(name)s: %(message)s",
    )

    app, sio, _ = create_app(cfg, with_gps=not args.no_gps,
                             with_robot=not args.no_robot)

    host = args.host or cfg.server.host
    port = args.port or cfg.server.port
    log.info("Solar Bot mission planner v2.0.0 — http://%s:%d",
             "localhost" if host == "0.0.0.0" else host, port)
    sio.run(app, host=host, port=port,
            debug=False, use_reloader=False, allow_unsafe_werkzeug=True)
    return 0


if __name__ == "__main__":  # pragma: no cover
    sys.exit(main())
