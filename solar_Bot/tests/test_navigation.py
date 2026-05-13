"""Navigator non-GPS behavior tests."""

from __future__ import annotations

import pytest

from mission_planner.config import Config, NavigationConfig, SafetyConfig
from mission_planner.geo import LatLon, bearing_deg
from mission_planner.navigation import Navigator, NavMode
from mission_planner.robot_bridge import RobotStatus
from mission_planner.safety import SafetyMonitor
from mission_planner.sensor_fusion import Estimator

BOUNDARY = [
    LatLon(19.07600, 72.87770),
    LatLon(19.07604, 72.87770),
    LatLon(19.07604, 72.87775),
    LatLon(19.07600, 72.87775),
]

WAYPOINTS = [
    LatLon(19.07601, 72.87771),
    LatLon(19.07601, 72.87774),
]


class FakeRobot:
    def __init__(self, *, heading: float = 0.0, battery_mv: int = 12_000) -> None:
        self.moves: list[tuple[int, int]] = []
        self._status = RobotStatus(
            heading=heading,
            battery_mv=battery_mv,
            connected=True,
        )

    def get_status(self) -> RobotStatus:
        return self._status

    def move(self, steer: int, throttle: int) -> None:
        self.moves.append((steer, throttle))

    def mission_start(self) -> None:
        pass

    def mission_pause(self) -> None:
        pass

    def mission_abort(self) -> None:
        pass

    def stop_robot(self) -> None:
        self.moves.append((0, 0))


def _make_nav(*, robot: FakeRobot | None = None, heading: float = 0.0) -> tuple[Navigator, Estimator]:
    cfg = Config(
        navigation=NavigationConfig(),
        safety=SafetyConfig(comms_timeout_s=999),
    )
    estimator = Estimator(cfg.navigation)
    fake_robot = robot or FakeRobot(heading=heading)
    safety = SafetyMonitor(cfg.safety)
    nav = Navigator(cfg, fake_robot, estimator, safety)
    nav.set_mode(NavMode.NON_GPS)
    nav.set_mission("test", BOUNDARY, WAYPOINTS)
    return nav, estimator


@pytest.mark.unit
def test_non_gps_start_aligns_heading_frame_to_first_segment() -> None:
    robot = FakeRobot(heading=37.0)
    nav, estimator = _make_nav(robot=robot)

    assert nav.start_mission() is True

    pose = estimator.pose()
    assert pose.has_position is True
    assert pose.source == "dead_reckon"
    assert pose.heading_deg == pytest.approx(bearing_deg(WAYPOINTS[0], WAYPOINTS[1]))


@pytest.mark.unit
def test_non_gps_turns_in_place_before_driving_large_heading_error() -> None:
    robot = FakeRobot(heading=0.0)
    nav, estimator = _make_nav(robot=robot)
    estimator.set_anchor(WAYPOINTS[0].lat, WAYPOINTS[0].lon)
    estimator.update_imu(0.0, speed_cms=0.0)

    nav._tick_non_gps(WAYPOINTS, 0, estimator.pose(), robot.get_status())

    assert robot.moves
    steer, throttle = robot.moves[-1]
    assert throttle == 0
    assert steer == nav.cfg.navigation.non_gps_turn_steer_pct


@pytest.mark.unit
def test_boundary_route_recovers_toward_nearest_segment() -> None:
    robot = FakeRobot(heading=180.0)
    nav, estimator = _make_nav(robot=robot)
    route = [BOUNDARY[0], BOUNDARY[1], BOUNDARY[2], BOUNDARY[3], BOUNDARY[0]]
    nav.set_mission("boundary", BOUNDARY, route, path_mode="boundary")
    inside = LatLon(19.07602, 72.877725)
    estimator.set_anchor(inside.lat, inside.lon)
    estimator.calibrate_yaw_to(180.0, 180.0)

    nav._tick_gps(route, 0, estimator.pose(), "boundary")

    assert robot.moves
    steer, throttle = robot.moves[-1]
    assert throttle > 0
    assert steer != 0


@pytest.mark.unit
def test_nearest_route_point_projects_inside_segment() -> None:
    nav, _ = _make_nav()
    here = LatLon(19.07602, 72.877725)
    route = [BOUNDARY[0], BOUNDARY[1], BOUNDARY[2], BOUNDARY[3], BOUNDARY[0]]

    dist, target, idx = nav._nearest_route_point_m(route, here)

    assert dist > 0
    assert idx in {0, 1, 2, 3}
    assert min(p.lat for p in BOUNDARY) <= target.lat <= max(p.lat for p in BOUNDARY)
    assert min(p.lon for p in BOUNDARY) <= target.lon <= max(p.lon for p in BOUNDARY)
