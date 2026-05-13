"""Robot bridge command behavior tests."""

from __future__ import annotations

import pytest

from mission_planner.robot_bridge import RobotBridge


@pytest.mark.unit
def test_move_allows_reverse_throttle() -> None:
    bridge = RobotBridge()
    bridge.move(10, -40)
    queued = bridge._cmd_q.get_nowait()
    assert queued == "MOVE 10 -40\n"
