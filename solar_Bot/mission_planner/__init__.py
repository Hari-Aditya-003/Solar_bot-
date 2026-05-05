"""Solar Panel Cleaning Robot — mission planner package.

Importable modules:
    config          Frozen-dataclass settings loader (config.yaml).
    geo             Geodesy + 2-D geometry helpers.
    path_planner    Boustrophedon coverage path generator.
    gps_reader      NEO-M9N NMEA reader (background thread).
    sensor_fusion   IMU + odometry dead-reckoning for non-GPS mode.
    robot_bridge    UART/UDP link to the Pico 2 W motor controller.
    navigation      Dual-mode (GPS / non-GPS) PID waypoint follower.
    safety          Geofence, e-stop, tilt/battery/comms watchdog.
    mission_store   Mission JSON load/save.
    app             Flask + Socket.IO server entry point.
"""

__version__ = "2.0.0"
