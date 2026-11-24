"""RosPort: everything RosBackend needs from ROS, as plain Python types (no rclpy import here).

Two implementations: `rclpy_port.RclpyPort` (real topics / actions / TF) and `fake_port.FakePort` (tests).
All positions are metres in the `map` frame, all times sim seconds (/clock). A method that returns None means
"this fact has no source right now" - the backend turns that into UNAVAILABLE or an explicit 'unknown'.
"""
from __future__ import annotations

from typing import Any, Protocol

NAV_RUNNING, NAV_OK, NAV_ABORTED, NAV_CANCELED = "running", "succeeded", "aborted", "canceled"


class RosPort(Protocol):
    # ---- facts (topics / TF)
    def now(self) -> float: ...                                   # /clock
    def pose(self) -> tuple[float, float, float] | None: ...      # TF map -> base_link: x, y, yaw
    def speed(self) -> float | None: ...                          # |v| from odometry, m/s
    def tilt_deg(self) -> float | None: ...                       # from the IMU
    def policy_status(self) -> str | None: ...                    # /policy/status: policy | stand:<reason> | tripod
    def semantic_objects(self) -> list[dict] | None: ...          # semantic_map/objects JSON "objects" list
    def map_id(self) -> str: ...
    def map_cell(self, x: float, y: float) -> str | None: ...     # "free" | "occupied" | "unknown" (/map)
    def mapped_fraction(self) -> float | None: ...                # known share of the interior, 0..1 (frontier.map_stats)
    def obstacle_sectors(self, radius_m: float) -> list[dict]: ...  # [{bearing_deg, distance_m}], from /scan
    def battery_pct(self) -> float | None: ...                    # /battery (battery_sim_node, SIMULATED); None until a message arrived
    def leg_states(self) -> dict[str, str] | None: ...            # /gurdy/leg_health (leg_health_node); None until a message arrived
    def drain_alerts(self) -> list[dict]: ...                     # alert / event topic, since last call
    # ---- Nav2
    def nav_ready(self) -> bool: ...
    def global_costmap_ready(self) -> bool: ...                    # /global_costmap/costmap has been received (the map reached the costmap)
    def lifecycle_state(self, node: str) -> str | None: ...        # lifecycle label ("active", ...) of a Nav2 node, None if not answering
    def nav_send(self, x: float, y: float, yaw: float | None) -> int | None: ...    # navigate_to_pose -> handle
    def nav_state(self, handle: int) -> dict[str, Any]: ...       # {state, distance_remaining, recoveries}
    def nav_cancel(self, handle: int) -> None: ...
    def compute_path(self, x: float, y: float, start: tuple[float, float] | None = None) -> dict | None: ...  # compute_path_to_pose (from `start`, default the robot pose) -> {length_m, points} | None
    # ---- exploration (the `explore` action of frontier_explorer_node, serve_action:=true)
    def explore_available(self) -> bool: ...
    def explore_start(self, max_time_s: float, area_rect: list[float] | None) -> int | None: ...   # rect = [x0, y0, x1, y1] map frame
    def explore_state(self, handle: int) -> dict[str, Any]: ...   # {state}
    def explore_cancel(self, handle: int) -> None: ...
    # ---- actuation / safety
    def send_cmd(self, vx: float, vy: float, wz: float) -> None: ...   # cmd_vel_smoothed (min-speed lift + collision monitor stay in the path)
    def hold_zero(self, seconds: float) -> None: ...             # keep publishing zero velocity for `seconds` (a stopped mission must really stop)
    def set_estop(self, active: bool) -> None: ...                # /estop (latched) - gate node is NEW
    def estop_active(self) -> bool: ...
