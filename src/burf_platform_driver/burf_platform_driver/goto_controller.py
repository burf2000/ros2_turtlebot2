"""
GoToController — minimal "drive to one (x, y, theta) goal" controller.

Pure Python: no ROS imports, no I/O, no threading. The caller is responsible
for feeding it the latest pose (in the map frame) on every `tick()` and
publishing the returned (linear_x, angular_z) tuple as a Twist on /cmd_vel.

State machine:
    idle -> rotating -> driving -> arrived
                                 -> rotating (if final yaw was specified)
    any  -> failed (stall: no progress for stall_timeout_s)
    any  -> idle   (on cancel)

The split lives in `Drivers/ros2_driver/` per the Option-B architecture
decision in projects/the-platform/prompts/ros2-waypoint-nav-option-b.md
(PA repo): keep the real-time control loop on-robot, keep the route
sequencing on the server. This class implements the on-robot half.
"""
from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Optional


def _wrap_angle(a: float) -> float:
    """Wrap angle to (-pi, pi]."""
    return math.atan2(math.sin(a), math.cos(a))


@dataclass
class _Goal:
    x: float
    y: float
    theta: Optional[float]   # None => no final orientation required


class GoToController:
    """Single-waypoint controller. See module docstring."""

    # State constants
    IDLE = 'idle'
    ROTATING = 'rotating'
    DRIVING = 'driving'
    ARRIVED = 'arrived'
    FAILED = 'failed'
    CANCELLED = 'cancelled'

    def __init__(
        self,
        *,
        lin_speed: float = 0.15,        # m/s default cruise
        ang_speed: float = 0.25,        # rad/s default rotation — slower than
                                        #     0.4 so SLAM gets cleaner scans
                                        #     during turns (less motion blur on
                                        #     scan correlation)
        goal_xy_tol: float = 0.15,      # m   accept-radius. Tighter than the
                                        #     UI's pre-flight 0.30 m check so
                                        #     valid clicks don't immediately
                                        #     arrive — they actually drive.
        goal_yaw_tol: float = 0.15,     # rad accept-cone for final theta
        rotate_align_tol: float = 0.35, # rad start driving once heading-to-goal
                                        #     is within ~20°
        slowdown_dist: float = 0.40,    # m  start tapering linear speed here
        stall_timeout_s: float = 15.0,  # s  no progress -> failed (longer to
                                        #     give slow rotation time to align)
    ):
        self.lin_speed = lin_speed
        self.ang_speed = ang_speed
        self.goal_xy_tol = goal_xy_tol
        self.goal_yaw_tol = goal_yaw_tol
        self.rotate_align_tol = rotate_align_tol
        self.slowdown_dist = slowdown_dist
        self.stall_timeout_s = stall_timeout_s

        self._state: str = self.IDLE
        self._goal: Optional[_Goal] = None
        self._distance_remaining: float = float('inf')
        self._last_progress_t: float = 0.0
        self._best_distance: float = float('inf')
        self._goal_id: int = 0

    # ─── Public API ─────────────────────────────────────────────────────────

    @property
    def state(self) -> str:
        return self._state

    @property
    def distance_remaining(self) -> float:
        return self._distance_remaining

    @property
    def goal_id(self) -> int:
        return self._goal_id

    def set_goal(self, x: float, y: float, theta: Optional[float] = None) -> None:
        """Begin driving toward (x, y), optionally facing theta on arrival."""
        self._goal = _Goal(x=float(x), y=float(y),
                           theta=(float(theta) if theta is not None else None))
        self._state = self.ROTATING
        self._distance_remaining = float('inf')
        self._best_distance = float('inf')
        self._last_progress_t = 0.0
        self._goal_id += 1

    def cancel(self) -> None:
        self._goal = None
        self._state = self.CANCELLED

    def is_active(self) -> bool:
        return self._state in (self.ROTATING, self.DRIVING)

    def tick(self, pose_in_map: tuple[float, float, float], now_s: float):
        """Advance the controller by one step.

        Args:
            pose_in_map: (x, y, theta) of base_link in the map frame.
            now_s: monotonic timestamp in seconds.

        Returns:
            (twist | None, status_dict)
            twist is (linear_x, angular_z) when motion should be commanded,
            or None when the caller should not publish anything this tick.
        """
        status = {
            'state': self._state,
            'distance_remaining': self._distance_remaining,
            'goal_id': self._goal_id,
        }

        if self._state in (self.IDLE, self.ARRIVED, self.FAILED, self.CANCELLED):
            return None, status
        if self._goal is None:
            self._state = self.IDLE
            return None, dict(status, state=self._state)

        px, py, ptheta = pose_in_map
        gx, gy = self._goal.x, self._goal.y
        dx, dy = gx - px, gy - py
        dist = math.hypot(dx, dy)
        self._distance_remaining = dist
        status['distance_remaining'] = dist

        # Initialise / track stall watchdog
        if self._best_distance == float('inf'):
            self._best_distance = dist
            self._last_progress_t = now_s
        elif dist < self._best_distance - 0.05:   # 5 cm tighter counts as progress
            self._best_distance = dist
            self._last_progress_t = now_s
        elif (now_s - self._last_progress_t) > self.stall_timeout_s:
            self._state = self.FAILED
            return (0.0, 0.0), dict(status, state=self._state)

        # If close enough in position, handle final-theta / arrival
        if dist <= self.goal_xy_tol:
            if self._goal.theta is None:
                self._state = self.ARRIVED
                return (0.0, 0.0), dict(status, state=self._state)
            yaw_err = _wrap_angle(self._goal.theta - ptheta)
            if abs(yaw_err) <= self.goal_yaw_tol:
                self._state = self.ARRIVED
                return (0.0, 0.0), dict(status, state=self._state)
            # rotate in place to face final theta
            self._state = self.ROTATING
            ang = math.copysign(self.ang_speed, yaw_err)
            return (0.0, ang), dict(status, state=self._state)

        # Heading to goal
        heading = math.atan2(dy, dx)
        yaw_to_heading = _wrap_angle(heading - ptheta)

        if self._state == self.ROTATING:
            if abs(yaw_to_heading) <= self.rotate_align_tol:
                self._state = self.DRIVING
                # fall through to driving on next tick
            else:
                ang = math.copysign(self.ang_speed, yaw_to_heading)
                return (0.0, ang), dict(status, state=self._state)

        # DRIVING: forward modulated by heading alignment.
        # Linear speed tapers near the goal (distance) AND tapers further when
        # the heading is off (cosine-weighted). Yaw correction uses a gentle
        # proportional gain that doesn't overshoot at small errors. Together
        # this avoids the circle-driving pathology where a too-eager yaw gain
        # overshoots the heading, the robot drops back into ROTATING, then
        # accelerates forward again before correcting, ad infinitum.
        speed_frac = min(1.0, dist / self.slowdown_dist) if self.slowdown_dist > 0 else 1.0
        yaw_factor = max(0.0, math.cos(yaw_to_heading))   # 1 when aligned, 0 at ±90°
        lin = self.lin_speed * max(0.2, speed_frac) * yaw_factor
        # Gentle P-gain — 0.8 rad/s per rad of error, hard-clamped to ang_speed
        ang = max(-self.ang_speed, min(self.ang_speed, 0.8 * yaw_to_heading))
        return (lin, ang), dict(status, state=self._state)
