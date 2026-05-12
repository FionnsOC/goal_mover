#!/usr/bin/env python3
"""
goal_mover_node.py  (ROS 2 Humble)

Three independent profiles computed once per goal:
  vx, vy  -- trapezoidal profile by distance, direction = unit vector to goal
  wz      -- rectangular profile scheduled by path distance s:
               [0, orange_end)        rotate onto path heading
               [orange_end, blue_start)  coast (wz=0)
               [blue_start, path_length] rotate onto goal heading
             rate [rad/s] = angular_rate_deg_per_cm * v * pi/1.8
             (constant deg/cm regardless of speed)

Topics
  Sub:  /goal_pose  (geometry_msgs/PoseStamped)  -- transient-local so late
        /odom       (nav_msgs/Odometry)              subscribers get last msg
  Pub:  /cmd_vel    (geometry_msgs/Twist)

Parameters
  desired_linear_vel        0.2  m/s
  linear_accel              0.4  m/s2
  linear_decel              0.6  m/s2
  angular_rate_deg_per_cm   1.0  deg/cm
  angular_deadband          0.02 rad
  xy_goal_tolerance         0.10 m
  yaw_goal_tolerance        0.05 rad
  control_rate             20.0  Hz
"""

import csv
import math
import os
import time
from dataclasses import dataclass
from typing import Optional

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry


# ---------------------------------------------------------------------------
# Helpers
# ---------------------------------------------------------------------------

def quat_to_yaw(q) -> float:
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_angle(a: float) -> float:
    while a >  math.pi: a -= 2.0 * math.pi
    while a < -math.pi: a += 2.0 * math.pi
    return a


# ---------------------------------------------------------------------------
# TrapezoidalProfile  (linear CoM speed)
# ---------------------------------------------------------------------------

class TrapezoidalProfile:
    def __init__(self, accel: float, decel: float, v_max: float, v_min: float = 0.04):
        self.accel = accel
        self.decel = decel
        self.v_max = v_max
        self.v_min = v_min

    def compute(self, dist_remaining: float, current_speed: float, dt: float) -> float:
        if dist_remaining <= 0.0:
            return 0.0
        brake_dist = (current_speed ** 2) / (2.0 * self.decel)
        if dist_remaining <= brake_dist:
            target = math.sqrt(2.0 * self.decel * max(dist_remaining, 0.0))
        else:
            target = self.v_max
        if target > current_speed:
            cmd = min(target, current_speed + self.accel * dt)
        else:
            cmd = target
        if cmd < self.v_min:
            return 0.0
        return max(0.0, min(cmd, self.v_max))


# ---------------------------------------------------------------------------
# AngularSchedule
# ---------------------------------------------------------------------------

@dataclass
class AngularSchedule:
    """
    Rectangular wz profiles keyed to path distance s.

      orange  [0,          orange_end)  wz = orange_sign * rate(v)
      coast   [orange_end, blue_start)  wz = 0
      blue    [blue_start, path_length] wz = blue_sign   * rate(v)

    rate(v) = angular_rate_deg_per_cm * v * pi/1.8   [rad/s]

    Overlap case (orange_dist + blue_dist > path_length):
      blue_start = orange_end  (hard switch, no coast)
    """
    orange_end:  float = 0.0
    blue_start:  float = 0.0
    orange_sign: float = 0.0
    blue_sign:   float = 0.0
    overlapping: bool  = False

    def evaluate(self, s: float, v: float,
                 angular_rate_deg_per_cm: float, deadband: float) -> float:
        rate_rad_per_m = angular_rate_deg_per_cm * (math.pi / 180.0) * 100.0
        wz_mag = rate_rad_per_m * v
        if s < self.orange_end:
            out = self.orange_sign * wz_mag
        elif s >= self.blue_start:
            out = self.blue_sign * wz_mag
        else:
            out = 0.0
        return 0.0 if abs(out) < deadband else out


# ---------------------------------------------------------------------------
# GoalMoverNode
# ---------------------------------------------------------------------------

class GoalMoverNode(Node):

    def __init__(self):
        super().__init__('goal_mover_rviz_node')

        # Parameters
        self.declare_parameter('desired_linear_vel',      0.2)
        self.declare_parameter('linear_accel',            0.4)
        self.declare_parameter('linear_decel',            0.6)
        self.declare_parameter('angular_rate_deg_per_cm', 1.0)
        self.declare_parameter('angular_deadband',        0.02)
        self.declare_parameter('xy_goal_tolerance',       0.15)   # 15 cm to handle end-of-path drift
        self.declare_parameter('yaw_goal_tolerance',      0.05)
        self.declare_parameter('control_rate',           20.0)
        self.declare_parameter('csv_filename',            '')

        self.desired_linear_vel      = self.get_parameter('desired_linear_vel').value
        self.linear_accel            = self.get_parameter('linear_accel').value
        self.linear_decel            = self.get_parameter('linear_decel').value
        self.angular_rate_deg_per_cm = self.get_parameter('angular_rate_deg_per_cm').value
        self.angular_deadband        = self.get_parameter('angular_deadband').value
        self.xy_goal_tolerance       = self.get_parameter('xy_goal_tolerance').value
        self.yaw_goal_tolerance      = self.get_parameter('yaw_goal_tolerance').value
        self.control_rate            = self.get_parameter('control_rate').value
        self.csv_filename            = self.get_parameter('csv_filename').value

        # Odometry state
        self.robot_x   = 0.0
        self.robot_y   = 0.0
        self.robot_yaw = 0.0
        self.odom_received = False

        # Per-goal state
        self.goal:             Optional[PoseStamped] = None
        self.angular_schedule: AngularSchedule       = AngularSchedule()
        self.path_length    = 0.0
        self.goal_start_x   = 0.0
        self.goal_start_y   = 0.0
        self.goal_dx        = 0.0   # relative offset to goal (goal - start)
        self.goal_dy        = 0.0
        self.goal_yaw       = 0.0   # desired final heading (rad)
        self.current_speed  = 0.0
        self.last_tick_time = None

        # ── CSV LOGGER: state ────────────────────────────────────────────────
        self._csv_file   = None
        self._csv_writer = None
        self._csv_t0     = 0.0   # time of first tick for this goal (seconds)
        log_dir = os.path.expanduser('~/goal_mover_logs')
        os.makedirs(log_dir, exist_ok=True)
        self._log_dir = log_dir
        # ── END CSV LOGGER: state ─────────────────────────────────────────

        # Subscriptions.
        # /goal_pose: plain depth=10 matches RViz '2D Goal Pose' defaults
        #             (Reliable + Volatile).  No custom QoS object needed.
        # /odom:      must be BEST_EFFORT to match the robot driver publisher.
        #             This is the only reason a QoS object is needed here --
        #             removing it would cause a silent DDS incompatibility and
        #             the node would never receive odometry.
        self.goal_sub = self.create_subscription(
            PoseStamped, '/goal_pose_rviz', self._goal_callback, 10)
        self.odom_sub = self.create_subscription(
            Odometry, '/odom', self._odom_callback,
            QoSProfile(reliability=ReliabilityPolicy.BEST_EFFORT,
                       history=HistoryPolicy.KEEP_LAST, depth=10))
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 1)

        period_s = 1.0 / self.control_rate
        self.timer = self.create_timer(period_s, self._control_loop)

        self.get_logger().info(
            f'[GoalMover RViz] Ready. '
            f'angular_rate={self.angular_rate_deg_per_cm} deg/cm  '
            f'linear_vel={self.desired_linear_vel} m/s  '
            f'rate={self.control_rate} Hz'
        )

    # -------------------------------------------------------------------------

    # ── CSV LOGGER: close helper ─────────────────────────────────────────────
    def _close_csv(self):
        """Flush and close the current CSV log file if one is open."""
        if self._csv_file is not None:
            self._csv_file.flush()
            self._csv_file.close()
            self._csv_file   = None
            self._csv_writer = None
    # ── END CSV LOGGER: close helper ─────────────────────────────────────────

    # -------------------------------------------------------------------------

    def _odom_callback(self, msg: Odometry):
        self.robot_x   = msg.pose.pose.position.x
        self.robot_y   = msg.pose.pose.position.y
        self.robot_yaw = quat_to_yaw(msg.pose.pose.orientation)
        if not self.odom_received:
            self.odom_received = True
            self.get_logger().info(
                f'[GoalMover] First odom received: '
                f'({self.robot_x:.3f}, {self.robot_y:.3f}, '
                f'yaw={math.degrees(self.robot_yaw):.1f} deg)'
            )

    # -------------------------------------------------------------------------

    def _goal_callback(self, msg: PoseStamped):
        # If odom hasn't arrived yet, delay until it has.
        # This prevents goal_start being set to (0,0).
        if not self.odom_received:
            self.get_logger().warn(
                '[GoalMover] Goal received but no odom yet -- ignoring. '
                'Make sure /odom is publishing before sending a goal.')
            return

        self.goal          = msg
        self.goal_start_x  = self.robot_x
        self.goal_start_y  = self.robot_y
        self.current_speed = 0.05   # bootstrap above v_min so profile escapes zero
        self.last_tick_time = None

        # ── CSV LOGGER: open file on new goal ────────────────────────────────
        self._close_csv()
        if self.csv_filename:
            name = self.csv_filename if self.csv_filename.endswith('.csv')                    else self.csv_filename + '.csv'
        else:
            name = f'goal_{time.strftime("%Y%m%d_%H%M%S")}.csv'
        csv_path = os.path.join(self._log_dir, name)
        self._csv_file   = open(csv_path, 'w', newline='')
        self._csv_writer = csv.writer(self._csv_file)
        self._csv_writer.writerow(
            ['t_s', 's_m', 'path_length_m', 'xy_dist_m',
             'vx_mps', 'vy_mps', 'wz_radps',
             'speed_mps', 'robot_yaw_deg', 'phase'])
        self._csv_t0 = 0.0
        self.get_logger().info(f'[GoalMover] Logging to {csv_path}')
        # ── END CSV LOGGER: open file on new goal ─────────────────────────

        # Store goal as relative offset from current robot position.
        # This makes the node frame-agnostic: it works correctly regardless
        # of where the odom origin is (Gazebo world coords vs RViz (0,0)).
        goal_x   = msg.pose.position.x
        goal_y   = msg.pose.position.y
        self.goal_dx  = goal_x - self.goal_start_x
        self.goal_dy  = goal_y - self.goal_start_y
        self.goal_yaw = quat_to_yaw(msg.pose.orientation)
        goal_yaw = self.goal_yaw

        self.path_length = math.hypot(self.goal_dx, self.goal_dy)

        path_yaw = math.atan2(self.goal_dy, self.goal_dx)

        self.angular_schedule = self._build_schedule(
            start_yaw=self.robot_yaw,
            path_yaw=path_yaw,
            goal_yaw=goal_yaw,
            path_length=self.path_length,
        )

        self.get_logger().info(
            f'[GoalMover] New goal | '
            f'path={self.path_length:.3f} m | '
            f'path_yaw={math.degrees(path_yaw):.1f} deg | '
            f'goal_yaw={math.degrees(goal_yaw):.1f} deg'
        )
        self.get_logger().info(
            f'[GoalMover] Schedule | '
            f'orange_end={self.angular_schedule.orange_end:.3f} m | '
            f'blue_start={self.angular_schedule.blue_start:.3f} m | '
            f'overlap={"YES" if self.angular_schedule.overlapping else "no"}'
        )

    # -------------------------------------------------------------------------

    def _build_schedule(self, start_yaw, path_yaw, goal_yaw, path_length):
        orange_error = wrap_angle(path_yaw - start_yaw)
        blue_error   = wrap_angle(goal_yaw  - path_yaw)

        # Distance each rotation requires at the configured rate.
        # k [rad/m] = angular_rate_deg_per_cm * pi/180 * 100
        k = self.angular_rate_deg_per_cm * (math.pi / 180.0) * 100.0

        theta_o = abs(orange_error)   # rad, magnitude only
        theta_b = abs(blue_error)     # rad, magnitude only

        orange_dist = theta_o / k    # metres needed for orange
        blue_dist   = theta_b / k    # metres needed for blue

        # orange_sign: direction to rotate from start_yaw onto path_yaw.
        # Use signum of wrap(path_yaw - start_yaw) — shortest turn.
        orange_sign = 1.0 if orange_error >= 0.0 else -1.0

        overlapping = (orange_dist + blue_dist) > path_length

        if not overlapping:
            # Normal case: orange finishes at path_yaw, then optional coast,
            # then blue starts.
            orange_end = orange_dist
            blue_start = path_length - blue_dist
            switch_angle_deg = None

            # blue_sign: direction from path_yaw to goal_yaw.
            # Recompute via signum(wrap(goal_yaw - path_yaw)) to avoid the
            # +-180 deg floating-point boundary that flips the sign.
            blue_sign = 1.0 if wrap_angle(goal_yaw - path_yaw) >= 0.0 else -1.0

        else:
            # Overlap case: solve for the intersection point s* where the
            # remaining orange rotation equals the remaining blue rotation.
            #
            #   s* = L/2 + (theta_o - theta_b) / (2*k)
            #
            # Derivation: at s*, set remaining orange = remaining blue:
            #   theta_o - k*s* = theta_b - k*(L - s*)
            #   => s* = L/2 + (theta_o - theta_b) / (2*k)
            #
            # Clamp to [0, L] to handle extreme cases where one rotation is
            # so much larger that the switch would be outside the path.
            s_star = path_length / 2.0 + (theta_o - theta_b) / (2.0 * k)
            s_star = max(0.0, min(path_length, s_star))

            # Angle at the switch point (how far orange has progressed).
            angle_star = theta_o - k * s_star   # rad remaining in orange dir
            angle_star = max(0.0, angle_star)   # clamp, should not go negative

            orange_end  = s_star
            blue_start  = s_star   # hard switch: blue starts exactly where orange ends
            switch_angle_deg = math.degrees(angle_star)

            # blue_sign: shortest turn from the actual heading at the switch
            # point to goal_yaw.  This is the key fix: in the overlap case the
            # robot is NOT at path_yaw when blue starts, so we must compute the
            # direction from the real heading at s*, not from path_yaw.
            # This also avoids the +-180 deg floating-point boundary issue.
            heading_at_switch = start_yaw + orange_sign * (theta_o - angle_star)
            blue_sign = 1.0 if wrap_angle(goal_yaw - heading_at_switch) >= 0.0 else -1.0

        self.get_logger().info(
            f'[GoalMover] Angles | '
            f'orange={math.degrees(orange_error):.1f} deg ({orange_dist:.3f} m) | '
            f'blue={math.degrees(blue_error):.1f} deg ({blue_dist:.3f} m)'
        )
        if overlapping:
            self.get_logger().info(
                f'[GoalMover] Overlap | '
                f's*={orange_end:.3f} m | '
                f'angle at switch={switch_angle_deg:.1f} deg'
            )

        return AngularSchedule(
            orange_end=orange_end,
            blue_start=blue_start,
            orange_sign=orange_sign,
            blue_sign=blue_sign,
            overlapping=overlapping,
        )

    # -------------------------------------------------------------------------

    def _control_loop(self):
        if self.goal is None:
            self.cmd_vel_pub.publish(Twist())
            return

        # dt
        now = self.get_clock().now()
        if self.last_tick_time is None:
            dt = 1.0 / self.control_rate
        else:
            dt = (now - self.last_tick_time).nanoseconds * 1e-9
            if dt <= 0.0 or dt > 1.0:
                dt = 1.0 / self.control_rate
        self.last_tick_time = now

        # Goal geometry — computed from relative offset so odom origin does
        # not matter (works with both Gazebo world coords and RViz (0,0)).
        goal_yaw = self.goal_yaw
        dx      = (self.goal_start_x + self.goal_dx) - self.robot_x
        dy      = (self.goal_start_y + self.goal_dy) - self.robot_y
        xy_dist = math.hypot(dx, dy)

        # s: distance travelled along path from start, clamped to path_length
        s = min(
            math.hypot(self.robot_x - self.goal_start_x,
                       self.robot_y - self.goal_start_y),
            self.path_length,
        )

        # Goal reached check
        yaw_error = wrap_angle(goal_yaw - self.robot_yaw)
        if xy_dist < self.xy_goal_tolerance and abs(yaw_error) < self.yaw_goal_tolerance:
            self.get_logger().info(
                f'[GoalMover] Goal reached. '
                f'xy_dist={xy_dist:.4f} m  yaw_err={math.degrees(yaw_error):.2f} deg'
            )
            self.cmd_vel_pub.publish(Twist())
            # ── CSV LOGGER: close on goal reached ────────────────────────
            self._close_csv()
            # ── END CSV LOGGER: close on goal reached ────────────────────
            self.goal = None
            self.current_speed = 0.0
            return

        # Linear profile: trapezoidal speed, direction = unit vector to goal in body frame
        profile = TrapezoidalProfile(self.linear_accel, self.linear_decel, self.desired_linear_vel)
        speed = profile.compute(xy_dist, self.current_speed, dt)
        self.current_speed = speed

        cos_r =  math.cos(self.robot_yaw)
        sin_r =  math.sin(self.robot_yaw)
        bx    =  cos_r * dx + sin_r * dy
        by    = -sin_r * dx + cos_r * dy
        bd    =  math.hypot(bx, by)

        if bd > 1e-6:
            vx = (bx / bd) * speed
            vy = (by / bd) * speed
        else:
            vx = vy = 0.0

        # Angular profile
        # When the linear profile has finished (speed==0 near end of path) or
        # the robot is already within xy_goal_tolerance, switch to a direct
        # yaw correction at a fixed rate independent of speed.
        # If odometry drift pushed xy_dist back above tolerance while speed==0,
        # also re-enable a slow translation correction so the robot creeps back.
        at_xy_goal      = xy_dist < self.xy_goal_tolerance
        linear_finished = (speed == 0.0 and s >= self.path_length * 0.99)

        if at_xy_goal or linear_finished:
            wz_rate = (self.angular_rate_deg_per_cm * (math.pi / 180.0)
                       * 100.0 * self.desired_linear_vel)
            wz = (0.0 if abs(yaw_error) < self.angular_deadband
                  else math.copysign(wz_rate, yaw_error))
            # Re-enable slow translation if drift pushed us outside tolerance.
            if not at_xy_goal and bd > 1e-6:
                correction_speed = min(self.desired_linear_vel * 0.5,
                                       xy_dist * 2.0)
                vx = (bx / bd) * correction_speed
                vy = (by / bd) * correction_speed
        else:
            wz = self.angular_schedule.evaluate(
                s, speed, self.angular_rate_deg_per_cm, self.angular_deadband)

        cmd = Twist()
        cmd.linear.x  = vx
        cmd.linear.y  = vy
        cmd.angular.z = wz
        self.cmd_vel_pub.publish(cmd)

        # ── CSV LOGGER: write row ────────────────────────────────────────────
        if self._csv_writer is not None:
            t_now = self.get_clock().now().nanoseconds * 1e-9
            if self._csv_t0 == 0.0:
                self._csv_t0 = t_now
            t_rel = t_now - self._csv_t0
            if at_xy_goal or linear_finished:
                phase = 'goal_heading'
            elif s < self.angular_schedule.orange_end:
                phase = 'orange'
            elif s >= self.angular_schedule.blue_start:
                phase = 'blue'
            else:
                phase = 'coast'
            self._csv_writer.writerow([
                f'{t_rel:.4f}', f'{s:.4f}', f'{self.path_length:.4f}',
                f'{xy_dist:.4f}', f'{vx:.4f}', f'{vy:.4f}', f'{wz:.4f}',
                f'{speed:.4f}', f'{math.degrees(self.robot_yaw):.2f}', phase,
            ])
        # ── END CSV LOGGER: write row ─────────────────────────────────────

        self.get_logger().info(
            f's={s:.3f}/{self.path_length:.3f} m | xy={xy_dist:.3f} m | '
            f'spd={speed:.3f} | vx={vx:.3f} vy={vy:.3f} wz={wz:.3f}'
        )


# ---------------------------------------------------------------------------

def main(args=None):
    rclpy.init(args=args)
    node = GoalMoverNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # ── CSV LOGGER: close on shutdown ────────────────────────────────
        node._close_csv()
        # ── END CSV LOGGER: close on shutdown ────────────────────────────
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
