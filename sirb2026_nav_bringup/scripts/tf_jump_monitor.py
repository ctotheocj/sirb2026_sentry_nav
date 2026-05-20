#!/usr/bin/env python3

import math

import rclpy
from rclpy.duration import Duration
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener


def _yaw(q):
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def _angle_diff(a, b):
    d = a - b
    while d > math.pi:
        d -= 2.0 * math.pi
    while d < -math.pi:
        d += 2.0 * math.pi
    return d


def _stamp_to_sec(stamp):
    return float(stamp.sec) + float(stamp.nanosec) * 1.0e-9


def _source_label(parent, child):
    if parent == "map" and child == "odom":
        return "relocalization/map_to_odom"
    if parent == "odom" and child == "base_footprint":
        return "lio_or_wheel_odom"
    if parent == "base_footprint" and child == "gimbal_yaw":
        return "base_to_gimbal_mount"
    if parent == "gimbal_yaw" and child == "gimbal_yaw_fake":
        return "fake_yaw_frame"
    return "unknown"


class TfJumpMonitor(Node):
    def __init__(self):
        super().__init__("tf_jump_monitor")

        self.declare_parameter("edges", [
            "map:odom",
            "odom:base_footprint",
            "base_footprint:gimbal_yaw",
            "gimbal_yaw:gimbal_yaw_fake",
        ])
        self.declare_parameter("poll_rate_hz", 30.0)
        self.declare_parameter("jump_distance_threshold", 0.20)
        self.declare_parameter("jump_speed_threshold", 6.0)
        self.declare_parameter("jump_yaw_threshold", 0.35)
        self.declare_parameter("odom_base_yaw_rate_threshold", 5.0)
        self.declare_parameter("max_dt", 0.50)
        self.declare_parameter("odom_base_max_dt", 0.70)
        self.declare_parameter("warn_period_sec", 0.30)
        self.declare_parameter("missing_warn_period_sec", 5.0)
        self.declare_parameter("startup_grace_sec", 5.0)
        self.declare_parameter("max_tf_age_sec", 0.50)

        self.start_time = self.get_clock().now()
        self.edges = []
        for item in self.get_parameter("edges").value:
            parts = str(item).split(":")
            if len(parts) != 2:
                self.get_logger().warn(f"Ignoring invalid TF edge '{item}', expected parent:child")
                continue
            self.edges.append((parts[0], parts[1]))

        self.dist_threshold = float(self.get_parameter("jump_distance_threshold").value)
        self.speed_threshold = float(self.get_parameter("jump_speed_threshold").value)
        self.yaw_threshold = float(self.get_parameter("jump_yaw_threshold").value)
        self.odom_base_yaw_rate_threshold = float(
            self.get_parameter("odom_base_yaw_rate_threshold").value
        )
        self.max_dt = float(self.get_parameter("max_dt").value)
        self.odom_base_max_dt = float(self.get_parameter("odom_base_max_dt").value)
        self.warn_period = float(self.get_parameter("warn_period_sec").value)
        self.missing_warn_period = float(self.get_parameter("missing_warn_period_sec").value)
        self.startup_grace = float(self.get_parameter("startup_grace_sec").value)
        self.max_tf_age = float(self.get_parameter("max_tf_age_sec").value)

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)
        self.last = {}
        self.last_warn = {}
        self.last_missing_warn = {}
        self.missing = set()

        rate = max(1.0, float(self.get_parameter("poll_rate_hz").value))
        self.timer = self.create_timer(1.0 / rate, self.tick)

    def tick(self):
        now = self.get_clock().now()
        for parent, child in self.edges:
            key = f"{parent}->{child}"
            source = _source_label(parent, child)
            try:
                transform = self.tf_buffer.lookup_transform(
                    parent,
                    child,
                    rclpy.time.Time(),
                    timeout=Duration(seconds=0.0),
                )
            except TransformException as exc:
                elapsed = (now - self.start_time).nanoseconds * 1.0e-9
                if elapsed < self.startup_grace:
                    self.missing.add(key)
                    continue
                last_warn = self.last_missing_warn.get(key)
                can_warn = (
                    last_warn is None
                    or (now - last_warn).nanoseconds * 1.0e-9 >= self.missing_warn_period
                )
                if can_warn:
                    self.last_missing_warn[key] = now
                    self.get_logger().warn(f"{key}: TF unavailable: {exc}")
                self.missing.add(key)
                continue

            if key in self.missing:
                self.missing.remove(key)
                self.get_logger().info(f"{key}: TF available")

            tr = transform.transform.translation
            rot = transform.transform.rotation
            stamp = _stamp_to_sec(transform.header.stamp)
            sample = (
                now,
                tr.x,
                tr.y,
                tr.z,
                _yaw(rot),
                stamp,
            )

            now_sec = now.nanoseconds * 1.0e-9
            age = now_sec - stamp
            if stamp == 0.0:
                pass
            elif age > self.max_tf_age:
                last_warn = self.last_warn.get(f"{key}:stale")
                can_warn = (
                    last_warn is None
                    or (now - last_warn).nanoseconds * 1.0e-9 >= self.warn_period
                )
                if can_warn:
                    self.last_warn[f"{key}:stale"] = now
                    self.get_logger().warn(
                        f"{key} stale source={source}: age={age:.3f}s "
                        f"limit={self.max_tf_age:.3f}s stamp={stamp:.3f}s"
                    )
            elif age < -0.05:
                last_warn = self.last_warn.get(f"{key}:future")
                can_warn = (
                    last_warn is None
                    or (now - last_warn).nanoseconds * 1.0e-9 >= self.warn_period
                )
                if can_warn:
                    self.last_warn[f"{key}:future"] = now
                    self.get_logger().warn(
                        f"{key} future-dated source={source}: age={age:.3f}s stamp={stamp:.3f}s"
                    )

            if key in self.last:
                prev = self.last[key]
                dt_wall = (sample[0] - prev[0]).nanoseconds * 1.0e-9
                dt_tf = sample[5] - prev[5]
                max_dt = (
                    self.odom_base_max_dt
                    if parent == "odom" and child == "base_footprint"
                    else self.max_dt
                )
                dt = dt_tf if 1.0e-4 < dt_tf < max_dt else dt_wall
                if 1.0e-4 < dt < max_dt:
                    dx = sample[1] - prev[1]
                    dy = sample[2] - prev[2]
                    dz = sample[3] - prev[3]
                    dist_xy = math.hypot(dx, dy)
                    dist_xyz = math.sqrt(dx * dx + dy * dy + dz * dz)
                    speed_xy = dist_xy / dt
                    dyaw = abs(_angle_diff(sample[4], prev[4]))
                    yaw_rate = dyaw / dt
                    trans_jump = (
                        dist_xy > self.dist_threshold
                        and speed_xy > self.speed_threshold
                    )
                    yaw_jump = dyaw > self.yaw_threshold
                    if parent == "odom" and child == "base_footprint":
                        yaw_jump = yaw_jump and yaw_rate > self.odom_base_yaw_rate_threshold
                    if trans_jump or yaw_jump:
                        last_warn = self.last_warn.get(key)
                        can_warn = (
                            last_warn is None
                            or (now - last_warn).nanoseconds * 1.0e-9 >= self.warn_period
                        )
                        if can_warn:
                            self.last_warn[key] = now
                            self.get_logger().warn(
                                f"{key} jump source={source}: dxy={dist_xy:.3f}m "
                                f"dxyz={dist_xyz:.3f}m dt_tf={dt_tf:.3f}s "
                                f"dt_wall={dt_wall:.3f}s "
                                f"speed_xy={speed_xy:.2f}m/s dyaw={dyaw:.3f}rad "
                                f"yaw_rate={yaw_rate:.2f}rad/s"
                            )

            self.last[key] = sample


def main():
    rclpy.init()
    node = TfJumpMonitor()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
