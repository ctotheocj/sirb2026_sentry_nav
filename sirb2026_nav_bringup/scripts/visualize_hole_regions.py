#!/usr/bin/env python3

import argparse
import math
from pathlib import Path

import rclpy
from rclpy.node import Node
from tf2_ros import Buffer, TransformException, TransformListener
from visualization_msgs.msg import Marker, MarkerArray
import yaml


def _load_holes(config_path):
    with Path(config_path).open("r", encoding="utf-8") as f:
        data = yaml.safe_load(f)

    params = (
        data.get("bt_navigator", {})
        .get("ros__parameters", {})
        .get("hole_pass", {})
    )
    hole_ids = params.get("hole_ids", [])
    holes = params.get("holes", {})
    result = []
    for hole_id in hole_ids:
        hole = holes.get(hole_id, {})
        for port_name in ("port_a_polygon", "port_b_polygon"):
            polygon = hole.get(port_name, [])
            if len(polygon) >= 6 and len(polygon) % 2 == 0:
                result.append((hole_id, port_name, _ordered_polygon([float(v) for v in polygon])))
    return result


def _ordered_polygon(polygon):
    if len(polygon) < 6 or len(polygon) % 2 != 0:
        return polygon
    points = [(polygon[i], polygon[i + 1]) for i in range(0, len(polygon), 2)]
    cx = sum(p[0] for p in points) / len(points)
    cy = sum(p[1] for p in points) / len(points)
    points.sort(key=lambda p: math.atan2(p[1] - cy, p[0] - cx))
    return [coord for point in points for coord in point]


def _point_in_polygon(x, y, polygon):
    if len(polygon) < 6 or len(polygon) % 2 != 0:
        return False
    inside = False
    points = [(polygon[i], polygon[i + 1]) for i in range(0, len(polygon), 2)]
    for i, p1 in enumerate(points):
        p0 = points[i - 1]
        if _distance_point_to_segment(x, y, p0[0], p0[1], p1[0], p1[1]) < 1e-6:
            return True
        crosses = ((p1[1] > y) != (p0[1] > y)) and (
            x < (p0[0] - p1[0]) * (y - p1[1]) / (p0[1] - p1[1] + 1e-9) + p1[0]
        )
        if crosses:
            inside = not inside
    return inside


def _distance_point_to_segment(px, py, ax, ay, bx, by):
    vx = bx - ax
    vy = by - ay
    wx = px - ax
    wy = py - ay
    length_sq = vx * vx + vy * vy
    if length_sq < 1e-9:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (wx * vx + wy * vy) / length_sq))
    return math.hypot(px - (ax + t * vx), py - (ay + t * vy))


class HoleRegionVisualizer(Node):
    def __init__(self, config_path_override=""):
        super().__init__("hole_region_visualizer")
        self.declare_parameter("config_path", "")
        self.declare_parameter("frame_id", "map")
        self.declare_parameter("topic", "hole_pass/hole_regions")
        self.declare_parameter("publish_period_sec", 1.0)
        self.declare_parameter("line_width", 0.04)
        self.declare_parameter("vertex_scale", 0.12)
        self.declare_parameter("z", 0.04)
        self.declare_parameter("robot_frame", "gimbal_yaw_fake")
        self.declare_parameter("print_region_state", True)

        self.frame_id = self.get_parameter("frame_id").value
        self.line_width = float(self.get_parameter("line_width").value)
        self.vertex_scale = float(self.get_parameter("vertex_scale").value)
        self.z = float(self.get_parameter("z").value)
        self.robot_frame = self.get_parameter("robot_frame").value
        self.print_region_state = bool(self.get_parameter("print_region_state").value)
        self.last_region = None

        self.tf_buffer = Buffer()
        self.tf_listener = TransformListener(self.tf_buffer, self)

        config_path = config_path_override or self.get_parameter("config_path").value
        if not config_path:
            raise RuntimeError("config_path parameter is required")

        self.regions = _load_holes(config_path)
        if not self.regions:
            self.get_logger().warn("No valid hole regions found in config")

        topic = self.get_parameter("topic").value
        self.pub = self.create_publisher(MarkerArray, topic, 1)
        period = max(0.1, float(self.get_parameter("publish_period_sec").value))
        self.timer = self.create_timer(period, self.publish_markers)
        self.region_timer = self.create_timer(period, self.print_robot_region)
        self.get_logger().info(
            f"Loaded {len(self.regions)} hole port regions from {config_path}, publishing {topic}"
        )

    def publish_markers(self):
        array = MarkerArray()
        stamp = self.get_clock().now().to_msg()
        marker_id = 0
        for idx, (hole_id, port_name, polygon) in enumerate(self.regions):
            color = self._color(idx)
            points = []
            for i in range(0, len(polygon), 2):
                p = self._point(polygon[i], polygon[i + 1])
                points.append(p)

            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = stamp
            line.ns = "hole_region_lines"
            line.id = marker_id
            marker_id += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.pose.orientation.w = 1.0
            line.scale.x = self.line_width
            line.color = color
            line.points = points + [points[0]]
            array.markers.append(line)

            vertices = Marker()
            vertices.header.frame_id = self.frame_id
            vertices.header.stamp = stamp
            vertices.ns = "hole_region_vertices"
            vertices.id = marker_id
            marker_id += 1
            vertices.type = Marker.SPHERE_LIST
            vertices.action = Marker.ADD
            vertices.pose.orientation.w = 1.0
            vertices.scale.x = self.vertex_scale
            vertices.scale.y = self.vertex_scale
            vertices.scale.z = self.vertex_scale
            vertices.color = color
            vertices.points = points
            array.markers.append(vertices)

            label = Marker()
            label.header.frame_id = self.frame_id
            label.header.stamp = stamp
            label.ns = "hole_region_labels"
            label.id = marker_id
            marker_id += 1
            label.type = Marker.TEXT_VIEW_FACING
            label.action = Marker.ADD
            label.pose.position = self._centroid(points)
            label.pose.position.z += 0.22
            label.pose.orientation.w = 1.0
            label.scale.z = 0.22
            label.color = color
            label.color.a = 1.0
            label.text = f"{hole_id}:{'A' if port_name == 'port_a_polygon' else 'B'}"
            array.markers.append(label)

        self.pub.publish(array)

    def print_robot_region(self):
        if not self.print_region_state:
            return
        try:
            tf = self.tf_buffer.lookup_transform(
                self.frame_id,
                self.robot_frame,
                rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
        except TransformException as ex:
            state = f"TF unavailable {self.frame_id}->{self.robot_frame}: {ex}"
            if state != self.last_region:
                self.get_logger().warn(state)
                self.last_region = state
            return

        x = tf.transform.translation.x
        y = tf.transform.translation.y
        matches = []
        for hole_id, port_name, polygon in self.regions:
            if _point_in_polygon(x, y, polygon):
                port = "A" if port_name == "port_a_polygon" else "B"
                matches.append(f"{hole_id} {port}")
        state = ", ".join(matches) if matches else "not in any hole region"
        if state != self.last_region:
            self.get_logger().info(f"robot at ({x:.3f}, {y:.3f}): {state}")
            self.last_region = state

    def _point(self, x, y):
        from geometry_msgs.msg import Point

        p = Point()
        p.x = float(x)
        p.y = float(y)
        p.z = self.z
        return p

    @staticmethod
    def _centroid(points):
        from geometry_msgs.msg import Point

        p = Point()
        if not points:
            return p
        p.x = sum(pt.x for pt in points) / len(points)
        p.y = sum(pt.y for pt in points) / len(points)
        p.z = sum(pt.z for pt in points) / len(points)
        return p

    @staticmethod
    def _color(index):
        from std_msgs.msg import ColorRGBA

        hue = (index * 0.61803398875) % 1.0
        r, g, b = _hsv_to_rgb(hue, 0.85, 1.0)
        c = ColorRGBA()
        c.r = float(r)
        c.g = float(g)
        c.b = float(b)
        c.a = 0.95
        return c


def _hsv_to_rgb(h, s, v):
    i = int(math.floor(h * 6.0))
    f = h * 6.0 - i
    p = v * (1.0 - s)
    q = v * (1.0 - f * s)
    t = v * (1.0 - (1.0 - f) * s)
    i %= 6
    if i == 0:
        return v, t, p
    if i == 1:
        return q, v, p
    if i == 2:
        return p, v, t
    if i == 3:
        return p, q, v
    if i == 4:
        return t, p, v
    return v, p, q


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--config", required=True, help="Path to nav2_params.yaml")
    args, ros_args = parser.parse_known_args()

    rclpy.init(args=ros_args)
    node = HoleRegionVisualizer(args.config)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
