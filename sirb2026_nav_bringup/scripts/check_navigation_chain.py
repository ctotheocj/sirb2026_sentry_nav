#!/usr/bin/env python3
"""Check the SIRB 2026 navigation acceptance topic chain."""

import argparse
import os
import re
import subprocess
import sys
import time
from dataclasses import dataclass
from typing import Iterable


@dataclass(frozen=True)
class TopicCheck:
    phase: str
    topic: str
    msg_type: str
    expected_rate: str
    required_publishers: tuple[str, ...] = ()
    required_subscribers: tuple[str, ...] = ()
    optional_subscribers: tuple[str, ...] = ()
    note: str = ""


@dataclass(frozen=True)
class AgeCheck:
    topic: str
    max_age_sec: float
    required: bool = True
    note: str = ""


@dataclass(frozen=True)
class TfCheck:
    target_frame: str
    source_frame: str
    timeout_sec: float = 1.0
    max_age_sec: float = 0.5
    required: bool = True
    note: str = ""


@dataclass(frozen=True)
class ActionCheck:
    name: str
    action_type: str
    required: bool = True
    note: str = ""


@dataclass(frozen=True)
class ServiceCheck:
    name: str
    srv_type: str
    required: bool = True
    note: str = ""


SUPPORT_CHECKS = (
    TopicCheck(
        phase="localization",
        topic="cloud_registered",
        msg_type="sensor_msgs/msg/PointCloud2",
        expected_rate="Point-LIO output",
        required_publishers=("point_lio",),
        required_subscribers=("loam_interface",),
        note="Point-LIO to registered_scan bridge input",
    ),
    TopicCheck(
        phase="localization",
        topic="lidar_odometry",
        msg_type="nav_msgs/msg/Odometry",
        expected_rate="registered_scan synchronized",
        required_publishers=("loam_interface",),
        required_subscribers=("sensor_scan_generation",),
        note="odom-frame lidar pose used to derive chassis odometry",
    ),
    TopicCheck(
        phase="localization",
        topic="registered_scan",
        msg_type="sensor_msgs/msg/PointCloud2",
        expected_rate="Point-LIO/loam output",
        required_publishers=("loam_interface",),
        optional_subscribers=(
            "ndt_omp_relocalization",
            "lidar_preprocessor",
            "sensor_scan_generation",
            "dynamic_point_detector",
        ),
        note="registered scan shared by perception and optional NDT fallback",
    ),
    TopicCheck(
        phase="localization",
        topic="odometry",
        msg_type="nav_msgs/msg/Odometry",
        expected_rate="sensor_scan_generation output",
        required_publishers=("sensor_scan_generation",),
        required_subscribers=(
            "dynamic_point_detector",
            "trajectory_manager",
            "controller_server",
            "grid_map_node",
            "fake_vel_transform",
        ),
        note="Shared state input for perception, trajectory ownership and MPC",
    ),
    TopicCheck(
        phase="static_costmap",
        topic="map",
        msg_type="nav_msgs/msg/OccupancyGrid",
        expected_rate="latched/static",
        required_publishers=("map_server",),
        required_subscribers=("global_costmap",),
        note="global_costmap.static_layer.map_topic",
    ),
    TopicCheck(
        phase="static_costmap",
        topic="obstacle_cloud",
        msg_type="sensor_msgs/msg/PointCloud2",
        expected_rate="registered_scan dependent",
        required_publishers=("lidar_preprocessor",),
        required_subscribers=("grid_map_node",),
        note="registered_scan filtered for online occupancy grid",
    ),
    TopicCheck(
        phase="static_costmap",
        topic="occupancy_grid",
        msg_type="nav_msgs/msg/OccupancyGrid",
        expected_rate="grid_map update",
        required_publishers=("grid_map_node",),
        required_subscribers=("global_costmap",),
        note="global_costmap.occupancy_grid_layer.topic",
    ),
)

SLAM_SUPPORT_CHECKS = (
    TopicCheck(
        phase="slam",
        topic="cloud_registered",
        msg_type="sensor_msgs/msg/PointCloud2",
        expected_rate="Point-LIO output",
        required_publishers=("point_lio",),
        required_subscribers=("loam_interface",),
        note="Point-LIO to registered_scan bridge input",
    ),
    TopicCheck(
        phase="slam",
        topic="registered_scan",
        msg_type="sensor_msgs/msg/PointCloud2",
        expected_rate="Point-LIO/loam output",
        required_publishers=("loam_interface",),
        optional_subscribers=("lidar_preprocessor", "sensor_scan_generation"),
        note="online map and obstacle-grid source in slam mode",
    ),
    TopicCheck(
        phase="slam",
        topic="map",
        msg_type="nav_msgs/msg/OccupancyGrid",
        expected_rate="slam_toolbox update",
        required_publishers=("slam_toolbox",),
        required_subscribers=("global_costmap",),
        note="global_costmap.static_layer.map_topic",
    ),
    TopicCheck(
        phase="slam",
        topic="obstacle_cloud",
        msg_type="sensor_msgs/msg/PointCloud2",
        expected_rate="registered_scan dependent",
        required_publishers=("lidar_preprocessor",),
        required_subscribers=("grid_map_node",),
        note="registered_scan filtered for online occupancy grid",
    ),
    TopicCheck(
        phase="slam",
        topic="occupancy_grid",
        msg_type="nav_msgs/msg/OccupancyGrid",
        expected_rate="grid_map update",
        required_publishers=("grid_map_node",),
        required_subscribers=("global_costmap",),
        note="global_costmap.occupancy_grid_layer.topic",
    ),
)

CORE_CHECKS = (
    TopicCheck(
        phase="trajectory",
        topic="trajectory_manager/trajectory_for_mpc",
        msg_type="sentry_nav_interfaces/msg/MincoTrajectory",
        expected_rate="20 Hz while active",
        required_publishers=("trajectory_manager",),
        required_subscribers=("controller_server",),
        optional_subscribers=("bt_navigator",),
        note="active trajectory owned by TrajectoryManager and tracked by MPC",
    ),
    TopicCheck(
        phase="control",
        topic="cmd_vel_controller",
        msg_type="geometry_msgs/msg/Twist",
        expected_rate="controller_frequency, 30 Hz configured",
        required_publishers=("controller_server",),
        required_subscribers=("velocity_smoother",),
        note="raw Nav2 controller output before velocity smoothing",
    ),
    TopicCheck(
        phase="control",
        topic="cmd_vel_selected",
        msg_type="geometry_msgs/msg/Twist",
        expected_rate="velocity_smoother rate, 30 Hz configured",
        required_publishers=("velocity_smoother",),
        required_subscribers=("fake_vel_transform",),
        note="smoothed Nav2 output consumed by fake_vel_transform",
    ),
    TopicCheck(
        phase="hole_pass",
        topic="mpc/hole_pass_cmd",
        msg_type="sentry_nav_interfaces/msg/HolePassCmd",
        expected_rate="navigation_mode_manager command period",
        required_publishers=("navigation_mode_manager",),
        note="lower/raise command plus yaw-rate command while hole-pass mode is active; raise otherwise",
    ),
    TopicCheck(
        phase="hole_pass",
        topic="navigation_mode_manager/mode",
        msg_type="std_msgs/msg/String",
        expected_rate="navigation_mode_manager command period",
        required_publishers=("navigation_mode_manager",),
        required_subscribers=("controller_server", "trajectory_manager"),
        note="mode gate for MPC predicted-collision stop and trajectory-manager forward collision check",
    ),
    TopicCheck(
        phase="control",
        topic="cmd_vel",
        msg_type="geometry_msgs/msg/Twist",
        expected_rate="final command stream",
        required_publishers=("fake_vel_transform",),
        note="terminal command; simulator or chassis subscriber depends on deployment",
    ),
)

PRIMARY_AGE_CHECKS = (
    AgeCheck("registered_scan", 0.5, note="localization/perception input freshness"),
    AgeCheck("odometry", 0.5, note="state estimate freshness"),
    AgeCheck("trajectory_manager/trajectory_for_mpc", 0.8, required=False, note="active only while navigating"),
)

DYNAMIC_OBSTACLE_CHECKS = (
    TopicCheck(
        phase="dynamic_perception",
        topic="registered_scan",
        msg_type="sensor_msgs/msg/PointCloud2",
        expected_rate="Point-LIO/loam output",
        required_publishers=("loam_interface",),
        required_subscribers=("dynamic_point_detector",),
        optional_subscribers=("lidar_preprocessor", "sensor_scan_generation"),
        note="entry point for dynamic point extraction",
    ),
    TopicCheck(
        phase="dynamic_perception",
        topic="dynamic_points",
        msg_type="sensor_msgs/msg/PointCloud2",
        expected_rate="registered_scan dependent",
        required_publishers=("dynamic_point_detector",),
        required_subscribers=("dynamic_obstacle_tracker",),
        note="M-detector output; must stay relative to namespace",
    ),
    TopicCheck(
        phase="dynamic_tracking",
        topic="dynamic_obstacles",
        msg_type="sentry_nav_interfaces/msg/TrackedObstacleArray",
        expected_rate="dynamic_points dependent",
        required_publishers=("dynamic_obstacle_tracker",),
        required_subscribers=("controller_server",),
        optional_subscribers=("bt_navigator", "smoother_server"),
        note="MPC/smoother dynamic obstacle constraints and BT trajectory replan checks",
    ),
)

DYNAMIC_OBSTACLE_AGE_CHECKS = (
    AgeCheck("dynamic_points", 0.8, note="M-detector output freshness"),
    AgeCheck("dynamic_obstacles", 0.8, note="tracked obstacle freshness"),
)

TF_CHECKS = (
    TfCheck("map", "odom", max_age_sec=0.35, note="localization transform"),
    TfCheck("odom", "gimbal_yaw_fake", max_age_sec=0.5, note="controller frame chain"),
    TfCheck("gimbal_yaw", "gimbal_yaw_fake", max_age_sec=0.5, required=False, note="fake_vel_transform yaw frame"),
)

ACTION_CHECKS = (
    ActionCheck(
        "safe_geometric_smoother/generate_minco_candidate",
        "sentry_nav_interfaces/action/GenerateMincoCandidate",
        note="BT candidate generation server",
    ),
    ActionCheck(
        "trajectory_manager/commit_trajectory",
        "sentry_nav_interfaces/action/CommitTrajectory",
        note="BT trajectory commit server",
    ),
)

SERVICE_CHECKS = (
    ServiceCheck(
        "navigation_mode_manager/set_navigation_mode",
        "sentry_nav_interfaces/srv/SetNavigationMode",
        note="BT hole-pass mode manager command service",
    ),
    ServiceCheck(
        "navigation_mode_manager/get_navigation_mode",
        "sentry_nav_interfaces/srv/GetNavigationMode",
        note="BT hole-pass mode manager state service",
    ),
    ServiceCheck(
        "global_costmap/occupancy_grid_layer/set_semantic_layer_mode",
        "sentry_nav_interfaces/srv/SetSemanticLayerMode",
        note="global occupancy_grid obstacle-layer suppression service",
    ),
    ServiceCheck(
        "local_costmap/occupancy_grid_layer/set_semantic_layer_mode",
        "sentry_nav_interfaces/srv/SetSemanticLayerMode",
        note="local occupancy_grid obstacle-layer suppression service",
    ),
)


def run(command: list[str], timeout: float = 5.0) -> tuple[int, str]:
    try:
        proc = subprocess.run(
            command,
            check=False,
            stdout=subprocess.PIPE,
            stderr=subprocess.STDOUT,
            text=True,
            timeout=timeout,
            env=ros_subprocess_env(),
        )
        return proc.returncode, proc.stdout
    except subprocess.TimeoutExpired as exc:
        output = exc.stdout or ""
        return 124, output + "\ncommand timed out"


def ros_subprocess_env() -> dict[str, str]:
    env = os.environ.copy()
    log_dir = env.get("ROS_LOG_DIR", "/tmp/sirb2026_nav_health_logs")
    os.makedirs(log_dir, exist_ok=True)
    env["ROS_LOG_DIR"] = log_dir
    return env


def scoped_topic(namespace: str, topic: str) -> str:
    if topic.startswith("/"):
        return topic
    ns = namespace.strip("/")
    if not ns:
        return "/" + topic
    return "/" + ns + "/" + topic


def scoped_action(namespace: str, name: str) -> str:
    return scoped_topic(namespace, name)


def parse_count(text: str) -> int:
    match = re.search(r"(\d+)", text)
    return int(match.group(1)) if match else 0


def full_node_name(namespace: str, name: str) -> str:
    ns = namespace.strip()
    if not ns or ns == "/":
        return "/" + name
    return ns.rstrip("/") + "/" + name


def parse_topic_info(text: str) -> dict:
    result = {
        "type": "",
        "publisher_count": 0,
        "subscription_count": 0,
        "publishers": [],
        "subscribers": [],
    }
    section = None
    pending_node = None

    for raw_line in text.splitlines():
        line = raw_line.strip()
        if line.startswith("Type:"):
            result["type"] = line.split(":", 1)[1].strip()
        elif line.startswith("Publisher count:"):
            result["publisher_count"] = parse_count(line)
            section = "publishers"
            pending_node = None
        elif line.startswith("Subscription count:"):
            result["subscription_count"] = parse_count(line)
            section = "subscribers"
            pending_node = None
        elif line.startswith("Node name:"):
            pending_node = line.split(":", 1)[1].strip()
        elif line.startswith("Node namespace:") and pending_node:
            namespace = line.split(":", 1)[1].strip()
            nodes = result["publishers"] if section == "publishers" else result["subscribers"]
            nodes.append(full_node_name(namespace, pending_node))
            pending_node = None

    return result


def node_matches(nodes: Iterable[str], expected: str) -> bool:
    suffix = "/" + expected
    return any(node == expected or node.endswith(suffix) for node in nodes)


def missing_expected(nodes: Iterable[str], expected: tuple[str, ...]) -> list[str]:
    return [name for name in expected if not node_matches(nodes, name)]


def topic_info(topic: str) -> tuple[bool, dict, str]:
    code, output = run(
        ["ros2", "topic", "info", "-v", "--no-daemon", "--spin-time", "1.0", topic],
        timeout=5.0,
    )
    info = parse_topic_info(output)
    exists = code == 0 and (info["type"] or info["publisher_count"] or info["subscription_count"])
    return exists, info, output


def list_topics() -> set[str]:
    code, output = run(["ros2", "topic", "list", "-t", "--no-daemon"], timeout=5.0)
    if code != 0:
        return set()
    topics = set()
    for line in output.splitlines():
        stripped = line.strip()
        if not stripped:
            continue
        topics.add(stripped.split()[0])
    return topics


def measure_hz(topic: str, duration: float) -> str:
    command = ["ros2", "topic", "hz", "--window", "20", "--wall-time", "--spin-time", "1.0", topic]
    proc = subprocess.Popen(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        env=ros_subprocess_env(),
    )
    try:
        time.sleep(max(1.0, duration))
        proc.terminate()
        output, _ = proc.communicate(timeout=2.0)
    except subprocess.TimeoutExpired:
        proc.kill()
        output, _ = proc.communicate()

    matches = re.findall(r"average rate:\s*([0-9.]+)", output or "")
    if matches:
        return matches[-1] + " Hz"
    if "no new messages" in (output or "").lower():
        return "no new messages"
    return "unavailable"


def parse_stamp_age(output: str) -> tuple[bool, float | None, str]:
    sec = None
    nanosec = None
    for raw_line in output.splitlines():
      line = raw_line.strip()
      if line.startswith("sec:") and sec is None:
          try:
              sec = int(line.split(":", 1)[1].strip())
          except ValueError:
              pass
      elif line.startswith("nanosec:") and sec is not None and nanosec is None:
          try:
              nanosec = int(line.split(":", 1)[1].strip())
          except ValueError:
              pass
      if sec is not None and nanosec is not None:
          break

    if sec is None or nanosec is None:
        return False, None, "header.stamp not found"
    if sec == 0 and nanosec == 0:
        return False, None, "header.stamp is zero"

    stamp = sec + nanosec * 1.0e-9
    return True, time.time() - stamp, ""


def parse_stamp(output: str) -> tuple[bool, float | None, str]:
    sec = None
    nanosec = None
    for raw_line in output.splitlines():
        line = raw_line.strip()
        if line.startswith("sec:") and sec is None:
            try:
                sec = int(line.split(":", 1)[1].strip())
            except ValueError:
                pass
        elif line.startswith("nanosec:") and sec is not None and nanosec is None:
            try:
                nanosec = int(line.split(":", 1)[1].strip())
            except ValueError:
                pass
        if sec is not None and nanosec is not None:
            break

    if sec is None or nanosec is None:
        return False, None, "stamp not found"
    if sec == 0 and nanosec == 0:
        return False, None, "stamp is zero"
    return True, sec + nanosec * 1.0e-9, ""


def current_ros_time_sec(use_sim_time: bool) -> tuple[bool, float | None, str]:
    if not use_sim_time:
        return True, time.time(), ""

    command = [
        "ros2", "topic", "echo", "--once", "--no-daemon", "--spin-time", "1.0",
        "--field", "clock", "/clock",
    ]
    code, output = run(command, timeout=4.0)
    if code != 0:
        return False, None, "could not read /clock"
    return parse_stamp(output)


def check_topic_age(check: AgeCheck, namespace: str, use_sim_time: bool) -> tuple[int, int]:
    topic = scoped_topic(namespace, check.topic)
    command = [
        "ros2", "topic", "echo", "--once", "--no-daemon", "--spin-time", "1.0",
        "--field", "header.stamp", topic,
    ]
    if use_sim_time:
        command.insert(4, "-s")
    code, output = run(command, timeout=4.0)
    if code != 0:
        status = "FAIL" if check.required else "WARN"
        print(f"[{status}] {topic} age unavailable")
        print(f"       note={check.note or 'message freshness'}")
        if output.strip():
            print("       ros2:", output.strip().splitlines()[-1])
        return (1, 0) if check.required else (0, 1)

    ok, stamp, reason = parse_stamp(output)
    if not ok or stamp is None:
        status = "FAIL" if check.required else "WARN"
        print(f"[{status}] {topic} age invalid: {reason}")
        return (1, 0) if check.required else (0, 1)

    clock_ok, now_sec, clock_reason = current_ros_time_sec(use_sim_time)
    if not clock_ok or now_sec is None:
        status = "FAIL" if check.required else "WARN"
        print(f"[{status}] {topic} age unavailable: {clock_reason}")
        return (1, 0) if check.required else (0, 1)

    age = now_sec - stamp
    if age < -0.05:
        status = "FAIL" if check.required else "WARN"
        print(f"[{status}] {topic} stamp is in the future: age={age:.3f}s")
        print(f"       note={check.note or 'message freshness'}")
        return (1, 0) if check.required else (0, 1)
    if age > check.max_age_sec:
        status = "FAIL" if check.required else "WARN"
        print(f"[{status}] {topic} stale: age={age:.3f}s limit={check.max_age_sec:.3f}s")
        print(f"       note={check.note or 'message freshness'}")
        return (1, 0) if check.required else (0, 1)

    clock_name = "sim-time" if use_sim_time else "wall-time"
    print(f"[PASS] {topic} fresh: age={age:.3f}s limit={check.max_age_sec:.3f}s ({clock_name})")
    return 0, 0


def parse_tf2_echo_time(output: str) -> tuple[bool, float | None, str]:
    match = re.search(r"At time\s+([0-9]+)(?:\.([0-9]+))?", output)
    if not match:
        return False, None, "transform stamp not found"
    sec_text = match.group(1)
    frac_text = match.group(2) or "0"
    frac = float("0." + frac_text) if frac_text else 0.0
    return True, float(sec_text) + frac, ""


def check_tf(check: TfCheck, namespace: str, use_sim_time: bool) -> tuple[int, int]:
    target = check.target_frame
    source = check.source_frame
    command = [
        "ros2", "run", "tf2_ros", "tf2_echo",
        target, source,
    ]
    ns = namespace.strip("/")
    if ns:
        command += [
            "--ros-args",
            "-r", f"/tf:=/{ns}/tf",
            "-r", f"/tf_static:=/{ns}/tf_static",
        ]
    code, output = run(command, timeout=max(4.0, check.timeout_sec + 3.0))
    if "Translation:" in output or "At time" in output:
        ok, stamp, reason = parse_tf2_echo_time(output)
        clock_ok, now_sec, clock_reason = current_ros_time_sec(use_sim_time)
        if ok and stamp == 0.0:
            print(f"[PASS] TF {target} <- {source} available with static/zero stamp")
        elif ok and stamp is not None and clock_ok and now_sec is not None:
            age = now_sec - stamp
            if age < -0.05:
                status = "FAIL" if check.required else "WARN"
                print(f"[{status}] TF {target} <- {source} stamp is in the future: age={age:.3f}s")
                return (1, 0) if check.required else (0, 1)
            if age > check.max_age_sec:
                status = "FAIL" if check.required else "WARN"
                print(
                    f"[{status}] TF {target} <- {source} stale: "
                    f"age={age:.3f}s limit={check.max_age_sec:.3f}s"
                )
                if check.note:
                    print(f"       note={check.note}")
                return (1, 0) if check.required else (0, 1)
            clock_name = "sim-time" if use_sim_time else "wall-time"
            print(
                f"[PASS] TF {target} <- {source} fresh: "
                f"age={age:.3f}s limit={check.max_age_sec:.3f}s ({clock_name})"
            )
        else:
            print(f"[PASS] TF {target} <- {source}")
            detail = reason if not ok else clock_reason
            print(f"       age not evaluated: {detail}")
        if check.note:
            print(f"       note={check.note}")
        return 0, 0

    status = "FAIL" if check.required else "WARN"
    print(f"[{status}] TF {target} <- {source} unavailable")
    if check.note:
        print(f"       note={check.note}")
    if output.strip():
        print("       tf2:", output.strip().splitlines()[-1])
    return (1, 0) if check.required else (0, 1)


def list_actions() -> dict[str, str]:
    code, output = run(["ros2", "action", "list", "-t"], timeout=5.0)
    if code != 0:
        return {}
    result = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        match = re.match(r"(\S+)\s+\[(.+)\]", line)
        if match:
            result[match.group(1)] = match.group(2)
    return result


def check_actions(namespace: str) -> tuple[int, int]:
    actions = list_actions()
    failures = 0
    warnings = 0
    if not actions:
        print("[FAIL] action graph unavailable")
        return 1, 0

    for check in ACTION_CHECKS:
        name = scoped_action(namespace, check.name)
        actual = actions.get(name)
        if actual == check.action_type:
            print(f"[PASS] action {name} type={actual}")
            if check.note:
                print(f"       note={check.note}")
            continue
        status = "FAIL" if check.required else "WARN"
        print(f"[{status}] action {name} missing or wrong type")
        print(f"       type={actual or '-'}, expected={check.action_type}")
        if check.note:
            print(f"       note={check.note}")
        if check.required:
            failures += 1
        else:
            warnings += 1
    return failures, warnings


def list_services() -> dict[str, str]:
    code, output = run(["ros2", "service", "list", "-t"], timeout=5.0)
    if code != 0:
        return {}
    result = {}
    for raw_line in output.splitlines():
        line = raw_line.strip()
        match = re.match(r"(\S+)\s+\[(.+)\]", line)
        if match:
            result[match.group(1)] = match.group(2)
    return result


def check_services(namespace: str) -> tuple[int, int]:
    services = list_services()
    failures = 0
    warnings = 0
    if not services:
        print("[FAIL] service graph unavailable")
        return 1, 0

    for check in SERVICE_CHECKS:
        name = scoped_topic(namespace, check.name)
        actual = services.get(name)
        if actual == check.srv_type:
            print(f"[PASS] service {name} type={actual}")
            if check.note:
                print(f"       note={check.note}")
            continue
        status = "FAIL" if check.required else "WARN"
        print(f"[{status}] service {name} missing or wrong type")
        print(f"       type={actual or '-'}, expected={check.srv_type}")
        if check.note:
            print(f"       note={check.note}")
        if check.required:
            failures += 1
        else:
            warnings += 1
    return failures, warnings


def check_topic(check: TopicCheck, namespace: str, measure: bool, hz_duration: float) -> tuple[int, int]:
    topic = scoped_topic(namespace, check.topic)
    exists, info, raw_output = topic_info(topic)
    failures = 0
    warnings = 0

    if not exists:
        print(f"[FAIL] {topic} missing ({check.phase})")
        print(f"       expected type: {check.msg_type}; reason: {check.note}")
        if raw_output.strip():
            print("       ros2:", raw_output.strip().splitlines()[-1])
        return 1, 0

    actual_type = info["type"]
    if actual_type != check.msg_type:
        failures += 1
        status = "FAIL"
        type_text = f"type {actual_type!r}, expected {check.msg_type!r}"
    else:
        status = "PASS"
        type_text = actual_type

    missing_pubs = missing_expected(info["publishers"], check.required_publishers)
    missing_subs = missing_expected(info["subscribers"], check.required_subscribers)
    if missing_pubs or missing_subs:
        failures += len(missing_pubs) + len(missing_subs)
        status = "FAIL"

    optional_present = [
        name for name in check.optional_subscribers if node_matches(info["subscribers"], name)
    ]
    hz_text = ""
    if measure:
        hz_text = f", measured={measure_hz(topic, hz_duration)}"

    print(f"[{status}] {topic} ({check.phase})")
    print(f"       type={type_text}, expected_rate={check.expected_rate}{hz_text}")
    print(f"       pub={', '.join(info['publishers']) or '-'}")
    print(f"       sub={', '.join(info['subscribers']) or '-'}")

    if missing_pubs:
        print(f"       missing publishers: {', '.join(missing_pubs)}")
    if missing_subs:
        print(f"       missing subscribers: {', '.join(missing_subs)}")
    if check.optional_subscribers:
        print(f"       optional subscribers present: {', '.join(optional_present) or '-'}")
    if check.note:
        print(f"       note={check.note}")

    return failures, warnings


def warn_stale_topics(namespace: str, strict: bool) -> tuple[int, int]:
    topics = list_topics()
    stale = [
        scoped_topic(namespace, "terrain_map"),
        scoped_topic(namespace, "terrain_map_ext"),
    ]
    found = [topic for topic in stale if topic in topics]
    if not found:
        return 0, 0

    print("[WARN] stale simulation topics detected: " + ", ".join(found))
    print("       simulation acceptance uses obstacle_cloud/occupancy_grid; terrain_map_ext is not part of the current chain")
    return (len(found) if strict else 0), len(found)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Validate the namespaced navigation topic chain and key graph endpoints."
    )
    parser.add_argument(
        "namespace",
        nargs="?",
        default="red_standard_robot1",
        help="robot namespace without leading slash; pass '' for no namespace",
    )
    parser.add_argument(
        "--no-support",
        action="store_true",
        help="check only the fixed acceptance chain, not map/odom/static-costmap support topics",
    )
    parser.add_argument(
        "--slam",
        action="store_true",
        help="check SLAM bringup support topics instead of prior-map localization topics",
    )
    parser.add_argument(
        "--measure-hz",
        action="store_true",
        help="sample topic frequencies; this adds a few seconds per topic",
    )
    parser.add_argument("--hz-duration", type=float, default=2.0, help="seconds to sample each topic")
    parser.add_argument(
        "--strict-stale",
        action="store_true",
        help="fail if old terrain_map or terrain_map_ext topics are still present in simulation",
    )
    parser.add_argument(
        "--dynamic-obstacles",
        action="store_true",
        help="also require the optional dynamic point detection/tracking chain",
    )
    parser.add_argument(
        "--skip-age",
        action="store_true",
        help="skip header stamp freshness checks",
    )
    parser.add_argument(
        "--skip-tf",
        action="store_true",
        help="skip TF lookup checks",
    )
    parser.add_argument(
        "--skip-actions",
        action="store_true",
        help="skip action server graph checks",
    )
    parser.add_argument(
        "--use-sim-time",
        action="store_true",
        help="subscribe with ROS time and do not compare message stamps against wall time",
    )
    args = parser.parse_args()

    namespace = args.namespace.strip("/")
    print("SIRB 2026 navigation acceptance chain")
    print(f"namespace=/{namespace}" if namespace else "namespace=<none>")

    support_checks = SLAM_SUPPORT_CHECKS if args.slam else SUPPORT_CHECKS
    checks = CORE_CHECKS if args.no_support else support_checks + CORE_CHECKS
    if args.dynamic_obstacles:
        checks = checks + DYNAMIC_OBSTACLE_CHECKS
    failures = 0
    warnings = 0
    for check in checks:
        check_failures, check_warnings = check_topic(
            check, namespace, args.measure_hz, args.hz_duration
        )
        failures += check_failures
        warnings += check_warnings

    stale_failures, stale_warnings = warn_stale_topics(namespace, args.strict_stale)
    failures += stale_failures
    warnings += stale_warnings

    if not args.skip_actions:
        action_failures, action_warnings = check_actions(namespace)
        failures += action_failures
        warnings += action_warnings
        service_failures, service_warnings = check_services(namespace)
        failures += service_failures
        warnings += service_warnings

    if not args.skip_tf:
        for check in TF_CHECKS:
            tf_failures, tf_warnings = check_tf(check, namespace, args.use_sim_time)
            failures += tf_failures
            warnings += tf_warnings

    if not args.skip_age:
        age_checks = PRIMARY_AGE_CHECKS
        if args.dynamic_obstacles:
            age_checks = age_checks + DYNAMIC_OBSTACLE_AGE_CHECKS
        for check in age_checks:
            age_failures, age_warnings = check_topic_age(check, namespace, args.use_sim_time)
            failures += age_failures
            warnings += age_warnings

    if failures:
        print(f"RESULT: FAIL ({failures} failure(s), {warnings} warning(s))")
        return 1

    print(f"RESULT: PASS ({warnings} warning(s))")
    return 0


if __name__ == "__main__":
    sys.exit(main())
