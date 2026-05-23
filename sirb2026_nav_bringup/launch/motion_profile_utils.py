import os
import tempfile

import yaml


class _NoAliasSafeDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def _node_params(data, name):
    params = data.get(name, {}).get("ros__parameters")
    if not isinstance(params, dict):
        raise KeyError(f"{name}.ros__parameters not found")
    return params


def _plugin_params(data, node, plugin):
    params = _node_params(data, node).get(plugin)
    if not isinstance(params, dict):
        raise KeyError(f"{node}.{plugin} not found")
    return params


def _set_xy(values, x, y, z):
    result = list(values) if isinstance(values, list) else []
    while len(result) < 3:
        result.append(0.0)
    result[0] = x
    result[1] = y
    result[2] = z
    return result


def _as_bool(value):
    if isinstance(value, bool):
        return value
    return str(value).strip().lower() in ("1", "true", "yes", "on")


def _read_pgm_size(image_path):
    with open(image_path, "rb") as f:
        magic = f.readline().strip()
        if magic not in (b"P2", b"P5"):
            raise ValueError(f"unsupported PGM magic {magic!r}")

        tokens = []
        while len(tokens) < 2:
            line = f.readline()
            if not line:
                break
            line = line.split(b"#", 1)[0]
            tokens.extend(line.split())

    if len(tokens) < 2:
        raise ValueError("PGM header does not contain width and height")
    return int(tokens[0]), int(tokens[1])


def apply_grid_map_bounds_from_map_yaml(data, map_yaml_path):
    if not map_yaml_path or not os.path.exists(map_yaml_path):
        return None

    with open(map_yaml_path, "r") as f:
        map_data = yaml.safe_load(f)
    if not isinstance(map_data, dict):
        raise ValueError(f"invalid map yaml: {map_yaml_path}")

    image_path = map_data["image"]
    if not os.path.isabs(image_path):
        image_path = os.path.join(os.path.dirname(map_yaml_path), image_path)

    width, height = _read_pgm_size(image_path)
    resolution = float(map_data["resolution"])
    origin = map_data.get("origin", [0.0, 0.0, 0.0])

    grid_map = _node_params(data, "grid_map_node").setdefault("map", {})
    grid_map["origin"] = [float(origin[0]), float(origin[1]), -0.5]
    grid_map["size"] = [round(width * resolution, 3), round(height * resolution, 3), 2.0]
    grid_map["resolution"] = resolution
    return grid_map["origin"], grid_map["size"]


def apply_motion_profile_to_dict(data):
    profile = _node_params(data, "motion_profile")
    max_speed = float(profile["max_speed"])
    max_accel = float(profile["max_accel"])
    control_speed = float(profile["control_speed_limit"])
    control_accel = float(profile["control_accel_limit"])
    planned_accel = min(max_accel, control_accel)
    tracking_soft = float(profile["tracking_error_soft"])
    tracking_hard = float(profile["tracking_error_hard"])
    localization_jump = float(profile["localization_jump_speed"])

    follow = _plugin_params(data, "controller_server", "FollowPath")
    smoother = _plugin_params(data, "smoother_server", "safe_geometric_smoother")
    manager = _node_params(data, "trajectory_manager")
    velocity = _node_params(data, "velocity_smoother")
    fake_vel = _node_params(data, "fake_vel_transform")

    follow["v_ref_max"] = control_speed
    follow["ax_max"] = control_accel
    follow["ay_max"] = control_accel
    follow["lateral_accel_limit"] = control_accel
    follow["v_circle_max"] = control_speed
    follow["pose_jump_speed_threshold"] = localization_jump
    follow["enable_lateral_error_ref_scaling"] = True
    follow["lateral_error_slow_threshold"] = tracking_soft
    follow["lateral_error_high_threshold"] = tracking_hard

    smoother["minco_v_ref"] = max_speed
    smoother["minco_v_max"] = control_speed
    smoother["minco_a_max"] = planned_accel
    smoother["minco_dynamic_realloc_max_segment_scale"] = max(
        float(smoother.get("minco_dynamic_realloc_max_segment_scale", 8.0)), 8.0)
    smoother["minco_max_segment_time"] = max(
        float(smoother.get("minco_max_segment_time", 4.0)), 4.0)
    smoother["minco_max_pieces"] = max(
        int(smoother.get("minco_max_pieces", 60)), 60)
    smoother["enable_trajectory_stitching"] = False
    smoother["reuse_cached_trajectory_on_minco_failure"] = False

    manager["publish_full_active_trajectory"] = True
    manager["publish_rate_hz"] = max(20.0, float(manager.get("publish_rate_hz", 20.0)))
    manager["switch_max_velocity_error"] = control_speed
    manager["switch_max_acceleration_error"] = planned_accel

    fake_vel["max_latest_cmd_age_sec"] = min(
        float(fake_vel.get("max_latest_cmd_age_sec", 0.12)), 0.12)
    velocity["velocity_timeout"] = max(
        float(velocity.get("velocity_timeout", 1.0)), 1.0)

    velocity["max_velocity"] = _set_xy(
        velocity.get("max_velocity", []), control_speed, control_speed,
        float(velocity.get("max_velocity", [0.0, 0.0, 0.0])[2]))
    velocity["min_velocity"] = _set_xy(
        velocity.get("min_velocity", []), -control_speed, -control_speed,
        float(velocity.get("min_velocity", [0.0, 0.0, 0.0])[2]))
    velocity["max_accel"] = _set_xy(
        velocity.get("max_accel", []), control_accel, control_accel,
        float(velocity.get("max_accel", [0.0, 0.0, 0.0])[2]))
    velocity["max_decel"] = _set_xy(
        velocity.get("max_decel", []), -control_accel, -control_accel,
        float(velocity.get("max_decel", [0.0, 0.0, 0.0])[2]))


def apply_dynamic_obstacle_mode(data, enabled):
    bt_nav = _node_params(data, "bt_navigator")
    follow = _plugin_params(data, "controller_server", "FollowPath")
    smoother = _plugin_params(data, "smoother_server", "safe_geometric_smoother")

    bt_nav["use_dynamic_obstacles"] = enabled

    follow["enable_dynamic_obstacle_avoidance"] = enabled
    follow.setdefault("dynamic_obstacle_topic", "dynamic_obstacles")

    smoother["dynamic_obstacle_enabled"] = enabled
    smoother.setdefault("dynamic_obstacle_topic", "dynamic_obstacles")


def apply_yaw_fusion_mode(data, enabled):
    fake_vel = _node_params(data, "fake_vel_transform")
    yaw_fusion = _node_params(data, "yaw_fusion")

    fake_vel["use_nav_yaw"] = enabled
    fake_vel.setdefault("nav_yaw_topic", "/Nav_yaw")

    yaw_fusion.setdefault("debug_logging", False)


def sync_hole_clear_corridors(data):
    bt_nav = _node_params(data, "bt_navigator")
    hole_pass = bt_nav.get("hole_pass", {})
    if not isinstance(hole_pass, dict):
        return 0
    holes = hole_pass.get("holes", {})
    hole_ids = hole_pass.get("hole_ids", [])
    if not isinstance(holes, dict) or not isinstance(hole_ids, list):
        return 0

    synced = 0
    for node_name in ("local_costmap", "global_costmap"):
        node = data.get(node_name, {})
        nested = node.get(node_name, {}) if isinstance(node, dict) else {}
        params = nested.get("ros__parameters") if isinstance(nested, dict) else None
        if not isinstance(params, dict):
            continue
        layer = params.get("occupancy_grid_layer")
        if not isinstance(layer, dict):
            continue

        layer["clear_hole_corridors"] = bool(layer.get("clear_hole_corridors", True))
        layer.setdefault("clear_hole_frame", "map")
        layer.setdefault("clear_hole_margin", 0.15)
        layer["clear_hole_ids"] = list(hole_ids)
        layer["clear_holes"] = {
            hole_id: {
                "port_a_polygon": list(holes.get(hole_id, {}).get("port_a_polygon", [])),
                "port_b_polygon": list(holes.get(hole_id, {}).get("port_b_polygon", [])),
            }
            for hole_id in hole_ids
            if isinstance(holes.get(hole_id), dict)
        }
        synced += 1
    return synced


def _replace_robot_namespace_placeholders(text, namespace):
    ns = namespace.strip("/")
    replacement = f"/{ns}" if ns else ""
    return text.replace("<robot_namespace>", replacement)


def prepare_navigation_params(context, params_file_config, namespace_config=None):
    if context.launch_configurations.get("_sirb2026_navigation_params_prepared") == "true":
        return []

    source = context.perform_substitution(params_file_config)
    if namespace_config is not None:
        namespace = context.perform_substitution(namespace_config)
    else:
        namespace = context.launch_configurations.get("namespace", "")

    with open(source, "r") as f:
        text = f.read()
    text = _replace_robot_namespace_placeholders(text, namespace)
    data = yaml.safe_load(text)
    synced_clear_corridors = sync_hole_clear_corridors(data)

    safe_ns = "".join(c if c.isalnum() or c in ("_", "-") else "_" for c in namespace)
    fd, output = tempfile.mkstemp(
        prefix=f"sirb2026_navigation_{safe_ns or 'root'}_", suffix=".yaml")
    os.close(fd)
    with open(output, "w") as f:
        yaml.dump(data, f, Dumper=_NoAliasSafeDumper, sort_keys=False, allow_unicode=True)

    context.launch_configurations["params_file"] = output
    context.launch_configurations["_sirb2026_navigation_params_prepared"] = "true"
    print(f"[navigation_params] generated params: {output}")
    print(f"[navigation_params] hole_clear_corridors synced_layers={synced_clear_corridors}")
    return []


def apply_launch_mode_guards(data, use_composition):
    if use_composition:
        return False
    smoother = _plugin_params(data, "smoother_server", "safe_geometric_smoother")
    if not bool(smoother.get("minco_use_esdf", False)):
        return False
    smoother["minco_use_esdf"] = False
    return True


def prepare_motion_profile_params(
    context, params_file_config, label, map_file_config=None,
    use_dynamic_obstacles_config=None, use_yaw_fusion_config=None):
    source = context.perform_substitution(params_file_config)
    with open(source, "r") as f:
        data = yaml.safe_load(f)
    apply_motion_profile_to_dict(data)
    if use_dynamic_obstacles_config is not None:
        use_dynamic_obstacles = _as_bool(
            context.perform_substitution(use_dynamic_obstacles_config))
    else:
        use_dynamic_obstacles = _as_bool(
            context.launch_configurations.get("use_dynamic_obstacles", False))
    apply_dynamic_obstacle_mode(data, use_dynamic_obstacles)
    if use_yaw_fusion_config is not None:
        use_yaw_fusion = _as_bool(context.perform_substitution(use_yaw_fusion_config))
    else:
        use_yaw_fusion = _as_bool(context.launch_configurations.get("use_yaw_fusion", False))
    apply_yaw_fusion_mode(data, use_yaw_fusion)
    use_composition = context.launch_configurations.get("use_composition", "False").lower() == "true"
    esdf_disabled = apply_launch_mode_guards(data, use_composition)
    map_yaml_path = ""
    if map_file_config is not None:
        map_yaml_path = context.perform_substitution(map_file_config)
    else:
        map_yaml_path = context.launch_configurations.get("map", "")
    try:
        bounds = apply_grid_map_bounds_from_map_yaml(data, map_yaml_path)
    except Exception as exc:
        print(f"[motion_profile] failed to apply grid_map bounds from {map_yaml_path}: {exc}")
        bounds = None

    namespace = context.launch_configurations.get("namespace", "robot")
    safe_ns = "".join(c if c.isalnum() or c in ("_", "-") else "_" for c in namespace)
    fd, output = tempfile.mkstemp(
        prefix=f"sirb2026_{label}_{safe_ns}_", suffix=".yaml")
    os.close(fd)
    with open(output, "w") as f:
        yaml.dump(data, f, Dumper=_NoAliasSafeDumper, sort_keys=False, allow_unicode=True)

    context.launch_configurations["params_file"] = output
    print(f"[motion_profile] generated params: {output}")
    if bounds:
        origin, size = bounds
        print(f"[motion_profile] grid_map bounds origin={origin} size={size}")
    if esdf_disabled:
        print("[motion_profile] use_composition=false; disabled smoother minco_use_esdf because GridMapRegistry is process-local")
    print(f"[motion_profile] dynamic_obstacles enabled={use_dynamic_obstacles}")
    print(f"[motion_profile] yaw_fusion enabled={use_yaw_fusion}")
    return []
