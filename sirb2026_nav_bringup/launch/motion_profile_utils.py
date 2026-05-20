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


def apply_motion_profile_to_dict(data):
    profile = _node_params(data, "motion_profile")
    max_speed = float(profile["max_speed"])
    max_accel = float(profile["max_accel"])
    control_speed = float(profile["control_speed_limit"])
    control_accel = float(profile["control_accel_limit"])
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
    follow["v_circle_max"] = control_speed
    follow["pose_jump_speed_threshold"] = localization_jump
    follow["enable_lateral_error_ref_scaling"] = True
    follow["lateral_error_slow_threshold"] = tracking_soft
    follow["lateral_error_high_threshold"] = tracking_hard

    smoother["minco_v_ref"] = max_speed
    smoother["minco_v_max"] = control_speed
    smoother["minco_a_max"] = max_accel
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
    manager["switch_max_velocity_error"] = max(
        control_speed, float(manager.get("switch_max_velocity_error", control_speed)))
    manager["switch_max_acceleration_error"] = max(
        max_accel, float(manager.get("switch_max_acceleration_error", max_accel)))

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
        velocity.get("max_accel", []), max(control_accel, max_accel), max(control_accel, max_accel),
        float(velocity.get("max_accel", [0.0, 0.0, 0.0])[2]))
    velocity["max_decel"] = _set_xy(
        velocity.get("max_decel", []), -max(control_accel, max_accel), -max(control_accel, max_accel),
        float(velocity.get("max_decel", [0.0, 0.0, 0.0])[2]))


def prepare_motion_profile_params(context, params_file_config, label):
    source = context.perform_substitution(params_file_config)
    with open(source, "r") as f:
        data = yaml.safe_load(f)
    apply_motion_profile_to_dict(data)

    namespace = context.launch_configurations.get("namespace", "robot")
    safe_ns = "".join(c if c.isalnum() or c in ("_", "-") else "_" for c in namespace)
    fd, output = tempfile.mkstemp(
        prefix=f"sirb2026_{label}_{safe_ns}_", suffix=".yaml")
    os.close(fd)
    with open(output, "w") as f:
        yaml.dump(data, f, Dumper=_NoAliasSafeDumper, sort_keys=False, allow_unicode=True)

    context.launch_configurations["params_file"] = output
    print(f"[motion_profile] generated params: {output}")
    return []
