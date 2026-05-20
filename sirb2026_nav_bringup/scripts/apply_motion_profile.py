#!/usr/bin/env python3
"""Check or apply motion_profile-derived Nav2 parameters."""

import argparse
import copy
import sys
from pathlib import Path

import yaml


class NoAliasSafeDumper(yaml.SafeDumper):
    def ignore_aliases(self, data):
        return True


def node_params(data, name):
    value = data.get(name, {})
    params = value.get("ros__parameters")
    if not isinstance(params, dict):
        raise KeyError(f"{name}.ros__parameters not found")
    return params


def nested_params(data, node, plugin=None):
    params = node_params(data, node)
    if plugin is None:
        return params
    value = params.get(plugin)
    if not isinstance(value, dict):
        raise KeyError(f"{node}.{plugin} not found")
    return value


def derived(profile):
    max_speed = float(profile["max_speed"])
    max_accel = float(profile["max_accel"])
    control_speed = float(profile["control_speed_limit"])
    control_accel = float(profile["control_accel_limit"])
    localization_jump = float(profile["localization_jump_speed"])
    return {
        "minco_v_ref": max_speed,
        "minco_v_max": control_speed,
        "minco_a_max": max_accel,
        "follow_v_ref_max": control_speed,
        "follow_ax_max": control_accel,
        "follow_ay_max": control_accel,
        "follow_v_circle_max": control_speed,
        "tracking_error_soft": float(profile["tracking_error_soft"]),
        "tracking_error_hard": float(profile["tracking_error_hard"]),
        "pose_jump_speed_threshold": localization_jump,
        "switch_max_velocity_error": control_speed,
        "switch_max_acceleration_error": max_accel,
        "velocity_max_xy": control_speed,
        "velocity_accel_xy": control_accel,
    }


def set_if_changed(container, key, value, changes):
    old = container.get(key)
    if old != value:
        changes.append((key, old, value))
        container[key] = value


def apply_profile(data):
    profile = node_params(data, "motion_profile")
    values = derived(profile)
    changes = []

    follow = nested_params(data, "controller_server", "FollowPath")
    smoother = nested_params(data, "smoother_server", "safe_geometric_smoother")
    manager = node_params(data, "trajectory_manager")
    vel = node_params(data, "velocity_smoother")
    fake_vel = node_params(data, "fake_vel_transform")

    set_if_changed(follow, "v_ref_max", values["follow_v_ref_max"], changes)
    set_if_changed(follow, "ax_max", values["follow_ax_max"], changes)
    set_if_changed(follow, "ay_max", values["follow_ay_max"], changes)
    set_if_changed(follow, "v_circle_max", values["follow_v_circle_max"], changes)
    set_if_changed(follow, "pose_jump_speed_threshold", values["pose_jump_speed_threshold"], changes)
    set_if_changed(follow, "enable_lateral_error_ref_scaling", True, changes)
    set_if_changed(follow, "lateral_error_slow_threshold", values["tracking_error_soft"], changes)
    set_if_changed(follow, "lateral_error_high_threshold", values["tracking_error_hard"], changes)

    set_if_changed(smoother, "minco_v_ref", values["minco_v_ref"], changes)
    set_if_changed(smoother, "minco_v_max", values["minco_v_max"], changes)
    set_if_changed(smoother, "minco_a_max", values["minco_a_max"], changes)
    set_if_changed(
        smoother,
        "minco_dynamic_realloc_max_segment_scale",
        max(float(smoother.get("minco_dynamic_realloc_max_segment_scale", 8.0)), 8.0),
        changes,
    )
    set_if_changed(
        smoother,
        "minco_max_segment_time",
        max(float(smoother.get("minco_max_segment_time", 4.0)), 4.0),
        changes,
    )
    set_if_changed(
        smoother,
        "minco_max_pieces",
        max(int(smoother.get("minco_max_pieces", 60)), 60),
        changes,
    )
    set_if_changed(smoother, "enable_trajectory_stitching", False, changes)
    set_if_changed(smoother, "reuse_cached_trajectory_on_minco_failure", False, changes)

    set_if_changed(manager, "publish_full_active_trajectory", True, changes)
    set_if_changed(
        manager,
        "publish_rate_hz",
        max(20.0, float(manager.get("publish_rate_hz", 20.0))),
        changes,
    )
    set_if_changed(
        manager,
        "switch_max_velocity_error",
        max(values["switch_max_velocity_error"], float(manager.get("switch_max_velocity_error", 0.0))),
        changes,
    )
    set_if_changed(
        manager,
        "switch_max_acceleration_error",
        max(values["switch_max_acceleration_error"], float(manager.get("switch_max_acceleration_error", 0.0))),
        changes,
    )

    max_velocity = list(vel.get("max_velocity", [0.0, 0.0, 0.0]))
    min_velocity = list(vel.get("min_velocity", [0.0, 0.0, 0.0]))
    max_accel = list(vel.get("max_accel", [0.0, 0.0, 0.0]))
    max_decel = list(vel.get("max_decel", [0.0, 0.0, 0.0]))
    while len(max_velocity) < 3:
        max_velocity.append(0.0)
    while len(min_velocity) < 3:
        min_velocity.append(0.0)
    while len(max_accel) < 3:
        max_accel.append(0.0)
    while len(max_decel) < 3:
        max_decel.append(0.0)

    max_velocity[0] = values["velocity_max_xy"]
    max_velocity[1] = values["velocity_max_xy"]
    min_velocity[0] = -values["velocity_max_xy"]
    min_velocity[1] = -values["velocity_max_xy"]
    xy_accel = max(values["velocity_accel_xy"], values["minco_a_max"])
    max_accel[0] = xy_accel
    max_accel[1] = xy_accel
    max_decel[0] = -xy_accel
    max_decel[1] = -xy_accel
    set_if_changed(vel, "max_velocity", max_velocity, changes)
    set_if_changed(vel, "min_velocity", min_velocity, changes)
    set_if_changed(vel, "max_accel", max_accel, changes)
    set_if_changed(vel, "max_decel", max_decel, changes)
    set_if_changed(vel, "velocity_timeout", max(float(vel.get("velocity_timeout", 1.0)), 1.0), changes)
    set_if_changed(
        fake_vel,
        "max_latest_cmd_age_sec",
        min(float(fake_vel.get("max_latest_cmd_age_sec", 0.12)), 0.12),
        changes,
    )

    return changes


def validate(data):
    warnings = []
    follow = nested_params(data, "controller_server", "FollowPath")
    smoother = nested_params(data, "smoother_server", "safe_geometric_smoother")
    vel = node_params(data, "velocity_smoother")
    manager = node_params(data, "trajectory_manager")

    minco_a = float(smoother.get("minco_a_max", 0.0))
    ax = float(follow.get("ax_max", 0.0))
    ay = float(follow.get("ay_max", 0.0))
    v_ref = float(smoother.get("minco_v_ref", 0.0))
    v_ctrl = float(follow.get("v_ref_max", 0.0))
    vel_accel = vel.get("max_accel", [0.0, 0.0])
    switch_accel = float(manager.get("switch_max_acceleration_error", 0.0))

    if ax > 0.0 and minco_a > ax * 1.5:
        warnings.append(f"minco_a_max {minco_a:.2f} > ax_max {ax:.2f} * 1.5")
    if ay > 0.0 and minco_a > ay * 1.5:
        warnings.append(f"minco_a_max {minco_a:.2f} > ay_max {ay:.2f} * 1.5")
    if v_ctrl > 0.0 and v_ref > v_ctrl:
        warnings.append(f"minco_v_ref {v_ref:.2f} > FollowPath.v_ref_max {v_ctrl:.2f}")
    if len(vel_accel) >= 2 and (float(vel_accel[0]) < ax or float(vel_accel[1]) < ay):
        warnings.append("velocity_smoother.max_accel is lower than FollowPath acceleration limit")
    if minco_a > 0.0 and switch_accel > 0.0 and switch_accel < minco_a * 0.7:
        warnings.append(
            f"switch_max_acceleration_error {switch_accel:.2f} is much lower than minco_a_max {minco_a:.2f}"
        )

    return warnings


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--file", required=True, type=Path)
    parser.add_argument("--write", action="store_true")
    args = parser.parse_args()

    data = yaml.safe_load(args.file.read_text())
    updated = copy.deepcopy(data)
    changes = apply_profile(updated)
    warnings = validate(updated)

    for key, old, new in changes:
        print(f"update {key}: {old!r} -> {new!r}")
    for warning in warnings:
        print(f"WARN: {warning}", file=sys.stderr)

    if args.write:
        args.file.write_text(
            yaml.dump(updated, Dumper=NoAliasSafeDumper, sort_keys=False, allow_unicode=True))
    elif changes:
        print("dry-run only; pass --write to update the YAML")

    return 1 if warnings else 0


if __name__ == "__main__":
    sys.exit(main())
