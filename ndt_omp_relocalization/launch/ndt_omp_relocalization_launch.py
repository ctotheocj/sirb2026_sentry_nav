# Copyright 2025 Lihan Chen
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")
    map_frame = LaunchConfiguration("map_frame")
    odom_frame = LaunchConfiguration("odom_frame")
    base_frame = LaunchConfiguration("base_frame")
    robot_base_frame = LaunchConfiguration("robot_base_frame")

    return LaunchDescription([
        DeclareLaunchArgument(
            "prior_pcd_file",
            default_value="",
            description="Prior PCD file in the map frame. Required for a useful NDT run.",
        ),
        DeclareLaunchArgument("use_sim_time", default_value="False"),
        DeclareLaunchArgument("map_frame", default_value="map"),
        DeclareLaunchArgument("odom_frame", default_value="odom"),
        DeclareLaunchArgument("base_frame", default_value="base_footprint"),
        DeclareLaunchArgument("robot_base_frame", default_value="gimbal_yaw"),
        Node(
            package="ndt_omp_relocalization",
            executable="ndt_omp_relocalization_node",
            name="ndt_omp_relocalization",
            output="screen",
            remappings=[("/tf", "tf"), ("/tf_static", "tf_static")],
            parameters=[
                {
                    "use_sim_time": use_sim_time,
                    "prior_pcd_file": prior_pcd_file,
                    "map_frame": map_frame,
                    "odom_frame": odom_frame,
                    "base_frame": base_frame,
                    "robot_base_frame": robot_base_frame,
                    "num_threads": 4,
                    "ndt_resolution": 1.0,
                    "ndt_step_size": 0.1,
                    "ndt_epsilon": 0.01,
                    "ndt_max_iterations": 30,
                    "ndt_search_method": 1,
                    "fitness_score_threshold": 1.0,
                    "registration_period_ms": 300,
                    "global_leaf_size": 0.25,
                    "registered_leaf_size": 0.25,
                    "min_source_points": 1000,
                    "min_filtered_points": 120,
                    "max_scan_age_sec": 0.5,
                    "init_pose": [0.0, 0.0, 0.0, 0.0, 0.0, 0.0],
                    "enable_roll_pitch_fix": True,
                    "trust_ndt_threshold": 5,
                    "jump_threshold_xy": 0.5,
                    "jump_threshold_yaw": 0.3,
                    "jump_threshold_rp": 0.1,
                    "enable_quality_gate": True,
                    "quality_sample_points": 1500,
                    "quality_max_corr_dist": 1.0,
                    "quality_min_valid_correspondences": 120,
                    "quality_min_overlap_ratio": 0.35,
                    "quality_max_median_residual": 0.35,
                    "quality_max_p90_residual": 1.0,
                    "publish_tf_only_when_trusted": True,
                    "freeze_tf_when_not_trusted": True,
                }
            ],
        ),
    ])
