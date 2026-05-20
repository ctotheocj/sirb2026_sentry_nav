# Copyright 2025 Pan
# Licensed under the Apache License, Version 2.0

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile
from nav2_common.launch import RewrittenYaml


def generate_launch_description():
    dodge_manager_dir = get_package_share_directory('dodge_manager')

    namespace = LaunchConfiguration('namespace')
    params_file = LaunchConfiguration('params_file')

    declare_namespace_cmd = DeclareLaunchArgument(
        'namespace',
        default_value='red_standard_robot1',
        description='Top-level namespace (should match nav2 namespace)',
    )

    declare_params_file_cmd = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(
            dodge_manager_dir, 'config', 'dodge_manager.yaml'
        ),
        description='Full path to the DodgeManager parameters file',
    )

    configured_params = ParameterFile(
        RewrittenYaml(
            source_file=params_file,
            root_key=namespace,
            param_rewrites={},
            convert_types=True,
        ),
        allow_substs=True,
    )

    dodge_manager_node = Node(
        package='dodge_manager',
        executable='dodge_manager_node',
        name='dodge_manager',
        namespace=namespace,
        parameters=[configured_params],
        output='screen',
        remappings=[
            # navigate_to_pose action 需要与 nav2 命名空间对齐
            # 节点在 /red_standard_robot1 下，相对路径会变成 /red_standard_robot1/navigate_to_pose
            # nav2 的 action server 通常也在同一命名空间，所以这里不需要 remap
            # 如果 nav2 在不同命名空间，取消下面注释并修改：
            # ('navigate_to_pose', '/red_standard_robot1/navigate_to_pose'),
        ],
    )

    ld = LaunchDescription()
    ld.add_action(declare_namespace_cmd)
    ld.add_action(declare_params_file_cmd)
    ld.add_action(dodge_manager_node)
    return ld
