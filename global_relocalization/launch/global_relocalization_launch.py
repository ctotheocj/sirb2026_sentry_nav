from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    params_file = LaunchConfiguration("params_file")
    prior_pcd_file = LaunchConfiguration("prior_pcd_file")
    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument("params_file", default_value=""),
        DeclareLaunchArgument("prior_pcd_file", default_value=""),
        DeclareLaunchArgument("use_sim_time", default_value="false"),
        Node(
            package="global_relocalization",
            executable="global_relocalization_node",
            name="global_relocalization",
            output="screen",
            parameters=[
                params_file,
                {"prior_pcd_file": prior_pcd_file},
                {"use_sim_time": use_sim_time},
            ],
        ),
    ])
