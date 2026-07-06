"""
Launches the butler_bt_navigator node with the waypoints param file and the
BT xml for whichever milestone you pass in.

Assumes Nav2 (amcl + planner + controller + bt_navigator's own NavigateToPose
action) is already up via your existing experiment8b_myrobot_bringup /
nav2 bringup launch, and that a map + localization are running.

Usage:
  ros2 launch cafe_butler_bt butler_bt.launch.py milestone:=milestone5_multi_table
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_share = get_package_share_directory('cafe_butler_bt')

    milestone_arg = DeclareLaunchArgument(
        'milestone',
        default_value='milestone1_basic_delivery',
        description=(
            'Which BT xml (without .xml) to run, e.g. '
            'milestone1_basic_delivery ... milestone7_skip_cancelled_table, '
            'or milestone_full_generic'
        ),
    )

    waypoints_yaml = os.path.join(pkg_share, 'config', 'waypoints.yaml')

    bt_xml_path = PathJoinSubstitution(
        [pkg_share, 'behavior_trees', [LaunchConfiguration('milestone'), '.xml']]
    )

    butler_bt_node = Node(
        package='cafe_butler_bt',
        executable='butler_bt_navigator',
        name='butler_bt_navigator',
        output='screen',
        parameters=[waypoints_yaml, {'bt_xml_path': bt_xml_path}],
    )

    return LaunchDescription([milestone_arg, butler_bt_node])
