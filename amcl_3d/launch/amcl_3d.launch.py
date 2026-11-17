from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    share = FindPackageShare('amcl_3d')
    arguments = [
        DeclareLaunchArgument('param_file', default_value=PathJoinSubstitution(
            [share, 'config', 'amcl_3d.param.yaml'])),
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('rviz', default_value='false'),
        DeclareLaunchArgument('input_map', default_value='/map_points'),
        DeclareLaunchArgument('input_points', default_value='/points'),
        DeclareLaunchArgument('input_odom', default_value='/odom'),
        DeclareLaunchArgument('input_imu', default_value='/imu'),
        DeclareLaunchArgument('input_initialpose', default_value='/initialpose'),
    ]

    amcl_3d = Node(
        package='amcl_3d',
        executable='amcl_3d_node',
        name='amcl_3d',
        output='screen',
        parameters=[LaunchConfiguration('param_file'),
                    {'use_sim_time': LaunchConfiguration('use_sim_time')}],
        remappings=[
            ('map', LaunchConfiguration('input_map')),
            ('points', LaunchConfiguration('input_points')),
            ('odom', LaunchConfiguration('input_odom')),
            ('imu', LaunchConfiguration('input_imu')),
            ('initialpose', LaunchConfiguration('input_initialpose')),
        ],
    )

    rviz = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', PathJoinSubstitution([share, 'rviz', 'amcl_3d.rviz'])],
        parameters=[{'use_sim_time': LaunchConfiguration('use_sim_time')}],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription(arguments + [amcl_3d, rviz])
