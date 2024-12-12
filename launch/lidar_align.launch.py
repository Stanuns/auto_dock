from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    return LaunchDescription([

        DeclareLaunchArgument(
        'use_sim_time',
        default_value='False',
        description='Use simulation (Gazebo) clock if true'),

        Node(
            package='auto_dock', 
            executable='lidar_align', 
            name='lidar_align',
            output='screen', 
            parameters=[{'use_sim_time':use_sim_time}]
        ),
    ])  