from launch import LaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
    use_sim_time = LaunchConfiguration('use_sim_time', default='false')

    # remappings = [('odom', 'odom_combined'),  #luxshare robot odom_combined
    #             ('relative_dock_pose', 'relative_dock_pose')]

    return LaunchDescription([

        DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'),

        Node(
            package='auto_dock', 
            executable='remote_auto_dock_hm', 
            name='remote_auto_dock_hm_client',
            output='screen', 
            parameters=[{'use_sim_time':use_sim_time}]
            # ,
            # remappings=remappings
        ),
    ])  