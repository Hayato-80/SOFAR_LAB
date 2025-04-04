from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    packages_name = 'my_pkg'
    rviz_file = 'lab1.rviz'
    rviz_config_file = PathJoinSubstitution(
            [FindPackageShare(packages_name), 'rviz', rviz_file]
    )
    # param_file = PathJoinSubstitution([FindPackageShare(FindPackageShare(packages_name)), 'config', 'turtlesim.yaml'])
    return LaunchDescription([
        Node(
            package='my_pkg',
            executable='my_node',
            name='my_node',
            output='screen',
        ),
        Node(
            package='turtlesim',
            # namespace='turtlesim1',
            
            executable='turtlesim_node',
            parameters=[{'background_b':23, 'background_g':23, 'backgroun_r':23}],
            # name='sim'
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            output='screen',
            parameters=[{'use_sim_time': True}],
            arguments=['-d', rviz_config_file]
        ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=['5.5', '5.5', '0', '0', '0', '0', 'odom', 'map']
        # )

    ])

