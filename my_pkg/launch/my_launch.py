from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import PathJoinSubstitution, LaunchConfiguration
from launch_ros.substitutions import FindPackageShare
from launch.actions import ExecuteProcess


def generate_launch_description():
    packages_name = 'my_pkg'
    rviz_file = 'lab1.rviz'
    rviz_config_file = PathJoinSubstitution(
            [FindPackageShare(packages_name), 'rviz', rviz_file]
    )
    v_param = LaunchConfiguration('v', default=0.0)
    delta_t_param = LaunchConfiguration('delta_t', default=0.0)

    # param_file = PathJoinSubstitution([FindPackageShare(FindPackageShare(packages_name)), 'config', 'turtlesim.yaml'])
    return LaunchDescription([
        Node(
            package='my_pkg',
            executable='lab_node',
            name='lab_node',
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
        Node(
            package='my_pkg',
            executable='lab2_node',
            name='lab2_node',
            output='screen',
            parameters=[{'v': v_param}, 
                        {'delta_t':delta_t_param}
            ],
            arguments=['--ros-args', '--log-level', 'INFO']

        ),
        # Node(
        #     package='my_pkg',
        #     executable='server_node',
        #     name='server_node',
        #     output='screen',
        # ),
        # ExecuteProcess(
        #     cmd=['ros2', 'service', 'call', '/spawn', 'turtlesim/srv/Spawn', 
        #          '{"x": 8.0, "y": 8.0, "theta": 0.0, "name": "robot2"}'],
        #     output='screen',
        # ),
        # Node(
        #     package='tf2_ros',
        #     executable='static_transform_publisher',
        #     name='static_tf',
        #     output='screen',
        #     parameters=[{'use_sim_time': True}],
        #     arguments=['5.5', '5.5', '0', '0', '0', '0', 'odom', 'map']
        # )

    ])

