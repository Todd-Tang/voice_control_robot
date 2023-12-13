import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():

    controller_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'controller.yaml')
    bt_navigator_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'bt_navigator.yaml')
    planner_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'planner_server.yaml')
    recovery_yaml = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'recovery.yaml')
    nav2_yaml = os.path.join(get_package_share_directory('localization_server'), 'config', 'amcl_config_initialized.yaml')
    map_file = os.path.join(get_package_share_directory('map_server'), 'config', 'turtlebot_area.yaml')
    rviz2_file = os.path.join(get_package_share_directory('path_planner_server'), 'config', 'slam.rviz')


    return LaunchDescription([     
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{'use_sim_time': True}, 
                        {'yaml_filename':map_file}]
        ),
            
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[nav2_yaml]
        ),
        Node(
            package='nav2_controller',
            executable='controller_server',
            name='controller_server',
            output='screen',
            parameters=[controller_yaml]),

        Node(
            package='nav2_planner',
            executable='planner_server',
            name='planner_server',
            output='screen',
            parameters=[planner_yaml]),
            
        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            name='recoveries_server',
            parameters=[recovery_yaml],
            output='screen'),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator',
            output='screen',
            parameters=[bt_navigator_yaml]),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager',
            output='screen',
            parameters=[{'autostart': True},
                        {'node_names': ['map_server',
                                        'amcl',
                                        'controller_server',
                                        'planner_server',
                                        'recoveries_server',
                                        'bt_navigator']}]),

        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz2_file]
        ),

        Node(
            package='path_planner_server',
            executable='nav_node',
            name='nav_node',
            output='screen'),

        # Node(
        #     package='path_planner_server',
        #     executable='point_action_node',
        #     name='point_action_node',
        #     output='screen'),

        Node(
            package='path_planner_server',
            executable='point_service_node',
            name='point_service_node',
            output='screen'),

        # Node(
        #     package='path_planner_server',
        #     executable='point_client_node',
        #     name='point_client_node',
        #     output='screen'),

        # Node(
        #     package='wr_package',
        #     executable='doa_to_point_node',
        #     name='doa_to_point_node',
        #     output='screen'),

        
    ])
