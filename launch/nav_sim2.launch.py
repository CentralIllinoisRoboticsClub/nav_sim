from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number

from launch.actions import ExecuteProcess, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

# https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py
from launch_ros.actions import LifecycleNode
from launch.actions import EmitEvent
from launch.actions import RegisterEventHandler
from launch_ros.events.lifecycle import ChangeState
from launch_ros.events.lifecycle import matches_node_name
from launch_ros.event_handlers import OnStateTransition
from launch.actions import LogInfo
from launch.events import matches_action
import lifecycle_msgs.msg

# https://answers.ros.org/question/306935/ros2-include-a-launch-file-from-a-launch-file/

# https://github.com/stereolabs/zed-ros2-wrapper/blob/master/zed_wrapper/launch/zed.launch.py
# Also need to add config to the CMakeLists.txt install(DIRECTORY
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    scan_sim_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('nav_sim') + '/launch/scan_sim2.launch.py')
    )
    
    avoid_obs_launch = IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                get_package_share_directory('nav_sim') + '/launch/sim_avoid_obstacles2.launch.py')
    )
    
    map_file = os.path.join(
        get_package_share_directory('nav_sim'),
        'config', 'big_run_map1u_clean_append6.yaml'
    )
    sim_bot_node = Node(
        package="nav_sim",
        executable="sim_bot.py",
        name="sim_bot_node",
        parameters=[{"map_yaml_file": map_file}]
    )
    
    load_map_node = Node(
        package="nav_sim",
        executable="load_map_client",
        output="screen",
        emulate_tty=True,
        parameters=[{"map_yaml_file": map_file}]
    )
    
    static_laser_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_laser_tf',
        arguments=['0', '0', '0.3', '0', '0', '0', 'base_link', 'laser']
    )
    
    static_map_tf_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='static_map_tf',
        arguments=['5.0', '0', '0', '0', '0', '0', 'map', 'odom']
    )
    
    config_rviz = os.path.join(
        get_package_share_directory('nav_sim'),
        'config', 'nav_sim_ros2.rviz'
    )
    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        arguments=['-d', config_rviz],
        #remappings=[("goal_pose", "wp_goal")]
    )
    
    # https://answers.ros.org/question/326070/ros2-nav2_map_server-can-not-load-map/
    # https://index.ros.org/p/nav2_map_server/
    map_node = LifecycleNode(
        package = 'nav2_map_server',
        executable='map_server',
        name='map_server',
        parameters=[
            {"yaml_filename": map_file}
        ]
    )
    # ros2 nav2_util lifecycle_bringup map_server
    # OR
    # ros2 lifecycle set map_server configure
    # ros2 lifecycle set map_server activate
    map_start_node = Node(
        package='nav2_util',
        executable='lifecycle_bringup',
        arguments=['map_server']
    )
    cmd_string = 'ros2 lifecycle set map_server configure & ros2 lifecycle set map_server activate'
    map_start_cmd = ExecuteProcess(
            cmd=cmd_string.split(' '),
            output='screen'
    )
    configure_map_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(map_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_map_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=map_node, goal_state='inactive',
            entities=[
                LogInfo(
                    msg="[LifecycleLaunch] map_server node is activating."),
                EmitEvent(event=ChangeState(
                    lifecycle_node_matcher=matches_action(map_node),
                    transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                )),
            ],
        )
    )
    
    # The map is only published once when activated, and scan_sim_launch misses it
    ld.add_action(scan_sim_launch)
    ld.add_action(sim_bot_node)
    ld.add_action(load_map_node)
    ld.add_action(static_laser_tf_node)
    ld.add_action(static_map_tf_node)
    #ld.add_action(rviz_node)
    ld.add_action(map_node)
    ld.add_action(configure_map_event)
    ld.add_action(activate_map_event)
    ld.add_action(avoid_obs_launch)
    
    return ld
