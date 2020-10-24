from launch import LaunchDescription
from launch_ros.actions import Node
from numpy import number

from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    ld = LaunchDescription()
    
    nav_states_node = Node(
        package="nav_sim",
        executable="nav_states",
        parameters=[{'plan_rate_hz': 10.0},
                    {'use_PotFields': False},
                    {'close_cone_to_bot_dist': 11.0},
                    {'valid_cone_to_wp_dist': 11.0},
                    {'near_path_dist': 1.0},
                    {'valid_end_of_path_dist': 5.0},
                    {'desired_speed': 1.0},
                    {'slow_speed': 0.4},
                    {'max_omega': 2.0},
                    {'max_fwd_heading_error_deg': 180.0},
                    {'search_time': 2.0},
                    {'search_omega': 0.8},
                    {'reverse_time': 1.0},
                    {'cmd_control_ver': 0},
                    {'scan_collision_db_limit': 4},
                    {'scan_collision_range': 0.5},
                    {'cone_detect_db_limit': 1},
                    {'cmd_speed_filter_factor': 0.5},
                    {'report_bumped_obstacles': True},
                    {'max_camera_search_time': 30.0},
                    {'slow_approach_distance': 2.0},
                    {'reverse_speed': 0.8},
                    {'bump_db_limit': 2},
                    {'path_step_size': 3},
                    {'waypoints_are_in_map_frame': True},
                    {'x_coords':          [17.0, 5.0, 7.0]},
                    {'y_coords':          [9.0, 29.0, 1.0]},
                    {'waypoint_types':     [0,   0,    0]},
                    {'hill_waypoint_list': [0,   0,    0]}
        ],
        #remappings=[('cmd_vel', 'ignore_cmd_vel')]
    )
    
    avoid_obs_node = Node(
        package="nav_sim",
        executable="avoid_obs",
        parameters=[{'plan_rate_hz': 10.0},
                    {'map_res_m': 0.5},
                    {'map_size': 301},
                    {'max_range': 40.0},
                    {'min_hill_range': 1.0},
                    {'plan_range': 35.0},
                    {'clear_decrement': -5},
                    {'adjacent_cost_offset': 2.0},
                    {'adjacent_cost_slope': 2.0},
                    {'inflation_factor': 2},
                    {'reinflate_cost_thresh': 30},
                    {'reinflate_radius': 5.0},
                    {'use_PotFields': False},
                    {'cone_search_radius': 1.0},
                    {'cone_obs_thresh': 0},
                    {'max_num_known_obstacles': 30},
                    {'known_obstacle_time_limit': 30.0}
        ]
    )
    
    astar_node = Node(
        package="nav_sim",
        executable="astar",
        output="screen",
        emulate_tty=True,
        parameters=[{'plan_rate_hz': 5.0},
                    {'obs_thresh': 30},
                    {'obs_weight': 0.1},
                    {'max_plan_time_sec': 3.0}
        ]
    )
    
    ld.add_action(avoid_obs_node)
    ld.add_action(nav_states_node)
    ld.add_action(astar_node)
    
    return ld
