// Copyright 2019 coderkarl. Subject to the BSD license.

// Eclipse, Project, Properties, C/C++ General, Paths and Symbols, Includes, Add /opt/ros/foxy

#ifndef NavStates_H
#define NavStates_H

//ROS Includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/point.hpp>

#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include <std_msgs/msg/int16.hpp>
#include <string>
#include <memory>
//#include <std_msgs/String.h>

#include "my_interfaces/srv/set_int.hpp"

class NavStates: public rclcpp::Node
{
public:
  NavStates(const rclcpp::NodeOptions& node_options);
  //returns parameter rate_ in hz
  // used to define ros::Rate
  double get_plan_rate();
  void update_states();

private:
  void update_mow_path();
  void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
  void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
  void clickedGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
  void camConeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr cone_pose_in);
  void obsConeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr obs_cone_pose_in);
  void bumpCallback(const std_msgs::msg::Int16::SharedPtr data);
  void pathCallback(const nav_msgs::msg::Path::SharedPtr path_in);
  void mapToOdomUpdateCallback(const std_msgs::msg::Int16::SharedPtr data);

  void pfWayPointUpdateCallback(const std_msgs::msg::Int16::SharedPtr data);

  void update_plan();
  bool check_for_cone_obstacle();
  void update_waypoint();
  void refresh_odom_goal();

  double distance_between_poses(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2);
  bool getPoseInFrame(const geometry_msgs::msg::PoseStamped& pose_in, std::string target_frame,
      geometry_msgs::msg::PoseStamped& pose_out);
  double getTargetHeading(const geometry_msgs::msg::PoseStamped& goal);

  bool nearPathPoint();
  void commandTo(const geometry_msgs::msg::PoseStamped& goal);
  void state_init();
  double get_time_in_state();

  // A function for each state in the state machine
  void track_path();
  void track_mow_path();
  void retreat();
  void search_in_place();
  void turn_to_target();
  void touch_target();
  void retreat_from_cone();
  void update_target();
  void update_target(geometry_msgs::msg::PoseStamped target_pose);
  void mow_area_callback(const my_interfaces::srv::SetInt::Request::SharedPtr request,
      const my_interfaces::srv::SetInt::Response::SharedPtr response);

  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist> > cmd_pub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped> > wp_goal_pub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped> > wp_cone_pub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped> > wp_static_map_goal_pub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int16> > nav_state_pub_;
  std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped> > known_obs_pub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int16> > hill_wp_pub_;
  std::shared_ptr<rclcpp::Publisher<std_msgs::msg::Int16> > valid_bump_pub_;
  
  //rclcpp::Subscription<sensor_msgs::msg::LaserScan>::SharedPtr scan2_sub_;
  std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan> > scan_sub_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry> > odom_sub_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > clicked_goal_sub_;
  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Path> > path_sub_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > cam_cone_pose_sub_;
  //std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > laser_cone_pose_sub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int16> > bump_sub_;
  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > obs_cone_sub_;
  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int16> > map_to_odom_update_sub_;

  std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int16> > pf_wp_update_sub_;

  rclcpp::Service<my_interfaces::srv::SetInt>::SharedPtr mow_area_server_;

  geometry_msgs::msg::PoseStamped bot_pose, map_goal_pose, odom_goal_pose;
  geometry_msgs::msg::PoseStamped camera_cone_pose, obs_cone_pose, camera_cone_pose_in_map;
  float bot_yaw;

  std::shared_ptr<tf2_ros::TransformListener> listener;
  std::shared_ptr<tf2_ros::Buffer> tfBuffer;

  nav_msgs::msg::Path m_path;
  std_msgs::msg::Int16 m_nav_state_msg, m_hill_wp_msg;

  std::vector<geometry_msgs::msg::Point> m_waypoints;
  int m_current_waypoint_type;
  int m_current_hill_type;

  bool m_collision, m_cone_detected, m_odom_received, m_path_received, m_obs_cone_received;
  bool m_init_wp_published, m_odom_goal_refresh_needed;
  bool m_first_search;
  bool m_valid_bump;
  int m_bump_switch;
  unsigned m_bump_count;
  unsigned m_num_waypoints, m_index_wp;
  int m_state;
  unsigned m_index_path;
  double m_speed, m_omega, m_filt_speed;
  unsigned m_scan_collision_db_count;
  unsigned m_cone_detect_db_count;

  double m_prev_heading_error;

  bool m_close_to_obs;

  bool m_update_pf_waypoint;

  rclcpp::Time state_start_time;
  rclcpp::Time m_bump_time;
  rclcpp::Time m_init_search_time;
  
  rclcpp::TimerBase::SharedPtr timer_;

  //parameters
  struct Parameters
  {
    double plan_rate; //Default 10 Hz, how often we use potential fields to update cmd_vel
    bool use_PotFields;
    double valid_cone_to_wp_dist;
    double near_path_dist;
    double valid_end_of_path_dist;
    double desired_speed;
    double slow_speed;
    double max_omega;
    double max_fwd_heading_error_deg;
    double search_time;
    double search_omega;
    double reverse_time;
    int cmd_control_ver;
    unsigned scan_collision_db_limit;
    double scan_collision_range;
    unsigned cone_detect_db_limit;
    double cmd_speed_filter_factor;
    bool report_bumped_obstacles;
    double max_camera_search_time;
    double slow_approach_distance;
    double reverse_speed;
    unsigned bump_db_limit;
    int path_step_size;
    //int min_new_path_size;
  }params;

  std::vector< std::vector<double> > x_coords;
  std::vector< std::vector<double> > y_coords;
  unsigned coords_index;
  std::vector<int64_t> waypoint_type_list;
  std::vector<int64_t> hill_wp_list;
  bool waypoints_are_in_map_frame;
  bool sim_mode;

  bool is_mow_boundary;
  bool mow_ccw;
  double mow_width;
  double mow_wp_spacing;
  double mow_inside_heading_offset;

};

#endif
