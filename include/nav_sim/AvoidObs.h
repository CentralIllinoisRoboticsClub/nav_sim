// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef AvoidObs_H
#define AvoidObs_H

//ROS Includes
#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <sensor_msgs/msg/laser_scan.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_ros/buffer.h>
#include "PotentialFields.h"
#include <std_msgs/msg/int16.hpp>
#include <deque>
//#include <std_msgs/String.h>


class AvoidObs: public rclcpp::Node
{
    public:
		AvoidObs();
        ~AvoidObs();
        
        //returns parameter rate_ in hz
        // used to define ros::Rate
        double get_plan_rate();
        void update_plan();
        
    private:
        void scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan);
        void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
        void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
        void coneCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
        void knownObstacleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr obs_pose);
        void foundConeCallback(const std_msgs::msg::Int16::SharedPtr msg);
        void hillWaypointCallback(const std_msgs::msg::Int16::SharedPtr msg);
        
        void update_cell(float x, float y, int val);
        
        bool get_map_indices(float x, float y, int& ix, int& iy);
        int get_cost(int ix, int iy);
        bool check_for_cone_obstacle();
        void check_known_obstacles();

        rclcpp::QoS qos_;

        rclcpp::TimerBase::SharedPtr m_timer;

        std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::OccupancyGrid> > costmap_pub_, pf_obs_pub_;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::Twist> > cmd_pub_;
        std::shared_ptr<rclcpp::Publisher<geometry_msgs::msg::PoseStamped> > obs_cone_pub_;

        std::shared_ptr<rclcpp::Subscription<sensor_msgs::msg::LaserScan> > scan_sub_;
        std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry> > odom_sub_;
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > goal_sub_;
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > wp_cone_sub_;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int16> > found_cone_sub_;
        std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > known_obstacle_sub_;
        std::shared_ptr<rclcpp::Subscription<std_msgs::msg::Int16> > hill_wp_sub_;
        
        PotentialFields pf;

        unsigned num_obs_cells;
        geometry_msgs::msg::Pose map_pose, pfObs_pose;
        nav_msgs::msg::OccupancyGrid costmap, pfObs;

        geometry_msgs::msg::Pose bot_pose, goal_pose;
        geometry_msgs::msg::PoseStamped camera_cone_poseStamped, obs_cone_poseStamped;
        float bot_yaw;
        
        std::shared_ptr<tf2_ros::TransformListener> listener;
        std::shared_ptr<tf2_ros::Buffer> tfBuffer;

        std::deque<geometry_msgs::msg::PoseStamped> knownObstacleDeq;

        double scan_range; //assigned to max_range_ or min_hill_range_ in hillWaypointCallback

        //parameters
        double plan_rate_; //Default 10 Hz, how often we use potential fields to update cmd_vel
        double map_res_; //Default 0.5 meters
        int n_width_, n_height_;
        double min_range_;
        double max_range_;
        double min_hill_range_;
        double plan_range_;
        int clear_decrement_, fill_increment_;
        double adjacent_cost_offset, adjacent_cost_slope;
        int inflation_factor_;
        double reinflate_radius_, cone_search_radius_;
        int reinflate_n_cells_, cone_search_n_cells_;
        int reinflate_cost_thresh_;
        bool use_PotFields_;
        int cone_obs_thresh_;
        int max_num_known_obstacles_;
        double known_obstacle_time_limit_;
};

#endif
