// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef AvoidObs_H
#define AvoidObs_H

//ROS Includes
#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_listener.h>
#include "PotentialFields.h"
#include <std_msgs/Int16.h>
#include <deque>
//#include <std_msgs/String.h>


class AvoidObs
{
    public:
		AvoidObs();
        ~AvoidObs();
        
        //returns parameter rate_ in hz
        // used to define ros::Rate
        double get_plan_rate();
        bool update_plan();
        
    private:
        void scanCallback(const sensor_msgs::LaserScan& scan);
        void odomCallback(const nav_msgs::Odometry& odom);
        void goalCallback(const geometry_msgs::PoseStamped& data);
        void coneCallback(const geometry_msgs::PoseStamped& data);
        void knownObstacleCallback(const geometry_msgs::PoseStamped& obs_pose);
        void foundConeCallback(const std_msgs::Int16& msg);
        void hillWaypointCallback(const std_msgs::Int16& msg);
        
        void update_cell(float x, float y, int val);
        
        bool get_map_indices(float x, float y, int& ix, int& iy);
        int get_cost(int ix, int iy);
        bool check_for_cone_obstacle();
        void check_known_obstacles();

        ros::NodeHandle nh_;
        ros::NodeHandle nh_p;
        ros::Publisher costmap_pub_, pf_obs_pub_;
        ros::Publisher cmd_pub_;
        ros::Publisher obs_cone_pub_;
        ros::Subscriber scan_sub_, odom_sub_, goal_sub_, wp_cone_sub_, found_cone_sub_, known_obstacle_sub_, hill_wp_sub_;
        
        PotentialFields pf;

        unsigned num_obs_cells;
        geometry_msgs::Pose map_pose, pfObs_pose;
        nav_msgs::OccupancyGrid costmap, pfObs;

        geometry_msgs::Pose bot_pose, goal_pose;
        geometry_msgs::PoseStamped camera_cone_poseStamped, obs_cone_poseStamped;
        float bot_yaw;
        
        tf::TransformListener listener;

        std::deque<geometry_msgs::PoseStamped> knownObstacleDeq;

        double scan_range; //assigned to max_range_ or min_hill_range_ in hillWaypointCallback

        //parameters
        double plan_rate_; //Default 10 Hz, how often we use potential fields to update cmd_vel
        double map_res_; //Default 0.5 meters
        int n_width_, n_height_;
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
