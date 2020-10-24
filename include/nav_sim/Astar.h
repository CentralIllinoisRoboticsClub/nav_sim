// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef Astar_H
#define Astar_H

#include <rclcpp/rclcpp.hpp>
#include <nav_msgs/msg/path.hpp>
#include <nav_msgs/msg/odometry.hpp>
#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <vector>

class Astar: public rclcpp::Node
{
public:
	Astar();
	~Astar();

	double get_plan_rate();
	bool get_path(geometry_msgs::msg::Pose pose, geometry_msgs::msg::Pose goal,
					nav_msgs::msg::OccupancyGrid map, nav_msgs::msg::Path& path);
	void findPath();

	//The a-star path-finding data of a map grid cell
	typedef struct {
		float f; //f = g+heuristic
		float g; //cumulative motion cost, should be actual distance, maybe weight reverse
		float x; //x coordinate in meters
		float y; //y coordinate in meters
		int c;
		int r;
	}Cell;

private:
	  rclcpp::TimerBase::SharedPtr m_timer;

	  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::Odometry> > odom_sub_;
	  std::shared_ptr<rclcpp::Subscription<geometry_msgs::msg::PoseStamped> > goal_sub_;
	  std::shared_ptr<rclcpp::Subscription<nav_msgs::msg::OccupancyGrid> > costmap_sub_;
    std::shared_ptr<rclcpp::Publisher<nav_msgs::msg::Path> > path_pub_;

    void odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom);
    void goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data);
    void costmapCallback(const nav_msgs::msg::OccupancyGrid::SharedPtr map);

	float simp_move(float next_pos[], float x1, float y1, int motion, float d);

	//bool compareCells(const Cell& a, const Cell& b);

	Cell new_cell(float f, float g, float x, float y, int c, int r);

	bool get_map_indices(float x, float y, int& ix, int& iy);

	int is_obs(nav_msgs::msg::OccupancyGrid map, int ix, int iy);

	int is_obs2(nav_msgs::msg::OccupancyGrid map, int ix, int iy);

	geometry_msgs::msg::Pose bot_pose, goal_pose;
	nav_msgs::msg::Path path;

	float map_x0, map_y0, map_res;
	int NUM_ROWS, NUM_COLS;

	nav_msgs::msg::OccupancyGrid m_map;

	//parameters
	int obs_thresh;
	double obs_weight;
	double plan_rate_;
	double max_plan_time_;

};

#endif
