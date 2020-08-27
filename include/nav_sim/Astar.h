// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef Astar_H
#define Astar_H

#include <ros/ros.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Pose.h>
#include <vector>

class Astar
{
public:
	Astar();
	~Astar();

	double get_plan_rate();
	bool get_path(geometry_msgs::Pose pose, geometry_msgs::Pose goal,
					nav_msgs::OccupancyGrid map, nav_msgs::Path& path);
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
	ros::NodeHandle nh_;
    ros::NodeHandle nh_p;
    ros::Subscriber odom_sub_, goal_sub_, costmap_sub_;
    ros::Publisher path_pub_;

    void odomCallback(const nav_msgs::Odometry& odom);
    void goalCallback(const geometry_msgs::PoseStamped& data);
    void costmapCallback(const nav_msgs::OccupancyGrid& map);

	float simp_move(float next_pos[], float x1, float y1, int motion, float d);

	//bool compareCells(const Cell& a, const Cell& b);

	Cell new_cell(float f, float g, float x, float y, int c, int r);

	bool get_map_indices(float x, float y, int& ix, int& iy);

	int is_obs(nav_msgs::OccupancyGrid map, int ix, int iy);

	int is_obs2(nav_msgs::OccupancyGrid map, int ix, int iy);

	geometry_msgs::Pose bot_pose, goal_pose;
	nav_msgs::Path path;

	float map_x0, map_y0, map_res;
	int NUM_ROWS, NUM_COLS;

	nav_msgs::OccupancyGrid m_map;

	//parameters
	int obs_thresh;
	double obs_weight;
	double plan_rate_;
	double max_plan_time_;

};

#endif
