// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef PotentialFields_H
#define PotentialFields_H

#include <nav_msgs/msg/occupancy_grid.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/vector3.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <vector>

// Inputs:
//    vector of obstacles
//    goal pose (just x,y used)
//    bot pose (just x,y used)
// Outputs:
//    cmd_vel, local vx, wz; published

class PotentialFields
{
public:
	PotentialFields();
	~PotentialFields();

	typedef struct {
			float radius; //f = g+heuristic
			float x; //x coordinate in meters
			float y; //y coordinate in meters
	}Obstacle;

	std::vector<Obstacle> obs_list;
	geometry_msgs::msg::Point bot, goal;



	geometry_msgs::msg::Twist update_cmd(float bot_yaw);

private:
	float c_attr, max_Fattr;
	float c_repel, obs_d0, obs_d_retreat;
	float alpha;
	geometry_msgs::msg::Vector3 get_Fattr();
	geometry_msgs::msg::Vector3 get_Frepel(geometry_msgs::msg::Vector3 Fattr);
	geometry_msgs::msg::Vector3 get_Frepel2(geometry_msgs::msg::Vector3 Fattr);
	geometry_msgs::msg::Vector3 get_vxvy();
	float get_cos_2d(float ax, float ay, float bx, float by);

};

#endif
