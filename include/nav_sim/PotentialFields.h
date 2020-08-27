// Copyright 2019 coderkarl. Subject to the BSD license.

#ifndef PotentialFields_H
#define PotentialFieldsd_H

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Vector3.h>
#include <geometry_msgs/Twist.h>
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
	geometry_msgs::Point bot, goal;



	geometry_msgs::Twist update_cmd(float bot_yaw);

private:
	float c_attr, max_Fattr;
	float c_repel, obs_d0, obs_d_retreat;
	float alpha;
	geometry_msgs::Vector3 get_Fattr();
	geometry_msgs::Vector3 get_Frepel(geometry_msgs::Vector3 Fattr);
	geometry_msgs::Vector3 get_Frepel2(geometry_msgs::Vector3 Fattr);
	geometry_msgs::Vector3 get_vxvy();
	float get_cos_2d(float ax, float ay, float bx, float by);

};

#endif
