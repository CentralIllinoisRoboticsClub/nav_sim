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

	geometry_msgs::msg::Twist update_cmd(float bot_yaw, bool& update_wp);

	void setParams(bool useLinear, float R1, float R2, float Kt, float offset_gamma,
	    float max_heading_error, float Kw, float des_speed, float min_omega, float d_retreat, float goal_thresh_m, bool useTan)
	{
	  m_useLinear = useLinear;
	  m_R1 = R1;
	  m_R2 = R2;
	  m_Kt = Kt;
	  m_gamma = offset_gamma;
	  m_max_heading_error = max_heading_error;
	  m_Kw = Kw;
	  m_des_speed = des_speed;
    m_min_omega = min_omega;
    obs_d_retreat = d_retreat;
    m_goal_thresh_m = goal_thresh_m;
    m_useTan = useTan;
	}

private:
	float c_attr, max_Fattr;
	float c_repel, obs_d0, obs_d_retreat;
	float m_goal_thresh_m;
	float alpha;

	bool m_useLinear;
	bool m_useTan;
	float m_R1, m_R2, m_Kt;
	float m_gamma, m_max_heading_error;
	float m_Kw;
	float m_des_speed;
        float m_min_omega;
	geometry_msgs::msg::Vector3 get_Fattr();
	geometry_msgs::msg::Vector3 get_Frepel(geometry_msgs::msg::Vector3 Fattr);
	geometry_msgs::msg::Vector3 get_Frepel2(geometry_msgs::msg::Vector3 Fattr);
	geometry_msgs::msg::Vector3 get_vxvy();
	geometry_msgs::msg::Vector3 get_vxvy_mow(float bot_yaw, bool &reverse, bool &met_goal, float& goal_dist);
	float get_cos_2d(float ax, float ay, float bx, float by);

};

#endif
