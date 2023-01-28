// Copyright 2019 coderkarl. Subject to the BSD license.

#include "nav_sim/PotentialFields.h"
#include <cmath>

PotentialFields::PotentialFields() :
	c_attr(2.0),
	max_Fattr(2.0),
	c_repel(100.0),
	obs_d0(3.0),
	obs_d_retreat(1.2),
	alpha(0.01),
	m_useLinear(false),
	m_useTan(false),
	m_R1(1.0), m_R2(2.0), m_Kt(10.0),
	m_gamma(M_PI/2), m_max_heading_error(M_PI/6),
	m_Kw(1.5),
	m_des_speed(1.0),
        m_min_omega(0.5)
{
	obs_list.clear();
	bot.x = 0;
	bot.y = 0;
	goal.x = 0;
	goal.y = 0;
}

PotentialFields::~PotentialFields(){}

geometry_msgs::msg::Twist PotentialFields::update_cmd(float bot_yaw, bool& update_wp)
{
	geometry_msgs::msg::Twist cmd;
	geometry_msgs::msg::Vector3 odom_vel;
	bool reverse = false;
	bool met_goal = false;
	float goal_dist = 0;
	if(m_useLinear)
	{
	  odom_vel = get_vxvy_mow(bot_yaw, reverse, met_goal, goal_dist);
	  update_wp = met_goal;
	}
	else
	{
	  odom_vel = get_vxvy();
	}
	float des_yaw;

	if(odom_vel.x == 0 && odom_vel.y == 0)
	{
		cmd.linear.x = 0;
		cmd.angular.z = 0;
		return cmd;
	}
	else
	{
		des_yaw = atan2(odom_vel.y, odom_vel.x);
	}
	float yaw_error = des_yaw - bot_yaw;
	if(m_useLinear && goal_dist < 1.0 && (cos(yaw_error) < -0.5))
	{
	  reverse = true;
	  yaw_error += M_PI;
	}
	if(yaw_error > M_PI)
		yaw_error -= M_PI*2;
	else if(yaw_error <= -M_PI)
		yaw_error += M_PI*2;

	if(m_useLinear)
	{
	  if(fabs(yaw_error) > m_max_heading_error)
	  {
	    cmd.linear.x = 0;
	  }
	  else
	  {
	    cmd.linear.x = m_des_speed * (1.0 - fabs(yaw_error) / m_max_heading_error);
	  }
	  float omega = m_Kw*yaw_error;
	  if(cmd.linear.x <= 0.5 && fabs(omega) < m_min_omega)
	  {
	    if(omega < 0.0)
	      omega = -m_min_omega;
	    else
	      omega = m_min_omega;
	  }
	  cmd.angular.z = omega;
	  if(reverse)
	  {
	    cmd.linear.x = -m_des_speed/2;
	    //cmd.angular.z = 0;
	  }
	}
	else
	{
    if(fabs(yaw_error) > M_PI*0.7)
      cmd.linear.x = -1;
    else
      cmd.linear.x = 1;

    float kp = 0.5;
    cmd.angular.z = kp*yaw_error;
	}

	return cmd;
}

geometry_msgs::msg::Vector3 PotentialFields::get_vxvy_mow(float bot_yaw, bool& reverse, bool& met_goal, float& goal_dist)
{
  reverse = false;
  geometry_msgs::msg::Vector3 vel, g_unit_vec, obs_unit_vec;
  vel.x = 0;
  vel.y = 0;

  float gdx = goal.x - bot.x;
  float gdy = goal.y - bot.y;
  goal_dist = sqrt(gdx*gdx + gdy*gdy);
  if(goal_dist < 0.7)
  {
    met_goal = true;
    //return vel;
  }
  g_unit_vec.x = gdx / goal_dist;
  g_unit_vec.y = gdy / goal_dist;

  if(g_unit_vec.x == 0 && g_unit_vec.y == 0)
  {
    met_goal = true;
    //return vel;
  }

  geometry_msgs::msg::Vector3 t_unit_vec;
  float odx, ody, obs_dist;
  float mu = sin(m_gamma);
  float min_odx = m_R2*2;
  float min_ody = m_R2*2;
  float min_obs_dist = m_R2*2;
  for(unsigned k = 0; k<obs_list.size(); ++k)
  {
    Obstacle obs = obs_list[k];

    // First check if goal is close to obstacle, skip this wp
    odx = obs.x - goal.x;
    ody = obs.y - goal.y;
    obs_dist = sqrt(odx*odx + ody*ody);
    if(obs_dist < 1.0)
    {
      met_goal = true;
      vel = g_unit_vec;
      return vel;
    }

    odx = obs.x - bot.x;
    ody = obs.y - bot.y;
    //forget obstacle radius for now
    obs_dist = sqrt(odx*odx + ody*ody);
    if(obs_dist < min_obs_dist)
    {
      min_obs_dist = obs_dist;
      min_odx = odx;
      min_ody = ody;
    }
  }

  if(min_obs_dist < m_R2)
  {
    obs_unit_vec.x = min_odx / min_obs_dist;
    obs_unit_vec.y = min_ody / min_obs_dist;
    float dot = cos(bot_yaw) * obs_unit_vec.x + sin(bot_yaw) * obs_unit_vec.y;
    if(min_obs_dist < m_R1 && dot > 0.7)
    {
      reverse = true;
    }
    t_unit_vec = obs_unit_vec;
    if(m_useTan)
    {
      t_unit_vec.x = -obs_unit_vec.y * mu;
      t_unit_vec.y =  obs_unit_vec.x * mu;

      dot = g_unit_vec.x * t_unit_vec.x + g_unit_vec.y * t_unit_vec.y;
      if(dot < -0.1)
      {
        t_unit_vec.x = -t_unit_vec.x;
        t_unit_vec.y = -t_unit_vec.y;
      }
    }
    float Ft = m_Kt * (m_R2 - min_obs_dist) / (m_R2 - m_R1);

    vel.x = (Ft*t_unit_vec.x + g_unit_vec.x) / (Ft + 1);
    vel.y = (Ft*t_unit_vec.y + g_unit_vec.y) / (Ft + 1);
  }
  else
  {
    vel = g_unit_vec;
  }

  return vel;
}

geometry_msgs::msg::Vector3 PotentialFields::get_vxvy()
{
	// velocity in the odom frame
	geometry_msgs::msg::Vector3 vel, Fattr, Frepel, Frepel_filt;
	float Fx, Fy;

	vel.x = 0;
	vel.y = 0;

	Fattr = get_Fattr();
	if(Fattr.x == 0 && Fattr.y == 0)
		return vel;

	Frepel = get_Frepel2(Fattr);

	Frepel_filt.x = alpha*Frepel_filt.x + (1-alpha)*Frepel.x;
	Frepel_filt.y = alpha*Frepel_filt.y + (1-alpha)*Frepel.y;

	Fx = Fattr.x + Frepel_filt.x;
	Fy = Fattr.y + Frepel_filt.y;

	float magF_sqd = Fx*Fx + Fy*Fy;
	float magF, dirx, diry;
	if(magF_sqd > 0)
	{
		magF = sqrt(magF_sqd);
		dirx = Fx/magF;
		diry = Fy/magF;
	}
	else
	{
		magF = sqrt(Fattr.x*Fattr.x + Fattr.y*Fattr.y);
		dirx = Fattr.x/magF;
		diry = Fattr.y/magF;
	}

	vel.x = dirx;
	vel.y = diry;

	return vel;
}

geometry_msgs::msg::Vector3 PotentialFields::get_Fattr()
{
	geometry_msgs::msg::Vector3 Fattr;
	Fattr.x = 0;
	Fattr.y = 0;

	float dx = goal.x - bot.x;
	float dy = goal.y - bot.y;
	if(fabs(dx) < 1 && fabs(dy) < 1)
		return Fattr;

	float ax = c_attr * dx;
	float ay = c_attr * dy;
	float mag_sqd = ax*ax + ay*ay;
	if(mag_sqd > max_Fattr*max_Fattr)
	{
		float mag = sqrt(mag_sqd);
		ax = ax/mag*max_Fattr;
		ay = ay/mag*max_Fattr;
	}
	Fattr.x = ax;
	Fattr.y = ay;

	return Fattr;
}
geometry_msgs::msg::Vector3 PotentialFields::get_Frepel(geometry_msgs::msg::Vector3 Fattr)
{
	geometry_msgs::msg::Vector3 Frepel;
	Frepel.x = 0;
	Frepel.y = 0;

	float dx, dy, dmag_sqd, d, w, Fx, Fy, Fxtan, Fytan;

	for(unsigned k = 0; k<obs_list.size(); ++k)
	{
		Obstacle obs = obs_list[k];
		dx = bot.x - obs.x;
		dy = bot.y - obs.y;
		//forget obstacle radius for now
		dmag_sqd = dx*dx + dy*dy;
		if(dmag_sqd < obs_d0*obs_d0)
		{
			d = sqrt(dmag_sqd);
			w = c_repel*(1/d - 1/(obs_d0)) / (d*d*d);
			Fx = w*dx;
			Fy = w*dy;
		}
		else
		{
			Fx = 0;
			Fy = 0;
		}

		if(d < obs_d_retreat || get_cos_2d(dx, dy, Fattr.x, Fattr.y) > 0.9) //we've past this obstacle
		{
			// Use radial force
			Frepel.x += Fx;
			Frepel.y += Fy;
		}
		else
		{
			// Use tangengtial force
			// TODO: choose mu based on a-star path
			float mu = 1.0;
			Fxtan = -mu*Fy;
			Fytan = mu*Fx;
			if(get_cos_2d(Fxtan, Fytan, Fattr.x, Fattr.y) < -0.9)
			{
				Fxtan = -Fxtan;
				Fytan = -Fytan;
			}
			Frepel.x += Fxtan;
			Frepel.y += Fytan;
		}
	}

	return Frepel;
}

geometry_msgs::msg::Vector3 PotentialFields::get_Frepel2(geometry_msgs::msg::Vector3 Fattr)
{
    geometry_msgs::msg::Vector3 Frepel;
    Frepel.x = 0;
    Frepel.y = 0;

    float dx, dy, dmag_sqd, d, w, Fx, Fy, Fxtan, Fytan;

    for(unsigned k = 0; k<obs_list.size(); ++k)
    {
        Obstacle obs = obs_list[k];
        dx = bot.x - obs.x;
        dy = bot.y - obs.y;
        //forget obstacle radius for now
        dmag_sqd = dx*dx + dy*dy;
        if(dmag_sqd < obs_d0*obs_d0)
        {
            //d = sqrt(dmag_sqd);
            w = c_repel*(1/dmag_sqd - 1/(obs_d0*obs_d0)) / (dmag_sqd*dmag_sqd);
            Fx = w*dx;
            Fy = w*dy;
        }
        else
        {
            Fx = 0;
            Fy = 0;
        }

        if(dmag_sqd < obs_d_retreat*obs_d_retreat)
        {
            // Use radial force
            Frepel.x += Fx;
            Frepel.y += Fy;
        }
        else
        {
            // Use tangengtial force
            // TODO: choose mu based on a-star path
            float mu = 1.0;
            Fxtan = -mu*Fy;
            Fytan = mu*Fx;
            Frepel.x += Fxtan;
            Frepel.y += Fytan;
        }
    }

    return Frepel;
}

float PotentialFields::get_cos_2d(float ax, float ay, float bx, float by)
{
	float magA, magB;
	magA = sqrt(ax*ax + ay*ay);
	magB = sqrt(bx*bx + by*by);
	if(magA == 0 || magB == 0)
		return 0;
	return (ax*bx + ay*by)/magA/magB;
}
