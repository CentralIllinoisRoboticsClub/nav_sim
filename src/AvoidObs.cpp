// Copyright 2019 coderkarl. Subject to the BSD license.

#include "avoid_obstacles/AvoidObs.h"
#include "avoid_obstacles/AvoidObsCommon.h"
#include <geometry_msgs/PointStamped.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>

/**********************************************************************
* Obstacle Avoidance using a nav_msgs/OccupacyGrid and A* path planning
* 
* Subscribe to /scan and Publish OccupacyGrid /costmap, Publish Path /path
* 
* The leddar.launch file launches this node as well as a static transform between base_link and laser
**********************************************************************/

//Constructor
AvoidObs::AvoidObs()
{
    //Topics you want to publish
    costmap_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("costmap", 1);
    cmd_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel",10);

    pf_obs_pub_ = nh_.advertise<nav_msgs::OccupancyGrid>("pfObs", 1);


    obs_cone_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("obs_cone_pose",1);

    //Topic you want to subscribe
    scan_sub_ = nh_.subscribe("scan", 50, &AvoidObs::scanCallback, this); //receive laser scan
    odom_sub_ = nh_.subscribe("odom", 10, &AvoidObs::odomCallback, this);
    goal_sub_ = nh_.subscribe("/move_base_simple/goal", 1, &AvoidObs::goalCallback, this);
    wp_cone_sub_ = nh_.subscribe("wp_cone_pose", 1, &AvoidObs::coneCallback, this);
    found_cone_sub_ = nh_.subscribe("found_cone",1, &AvoidObs::foundConeCallback, this);
    known_obstacle_sub_ = nh_.subscribe("known_obstacle",1,&AvoidObs::knownObstacleCallback, this);
    hill_wp_sub_ = nh_.subscribe("hill_wp",1,&AvoidObs::hillWaypointCallback, this);
    nh_p  = ros::NodeHandle("~");
    nh_p.param("plan_rate_hz", plan_rate_, 1.0); //set in avoid_obs.launch
    nh_p.param("map_res_m", map_res_, 0.5);
    nh_p.param("map_size", n_width_, 200);
    nh_p.param("map_size", n_height_, 200);
    nh_p.param("max_range", max_range_, 40.0);
    nh_p.param("min_hill_range", min_hill_range_, 1.0);
    nh_p.param("plan_range_m",plan_range_, 40.0);
    nh_p.param("clear_decrement",clear_decrement_,-5);
    nh_p.param("fill_increment",fill_increment_,10);
    nh_p.param("adjacent_cost_offset", adjacent_cost_offset, 2.0);
    nh_p.param("adjacent_cost_slope", adjacent_cost_slope, 1.0);
    nh_p.param("inflation_factor", inflation_factor_, 2);
    nh_p.param("reinflate_radius", reinflate_radius_, 2.5);
    nh_p.param("cone_search_radius", cone_search_radius_, 1.0);
    nh_p.param("reinflate_cost_thresh", reinflate_cost_thresh_, 30);
    nh_p.param("use_PotFields", use_PotFields_,false);
    nh_p.param("cone_obs_thresh", cone_obs_thresh_, 20);
    nh_p.param("max_num_known_obstacles", max_num_known_obstacles_, 20);
    nh_p.param("known_obstacle_time_limit", known_obstacle_time_limit_, 30.0);

    scan_range = max_range_;

    ROS_INFO("map_size (n cells): %d", n_width_);
    
    reinflate_n_cells_ = boost::math::iround(reinflate_radius_/map_res_);
    cone_search_n_cells_ = boost::math::iround(cone_search_radius_/map_res_);

    //listener.setExtrapolationLimit(ros::Duration(0.1));
    listener.waitForTransform("laser", "odom", ros::Time(0), ros::Duration(10.0));

    num_obs_cells = 0; //number of obstacle cells
    
    map_pose.position.x = -(n_width_/2)*map_res_-map_res_/2;//1.0 -(n_width_/2)*map_res_ + map_res_/2; // I believe this is zero by default, check by echoing costmap
    map_pose.position.y = -(n_height_/2)*map_res_-map_res_/2;//1.0 -(n_height_/2)*map_res_ + map_res_/2; // will need to update x and y as we move
    ROS_INFO("map_pose x,y: %0.2f, %0.2f",map_pose.position.x, map_pose.position.y);
    map_pose.orientation.w = 1.0;
    
    costmap.header.stamp = ros::Time::now();
    costmap.header.frame_id = "odom";
    costmap.info.resolution = map_res_;
    costmap.info.width = n_width_;
    costmap.info.height = n_height_;
    costmap.info.origin = map_pose;
    // Fill costmap with zeros
    // cost(ix,iy) = costmap.data[ix*n_height + iy], x-RIGHT, y-UP, 0,0 is bottom left
    costmap.data.resize(n_width_*n_height_);

    goal_pose.orientation.w = 1.0;
    bot_pose.orientation.w = 1.0;
    bot_yaw = 0.0;

    //Potential Fields Obstacle Map for debugging
    pfObs.header = costmap.header;
    pfObs.info = costmap.info;
    pfObs.data.resize(n_width_*n_height_);

    camera_cone_poseStamped.header.frame_id = "odom";
    obs_cone_poseStamped.header.frame_id = "odom";
    camera_cone_poseStamped.pose.orientation.w = 1.0;
    obs_cone_poseStamped.pose.orientation.w = 1.0;
}

AvoidObs::~AvoidObs(){}

void AvoidObs::knownObstacleCallback(const geometry_msgs::PoseStamped& obs_pose)
{
  // TODO: transform known obstacle to odom frame if not already
  knownObstacleDeq.push_back(obs_pose);
  //TODO: Consider using boost circular buffer
  if(knownObstacleDeq.size() > max_num_known_obstacles_)
  {
    knownObstacleDeq.pop_front();
  }
}

void AvoidObs::hillWaypointCallback(const std_msgs::Int16& msg)
{
  if(msg.data == 1)
  {
    scan_range = min_hill_range_;
  }
  else
  {
    scan_range = max_range_;
  }
}

void AvoidObs::coneCallback(const geometry_msgs::PoseStamped& data)
{
    camera_cone_poseStamped = data;
}

void AvoidObs::foundConeCallback(const std_msgs::Int16& msg)
{
  if(msg.data == 0) // need to reset the camera_cone_poseStamped so the just bumped cone now becomes an obstacle again
  {
    // TODO: add a bool flag when we bump a cone and update a waypoint to next cone, clear the flag in next coneCallback
    //  do not check_for_cone_obstacle if this flag is set
    camera_cone_poseStamped.pose.position.x = 0;
    camera_cone_poseStamped.pose.position.y = 0;
  }
}

bool AvoidObs::check_for_cone_obstacle()
{
    double sec_since_cone = (ros::Time::now()-camera_cone_poseStamped.header.stamp).toSec();
    unsigned min_cell_dist = 4*cone_search_n_cells_;
    if(sec_since_cone < 2.0)
    {
        int cost, max_nearby_cost = cone_obs_thresh_;
        int cx, cy;
        get_map_indices(camera_cone_poseStamped.pose.position.x, camera_cone_poseStamped.pose.position.y, cx, cy);
        for(int dx = -cone_search_n_cells_; dx <= cone_search_n_cells_; ++dx)
        {
            for(int dy = -cone_search_n_cells_; dy <= cone_search_n_cells_; ++dy)
            {
                cost = get_cost(cx+dx,cy+dy);
                if(cost > cone_obs_thresh_ and cost <= 100)
                {
                    costmap.data[(cy+dy) * n_width_ + (cx+dx)] = 0;
                    unsigned cell_dist = abs(dx)+abs(dy);
                    //if(cell_dist <= min_cell_dist)
                    if(cost >= max_nearby_cost)
                    {
                        max_nearby_cost = cost;
                        min_cell_dist = cell_dist;
                        obs_cone_poseStamped.header.stamp = ros::Time::now();
                        obs_cone_poseStamped.pose.position.x = map_pose.position.x + (cx+dx)*map_res_ + map_res_/2;
                        obs_cone_poseStamped.pose.position.y = map_pose.position.y + (cy+dy)*map_res_ + map_res_/2;
                    }
                }
            }
        }
        if(max_nearby_cost > cone_obs_thresh_)
        {
            obs_cone_pub_.publish(obs_cone_poseStamped);
            return true;
        }
    }
    return false;
}

double AvoidObs::get_plan_rate()
{
    return plan_rate_;
}

void AvoidObs::update_cell(float x, float y, int val)
{
    int ix, iy;
    get_map_indices(x, y, ix, iy);
    if (inflation_factor_ <= ix && ix < n_width_-inflation_factor_ && inflation_factor_ <= iy && iy < n_height_-inflation_factor_)
    {
        int cur_val = costmap.data[iy * n_width_ + ix];
        if (val > 0 && cur_val == 0) //ignore first hits
        {
            costmap.data[iy * n_width_ + ix] = 1;
        }
        else
        {
            if(cur_val + val > 100)
                cur_val = 100;
            else if (cur_val + val < 0)
                cur_val = 0;
            else
                cur_val += val;
            costmap.data[iy*n_width_ + ix] = cur_val;

            //adjacent cell inflation
            if(val > 0)
            {
                for(int dx = -inflation_factor_; dx <= inflation_factor_; ++dx)
                {
                    for(int dy = -inflation_factor_; dy <= inflation_factor_; ++dy)
                    {
                        if((dx != 0 or dy != 0))
                        {
                            int adj_val = costmap.data[(iy+dy)*n_width_ + ix+dx];
                            int new_val = double(cur_val)/(adjacent_cost_offset+double(abs(dx)+abs(dy))*adjacent_cost_slope);
                            if(adj_val < new_val)
                                costmap.data[(iy+dy)*n_width_ + ix+dx] = new_val;
                        }
                    }
                }
            }
        }
    }
}

bool AvoidObs::update_plan()
{
	/*path.poses.clear();
	geometry_msgs::PoseStamped wp;
	wp.pose.position.x = 5.0;
	wp.pose.position.y = 5.0;
	path.poses.push_back(wp);
	*/

	geometry_msgs::Pose start, temp_goal = goal_pose;

	float dx = goal_pose.position.x - bot_pose.position.x;
	float dy = goal_pose.position.y - bot_pose.position.y;
	float goal_dist_sqd = dx*dx + dy*dy;
	if(goal_dist_sqd > plan_range_*plan_range_)
	{
		float dir_rad;
		if(dx == 0 && dy == 0)
			dir_rad = 0.0;
		else
			dir_rad = atan2(dy,dx);
		temp_goal.position.x = bot_pose.position.x+plan_range_*cos(dir_rad);
		temp_goal.position.y = bot_pose.position.y+plan_range_*sin(dir_rad);
	}

	if(use_PotFields_)
	{
		//Potential Fields Test
		// pfObs is an odom grid map, same info as costmap
		pfObs.header.stamp = ros::Time::now();
		pfObs.data.clear();
		pfObs.data.resize(n_width_*n_height_);

		pf.obs_list.clear();
		int bot_ix, bot_iy;
		get_map_indices(pf.bot.x, pf.bot.y, bot_ix, bot_iy);

		for(int ix = bot_ix - 5; ix < bot_ix+5; ++ix)
		{
			for(int iy = bot_iy - 5; iy < bot_iy+5; ++iy)
			{
				if(get_cost(ix,iy) > 30)
				{
					pfObs.data[iy*n_width_ + ix] = 95;
					PotentialFields::Obstacle obs;
					obs.x = map_pose.position.x + ix*map_res_;
					obs.y = map_pose.position.y + iy*map_res_;
					//ROS_INFO("Added pfObs x,y: %0.0f, %0.0f",obs.x, obs.y);
					pf.obs_list.push_back(obs);
				}
			}
		}

		//testing hard coded obstacle
		// revealed need to use round for x,y to ix,iy and also offset map by res/2
		//pf.obs_list.clear();
		PotentialFields::Obstacle obs;
		float x = 7.0, y=1.0;
		int ix, iy;
		for(int k=0; k<=0; ++k)
		{
			get_map_indices(x, y+k, ix, iy);
			pfObs.data[iy*n_width_ + ix] = 50;
			obs.x = x;
			obs.y = y+k;
			pf.obs_list.push_back(obs);
		}
		obs.x = 3; obs.y = 3;
		get_map_indices(obs.x, obs.y, ix, iy);
		pfObs.data[iy*n_width_ + ix] = 50;
		pf.obs_list.push_back(obs);
		obs.x = 1; obs.y = 1;
        get_map_indices(obs.x, obs.y, ix, iy);
        pfObs.data[iy*n_width_ + ix] = 50;
        pf.obs_list.push_back(obs);
		// end testing hard coded obstacle list
		bot_yaw = get_yaw(bot_pose);
		geometry_msgs::Twist cmd = pf.update_cmd(bot_yaw);
		//ROS_INFO("bot_yaw: %0.2f", bot_yaw);

		cmd_pub_.publish(cmd);
		pf_obs_pub_.publish(pfObs);
	}
}

bool AvoidObs::get_map_indices(float x, float y, int& ix, int& iy)
{
	ix = boost::math::iround((x-map_pose.position.x)/map_res_ - map_res_);
	iy = boost::math::iround((y-map_pose.position.y)/map_res_ - map_res_);
	return true;
}

int AvoidObs::get_cost(int ix, int iy)
{
	if(0 > ix || ix >= n_width_ || 0 > iy || iy >= n_height_)
		return 200;
	return costmap.data[iy*n_width_ + ix];
}

void AvoidObs::odomCallback(const nav_msgs::Odometry& odom)
{
	bot_pose.position = odom.pose.pose.position;
	bot_pose.orientation = odom.pose.pose.orientation;

	pf.bot.x = bot_pose.position.x;
	pf.bot.y = bot_pose.position.y;
}

void AvoidObs::goalCallback(const geometry_msgs::PoseStamped& data)
{
	goal_pose = data.pose;
	pf.goal.x = goal_pose.position.x;
	pf.goal.y = goal_pose.position.y;
}

void AvoidObs::scanCallback(const sensor_msgs::LaserScan& scan) //use a point cloud instead, use laser2pc.launch
{
    //ROS_INFO("NEW SCAN");
	// Transform scan to map frame, clear and fill costmap
    listener.waitForTransform("laser", "odom", scan.header.stamp, ros::Duration(10.0)); // ros::Time(0) causes exceptions

	geometry_msgs::PointStamped laser_point, odom_point;
	laser_point.header.frame_id = "laser";
	laser_point.header.stamp = scan.header.stamp;//ros::Time();
	laser_point.point.z = 0;
	
	for (int i = 0; i < scan.ranges.size();i++)
	{
	    float range = scan.ranges[i];
	    float angle  = scan.angle_min +(float(i) * scan.angle_increment);

	    //clear map cells
	    // only clear at range >= 0.5 meters
        for (double r = 0.5; r < (range - map_res_*2.0); r += map_res_)
        {
            double angle_step = map_res_ / r;
            //clearing as we pass obstacles, try angle_increment/3 vs /2 (reduce clearing fov per laser)
            for (double a = (angle - scan.angle_increment / 2); a < (angle + scan.angle_increment / 2); a += angle_step)
            {
                laser_point.point.x = r * cos(a);
                laser_point.point.y = r * sin(a);
                try
                {
                    listener.transformPoint("odom", laser_point,
                            odom_point);
                    update_cell(odom_point.point.x, odom_point.point.y,
                            clear_decrement_); //CLEAR_VAL_DECREASE
                }
                catch (tf::TransformException& ex)
                {
                    int xa;
                    ROS_ERROR("AvoidObs clear Received an exception trying to transform a point : %s", ex.what());
                }

            }
        }

	    // fill obstacle cells
	    if(range < scan_range)
	    {
			laser_point.point.x = range*cos(angle) ;
			laser_point.point.y = range*sin(angle) ;
			int count = 0;
			while(count < 3)
			{
				++count;
				try{
					listener.transformPoint("odom", laser_point, odom_point);
					update_cell(odom_point.point.x, odom_point.point.y, fill_increment_);
					break;
				}
				catch(tf::TransformException& ex){
					int xa;
					ROS_ERROR("AvoidObs fill Received an exception trying to transform a point : %s", ex.what());
				}
			}
	    }
	}

	// Re-apply inflation to all obstacles within a radius of the bot, this covers outside the scan FOV
	int radius_ind = reinflate_n_cells_;
	int bot_ix, bot_iy;
    get_map_indices(bot_pose.position.x, bot_pose.position.y, bot_ix, bot_iy);

    for(int ix = bot_ix - radius_ind; ix < bot_ix+radius_ind; ++ix)
    {
        for(int iy = bot_iy - radius_ind; iy < bot_iy+radius_ind; ++iy)
        {
            if(get_cost(ix,iy) > reinflate_cost_thresh_)
            {
                int cur_val = costmap.data[iy * n_width_ + ix];
                //adjacent cell inflation
                for(int dx = -1; dx <= 1; ++dx)
                {
                    for(int dy = -1; dy <= 1; ++dy)
                    {
                        if((dx != 0 or dy != 0))
                        {
                            int iix = ix+dx;
                            int iiy = iy+dy;
                            if (0 <= iix && iix < n_width_ && 0 <= iiy && iiy < n_height_)
                            {
                                int adj_val = costmap.data[(iiy)*n_width_ + iix];
                                int new_val = double(cur_val)/(adjacent_cost_offset+double(abs(dx)+abs(dy))*adjacent_cost_slope);
                                if(adj_val < new_val)
                                    costmap.data[(iiy)*n_width_ + iix] = new_val;
                            }
                        }
                    }
                }
            }
        }
    }
    // END Re-apply inflation


	//odom_point.point.x = 10.0;
	//odom_point.point.y = -5.0;
	//update_cell(odom_point.point.x, odom_point.point.y, 100.0);

    check_for_cone_obstacle(); //clears obstacles near cone pose estimated from camera

    // ensure costmap has known obstacles
    check_known_obstacles();

    costmap.header.stamp = scan.header.stamp;
    costmap_pub_.publish(costmap);
}

void AvoidObs::check_known_obstacles()
{
  if(knownObstacleDeq.size() > 0)
  {
    // check if oldest obstacle has expired
    if( (ros::Time::now() - knownObstacleDeq[0].header.stamp).toSec() > known_obstacle_time_limit_)
    {
      knownObstacleDeq.pop_front();
    }

    // insert known obstacles into costmap
    for(unsigned k=0; k<knownObstacleDeq.size(); ++k)
    {
      const geometry_msgs::PoseStamped& obs_pose = knownObstacleDeq.at(k);
      update_cell(obs_pose.pose.position.x, obs_pose.pose.position.y, 100);
    }
  }
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "avoid_obs");

    AvoidObs avoid_obs;
    ROS_INFO("Starting Obstacle Avoidance");
    int loop_hz = avoid_obs.get_plan_rate();
    ros::Rate rate(loop_hz);

    while(ros::ok())
    {
        ros::spinOnce();
        avoid_obs.update_plan();
        rate.sleep();
    }

    return 0;
}
