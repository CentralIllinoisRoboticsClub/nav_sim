// Copyright 2019 coderkarl. Subject to the BSD license.

#include "nav_sim/AvoidObs.h"
//#include "nav_sim/AvoidObsCommon.h"
#include <geometry_msgs/msg/point_stamped.h>
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/utils.h> // tf2::getYaw(orientation or quat)

/**********************************************************************
* Obstacle Avoidance using a nav_msgs/OccupacyGrid and A* path planning
* 
* Subscribe to /scan and Publish OccupacyGrid /costmap, Publish Path /path
* 
**********************************************************************/

using std::placeholders::_1;

//Constructor
AvoidObs::AvoidObs() :
    Node("avoid_obs"),
    qos_(rclcpp::QoSInitialization::from_rmw(rmw_qos_profile_sensor_data))
{
    // https://github.com/ros-planning/navigation2/blob/foxy-devel/nav2_amcl/src/amcl_node.cpp
    tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
    auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
        get_node_base_interface(),
        get_node_timers_interface());
    tfBuffer->setCreateTimerInterface(timer_interface);
    listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

    //Topics you want to publish
    costmap_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("costmap", 1);
    cmd_pub_ = create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

    pf_obs_pub_ = create_publisher<nav_msgs::msg::OccupancyGrid>("pfObs", 1);

    obs_cone_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("obs_cone_pose",1);

    //Topic you want to subscribe
    scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", rclcpp::SensorDataQoS(), std::bind(&AvoidObs::scanCallback, this, _1)); //receive laser scan
    odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", 10, std::bind(&AvoidObs::odomCallback, this, _1));

    // "/move_base_simple/goal"
    goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("wp_goal", 10, std::bind(&AvoidObs::goalCallback, this, _1));
    wp_cone_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("wp_cone_pose", 10, std::bind(&AvoidObs::coneCallback, this, _1));
    found_cone_sub_ = create_subscription<std_msgs::msg::Int16>("found_cone", 10, std::bind(&AvoidObs::foundConeCallback, this, _1));
    known_obstacle_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("known_obstacle", 10, std::bind(&AvoidObs::knownObstacleCallback, this, _1));
    hill_wp_sub_ = create_subscription<std_msgs::msg::Int16>("hill_wp", 10, std::bind(&AvoidObs::hillWaypointCallback, this, _1));

    plan_rate_ = declare_parameter("plan_rate_hz", 1.0);

    map_res_ = declare_parameter("map_res_m", 0.5);
    n_width_ = declare_parameter("map_size", 200);
    n_height_ = n_width_;
    min_range_ = declare_parameter("min_range", 0.05);
    max_range_ = declare_parameter("max_range", 40.0);
    min_hill_range_ = declare_parameter("min_hill_range", 1.0);
    plan_range_ = declare_parameter("plan_range_m", 40.0);
    clear_decrement_ = declare_parameter("clear_decrement", -5);
    fill_increment_ = declare_parameter("fill_increment", 10);
    adjacent_cost_offset = declare_parameter("adjacent_cost_offset", 2.0);
    adjacent_cost_slope = declare_parameter("adjacent_cost_slope", 1.0);
    inflation_factor_ = declare_parameter("inflation_factor", 2);
    reinflate_radius_ = declare_parameter("reinflate_radius", 2.5);
    cone_search_radius_ = declare_parameter("cone_search_radius", 1.0);
    reinflate_cost_thresh_ = declare_parameter("reinflate_cost_thresh", 30);
    use_PotFields_ = declare_parameter("use_PotFields", false);
    cone_obs_thresh_ = declare_parameter("cone_obs_thresh", 20);
    max_num_known_obstacles_ = declare_parameter("max_num_known_obstacles", 20);
    known_obstacle_time_limit_ = declare_parameter("known_obstacle_time_limit", 30.0);

    scan_range = max_range_;

    RCLCPP_INFO(get_logger(), "map_size (n cells): %d", n_width_);
    
    reinflate_n_cells_ = boost::math::iround(reinflate_radius_/map_res_);
    cone_search_n_cells_ = boost::math::iround(cone_search_radius_/map_res_);

    //listener.setExtrapolationLimit(ros::Duration(0.1));
    //listener.waitForTransform("laser", "odom", ros::Time(0), ros::Duration(10.0));
    try{
      tfBuffer->lookupTransform("laser", "odom", rclcpp::Time(0), rclcpp::Duration(10.0));
    } catch (tf2::TransformException &ex) {
      RCLCPP_WARN(get_logger(), "AvoidObs: %s",ex.what());
    }

    num_obs_cells = 0; //number of obstacle cells
    
    map_pose.position.x = -(n_width_/2)*map_res_-map_res_/2;//1.0 -(n_width_/2)*map_res_ + map_res_/2; // I believe this is zero by default, check by echoing costmap
    map_pose.position.y = -(n_height_/2)*map_res_-map_res_/2;//1.0 -(n_height_/2)*map_res_ + map_res_/2; // will need to update x and y as we move
    RCLCPP_INFO(get_logger(), "map_pose x,y: %0.2f, %0.2f",map_pose.position.x, map_pose.position.y);
    map_pose.orientation.w = 1.0;
    
    costmap.header.stamp = now();
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

    m_timer = create_wall_timer(std::chrono::milliseconds((int)(1000./plan_rate_) ),
              std::bind(&AvoidObs::update_plan, this) );

    RCLCPP_INFO(get_logger(), "Starting Obstacle Avoidance");
}

AvoidObs::~AvoidObs(){}

void AvoidObs::knownObstacleCallback(const geometry_msgs::msg::PoseStamped::SharedPtr obs_pose)
{
  // TODO: transform known obstacle to odom frame if not already
  knownObstacleDeq.push_back(*obs_pose);
  //TODO: Consider using boost circular buffer
  if(knownObstacleDeq.size() > max_num_known_obstacles_)
  {
    knownObstacleDeq.pop_front();
  }
}

void AvoidObs::hillWaypointCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  if(msg->data == 1)
  {
    scan_range = min_hill_range_;
  }
  else
  {
    scan_range = max_range_;
  }
}

void AvoidObs::coneCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
    camera_cone_poseStamped = *data;
}

void AvoidObs::foundConeCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  if(msg->data == 0) // need to reset the camera_cone_poseStamped so the just bumped cone now becomes an obstacle again
  {
    // TODO: add a bool flag when we bump a cone and update a waypoint to next cone, clear the flag in next coneCallback
    //  do not check_for_cone_obstacle if this flag is set
    camera_cone_poseStamped.pose.position.x = 0;
    camera_cone_poseStamped.pose.position.y = 0;
  }
}

bool AvoidObs::check_for_cone_obstacle()
{
    double sec_since_cone = (now()-camera_cone_poseStamped.header.stamp).seconds();
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
                        obs_cone_poseStamped.header.stamp = now();
                        obs_cone_poseStamped.pose.position.x = map_pose.position.x + (cx+dx)*map_res_ + map_res_/2;
                        obs_cone_poseStamped.pose.position.y = map_pose.position.y + (cy+dy)*map_res_ + map_res_/2;
                    }
                }
            }
        }
        if(max_nearby_cost > cone_obs_thresh_)
        {
            obs_cone_pub_->publish(obs_cone_poseStamped);
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

void AvoidObs::update_plan()
{
	/*path.poses.clear();
	geometry_msgs::msg::PoseStamped wp;
	wp.pose.position.x = 5.0;
	wp.pose.position.y = 5.0;
	path.poses.push_back(wp);
	*/

	geometry_msgs::msg::Pose start, temp_goal = goal_pose;

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
		pfObs.header.stamp = now();
		pfObs.data.clear();
		pfObs.data.resize(n_width_*n_height_);

		pf.obs_list.clear();
		int bot_ix, bot_iy;
		get_map_indices(pf.bot.x, pf.bot.y, bot_ix, bot_iy);

		for(int ix = bot_ix - 15; ix < bot_ix+15; ++ix)
		{
			for(int iy = bot_iy - 15; iy < bot_iy+15; ++iy)
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
		bot_yaw = tf2::getYaw(bot_pose.orientation); //get_yaw(bot_pose);
		geometry_msgs::msg::Twist cmd = pf.update_cmd(bot_yaw);
		//ROS_INFO("bot_yaw: %0.2f", bot_yaw);

		cmd_pub_->publish(cmd);
		pf_obs_pub_->publish(pfObs);
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

void AvoidObs::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  //RCLCPP_WARN(get_logger(), "odomCallback");
	bot_pose.position = odom->pose.pose.position;
	bot_pose.orientation = odom->pose.pose.orientation;

	pf.bot.x = bot_pose.position.x;
	pf.bot.y = bot_pose.position.y;
}

void AvoidObs::goalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
	goal_pose = data->pose;
	pf.goal.x = goal_pose.position.x;
	pf.goal.y = goal_pose.position.y;
}

void AvoidObs::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) //use a point cloud instead, use laser2pc.launch
{
  //RCLCPP_WARN(get_logger(), "scanCallback");
  //ROS_INFO("NEW SCAN");
	// Transform scan to map frame, clear and fill costmap
  //listener.waitForTransform("laser", "odom", scan->header.stamp, ros::Duration(10.0)); // ros::Time(0) causes exceptions

  // https://answers.ros.org/question/273205/transfer-a-pointxyz-between-frames/
  geometry_msgs::msg::TransformStamped transformStamped;
  try{
    //transformStamped = tfBuffer->lookupTransform("laser", "odom", scan->header.stamp, rclcpp::Duration(0.05));
    transformStamped = tfBuffer->lookupTransform("odom", "laser", rclcpp::Time(0) );
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "AvoidObs: %s",ex.what());
    return;
  }
  //RCLCPP_WARN(get_logger(), "Post get transform");

	geometry_msgs::msg::PointStamped laser_point, odom_point;
	laser_point.header.frame_id = "laser";
	laser_point.header.stamp = scan->header.stamp;//ros::Time();
	laser_point.point.z = 0;
	
	/*
	laser_point.point.x = 1.0;
	laser_point.point.y = 0.0;
	tf2::doTransform(laser_point, odom_point, transformStamped);
	RCLCPP_WARN(get_logger(), "out1 %.2f, %.2f", odom_point.point.x, odom_point.point.y);

	odom_point.header.frame_id = "odom";
	tf2::doTransform(laser_point, odom_point, transformStamped);
	RCLCPP_WARN(get_logger(), "out2 %.2f, %.2f", odom_point.point.x, odom_point.point.y);
	*/

	for (int i = 0; i < scan->ranges.size();i++)
	{
	    float range = scan->ranges[i];
	    if(range < min_range_)
	    	continue;

	    bool clear_only = false;
	    if (not std::isfinite(range))
	    {
	      range = max_range_;
	      clear_only = true;
	      continue;
	    }

	    float angle  = scan->angle_min +(float(i) * scan->angle_increment);

	    //clear map cells
	    // only clear at range >= 0.5 meters
	    //RCLCPP_WARN(get_logger(), "CLEAR");
        for (double r = 0.1; r < (range - map_res_*2.0); r += map_res_)
        {
            double angle_step = map_res_ / r;
            //RCLCPP_WARN(get_logger(), "map_res_ %.2f, r %.2f, step %.3f", map_res_, r, angle_step);
            //clearing as we pass obstacles, try angle_increment/3 vs /2 (reduce clearing fov per laser)

            for (double a = (angle - scan->angle_increment / 6); a < (angle + scan->angle_increment / 6); a += angle_step)
            {
                //RCLCPP_WARN(get_logger(),  "\na %.2f, limit %.2f", a, angle + scan->angle_increment/2);
                laser_point.point.x = r * cos(a);
                laser_point.point.y = r * sin(a);
                // Should now be able to remove try catch here
                try
                {
                	//RCLCPP_WARN(get_logger(), "preupdateCell");
                    //listener.transformPoint("odom", laser_point,odom_point);
                    //tfBuffer->transform(laser_point, odom_point, "odom", tf2::durationFromSec(1.0));
                    tf2::doTransform(laser_point, odom_point, transformStamped);
                    update_cell(odom_point.point.x, odom_point.point.y,
                            clear_decrement_); //CLEAR_VAL_DECREASE
                }
                catch (tf2::TransformException& ex)
                {
                    int xa;
                    RCLCPP_ERROR(get_logger(), "AvoidObs clear Received an exception trying to transform a point : %s", ex.what());
                }
            }
        }

        if(clear_only)
        {
          continue;
        }

      //RCLCPP_WARN(get_logger(), "FILL");
	    // fill obstacle cells
        bool good_tf = true;
	    if(range < scan_range)
	    {
			laser_point.point.x = range*cos(angle) ;
			laser_point.point.y = range*sin(angle) ;
			int count = 0;
			while(count < 3)
			{
				++count;
				try{
					//listener.transformPoint("odom", laser_point, odom_point);
					//tfBuffer->transform(laser_point, odom_point, "odom", tf2::durationFromSec(1.0));
				  tf2::doTransform(laser_point, odom_point, transformStamped);
					update_cell(odom_point.point.x, odom_point.point.y, fill_increment_);
					good_tf = true; // Without this, must have good tf on first try
					break;
				}
				catch(tf2::TransformException& ex){
					good_tf = false;
					int xa;
					RCLCPP_ERROR(get_logger(), "AvoidObs fill Received an exception trying to transform a point : %s", ex.what());
				}
			}
			if(!good_tf)
			{
				break;
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

    //RCLCPP_WARN(get_logger(), "PUBLISH COSTMAP");
    costmap.header.stamp = scan->header.stamp;
    costmap_pub_->publish(costmap);
}

void AvoidObs::check_known_obstacles()
{
  if(knownObstacleDeq.size() > 0)
  {
    // check if oldest obstacle has expired
    if( (now() - knownObstacleDeq[0].header.stamp).seconds() > known_obstacle_time_limit_)
    {
      knownObstacleDeq.pop_front();
    }

    // insert known obstacles into costmap
    for(unsigned k=0; k<knownObstacleDeq.size(); ++k)
    {
      const geometry_msgs::msg::PoseStamped& obs_pose = knownObstacleDeq.at(k);
      update_cell(obs_pose.pose.position.x, obs_pose.pose.position.y, 100);
    }
  }
}

int main(int argc, char **argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AvoidObs>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}
