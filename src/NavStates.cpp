// Copyright 2019 coderkarl. Subject to the BSD license.

#include "nav_sim/NavStates.h"
//#include "avoid_obstacles/AvoidObsCommon.h" // get_yaw()
#include <geometry_msgs/msg/point_stamped.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <math.h>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>
#include <tf2_ros/create_timer_ros.h>
#include <tf2/utils.h> // tf2::getYaw(orientation or quat)

/**********************************************************************
 * State machine for navigating to waypoints and perception targets
 *
 * Subscribe to /scan, /odom, /path and Publish /cmd_vel, /wp_goal
 **********************************************************************/

#define STATE_RETREAT_FROM_CONE -2
#define STATE_RETREAT -1
#define STATE_TRACK_PATH 0
#define STATE_TURN_TO_TARGET 1
#define STATE_TOUCH_TARGET 2
#define STATE_SEARCH_IN_PLACE 3
#define STATE_TRACK_MOW_PATH 10

#define WP_TYPE_INTER 0
#define WP_TYPE_CONE 1

using std::placeholders::_1;
using std::placeholders::_2;

//Constructor
NavStates::NavStates(const rclcpp::NodeOptions& node_options) :
Node("nav_states", node_options),
m_current_waypoint_type(WP_TYPE_INTER),
m_current_hill_type(0),
m_collision(false),
m_cone_detected(false),
m_odom_received(false),
m_path_received(false),
m_obs_cone_received(false),
m_init_wp_published(false),
m_odom_goal_refresh_needed(false),
m_first_search(true),
m_valid_bump(false),
m_bump_switch(0),
m_bump_count(0),
m_num_waypoints(0),
m_index_wp(0),
m_state(STATE_TRACK_PATH),
m_index_path(0),
m_speed(0.0),
m_omega(0.0),
m_filt_speed(0.0),
m_scan_collision_db_count(0),
m_cone_detect_db_count(0),
m_prev_heading_error(0.0),
m_close_to_obs(false),
m_update_pf_waypoint(false)
{
  // https://github.com/ros-planning/navigation2/blob/foxy-devel/nav2_amcl/src/amcl_node.cpp
  tfBuffer = std::make_shared<tf2_ros::Buffer>(get_clock());
  auto timer_interface = std::make_shared<tf2_ros::CreateTimerROS>(
  		get_node_base_interface(),
  		get_node_timers_interface());
  tfBuffer->setCreateTimerInterface(timer_interface);
  listener = std::make_shared<tf2_ros::TransformListener>(*tfBuffer);

  //Topics you want to publish
  cmd_pub_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel",10);

  wp_static_map_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("static_map_goal",1);
  wp_goal_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("wp_goal",1);
  wp_cone_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("wp_cone_pose",1); //temp, Need to send this to avoid_obs to clear the costmap at cone
  nav_state_pub_ = create_publisher<std_msgs::msg::Int16>("nav_state",1);
  known_obs_pub_ = create_publisher<geometry_msgs::msg::PoseStamped>("known_obstacle",1);
  hill_wp_pub_ = create_publisher<std_msgs::msg::Int16>("hill_wp",1);
  valid_bump_pub_ = create_publisher<std_msgs::msg::Int16>("valid_bump",1);

  //Topic you want to subscribe
  scan_sub_ = create_subscription<sensor_msgs::msg::LaserScan>("scan", 50, std::bind(&NavStates::scanCallback, this, _1)); //receive laser scan
  odom_sub_ = create_subscription<nav_msgs::msg::Odometry>("odom", rclcpp::SensorDataQoS(), std::bind(&NavStates::odomCallback, this, _1));
  clicked_goal_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("goal_pose", 1, std::bind(&NavStates::clickedGoalCallback, this, _1));
  cam_cone_pose_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("cam_cone_pose", 1, std::bind(&NavStates::camConeCallback, this, _1));
  obs_cone_sub_ = create_subscription<geometry_msgs::msg::PoseStamped>("obs_cone_pose", 1, std::bind(&NavStates::obsConeCallback, this, _1));
  bump_sub_ = create_subscription<std_msgs::msg::Int16>("bump_switch",1, std::bind(&NavStates::bumpCallback, this, _1));
  path_sub_ = create_subscription<nav_msgs::msg::Path>("path",5, std::bind(&NavStates::pathCallback, this, _1));
  map_to_odom_update_sub_ = create_subscription<std_msgs::msg::Int16>("map_to_odom_update", 1, std::bind(&NavStates::mapToOdomUpdateCallback, this, _1));
  
  pf_wp_update_sub_ = create_subscription<std_msgs::msg::Int16>("pf_wp_update", 1, std::bind(&NavStates::pfWayPointUpdateCallback, this, _1));

  mow_area_server_ = create_service<my_interfaces::srv::SetInt>(
                "set_mow_area", std::bind(&NavStates::mow_area_callback, this, _1, _2) );


  params.plan_rate = declare_parameter("plan_rate_hz", 10.0);
  params.use_PotFields = declare_parameter("use_PotFields", false);
  params.valid_cone_to_wp_dist = declare_parameter("valid_cone_to_wp_dist", 1.0);
  params.near_path_dist = declare_parameter("near_path_dist", 1.0);
  params.valid_end_of_path_dist = declare_parameter("valid_end_of_path_dist", 5.0);
  params.desired_speed = declare_parameter("desired_speed", 0.6);
  params.slow_speed = declare_parameter("slow_speed", 0.3);
  params.max_omega = declare_parameter("max_omega", 0.5);
  params.max_fwd_heading_error_deg = declare_parameter("max_fwd_heading_error_deg", 90.0);
  params.search_time = declare_parameter("search_time",  1.0);
  params.search_omega = declare_parameter("search_omega",  0.1);
  params.reverse_time = declare_parameter("reverse_time",  2.0);
  params.cmd_control_ver = declare_parameter("cmd_control_ver",  0);
  params.scan_collision_db_limit = declare_parameter("scan_collision_db_limit",  2);
  params.scan_collision_range = declare_parameter("scan_collision_range",  0.5);
  params.cone_detect_db_limit = declare_parameter("cone_detect_db_limit",  2);
  params.cmd_speed_filter_factor = declare_parameter("cmd_speed_filter_factor",  0.5);
  params.report_bumped_obstacles = declare_parameter("report_bumped_obstacles",  false);
  params.max_camera_search_time = declare_parameter("max_camera_search_time",  7.0);
  params.slow_approach_distance = declare_parameter("slow_approach_distance",  1.0);
  params.reverse_speed = declare_parameter("reverse_speed",  0.8);
  params.bump_db_limit = declare_parameter("bump_db_limit",  2);
  params.path_step_size = declare_parameter("path_step_size",  3);

  std::vector<double> xvec, yvec;
  xvec = declare_parameter("x_coords0", std::vector<double>({5.0, 0.0})); //.as_double_array();
  yvec = declare_parameter("y_coords0", std::vector<double>({5.0, 0.0}));
  x_coords.push_back(xvec);
  y_coords.push_back(yvec);
  xvec = declare_parameter("x_coords1", std::vector<double>({5.0, 0.0})); //.as_double_array();
  yvec = declare_parameter("y_coords1", std::vector<double>({5.0, 0.0}));
  x_coords.push_back(xvec);
  y_coords.push_back(yvec);
  xvec = declare_parameter("x_coords2", std::vector<double>({5.0, 0.0})); //.as_double_array();
  yvec = declare_parameter("y_coords2", std::vector<double>({5.0, 0.0}));
  x_coords.push_back(xvec);
  y_coords.push_back(yvec);
  xvec = declare_parameter("x_coords3", std::vector<double>({5.0, 0.0})); //.as_double_array();
  yvec = declare_parameter("y_coords3", std::vector<double>({5.0, 0.0}));
  x_coords.push_back(xvec);
  y_coords.push_back(yvec);
  coords_index = 0;
  //rclcpp::Parameter double_array_param = get_parameter("x_coords");
  //x_coords = double_array_param.as_double_array();
  //x_coords = declare_parameter<std::vector<double> >("x_coords");
  //y_coords = declare_parameter<std::vector<double> >("y_coords");
  //double_array_param = get_parameter("y_coords");
  //y_coords = double_array_param.as_double_array();
  
  waypoint_type_list = declare_parameter("waypoint_types", std::vector<int64_t>({0,0}));
  hill_wp_list = declare_parameter("hill_waypoint_list", std::vector<int64_t>({0,0}));

  waypoints_are_in_map_frame = declare_parameter("waypoints_are_in_map_frame", true);
  sim_mode = declare_parameter("sim_mode", false);

  is_mow_boundary = declare_parameter("is_mow_boundary", false);
  mow_ccw = declare_parameter("mow_ccw", true);
  mow_width = declare_parameter("mow_width", 0.4);
  mow_wp_spacing = declare_parameter("mow_wp_spacing", 2.0);
  mow_inside_heading_offset = mow_ccw ? M_PI/2 : -M_PI/2;
  RCLCPP_INFO(get_logger(), "mow inside heading offset = %.2f", mow_inside_heading_offset);

  //listener.setExtrapolationLimit(ros::Duration(0.1));
  //listener.waitForTransform("laser", "odom", rclcpp::Time(0), ros::Duration(10.0)); //TODO: rclcpp::Time(0) or ::now() ??
  //listener.waitForTransform("base_link", "odom", rclcpp::Time(0), ros::Duration(10.0));
  try{
    tfBuffer->lookupTransform("laser", "odom", rclcpp::Time(0), rclcpp::Duration(10, 0));
    tfBuffer->lookupTransform("base_link", "odom", rclcpp::Time(0), rclcpp::Duration(10, 0));
  } catch (tf2::TransformException &ex) {
    RCLCPP_WARN(get_logger(), "NavStates: %s",ex.what());
  }

  if(waypoints_are_in_map_frame)
  {
    map_goal_pose.header.frame_id = "map";
  }
  else
  {
    map_goal_pose.header.frame_id = "odom";
  }
  bot_pose.header.frame_id = "odom";
  map_goal_pose.pose.orientation.w = 1.0;
  camera_cone_pose_in_map = map_goal_pose;

  bot_pose.pose.orientation.w = 1.0;
  odom_goal_pose = bot_pose;
  bot_yaw = 0.0;

  camera_cone_pose.header.frame_id = "odom";
  obs_cone_pose.header.frame_id = "odom";
  camera_cone_pose.pose.orientation.w = 1.0;
  obs_cone_pose.pose.orientation.w = 1.0;

  update_mow_path();
  update_waypoint();

  m_state = STATE_TRACK_PATH;
  if(is_mow_boundary)
  {
    RCLCPP_INFO(get_logger(), "MOW BOUNDARY");
    m_state = STATE_TRACK_MOW_PATH;
  }

  state_init();

  RCLCPP_INFO(get_logger(), "Initialized Navigation State Manager");

  //std::chrono::milliseconds period_msec = (int)(1000.0/get_plan_rate());
  //timer_ = rclcpp::create_wall_timer(std::chrono::milliseconds, std::bind(&NavStates::update_states, this) );
}

void NavStates::update_mow_path()
{
  bool use_lanes = true;
  const std::vector<double>& xc = x_coords[coords_index];
  const std::vector<double>& yc = y_coords[coords_index];

  // TODO: index vec for each mow area?
  // TODO: save and load index to/from file ?
  m_index_wp = 0;

  if(!is_mow_boundary)
  {
    m_waypoints.clear();
    m_num_waypoints = xc.size();
    for(unsigned k=0; k<m_num_waypoints; ++k)
    {
      geometry_msgs::msg::Point wp; wp.x = xc[k]; wp.y = yc[k];
      m_waypoints.push_back(wp);
    }
  }
  else if(use_lanes)
  {
    std::vector<geometry_msgs::msg::Point> bound1;
    bound1.clear();
    m_waypoints.clear();
    unsigned numKeyPts = xc.size();
    for(unsigned k=0; k<numKeyPts; ++k)
    {
      geometry_msgs::msg::Point wp; wp.x = xc[k]; wp.y = yc[k];
      m_waypoints.push_back(wp);
      bound1.push_back(wp);
    }

    if(numKeyPts == 4)
    {
      m_waypoints.clear();
      float c = mow_wp_spacing;
      float d = mow_width;
      geometry_msgs::msg::Point pa, pb, pc;
      pa = bound1[0]; pb = bound1[1]; pc = bound1[2];
      float dx = pb.x - pa.x;
      float dy = pb.y - pa.y;
      float distDown = sqrt(dx*dx + dy*dy);
      float xDown = dx/distDown;
      float yDown = dy/distDown;

      dx = pc.x - pb.x;
      dy = pc.y - pb.y;
      float distCross = sqrt(dx*dx + dy*dy);
      float xCross = dx/distCross;
      float yCross = dy/distCross;

      int nLanes = distCross/d + 1;
      int nSteps = distDown/c;
      float gap = distDown/nSteps;

      geometry_msgs::msg::Point wp = pa;
      for(int j=0; j<=nLanes; ++j)
      {
        for(int k=0; k<=nSteps; ++k)
        {
          std::cout << "wp," << wp.x << "," << wp.y << "\n";
          m_waypoints.push_back(wp);
          wp.x += gap*xDown;
          wp.y += gap*yDown;
        }
        std::cout << "wp," << wp.x << "," << wp.y << "\n";
        m_waypoints.push_back(wp);
        wp.x += d*xCross;
        wp.y += d*yCross;
        xDown = -xDown;
        yDown = -yDown;
      }
    }
    m_num_waypoints = m_waypoints.size();
  } // end if use lanes mowing
  else // mow boundary, make more frequent waypoints
  {
    float offset_gamma = mow_inside_heading_offset;
    std::vector<geometry_msgs::msg::Point> bound1, bound2, bound_offset;
    bound1.clear(); bound2.clear(); bound_offset.clear();
    unsigned numKeyPts = xc.size();
    for(unsigned k=0; k<numKeyPts; ++k)
    {
      geometry_msgs::msg::Point wp; wp.x = xc[k]; wp.y = yc[k];
      bound1.push_back(wp);
    }

    // create up to nBound shrinking boundary loops
    float c = mow_wp_spacing;
    float d = mow_width;
    unsigned nBound = 10;
    m_waypoints.clear();
    bool done = false;
    for(unsigned loopNum=0; loopNum<nBound; ++loopNum)
    {
      if(done)
        break;

      for(unsigned bk=0; bk<numKeyPts; ++bk)
      {
        const geometry_msgs::msg::Point& p = bound1[bk];
        const geometry_msgs::msg::Point& n = (bk==(numKeyPts-1) ? bound1.front() : bound1[bk+1]);
        //m_waypoints.push_back(p);
        std::cout << "keyPoint," << p.x << "," << p.y << "\n";
        if(isnan(p.y))
        {
          done = true;
          break;
        }

        float dx = n.x - p.x;
        float dy = n.y - p.y;
        float dist_limit = sqrt(dx*dx + dy*dy);
        if(dist_limit < d)
        {
          done = true;
          break;
        }
        float ux = dx/dist_limit;
        float uy = dy/dist_limit;

        // Next loop boundary points
        if(loopNum == 0)
        {
          // Prepare next boundary points
          geometry_msgs::msg::Point prev;
          if(bk==0)
            prev = bound1.back();
          else
            prev = bound1[bk-1];
          float prev_dx = p.x - prev.x;
          float prev_dy = p.y - prev.y;
          float mag = sqrt(prev_dx*prev_dx + prev_dy*prev_dy);
          float vx = prev_dx / mag;
          float vy = prev_dy / mag;
          float netx = ux + vx;
          float nety = uy + vy;
          float net_mag = sqrt(netx*netx + nety*nety);
          geometry_msgs::msg::Point t;
          t.x = -nety/net_mag*sin(offset_gamma);
          t.y =  netx/net_mag*sin(offset_gamma);
          bound_offset.push_back(t);
        }
        else
        {
          if(bound_offset.size() != numKeyPts)
          {
            RCLCPP_INFO(get_logger(), "BOUND OFFSETS ERROR, MOW BOUNDARY FAILED");
          }
        }

        geometry_msgs::msg::Point p2;
        p2.x = p.x + d*bound_offset[bk].x;
        p2.y = p.y + d*bound_offset[bk].y;
        bound2.push_back(p2);
        // End next loop boundary points

        geometry_msgs::msg::Point cp;
        cp.x = p.x + c*ux;
        cp.y = p.y + c*uy;
        float dist = c;

        // finer points between p and n
        while(dist < (dist_limit - c/2))
        {
          m_waypoints.push_back(cp);
          std::cout << "cp," << cp.x << "," << cp.y << "\n";
          if(isnan(cp.y))
          {
            done = true;
            break;
          }
          cp.x += c*ux;
          cp.y += c*uy;
          dist += c;
        }
      }

      if(!done)
      {
        bound1 = bound2;
        bound2.clear();
      }
    }

    m_num_waypoints = m_waypoints.size();
  } // end if mow boundary
}

void NavStates::update_waypoint()
{
  m_first_search = true; // used to reset the m_init_search_time in search_in_place state
  map_goal_pose.header.stamp = now();
  if(!is_mow_boundary)
  {
    m_current_waypoint_type = waypoint_type_list[m_index_wp];
    m_current_hill_type = hill_wp_list[m_index_wp];
  }
  map_goal_pose.pose.position.x = m_waypoints[m_index_wp].x;
  map_goal_pose.pose.position.y = m_waypoints[m_index_wp].y;
  camera_cone_pose_in_map = map_goal_pose;
  RCLCPP_INFO(get_logger(), "Update Waypoint");
  RCLCPP_INFO(get_logger(), "map x,y = %0.1f, %0.1f",map_goal_pose.pose.position.x, map_goal_pose.pose.position.y);
  wp_static_map_goal_pub_->publish(map_goal_pose);
  geometry_msgs::msg::PoseStamped odom_pose;
  odom_pose.header.frame_id = "odom";
  if(getPoseInFrame(map_goal_pose, "odom", odom_pose))
  {
    odom_goal_pose = odom_pose;
    RCLCPP_INFO(get_logger(), "odom x,y = %0.1f, %0.1f",odom_goal_pose.pose.position.x, odom_goal_pose.pose.position.y);
    ++m_index_wp;
    if(m_index_wp == m_num_waypoints)
    {
      m_index_wp = 0;
    }
    wp_goal_pub_->publish(odom_goal_pose);
    if(m_current_waypoint_type == WP_TYPE_CONE)
    {
      camera_cone_pose = odom_goal_pose;
      wp_cone_pub_->publish(camera_cone_pose); // publish this once to avoid obs so it will clear the costmap at the initial goal, until it finds an updated goal from the camera
    }
    RCLCPP_INFO(get_logger(), "wp_goal published");
    m_init_wp_published = true;
    m_path_received = false; //avoid updating waypoint again if Astar has not yet responded to the just updated waypoint
    m_obs_cone_received = false;
    m_cone_detect_db_count = 0;
  }
}

// Because can2ros now updates map_to_odom, update the goal in the new transformed odom frame
//  This is triggered by the mapToOdomUpdateCallback to topic "map_to_odom_update"
//  This will continue to be called until the getPoseinFrame is successful after a triggered refresh
void NavStates::refresh_odom_goal()
{
  //camera_cone_pose_in_map.header.stamp = now(); // Moved to the mapToOdomUpdateCallback
  geometry_msgs::msg::PoseStamped odom_pose;
  odom_pose.header.frame_id = "odom";
  if(getPoseInFrame(camera_cone_pose_in_map, "odom", odom_pose))
  {
    odom_goal_pose = odom_pose;

    wp_goal_pub_->publish(odom_goal_pose);
    if(m_current_waypoint_type == WP_TYPE_CONE)
    {
      camera_cone_pose = odom_goal_pose;
      wp_cone_pub_->publish(camera_cone_pose); // publish this once to avoid obs so it will clear the costmap at the initial goal, until it finds an updated goal from the camera
    }
    m_odom_goal_refresh_needed = false;
  }
}

void NavStates::pathCallback(const nav_msgs::msg::Path::SharedPtr path_in)
{
  // Verify m_path_received does not become true between update_waypoint() setting m_init_wp_published true and Astar publishing a new path to the old waypoint
  unsigned path_size = path_in->poses.size();
  if(path_size > 0)
  {
    // It is assumed that the next waypoint is far enough away such that the distance between the newly updated odom_goal_pose and a path to an old goal will be > valid_end_of_path_dist
    if(distance_between_poses(path_in->poses[path_size-1], odom_goal_pose) < params.valid_end_of_path_dist) //both poses should be in the odom frame
    {
      m_path = *path_in;
      m_path_received = true;
      m_index_path = 0;
    }
  }

}

void NavStates::camConeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr cone_pose_in)
{
  // TODO: If state = search in place, set a boolean for search in place to move to state = STOP_AND_SEARCH
  //   we also want to do this from the track path state if we are near the end of path

  if(m_state != STATE_RETREAT_FROM_CONE && m_current_waypoint_type == WP_TYPE_CONE) //TODO: Verify we should ignore this for intermediate waypoints
  {
    geometry_msgs::msg::PoseStamped cone_in_map;
    if(waypoints_are_in_map_frame)
    {
      cone_in_map.header.frame_id = "map";
    }
    else
    {
      cone_in_map.header.frame_id = "odom";
    }
    if(getPoseInFrame(*cone_pose_in, cone_in_map.header.frame_id, cone_in_map))
    {
      double dist = distance_between_poses(cone_in_map, map_goal_pose);
      RCLCPP_INFO(get_logger(), "camConeCallback, dist btwn cone and goal in map = %0.1f",dist);
      if(dist < params.valid_cone_to_wp_dist)
      {
        geometry_msgs::msg::PoseStamped cone_in_odom;
        cone_in_odom.header.frame_id = "odom";
        if(getPoseInFrame(*cone_pose_in, "odom", cone_in_odom) )
        {
          ++m_cone_detect_db_count;
          // TODO: Consider still checking bot to cone distance even though cone_finder also checks this
          if(m_cone_detect_db_count >= params.cone_detect_db_limit)
          {
            camera_cone_pose_in_map = cone_in_map;
            camera_cone_pose = cone_in_odom;
            wp_cone_pub_->publish(camera_cone_pose);
            m_cone_detect_db_count = params.cone_detect_db_limit;
            m_cone_detected = true;
            RCLCPP_INFO(get_logger(), "cone detected");
          }
        }
        else if(m_cone_detect_db_count > 0)
        {
          --m_cone_detect_db_count;
        }
      }
    }
  }
}

void NavStates::obsConeCallback(const geometry_msgs::msg::PoseStamped::SharedPtr obs_cone_pose_in)
{
  obs_cone_pose = *obs_cone_pose_in;
  m_obs_cone_received = true;
}

double NavStates::distance_between_poses(const geometry_msgs::msg::PoseStamped& pose1, const geometry_msgs::msg::PoseStamped& pose2)
{
  double dx = pose1.pose.position.x - pose2.pose.position.x;
  double dy = pose1.pose.position.y - pose2.pose.position.y;
  return sqrt(dx*dx + dy*dy);
}

bool NavStates::getPoseInFrame(const geometry_msgs::msg::PoseStamped& pose_in, std::string target_frame,
    geometry_msgs::msg::PoseStamped& pose_out)
{
  try
  {
    tfBuffer->transform(pose_in, pose_out, target_frame, tf2::durationFromSec(10.0));
    //geometry_msgs::msg::TransformStamped tf =
    //	tfBuffer.lookupTransform(target_frame, pose_in.header.frame_id, req_time, rclcpp::Duration(10.0));
    return true;
  }
  catch (tf2::TransformException& ex)
  {
    RCLCPP_ERROR(get_logger(), "NavStates getPose Received an exception trying to transform a pose : %s", ex.what());
    return false;

  }
  return true;
}

void NavStates::bumpCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  m_bump_switch = msg->data;
  m_bump_count += m_bump_switch;
  if(m_bump_switch == 0)
  {
    m_bump_count = 0;
  }

  if(m_bump_count >= params.bump_db_limit)
  {
    m_valid_bump = true;
  }
  else
  {
    m_valid_bump = false;
  }

  if(m_valid_bump && params.report_bumped_obstacles)
  {
    // TODO: Revisit the storage of known obs poses in Avoid Obs now that we will add potentially many known obstacles using the bumper
    //       Check for duplicate known obstacles in the deque, store them with low resolution so very close obstacles are considered duplicates
    //       Store the unique costmap index number of known obstacles and use that to check for duplicates
    // Known obstacle pose should be published in the odom frame for AvoidObs costmap
    if( (now()-m_bump_time).seconds() > params.reverse_time)
    {
      known_obs_pub_->publish(bot_pose);
      m_bump_time = now();
    }

  }
  std_msgs::msg::Int16 valid_bump_msg;
  valid_bump_msg.data = m_valid_bump;
  valid_bump_pub_->publish(valid_bump_msg);
}

void NavStates::mapToOdomUpdateCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "mapToOdomUpdate %d", msg->data);
  m_odom_goal_refresh_needed = true;
  camera_cone_pose_in_map.header.stamp = now();
}

void NavStates::pfWayPointUpdateCallback(const std_msgs::msg::Int16::SharedPtr msg)
{
  RCLCPP_DEBUG(get_logger(), "pfWayPointUpdate %d", msg->data);
  if(m_init_wp_published)
    m_update_pf_waypoint = true;
}

double NavStates::get_plan_rate()
{
  return params.plan_rate;
}

void NavStates::odomCallback(const nav_msgs::msg::Odometry::SharedPtr odom)
{
  bot_pose.header.stamp = odom->header.stamp;
  bot_pose.pose.position = odom->pose.pose.position;
  bot_pose.pose.orientation = odom->pose.pose.orientation;
  bot_yaw = tf2::getYaw(bot_pose.pose.orientation); //get_yaw(bot_pose.pose);
  m_odom_received = true;
}

void NavStates::clickedGoalCallback(const geometry_msgs::msg::PoseStamped::SharedPtr data)
{
  geometry_msgs::msg::PoseStamped odomPose;
  odomPose.header.frame_id = "odom";
  if(getPoseInFrame(*data,"odom",odomPose))
  {
    odom_goal_pose = odomPose;
    wp_goal_pub_->publish(odom_goal_pose);
    RCLCPP_INFO(get_logger(), "Updated wp_goal via clicked point");
  }
}

void NavStates::scanCallback(const sensor_msgs::msg::LaserScan::SharedPtr scan) //use a point cloud instead, use laser2pc.launch
{
  m_collision = false;
  unsigned close_count = 0;

  // Declare "collision" if multiple lasers see an obstacle within close range
  // TODO: Also calculate a close_obs_range to control m_speed in commandTo instead of using distance_between_poses
  //       Set a boolean here, m_close_to_obs = close_obs_range < params.slow_approach_distance
  double close_obs_range = DBL_MAX;
  for (unsigned i = 0; i < scan->ranges.size();i++)
  {
    float range = scan->ranges[i];
    //float angle  = scan->angle_min +(float(i) * scan->angle_increment);
    if(range < params.scan_collision_range)
    {
      ++close_count;
    }
    if(range < close_obs_range)
    {
      close_obs_range = range;
    }
  }

  m_close_to_obs = close_obs_range < params.slow_approach_distance;

  if(close_count > 0) // TODO: Parameter, how many lasers in one scan need to see an obstacle
  {
    ++m_scan_collision_db_count;
    if(m_scan_collision_db_count >= params.scan_collision_db_limit)
    {
      m_scan_collision_db_count = params.scan_collision_db_limit;
      m_collision = true;
    }
  }
  else
  {
    m_scan_collision_db_count = 0;
  }
}

bool NavStates::nearPathPoint()
{
  return distance_between_poses(bot_pose, m_path.poses[m_index_path]) < params.near_path_dist;
}

//getTargetHeading assumes goal is in odom frame
double NavStates::getTargetHeading(const geometry_msgs::msg::PoseStamped& goal)
{
  return atan2(goal.pose.position.y - bot_pose.pose.position.y, goal.pose.position.x - bot_pose.pose.position.x);
}

void NavStates::commandTo(const geometry_msgs::msg::PoseStamped& goal)
{
  m_speed = params.desired_speed;
  // use m_close_to_obs from scanCallback
  if(m_state == STATE_TOUCH_TARGET &&
      ((distance_between_poses(bot_pose, odom_goal_pose) < params.slow_approach_distance) || m_close_to_obs) )
  {
    m_speed = params.slow_speed;
  }
  double des_yaw = getTargetHeading(goal);
  double heading_error = des_yaw - bot_yaw;

  if(heading_error >= M_PI)
  {
    heading_error -= 2*M_PI;
  }
  else if(heading_error < -M_PI)
  {
    heading_error += 2*M_PI;
  }

  double delta_heading_error = heading_error - m_prev_heading_error;
  if(delta_heading_error >= M_PI)
  {
	delta_heading_error -= 2*M_PI;
  }
  else if(delta_heading_error < -M_PI)
  {
	delta_heading_error += 2*M_PI;
  }

  m_prev_heading_error = heading_error;

  if(params.cmd_control_ver == 0)
  {
    // ********** FROM DiffDriveController ********
    double ka= 2.0;
    double ka_d = 2.0;
    double kb= 0.001;
    m_omega = ka*heading_error + kb*des_yaw + ka_d * delta_heading_error;
    // **********************************************
  }
  else
  {
    m_omega = 0.5*(heading_error); // TODO: parameter
    if(fabs(m_omega) > 1.0) // slow for tight turns
    {
      m_omega = 1.0*m_omega/fabs(m_omega);
      m_speed = params.slow_speed;
    }
  }

  if(fabs(heading_error) > params.max_fwd_heading_error_deg*M_PI/180)
  {
    m_speed = -params.reverse_speed; //-m_speed
  }

  //RCLCPP_INFO(get_logger(), "des_yaw, bot_yaw, omega: %0.1f, %0.1f, %0.1f",des_yaw*180/M_PI, bot_yaw*180/M_PI, m_omega);
}

void NavStates::state_init()
{
  state_start_time = now();
}
double NavStates::get_time_in_state()
{
  return (now() - state_start_time).seconds();
}

void NavStates::track_mow_path()
{
  //RCLCPP_INFO(get_logger(), "TRACK MOW PATH");
  if(!m_init_wp_published)
  {
    update_waypoint();
  }

  // Next waypoint
  if(m_update_pf_waypoint)
  {
    RCLCPP_INFO(get_logger(), "UPDATE PF WAYPOINT");
    m_update_pf_waypoint = false;
    m_init_wp_published = false;
    update_waypoint();
  }
}

void NavStates::track_path()
{
  if(!m_init_wp_published)
  {
    update_waypoint();
  }

  if(!(m_odom_received && m_path_received) || m_path.poses.size() == 0)
  {
	//RCLCPP_INFO(get_logger(), "odom_received, path_received, path size: %d, %d, %d",(int)m_odom_received, (int)m_path_received, (int)m_path.poses.size());
    //m_state = STATE_SEARCH_IN_PLACE;
    return;
  }

  if(m_valid_bump || m_collision)
  {
    m_state = STATE_RETREAT;
    m_speed = 0.0;
    m_omega = 0.0;
    return;
  }

  if(m_cone_detected && m_current_waypoint_type == WP_TYPE_CONE) //TODO: Verify we should ignore m_cone_detected for intermediate waypoints
  {
    m_state = STATE_TURN_TO_TARGET;
    return;
  }

  // end of path?, See pathCallback
  // Verify m_path_received does not become true between update_waypoint() setting m_init_wp_published true and Astar publishing a new path to the old waypoint
  if(m_path_received && m_index_path == m_path.poses.size()-1)
  {
    if(m_current_waypoint_type == WP_TYPE_CONE)
    {
      m_state = STATE_SEARCH_IN_PLACE;
      return;
    }
    else if(m_init_wp_published)
    {
      m_init_wp_published = false;
      update_waypoint();
    }
  }
  //check if close to next path point and update
  if(nearPathPoint())
  {
    m_index_path += params.path_step_size; //TODO: PARAMETER
    if(m_index_path >= m_path.poses.size())
    {
      m_index_path = m_path.poses.size()-1;
    }
  }
  geometry_msgs::msg::PoseStamped path_pose = m_path.poses[m_index_path];
  commandTo(path_pose);
}
void NavStates::retreat()
{
  m_speed = -params.reverse_speed;
  m_omega = 0.0; //eventually retreat with m_omega = omega from path control
  if(get_time_in_state() > params.reverse_time)
  {
    m_speed = 0.0;
    m_state = STATE_TRACK_PATH;
  }
}
void NavStates::search_in_place()
{
  if(m_first_search)
  {
    m_init_search_time = now();
    m_first_search = false;
  }

  m_speed = params.slow_speed;
  m_omega = params.search_omega;
  if(m_cone_detected)
  {
    m_state = STATE_TOUCH_TARGET;
    update_target();
  }
  // If still at end of path, it will come right back here from STATE_TRACK_PATH
  if(get_time_in_state() > params.search_time || m_collision)
  {
    m_state = STATE_TRACK_PATH;
    if( (now()-m_init_search_time).seconds() > params.max_camera_search_time)
    {
      if(m_obs_cone_received)
      {
        camera_cone_pose = obs_cone_pose;
      }
      else
      {
        camera_cone_pose = odom_goal_pose;
      }
      m_cone_detected = true; //could also not set this to still require camera cone detection
      update_target();
      //OR just update the goal for Astar to be the obs_cone_pose
      // update_target(obs_cone_pose); //still requires the camera to find the cone, but the path will be updated to go to obs_cone_pose
    }
  }
}
void NavStates::turn_to_target()
{
  double des_yaw = getTargetHeading(camera_cone_pose);
  m_speed = params.slow_speed;
  m_omega = 1.0*(des_yaw - bot_yaw);

  if(fabs(m_omega) > params.max_omega) // TODO: PARAMETER
    m_omega = params.max_omega*m_omega/fabs(m_omega); //TODO: PARAMETER
  if(fabs(m_omega) < params.search_omega)
    m_omega = params.search_omega*m_omega/fabs(m_omega); //TODO: PARAMETER
  if(m_cone_detected)
  {
    m_state = STATE_TOUCH_TARGET;
    update_target();
    return;
  }
  if(fabs(des_yaw - bot_yaw) < 5*M_PI/180 || get_time_in_state() > 3.0) //TODO: PARAMETER
  {
    m_state = STATE_TRACK_PATH;
  }
}
void NavStates::touch_target()
{
  commandTo(camera_cone_pose);
  if(m_valid_bump || m_collision)
  {
    // this causes update_waypoint() to be called again until it is successful
    m_init_wp_published = false;
    m_cone_detected = false;
    m_cone_detect_db_count = 0;

    update_waypoint();
    m_speed = 0.0;
    m_omega = 0.0;
    m_state = STATE_RETREAT_FROM_CONE;
    // Known obstacle pose should be published in the odom frame for AvoidObs costmap
    known_obs_pub_->publish(bot_pose);
  }
}
void NavStates::retreat_from_cone()
{
  // NOTE, this state should block target updates
  //   Do not allow the just touched cone to revert the updated waypoint

  if(!m_init_wp_published)
  {
    update_waypoint();
  }

  m_speed = -params.reverse_speed;
  m_omega = 0.0;
  if(get_time_in_state() > params.reverse_time)
  {
    m_speed = 0.0;
    m_state = STATE_TRACK_PATH;
  }
}
void NavStates::update_target()
{
  wp_goal_pub_->publish(camera_cone_pose);
}

void NavStates::update_target(geometry_msgs::msg::PoseStamped target_pose)
{
  wp_goal_pub_->publish(target_pose);
}

void NavStates::mow_area_callback(const my_interfaces::srv::SetInt::Request::SharedPtr request,
    const my_interfaces::srv::SetInt::Response::SharedPtr response)
{
  response->success = true;
  if(0 <= request->data && request->data <= 3)
  {
    coords_index = request->data;
    update_mow_path();
    update_waypoint();
  }
  else
  {
    response->success = false;
  }
}

void NavStates::update_states()
{
  int prev_state = m_state;
  switch(m_state) {
  case STATE_TRACK_MOW_PATH:
    track_mow_path();
    break;
  case STATE_TRACK_PATH:
    track_path();
    break;
  case STATE_TURN_TO_TARGET:
    turn_to_target();
    break;
  case STATE_TOUCH_TARGET:
    touch_target();
    break;
  case STATE_SEARCH_IN_PLACE:
    search_in_place();
    break;
  case STATE_RETREAT:
    retreat();
    break;
  case STATE_RETREAT_FROM_CONE:
    retreat_from_cone();
    break;
  default: //optional
  RCLCPP_INFO(get_logger(), "Nav State not defined");
  }

  if(prev_state != m_state)
  {
    state_init();
  }

  if(!(m_odom_received && m_path_received) )
  {
    m_speed = 0.0;
    m_omega = 0.0;
  }

  if(fabs(m_omega) > params.max_omega)
  {
    m_omega = params.max_omega*m_omega/fabs(m_omega);
    m_speed = params.slow_speed*m_speed/fabs(m_speed);
  }

  if(m_speed <= 0.0 && m_valid_bump)
  {
    m_filt_speed = m_speed;
  }
  else
  {
    double alpha = params.cmd_speed_filter_factor;
    m_filt_speed = m_filt_speed*alpha + m_speed*(1.0-alpha);
  }

  if(fabs(m_filt_speed) > params.desired_speed)
  {
    m_filt_speed = 0.0;
  }
  if(fabs(m_omega) > params.max_omega)
  {
    m_omega = 0.0;
  }

  geometry_msgs::msg::Twist cmd;
  cmd.linear.x = m_filt_speed;
  cmd.angular.z = m_omega;
  cmd_pub_->publish(cmd);

  m_nav_state_msg.data = m_state;
  nav_state_pub_->publish(m_nav_state_msg);

  m_hill_wp_msg.data = m_current_hill_type;
  hill_wp_pub_->publish(m_hill_wp_msg);

  if(m_odom_goal_refresh_needed)
  {
    refresh_odom_goal();
  }
}

int main(int argc, char **argv)
{
  //Initiate ROS
  rclcpp::init(argc, argv);
  rclcpp::NodeOptions node_options;
  auto nav_states_node = std::make_shared<NavStates>(node_options);

  RCLCPP_INFO(nav_states_node->get_logger(), "Starting Navigation State Manager");
  int loop_hz = nav_states_node->get_plan_rate();
  rclcpp::Rate rate(loop_hz);

  //rclcpp::spin(nav_states_node);
  while(rclcpp::ok())
  {
	rclcpp::spin_some(nav_states_node);
	nav_states_node->update_states();
    rate.sleep();
  }

  return 0;
}
