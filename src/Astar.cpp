// Copyright 2019 coderkarl. Subject to the BSD license.

#include "avoid_obstacles/Astar.h"
#include "avoid_obstacles/AvoidObsCommon.h"

#include <geometry_msgs/PoseStamped.h>
#include <tf/transform_datatypes.h>
#include <cstdlib>
#include <cstdio>
#include <cmath>
#include <boost/math/special_functions/round.hpp>
#include <algorithm>

bool compareCells (const Astar::Cell& cellA, const Astar::Cell& cellB)
{
  //return cellB.f > cellA.f? 1 : -1;
  return cellB.f > cellA.f; //sort vector small f (at 0) to large f (at end)
}

Astar::Astar():
		map_x0(0.0),
		map_y0(0.0),
		map_res(0.5),
		NUM_ROWS(0),
		NUM_COLS(0),
		m_map()
{
    path_pub_ = nh_.advertise<nav_msgs::Path>("path", 1);
    odom_sub_ = nh_.subscribe("odom", 1, &Astar::odomCallback, this);
    goal_sub_ = nh_.subscribe("wp_goal", 1, &Astar::goalCallback, this);
    costmap_sub_ = nh_.subscribe("costmap", 1, &Astar::costmapCallback, this);
    nh_p  = ros::NodeHandle("~");
    nh_p.param("obs_thresh", obs_thresh, 50);
    nh_p.param("obs_weight", obs_weight, 0.1);
    nh_p.param("plan_rate_hz", plan_rate_, 1.0);
    nh_p.param("max_plan_time_sec", max_plan_time_, 10.0);

    path.header.stamp = ros::Time::now();
    path.header.frame_id = "odom";
    //path.poses.push_back(geometry_msgs::PoseStamped)

    bot_pose.orientation.w = 1.0;

	std::cout << "Astar initialized\n";
}

Astar::~Astar(){}

double Astar::get_plan_rate()
{
    return plan_rate_;
}

void Astar::odomCallback(const nav_msgs::Odometry& odom)
{
    bot_pose.position = odom.pose.pose.position;
    bot_pose.orientation = odom.pose.pose.orientation;
}

void Astar::goalCallback(const geometry_msgs::PoseStamped& data)
{
    goal_pose = data.pose;
}

void Astar::costmapCallback(const nav_msgs::OccupancyGrid& map)
{
  m_map = map;
}

void Astar::findPath()
{
  float dx = goal_pose.position.x - bot_pose.position.x;
  float dy = goal_pose.position.y - bot_pose.position.y;
  float goal_dist_sqd = dx*dx + dy*dy;
  if(goal_dist_sqd > 0.25)
  {
      path.header.stamp = ros::Time::now();
      if(get_path(bot_pose, goal_pose, m_map, path))
          path_pub_.publish(path);
      double duration = (ros::Time::now()-path.header.stamp).toSec();
      if(duration > 0.05)
          ROS_WARN("Astar took %0.2f sec",duration);
  }
}

bool Astar::get_map_indices(float x, float y, int& ix, int& iy)
{
	ix = boost::math::iround((x-map_x0)/map_res - map_res);
	iy = boost::math::iround((y-map_y0)/map_res - map_res);
	return true;
}

int Astar::is_obs(nav_msgs::OccupancyGrid map, int ix, int iy)
{
	int ind = iy*NUM_COLS + ix;
	if(ind > map.data.size())
		return 100;
	/*if(map.data[iy*NUM_COLS + ix] > obs_thresh)
	{
		return 1;
	}
	return 0;*/
	return map.data[ind];
}
int Astar::is_obs2(nav_msgs::OccupancyGrid map, int ix, int iy)
{
	if(is_obs(map, ix, iy) == 1)
	{
		return 1;
	}
	else // check if two adjacent cells are obstacles
	{
		int count = 0;
		for(int dx = -1; dx <= 1; ++dx)
		{
			for(int dy = -1; dy <= 1; ++dy)
			{
				if(is_obs(map,ix+dx, iy+dy))
				{
					++count;
					if(count == 2)
						return 1;
				}
			}
		}
	}
	return 0;
}
bool Astar::get_path(geometry_msgs::Pose pose, geometry_msgs::Pose goal,
						nav_msgs::OccupancyGrid map, nav_msgs::Path& path)
{
	ros::Time start_time = ros::Time::now();
	ros::Duration MAX_PLAN_TIME(max_plan_time_);

	std::vector<Cell> open;
	std::vector<Cell>::iterator it_insert;
	//it_insert = std::upper_bound (v.begin(), v.end(), new_cell, compareCells);
	//open.insert(it_insert, new_cell)

	/*open.push_back(new_cell(0,1,0,1,0,0));
	open.push_back(new_cell(1,2,0,2,0,0));
	open.push_back(new_cell(0.5,3,3,0,0,0));
	open.push_back(new_cell(2.0,4,4,0,0,0));
	open.push_back(new_cell(1.5,5,5,0,0,0));

	ROS_INFO("Get path test");
	printf("before sort\n");
	for(unsigned k=0; k<open.size(); ++k)
		printf("%f, %f, %f, %f\n", open[k].f, open[k].g, open[k].x, open[k].y);
	std::sort(open.begin(), open.end(), compareCells);
	Cell cc = new_cell(1.1,6,6,0,0,0);
	it_insert = std::upper_bound(open.begin(),open.end(),cc,compareCells);
	open.insert(it_insert,cc);
	printf("after sort and insert\n");
	for(unsigned k=0; k<open.size(); ++k)
		printf("%f, %f, %f, %f\n", open[k].f, open[k].g, open[k].x, open[k].y);
	open.clear();*/

	map_res = map.info.resolution;
	if(map_res == 0)
	    return false;
	map_x0 = map.info.origin.position.x;
	map_y0 = map.info.origin.position.y;
	NUM_ROWS = map.info.height;
	NUM_COLS = map.info.width;

	int finished[NUM_ROWS][NUM_COLS] = {{0}};
	int action[NUM_ROWS][NUM_COLS] = {{0}};
	float open_g[NUM_ROWS][NUM_COLS] = {{0}};
	//std::vector<int> finished;
	//std::vector<int> action;

	float x_init = pose.position.x;
	float y_init = pose.position.y;

	float xg = goal.position.x;
	float yg = goal.position.y;

	int rg; // = boost::math::iround(-yg);
	int cg; // = boost::math::iround(xg);
	get_map_indices(xg, yg, cg, rg);

	// Do not try to plan to obstacle goal
	// TODO: Move goal until not an obstacle
	//    Another option: move within a large radius of the goal (until c2,r2 is near cg,rg)
	if(is_obs2(map,cg,rg))
	{
	  // step toward bot in 0.5 meter increments until no obstacle goal
	  bool goal_is_obs = true;
	  double dx = x_init - xg;
	  double dy = y_init - yg;
	  double range = sqrt(dx*dx+dy*dy);
	  double step_x = dx/range*0.5;
	  double step_y = dy/range*0.5;
	  for(int nSteps=1;nSteps<=5;++nSteps)
	  {
	    get_map_indices(xg+step_x*nSteps, yg+step_y*nSteps, cg, rg);
	    if(!is_obs2(map,cg,rg))
	    {
	      goal_is_obs = false;
	      break;
	    }
	  }
	  if(goal_is_obs) // step away from bot if still no clear cell found
	  {
	    for(int nSteps=1;nSteps<=5;++nSteps)
	      {
	        get_map_indices(xg-step_x*nSteps, yg-step_y*nSteps, cg, rg);
	        if(!is_obs2(map,cg,rg))
	        {
	          goal_is_obs = false;
	          break;
	        }
	      }
	  }
	  if(goal_is_obs)
	  {
      ROS_WARN("Astar won't plan to obstacle goal");
      return false;
	  }
	}

	float x1,y1,g1,f2,g2,h2,DIST,x2,y2,cost;
	float next_pos[2] = {0,0};
	int r1,c1, nOpen, dCount, m, r2,c2, r_init, c_init, count;
	int numMotions = 8, rev_motion;
	int motions[8] = {1,-1,2,-2,3,-3,4,-4}; //simp_move
	int done, no_sol, a;
	x1 = x_init;
	y1 = y_init;
	//r1 = boost::math::iround(-y_init);
	//c1 = boost::math::iround(x_init);
	get_map_indices(x_init, y_init, c1, r1);

	finished[r1][c1] = 1;
	g1 = 0;
	//h2 = sqrt((xg-x1)^2 + (yg-y1)^2);
	//f2 = g1+h2;
	nOpen = 0;
	done = 0;
	no_sol = 0;
	DIST = map_res;

	//printf("Begin while\n");
	dCount = 0;
	while(!done && !no_sol)
	{
		//md(5);
		for(m = 0; m<numMotions; m++)
		{
			cost = simp_move(next_pos,x1,y1,motions[m],DIST);
			if(next_pos[0] == 32767)
			{
				printf("Invalid motion in search move\n");
				return 1;
			}
			x2 = next_pos[0];
			y2 = next_pos[1];
			//r2 = boost::math::iround(-y2);
			//c2 = boost::math::iround(x2);
			get_map_indices(x2, y2, c2, r2);

            if (r2 >= 0 && r2 < NUM_ROWS && c2 >= 0 && c2 < NUM_COLS)
            {
                int obs_cost = is_obs(map, c2, r2);
                if (obs_cost < obs_thresh && finished[r2][c2] == 0)
                {
                    g2 = g1 + cost + float(obs_cost) * obs_weight;
                    if (open_g[r2][c2] == 0 or g2 < open_g[r2][c2])
                    {
                        open_g[r2][c2] = g2;

                        h2 = sqrt((xg-x2)*(xg-x2) + (yg-y2)*(yg-y2));
                        //h2 = (xg-x2)*(xg-x2) + (yg-y2)*(yg-y2);
                        f2 = g2 + h2;

                        nOpen += 1;
                        if(nOpen == 1)
                        {
                            open.push_back(new_cell(f2,g2,x2,y2,c2,r2));
                        }
                        else
                        {
                            Cell cc = new_cell(f2,g2,x2,y2,c2,r2);
                            it_insert = std::upper_bound(open.begin(),open.end(),cc,compareCells);
                            open.insert(it_insert,cc);
                        }

                        //finished[r2][c2] = 1;
                        action[r2][c2] = motions[m];
                    }
                }
            }
        }
		if(nOpen == 0)
		{
			no_sol = 1;
			printf("No Solution found\n");
		}
		else if( (ros::Time::now()-start_time)> MAX_PLAN_TIME )
		{
			no_sol = 1;
			printf("Timed Out, No Solution\n");
		}
		else
		{
			g1 = open[0].g;
			x1 = open[0].x;
			y1 = open[0].y;
			c1 = open[0].c;
			r1 = open[0].r;
			open.erase(open.begin());
			finished[r1][c1] = 1;
			dCount = dCount + 1;

			nOpen -= 1;
			if(nOpen < 0)
			{
				nOpen = 0;
			}
			if(r1 == rg && c1 == cg)
			{
				done = 1;
				//printf("done\n");
			}
		}
	}

	if(done)
	{
		get_map_indices(x_init,y_init,c_init,r_init);

		//Step backwards and store the optimum path
		//printf("x: %.1f, y: %.1f\n",x1,y1);
		//fprintf(fpPath,"%.1f %.1f\n",x1,y1);
		path.poses.clear();
		geometry_msgs::PoseStamped wp;
		wp.pose.position.x = x1;
		wp.pose.position.y = y1;
		path.poses.push_back(wp);

		while( (abs(r1-r_init) > 0 || abs(c1-c_init) > 0) && dCount >= -2)
		{
			//printf("action: %d\n",action[r1][c1][p1]);
			rev_motion = -action[r1][c1];
			cost = simp_move(next_pos,x1,y1,rev_motion,DIST);
			if(next_pos[0] == 32767)
			{
				printf("Invalid motion in move\n");
				return 1;
			}
			x1 = next_pos[0];
			y1 = next_pos[1];
			//r1 = boost::math::iround(-y1);
			//c1 = boost::math::iround(x1);
			get_map_indices(x1, y1, c1, r1);
			//count = count+1;
			//x(count) = x1;
			//y(count) = y1;
			//printf("x: %.1f, y: %.1f\n",x1,y1);
			//fprintf(fpPath,"%.1f %.1f\n",x1,y1);
			wp.pose.position.x = x1;
			wp.pose.position.y = y1;
			path.poses.push_back(wp);
			dCount = dCount - 1;
		}
		std::reverse(path.poses.begin(),path.poses.end());
		//plot(x,y,'g')
	}
	if(nOpen > 500)
	    ROS_INFO("nOpen = %d",nOpen);

	/*
	path.poses.clear();
	geometry_msgs::PoseStamped wp;
	wp.pose.position.x = 5.0;
	wp.pose.position.y = 5.0;
	path.poses.push_back(wp);
	wp.pose.position.x = 15.0;
	path.poses.push_back(wp);
	*/
	if(no_sol)
	{
	  ROS_WARN("Astar no solution");
	  return false;
	}
	return true;
}

Astar::Cell Astar::new_cell(float f, float g, float x, float y, int c, int r)
{
	Cell ce = {f,g,x,y,c,r};
	return ce;
}

float Astar::simp_move(float next_pos[], float x1, float y1, int motion, float d)
{
	float cost;
	#define up 1
	#define down -1
	#define left 2
	#define right -2
	#define upLeft 3
	#define downRight -3
	#define upRight 4
	#define downLeft -4

	if(abs(motion) <= 2)
		cost = d;
	else
		cost = 1.42*d;

	float x2,y2;

	switch (motion)
	{
		case up:
			x2 = x1;
			y2 = y1 + d;
			break;
		case down:
			x2 = x1;
			y2 = y1 - d;
			break;
		case left:
			x2 = x1-d;
			y2 = y1;
			break;
		case right:
			x2 = x1+d;
			y2 = y1;
			break;
		case upLeft:
			x2 = x1 - d;
			y2 = y1 + d;
			break;
		case downRight:
			x2 = x1 + d;
			y2 = y1 - d;
			break;
		case upRight:
			x2 = x1+d;
			y2 = y1+d;
			break;
		case downLeft:
			x2 = x1-d;
			y2 = y1-d;
			break;
		default:
			//printf("invalid motion value\n");
			x2 = 32767;
			y2 = 32767;
	}
	next_pos[0] = x2;
	next_pos[1] = y2;

	return cost;
}

int main(int argc, char **argv)
{
    //Initiate ROS
    ros::init(argc, argv, "astar");

    Astar astar;
    ROS_INFO("Starting A-star path planning");
    //ros::spin();
    double loop_hz = astar.get_plan_rate();
    ros::Rate rate(loop_hz);

    while(ros::ok())
    {
        astar.findPath();
        ros::spinOnce();
        rate.sleep();
    }

    return 0;
}
