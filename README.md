# Simple ROS2 simulation for path planning with lidar
clone or download zip to catkin_ws/src/
`git clone --branch ros2 --single-branch https://github.com/CentralIllinoisRoboticsClub/nav_sim.git`

# ROS2 WIP status
See nav_sim/launch/nav_sim2.launch.py  
This tests sim_bot.py, the static map, nav_sim_ros2.rviz, avoid_obs, astar, and light_scan_sim  
light_scan_sim for ros2 is at  
`git clone --branch ros2 --single-branch https://github.com/CentralIllinoisRoboticsClub/light_scan_sim.git`  

avoid_obs builds the costmap using the laser scan.  
Astar receives the costmap and publishes /path for nav_states.  

colcon build --symlink-install --packages-select nav_sim  
colcon build --symlink-install --packages-select light_scan_sim  
`ros2 launch nav_sim nav_sim2.launch.py`  

Because of the map_server activate issue, wait about 10 seconds for the laser scan in rviz.  
nav_states sends an initial wp_goal of 17.0, 9.0 and the bot will move.  

You can specify a new goal at any time using rviz 2D Goal Pose at the top.  

Currently, in launch/sim_avoid_obstacles2.launch.py, avoid_obs use_PotFields = False  

If you enable use_PotFields, remap nav_states cmd_vel to ignore_vel b/c avoid_obs will use potential fields to publish cmd_vel.  

Note the potential fields puts a tangential CCW force field around obstacles.  
So sometimes the bot gets sucked into going the long way around an obstacle.  
It is difficult to know whether the tangential field should be CW or CCW.  
Maybe see which direction points toward the goal and use that direction.  
But if you are approaching a wall perpendicular to you, do you go left or right?  
We talked about using an Astar + potential fields hybrid solution to help with this.  

## Obsolete issues now resolved... sort of
After launching, the map_server has not yet published /map  
See the links commented in nav_sim2.launch.py  
In another terminal:  
```
ros2 lifecycle set map_server configure
ros2 lifecycle set map_server activate
```
[Reference launch file](https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py)

Finally in another terminal:  
`ros2 run turtlesim turtle_teleop_key --ros-arg -r turtle1/cmd_vel:=cmd_vel`

## MISC planner notes

We recently tried our own simpler local planner that just tries to follow the global path plan.
See the test_wheele_local_planner branch in this nav_sim repo.
remap cmd_vel to something else, and run the wheele_local_planner_sim.py in the updated wheele repo

## Dependencies
```
sudo apt-get install ros-kinetic-navigation
sudo apt-get install ros-kinetic-teb-local-planner
sudo apt-get install ros-kinetic-move-base
sudo apt-get install ros-kinetic-map-server
sudo apt-get install ros-kinetic-global-planner
cd ~/catkin_ws/src/
git clone https://github.com/josephduchesne/light_scan_sim.git
cd ~/catkin_ws/
catkin_make
```
Potential Future Dependencies:
```
sudo apt-get install ros-kinetic-amcl
sudo apt-get install ros-kinetic-gmapping
```
In general, if you get errors about packages and it is not one of the catkin_ws/src packages, try to install the package

## Usage
`roslaunch nav_sim nav_sim.launch`
(A roscore master will automatically start for your first launch file)
Ignore the following warning for now:
LightScanSim: Could not find a connection between 'map_image' and 'laser' because they are not part of the same tree.Tf has two or more unconnected trees.
I do not understand the purpose of map_image.

`roslaunch nav_sim move_base_nav.launch`

If ROS cannot find the .py scripts, try changing the file permissions:
`chmod +x /path/to/filename.py`

Use the 2D Nav Goal button at the top of RVIZ. Click and drag to specify goal.
Turn on/off Map, local_costmap, global_costmap as desired.
NOTE, you cannot specify a goal outside of global_costmap.
Increase global_costmap size if desired.
Try a non-static global_costmap.

## Tuning
Start with trying to understand the effects of the following files:
base_local_planner.yaml (teb_local_planner is another option to try later)
costmap_common_params.yaml
local_costmap_params.yaml
global_costmap_params.yaml

Increasing the global_costmap inflation radius gives better paths and reduces the chance of getting stuck near obstacles. I recently increased it to 0.4 meters.

These files are used by move_base_nav.launch, which can be restarted separately while leaving your simulation state as is.

If the robot gets stuck, which is likely to happen so far, restart both launch files.

## Limited turning radius model
Currently, scripts/sim_bot.py has the function sim_cmd_callback(self,data)
This function modifies the linear velocity and angular velocity commands from the navigation planning.
It ensures the maximum curvature (2.0 1/meters for wheele) is not exceeded.
However, this is one reason the sim bot gets stuck easily.
The navigation move_base node thinks the bot can spin in place.
We either need to increase the robot and/or costmap inflation radius, or find a local planner that does a better job of using reverse when needed.
The teb_local_planner (see move_base_nav.launch) can be used instead of base_local_planner.
teb_local_planner does allow reverse. It supposedly controls minimum turning radius, but I could not get it to work.
