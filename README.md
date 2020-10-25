# Simple ROS2 simulation for path planning with lidar
clone or download zip to ros2_ws/src/
`git clone --branch ros2 --single-branch https://github.com/CentralIllinoisRoboticsClub/nav_sim.git`

light_scan_sim for ros2 is at  
`git clone --branch ros2 --single-branch https://github.com/CentralIllinoisRoboticsClub/light_scan_sim.git`  

# ROS2 WIP status
See nav_sim/launch/nav_sim2.launch.py  
This tests sim_bot.py, the static map, nav_sim_ros2.rviz, avoid_obs, astar, and light_scan_sim  

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

## ROS2 dependencies
```
sudo apt install ros-foxy-desktop
sudo apt install ros-foxy-nav2-core
sudo apt install ros-foxy-tf2
sudo apt install ros-foxy-tf2-ros
sudo apt install ros-foxy-nav2-amcl
sudo apt install ros-foxy-nav2-map-server 

# add to ~/.bashrc
source /opt/ros/foxy/setup.bash
source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash
source /home/$USER/ros2_ws/install/setup.bash
```

## py and cpp in one package
[https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/](https://roboticsbackend.com/ros2-package-for-both-python-and-cpp-nodes/)

If the nav_sim2.launch.py says no executable sim_bot.py:  
At the end of nav_sim/launch/nav_sim2.launch.py, comment:  
`#ld.add_action(sim_bot_node)`  
Then do the following in two terminals:  
`python3 ~/ros2_ws/src/nav_sim/nav_sim/sim_bot.py`  
`ros2 launch nav_sim nav_sim2.launch.py`  

Another option is to put the sim_bot.py script in a separate nav_sim_py package and modify the nav_sim2.launch.py to call sim_bot from there.  

## map_server activate
After launching, the map_server has not yet published /map  
light_scan_sim sends a change_state request to resolve this, but it takes a long time (10 sec).  
The following is what was done before light_scan_sim was modified:  
In another terminal:  
```
ros2 lifecycle set map_server configure
ros2 lifecycle set map_server activate
```
[Reference launch file](https://github.com/ros-drivers/ros2_ouster_drivers/blob/eloquent-devel/ros2_ouster/launch/os1_launch.py)
