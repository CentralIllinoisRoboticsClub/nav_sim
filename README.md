# Simple ROS simulation for path planning with lidar
clone or download zip to catkin_ws/src/
`git clone https://github.com/CentralIllinoisRoboticsClub/nav_sim.git`

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
