# ForzaETH Race Stack Installation
After following the [prerequisites](#prerequisites) you will need to choose either [Car installation](#car-installation) or [Sim installation](#sim-installation). 
## Prerequisites 
To run the ForzaETH race stack make sure that you have assembled and setup a car following the [official F1TENTH instructions](https://f1tenth.org/build). Follow the instructions up to the point *Install F1TENTH Driver Stack » 3. F1TENTH Driver Stack Setup » 1. udev Rules Setup*.

For both the car and the sim installation, be sure to have installed:
  - [ROS Noetic](http://wiki.ros.org/noetic/Installation/Ubuntu)
  - [Catkin Build Tools](https://catkin-tools.readthedocs.io/en/latest/installing.html)

**Note**: 
The two installations are not needed in the case of the [Docker](#docker) setup.

**Note**:
Be sure to have included the sourcing lines in your `~/.bashrc` file, in order to properly setup ROS in every terminal you open. 
The two lines to be added are (if you are using `bash`)
```
source /opt/ros/noetic/setup.bash
source <path to your catkin_ws>/devel/setup.bash
```

## Native installation
The following steps assume you have a catkin workspace set up and are working from within the ```src``` folder. For example:
```
cd ~/catkin_ws/src
```
### Car installation
#### [Step 1 of 5] 
First you'll have to clone the repository with all the submodules in it **(in case you haven't done so from above)**. 
```
git clone --recurse-submodules git@github.com:ForzaETH/race_stack.git 
cd race_stack
```
From now on the installation assumes you are in the `race_stack` folder.

#### [Step 2 of 5]
Install all the dependencies
```
# ubuntu packages dependencies
xargs sudo apt-get install -y < ./.install_utils/linux_req_car.txt

# python dependencies
pip install -r ./.devcontainer/.install_utils/requirements.txt
pip install ~/catkin_ws/src/race_stack/f110_utils/libs/ccma
pip install -e ~/catkin_ws/src/race_stack/planner/graph_based_planner/src/GraphBasedPlanner
```


#### [Step 3 of 5]
Then you will need to install the cartographer version which we modified (and which was just downloaded as submodule)
```
chmod +x ./.devcontainer/.install_utils/cartographer_setup.sh
sudo ./.devcontainer/.install_utils/cartographer_setup.sh
```
#### [Step 4 of 5]
Similarly you will need to install the SynPF particle filter (which was just downloaded as submodule)
```
chmod +x ./.devcontainer/.install_utils/pf_setup.sh
sudo ./.devcontainer/.install_utils/pf_setup.sh
```
####  [Step 5 of 5]

**Optional**: if you want you can erase the simulator folder, so to not install it on the car too. 
It can be done with the following command 
```
# Optional
rm -rf ./base_system/f110-simulator
``` 

Then build the whole system!
```
catkin build
```

The car is now ready to be tested. For examples on how to run the different modules on the car, refer to the [`stack_master` README](./stack_master/README.md). As a further example, the [time-trials](./stack_master/checklists/TimeTrials.md) or the [head-to-head](./stack_master/checklists/HeadToHead.md) checklists are a good starting point.

### Sim installation
#### [Step 1 of 2]
First you'll have to clone the repository **(in case you haven't done so from above)**.
```
git clone --recurse-submodules git@github.com:ForzaETH/race_stack.git 
cd race_stack
```
From now on the installation assumes you are in the `race_stack` folder.

#### [Step 2 of 3]
Install all the dependencies
```
# ubuntu packages dependencies
xargs sudo apt-get install -y < ./.devcontainer/.install_utils/linux_req_sim.txt

# python dependencies
pip install -r ./.devcontainer/.install_utils/requirements.txt
pip install ~/catkin_ws/src/race_stack/f110_utils/libs/ccma
pip install -e ~/catkin_ws/src/race_stack/planner/graph_based_planner/src/GraphBasedPlanner
```
#### [Step 3 of 3]
**Optional**: if you want you can erase the simulator folder, so to not install it on the car too. 
It can be done with the following command 
```
# Optional
rm -rf ./base_system/f1tenth_system
``` 

Then build the whole system!
```
catkin build
```

After installation you can test simulation functionalities, for example running the [time-trials](./stack_master/checklists/TimeTrials.md) or the [head-to-head](./stack_master/checklists/HeadToHead.md) checklists. For a more general overview of how to use the `race_stack` refer to the [`stack_master` README](./stack_master/README.md).

```
Calibration of use_imu_data and use_odometry (Manual, No Bag Files)

Goal: Calibrate odometry and IMU parameters using simple tools and control inputs—no rosbag or playback required.
Tools Required:

Tape measure (≥ 5 meters)

Flat, marked test course

Manual control over throttle and steering (e.g., joystick or keyboard)

Step 1: Straight Line Test for Odometry (Distance-Based)

Mark a start and end point 5.0 m apart.

Drive robot straight and observe:

Commanded ERPM (or throttle value)

Actual traveled distance

If odometry under-reports (e.g., shows 4.0 m):

Multiply speed_to_erpm_gain by (5.0 / 4.0) = 1.25

If time is known, use:

speed_to_erpm_gain = ERPM * time / distance

Step 2: Circle Test for Steering Calibration

Command a fixed steering value (e.g., ±0.2 rad)

Drive a full circle slowly, measure turning radius R

Use known wheelbase L to estimate angle:

angle_rad = arctan(L / R)

Then solve:

steering_to_servo_gain = commanded_value / angle_rad

Update the gain until your robot's turning radius matches expected radius.

Step 3: IMU Rotation Check

With robot stationary, rotate it 90° by hand

Observe change in yaw from /imu/data

Should be ≈ 1.57 rad

If not:

Check IMU mounting direction

Check for duplicate IMU publishers

Ensure TF from base_link → imu is correct

Step 4: Loop and Refine

Repeat above steps to fine-tune each gain

Confirm /odom tracks real-world distances and /imu/data reflects orientation changes

Tips:

rostopic echo /odom to verify position

rostopic hz /odom to ensure steady publishing

rosrun tf tf_echo base_link odom to observe pose change over time

```
Cartographer needs adjusted for low speed RPLIDAR sensored vesc and wimoton IMU
Key Lua Parameters Overview (Code Block Format)   for f110_2d.lua, and f110_2d_loc.lua in racecar config (NUC2) 

Parameter                                 | Typical Range | Impact / When to Tune                                 | Increment | Calibration Needed
---------------------------------------- | --------------|------------------------------------------------------|-----------|-------------------
min_range                                | 0.05 – 0.2 m   | Filters close returns (robot body, bumper)           | ±0.05 m   | No
max_range                                | 8 – 25 m       | Filters noisy returns beyond LiDAR max range         | ±1.0 m    | No
missing_data_ray_length                  | 1.0 – 5.0 m    | Length assumed for missing LiDAR returns             | ±1.0 m    | No
use_imu_data                             | true / false   | Enables IMU orientation data                         | -         | Yes
use_odometry                             | true / false   | Enables VESC or encoder odometry                     | -         | Yes
num_accumulated_range_data               | 1 – 10         | Higher = smoother scan fusion                        | ±1        | No
motion_filter.max_angle_radians          | 0.05 – 0.5 rad | Suppresses small angular updates                     | ±0.05 rad | No
motion_filter.max_distance_meters        | 0.1 – 1.0 m    | Suppresses minor linear updates                      | ±0.1 m    | No
motion_filter.max_time_seconds           | 0.1 – 1.0 s    | Minimum time before update is triggered              | ±0.1 s    | No
ceres_scan_matcher.translation_weight    | 1 – 100        | Trust in odom position during scan match             | ±5        | No
ceres_scan_matcher.rotation_weight       | 1 – 100        | Trust in IMU/odom heading during scan match          | ±5        | No
submaps.num_range_data                   | 20 – 100       | Size of submap before finalization                   | ±10       | No
min_score                                | 0.4 – 0.8      | Minimum scan match score to accept                   | ±0.05     | No
global_localization_min_score            | 0.4 – 0.7      | Score threshold for global re-localization           | ±0.05     | No
optimize_every_n_nodes                   | 20 – 100       | How often pose graph is optimized                    | ±5        | No
constraint_builder.min_score             | 0.4 – 0.8      | Threshold for loop closure constraint acceptance     | ±0.05     | No
constraint_builder.max_constraint_distance| 5 – 15 m      | Max distance to attempt constraint match             | ±1.0 m    | No
optimization_problem.huber_scale         | 1 – 10         | Outlier rejection strength in optimization           | ±1.0      | No




