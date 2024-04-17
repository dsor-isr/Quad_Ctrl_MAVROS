# Quad_Ctrl_MAVROS

The **Quad_Ctrl_MAVROS** package offers a testing software environment to efficiently implement trajectory tracking 
controllers for multirotors equipped with the [PX4](https://px4.io/) autopilot by leveraging the 
features of ROS and the [MAVROS](http://wiki.ros.org/mavros) package.

At the current version, the testing software environment allows to control two vehicles simultaneously,
while exchanging _messages_ between them. One may extend the functionalities of this software to
support a larger number of vehicles, however, bear in mind that further code optimisation may be required.

## Main features

* Load reference trajectories from CSV files
* Send failsafe commands from an offboard computer in emergency circumstances
* Set a flying safe zone
* Synchronise two vehicles in order to start tracking a trajectory simultaneously
* Log flight data in rosbags and make plots and animations using the MATLAB functions provided

<p align = "center">
<img src="media/relay.gif" alt="Relay Manoeuvre Arena" height="300" align="center"/>
</p>

## Citation

If you find Quad_Ctrl_MAVROS useful in your academic work, please cite the paper below.
```
Coming soon
```

## Running SITL Simulations in Gazebo Classic

* At your **ROS workspace**, clone the PX4 autopilot repository into your 'src' folder
and checkout to your desired version, preferably the latest stable release, by using the following commands:
```bash
git clone https://github.com/PX4/PX4-Autopilot.git
cd PX4-Autopilot
git checkout <tag>
git submodule update --init --recursive
```

* [Install](https://github.com/mavlink/mavros/blob/master/mavros/README.md#installation) MAVROS. Note that the default instructions are for the ROS Kinetic distribution; 

* Setup your SITL and MAVROS environment by [launching Gazebo with ROS wrappers](https://docs.px4.io/main/en/simulation/ros_interface.html#launching-gazebo-classic-with-ros-wrappers):
```bash
cd <PX4-Autopilot_clone>
DONT_RUN=1 make px4_sitl_default gazebo-classic
source ~/catkin_ws/devel/setup.bash    # (optional)
source Tools/simulation/gazebo-classic/setup_gazebo.bash $(pwd) $(pwd)/build/px4_sitl_default
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$(pwd)/Tools/simulation/gazebo-classic/sitl_gazebo-classic
roslaunch px4 mavros_posix_sitl.launch
```

* Use the provided launch files for SITL simulations to test your controller (an example of a tracking controller is in [src/main](src/main.cpp)):
```bash
# OPTION 1: Single vehicle
roslaunch quad_ctrl_mavros quad_ctrl_sitl.launch
# OPTION 2: Multiple vehicles
roslaunch quad_ctrl_mavros multi_quad_ctrl_sitl.launch
```

## Experimental Tests

This package has been successfully tested in an indoor flying arena, which features a motion capture system to obtain estimates of the position and yaw angle of the vehicles involved in the experiments.

* Use the provided launch files for real-world experiments:
```bash
# OPTION 1: Single vehicle
roslaunch quad_ctrl_mavros quad_ctrl.launch
# OPTION 2: Multiple vehicles
roslaunch quad_ctrl_mavros multi_quad_ctrl.launch
```

Change the `ID` and `fcu_url` of each vehicle according to your system.

## Licenses

Quad_Ctrl_MAVROS is released under [BSD-3 License](LICENSE).

PX4-Autopilot is available as an open-source project under [BSD-3 License](https://github.com/PX4/PX4-Autopilot).

MAVROS is available [triple-licensed](https://github.com/mavlink/mavros) as an open-source project. 

## Project Sponsors

- Dynamical Systems and Ocean Robotics (DSOR) -- a group of the Institute for Systems and Robotics (ISR). ISR is a research unit of the Laboratory of Robotics and Engineering Systems (LARSyS)

- Instituto Superior Técnico, Universidade de Lisboa

The work of João Pinto was supported by a PhD grant funded by Fundação para a Ciência e Tecnologia (FCT).

<p float="left" align="center">
  <img src="media/logos/dsor_logo.png" width="90" align="center"/>
  <img src="media/logos/isr_logo.png" width="200" align="center"/> 
  <img src="media/logos/larsys_logo.png" width="200" align="center"/> 
  <img src="media/logos/ist_logo.png" width="200" align="center"/> 
  <img src="media/logos/fct_logo.png" width="200" align="center"/> 
</p>


