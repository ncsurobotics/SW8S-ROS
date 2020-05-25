# Seawolf-8-Software
This repository contains the ROS powered code that runs on Seawolf-8, an autonomous submarine built by North Carolina State Universities underwater robotics club.

## Important ROS Topics
The following is a list of major ROS topics in this project organized by the package they belong to
### wolf_serial
* wolf_vertical
    * contains Float64 messages that determine the downwards velocity of the sub, accepts parameters from -1 to 1, where 1 is the maximum upwards thrust and vice versa
* wolf_lateral
    * contains Float64 messages that determine the lateral velocity of the sub, accepts parameters from -1 to 1, where 1 is the maximum forwards thrust and vice versa
* wolf_rotation
    * contains Float64 messages that determine the yaw velocity of the sub, accepts parameters from -1 to 1, where 1 is the maximum clockwise thrust and vice versa
* wolf_imu_euler
    * contains Vector3 messages that describe the subs orientation using euler angles
    
    
## Launching in simulation
Please see the wiki for how to install proper simulation software, once it is installed edit the "dir" var in "parallel_start.sh" to reflect where all the software is on your system, then this script can be run to start the simulation 