# Seawolf-8-Software
This repository contains the ROS powered code that runs on Seawolf-8, an autonomous submarine built by North Carolina State Universities underwater robotics club.

## Launching in simulation
Please see the wiki for how to install proper simulation software, once it is installed set the system var SW8_SOFTWARE_DIR to point to where you installed all of your software. Now you can run the bash script "parallel_start.sh" to start the simulation, this will run the "seawolf8.launch" file by default

## Important ROS Topics
The following is a list of major ROS topics in this project that a developer should be aware of
* wolf_twist_setpoint
    * Sets setpoints for the sub to move towards
        * angular x y z are in terms of the subs magnetic compass and setting these will make the pose controller align the sub to this magnetic heading
        * linear z is in terms of depth below surface, and setting it will cause the sub to submerge to the given depth
        * linear x,y act as a speed vectors and exist in terms of "world" coordinates, an absolute reference frame. These values should generally not be set above 0.3, and control the speed and direction the sub travels in. 
* wolf_twist
    * contains geometry_msgs/Twist messages that set the sub's linear (up, forward, left, etc.) and angular (pitch, roll, yaw) velocities directly
    * should not be set outside of pose controllers
* wolf_yaw
    * current yaw of the sub in degrees (uses magnetic compass but can be changed)
* wolf_depth
    * depth of the sub in meters

## Important TF2 Frames
* world
    * just a constant reference frame set on startup
* wolf_hull
    * reference frame oriented with the general (center) hull of the sub. 

## Running ROS Tests
To run the tests, use this command

`rostest PACKAGE TEST`

most commonly tests will be "sanity check" tests which check the validity of topic IO of nodes within a package, these can run like this:

`rostest wolf_serial sanity_checks.test`