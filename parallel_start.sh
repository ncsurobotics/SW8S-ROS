#!/bin/bash

# Make sure to kill background processes on exit
trap 'kill 0; pkill gzserver; pkill gzclient' EXIT

source ~/programming/Seawolf-8-Software/devel/setup.bash

# Start all processes with & at the end
{
	cd ~/programming/bluerov_ros_playground
	source gazebo.sh
	exec gazebo worlds/underwater.world
} &
sleep 5

{
	exec roslaunch ~/programming/Seawolf-8-Software/seawolf8.launch
} &

{
	cd ~/programming/ardupilot/build/sitl/ArduSub
	exec sim_vehicle.py -f gazebo-bluerov2 -L RATBeach --out=udp:0.0.0.0:14550 --console
}



wait
