#!/bin/bash

# Make sure to kill background processes on exit
trap 'kill 0; pkill gzserver; pkill gzclient' EXIT

dir=${SW8_SOFTWARE_DIR-}

source $dir/Seawolf-8-Software/devel/setup.sh

# Start all processes with & at the end
{
	cd $dir/bluerov_ros_playground
	source gazebo.sh
	gzserver worlds/underwater.world
} &
sleep 5

{
	rostest wolf_control test_pose.test --text
} &

{
	cd $dir/ardupilot/build/sitl/ArduSub
	sim_vehicle.py -f gazebo-bluerov2 -L RATBeach --out=udp:0.0.0.0:14550 --console
}

wait
