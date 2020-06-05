#!/bin/bash

# Make sure to kill background processes on exit
trap 'kill 0; pkill gzserver; pkill gzclient' EXIT

dir=${SW8_SOFTWARE_DIR-~/programming}

source $dir/Seawolf-8-Software/devel/setup.bash

# Start all processes with & at the end
{
	cd $dir/bluerov_ros_playground
	source gazebo.sh
	exec gazebo worlds/underwater.world
} &
sleep 5

{
	exec rostest wolf_control test_pose.test
} &

{
	cd $dir/ardupilot/build/sitl/ArduSub
	exec sim_vehicle.py -f gazebo-bluerov2 -L RATBeach --out=udp:0.0.0.0:14550 --console
}

wait