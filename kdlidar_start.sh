#! /bin/bash
#set -x
echo "Sourcing..."

source /opt/ros/humble/setup.bash
source /mnt/robot_ws/install/setup.bash --extend
source /mnt/kdvisual_ws/install/setup.bash --extend

echo "Launching Node..."

echo "Debug: Script started with arguments: $@"

params="/mnt/robot_ws/kdlidar.yaml"
#settings="/mnt/kdvisual_ws/install/kdlidar_ros2/share/kdlidar_ros2/config/ouster.ini"   
settings="/mnt/robot_ws/ouster.ini"

test=$1
#DIRECTORY="/mnt/robot_ws/$(date +"%Y_%m_%d")"
DIRECTORY="/mnt/robot_ws/2024_09_11"

echo "Params: $params"
echo "Settings: $settings"


ros2 launch kdlidar_ros2 kdlidar.launch.py license_file:=/mnt/kdvisual_ws/install/kdlidar_ros2/share/kdlidar_ros2/config/kdlidar.kdlicense2 params_file:=$params settings_file:=$settings rviz:=true & 
NODE_PID=$!
disown $NODE_PID


prep_directories() {
	if [[ -d "$DIRECTORY" ]]; then
		mkdir $DIRECTORY/$test/
	elif [[ ! -d "$DIRECTORY" ]]; then
		mkdir $DIRECTORY
		mkdir $DIRECTORY/$test/
	fi
}		
	
call_rosservice() {
		prep_directories
		ros2 service call /kdlidar_ros/save_2d_map kdcommon_ros2_msgs/srv/SaveFile '{name: '$DIRECTORY'/'$test'/floorplan}'
		ros2 service call /kdlidar_ros/save_map kdcommon_ros2_msgs/srv/SaveFile '{name: '$DIRECTORY'/'$test'/map.kdlm}'
		ros2 service call /kdlidar_ros/save_las kdcommon_ros2_msgs/srv/SaveFile '{name: '$DIRECTORY'/'$test'/map.las}'
		ros2 service call /kdlidar_ros/save_ply kdcommon_ros2_msgs/srv/SaveFile '{name: '$DIRECTORY'/'$test'/map.ply}'
		ros2 service call /kdlidar_ros/save_trajectory kdcommon_ros2_msgs/srv/SaveFile '{name: '$DIRECTORY'/'$test'/trajectory.tum}'
		ros2 service call /kdlidar_ros/save_settings kdcommon_ros2_msgs/srv/SaveFile '{name: '$DIRECTORY'/'$test'/settings.ini}'	
		
		cd $DIRECTORY/$test/
		pwd
		echo "Dumping Params"
		ros2 param dump /kdlidar_ros/kdlidar_ros2_node > param_dump.yaml
		echo "Finished"
		echo "Dumping Frames"
		ros2 run tf2_tools view_frames
		echo "Finished"
		cd ../..
}

cleanup() {
	echo "Terminating KdLidar"
	kill $NODE_PID
	
}

rosbag_record() {
		prep_directories
		cd $DIRECTORY/$test/
		echo "Recording ROSBag.."
		nohup ros2 bag record /cmd_vel \
/odom \
/ouster/imu \
/ouster/metadata \
/ouster/nearir_image \
/ouster/os_driver/transition_event \
/ouster/points \
/ouster/range_image \
/ouster/reflec_image \
/ouster/scan \
/ouster/signal_image \
/tf \
/tf_static \
/kdlidar_ros/pose \
/kdlidar_ros/path \
/vrpn_mocap/TurtleBot3/pose > rosbag_record.log 2>&1 &
		NODE_PID_BAG=$!
		echo "$NODE_PID_BAG"
		cd /mnt/robot_ws
		
}

rosbag_record_kill() {
    echo "Killing rosbag recording..."
    if [[ -n "$NODE_PID_BAG" ]]; then
        kill $NODE_PID_BAG
        sleep 1
        if ps -p $NODE_PID_BAG > /dev/null; then
            echo "Process $NODE_PID_BAG did not terminate, force killing..."
            kill -9 $NODE_PID_BAG
        fi
        wait $NODE_PID_BAG 2>/dev/null
        echo "Rosbag recording killed."
        NODE_PID_BAG=""
    else
        echo "No rosbag recording to kill."
    fi
}

trap cleanup SIGINT


echo "ROS node launched with PID $NODE_PID. Press any key to call rosservice. Press Ctrl+C to exit."

# Wait for key press and call rosservice
while true; do

    read -n 1 -s key  # Wait for a key press and store it in the variable 'key'
    if [[ $key == "t" ]]; then
        call_rosservice
    elif [[ $key == "q" ]]; then  # Detect Ctrl+C
        cleanup
        break
    fi
    if [[ $key == "r" ]]; then
        rosbag_record
    elif [[ $key == "e" ]]; then
	rosbag_record_kill
    fi
done


echo "EXIT"
