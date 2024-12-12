#! /bin/bash
#set -x
RED='\e[31m'
GREEN='\e[32m'
YELLOW='\e[33m'
BLUE='\e[34m'
RESET='\e[0m'

echo "Sourcing..."

export ROS_IP=192.168.10.2
export ROS_MASTER_URI=http://192.168.10.1:11311

source /opt/ros/noetic/setup.bash --extend
source /mnt/kdvisual_ws/install/setup.bash --extend
source /mnt/robot_ws/devel/setup.bash --extend

echo "Debug: Script started with arguments: $@"

## Default Variables
settings="/mnt/robot_ws/good_settings.ini"
test=$3

if [[ (${1:-} == "multi" && ${2:-} == "stereo") || (${1:-} == "" && ${2:-} == "")]]; then
	params="/mnt/robot_ws/multi_stereo_config.yaml"	
	rosbag_arg="${4:-}"
elif [[ (${1:-} == "single" && ${2:-} == "stereo") ]]; then
	params="/mnt/robot_ws/single_stereo_config.yaml"
	rosbag_arg="${4:-}"
elif [[ ($1 == "multi" && $2 == "rgbd") ]]; then
	params="/mnt/robot_ws/multi_rgbd_config.yaml"
	rosbag_arg="${4:-}"
elif [[ ($1 == "single" && $2 == "rgbd") ]]; then
	params="/mnt/robot_ws/single_rgbd_config.yaml"
	rosbag_arg="${4:-}"
elif [[ ($1 == "atheon") ]]; then
	params="/mnt/robot_ws/single-stereo-atheon.yaml"
	settings="/mnt/robot_ws/kdvisual_multiple_realsense.ini"
	test=$2
	rosbag_arg="${3:-}"
fi    

if [[ -n "$rosbag_arg" ]]; then
    echo -e "${GREEN}Using rosbag file: $rosbag_arg${RESET}"
else
    echo -e "${YELLOW}No rosbag file provided, skipping rosbag argument...${RESET}"
fi

echo "Single/Multi: $1"
echo "Stereo/RGBD: $2"
echo -e "${GREEN}<---------- Params: $params ---------->${RESET}"
echo -e "${GREEN}<---------- Settings: $settings ---------->${RESET}"

roslaunch kdvisual_ros kdvisual.launch license_file:=/mnt/robot_ws/KdVisualGoodLicense.kdlicense2 params_file:=$params settings_file:=$settings rviz:=true ${rosbag_arg:+"rosbag:=$rosbag_arg"} &
#roslaunch lidar_odometry_atheon.launch params_file:=$params settings_file:=$settings ${rosbag_arg:+"rosbag:=$rosbag_arg"} &

#DIRECTORY="/mnt/robot_ws/$(date +"%Y_%m_%d")"
DIRECTORY="/mnt/robot_ws/2024_12_04"

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
		rosservice call /kdvisual_ros/save_2d_map "name: '$DIRECTORY/$test/floorplan'"
		rosservice call /kdvisual_ros/save_map "name: '$DIRECTORY/$test/kdmap.kdvm'"
		rosservice call /kdvisual_ros/save_trajectory "name: '$DIRECTORY/$test/trajectory.tum'"
		rosservice call /kdvisual_ros/save_log "name: '$DIRECTORY/$test/logs.log'"
		rosservice call /kdvisual_ros/save_settings "name: '$DIRECTORY/$test/settings.ini'"	
		
		cd $DIRECTORY/$test/
		rosparam dump param_dump.yaml
		rosrun pcl_ros pointcloud_to_pcd input:=/kdvisual_ros/map_points
			
}

cleanup() {
	echo -e "${RED}Terminating KdVisual${RESET}"
	ps aux | grep kdvisual | grep -v grep | awk '{ print "kill -9", $2 }' | sh
	ps aux | grep rf2o | grep -v grep | awk '{ print "kill -9", $2 }' | sh
	ssh handsfree@192.168.10.1 "echo 'handsfree' | sudo -S systemctl restart roscore.service" &
	ssh handsfree@192.168.10.1 "echo 'handsfree' | sudo -S systemctl restart rundriverlaser.service" &
	ssh handsfree@192.168.10.1 "echo 'handsfree' | sudo -S systemctl restart runlaser.service" &
	cd /mnt/robot_ws
	sleep 2
	printf "${GREEN}\n<----- Restarting ROSCORE & Gemini nodes.. Please wait.. ----->\n${RESET}"
	unset params
	unset settings
	sleep 6
}

rosbag_record() {
		prep_directories
		cd $DIRECTORY/$test/
		rosbag record  /camera_back/infra1/camera_info \
		/camera_back/infra1/image_rect_raw \
		/camera_back/infra2/camera_info \
		/camera_back/infra2/image_rect_raw \
		/camera_front/infra1/camera_info \
		/camera_front/infra1/image_rect_raw \
		/camera_front/infra2/camera_info \
		/camera_front/infra2/image_rect_raw \
		/camera_front/imu \
		/tf \
		/tf_static \
		/lidar_odom \
		/mobile_base/mobile_base_controller/odom \
		/mobile_base/mobile_base_controller/cmd_vel \
		/base_scan \
		/camera_front/color/image_raw \
		/camera_front/color/camera_info \
		/camera_front/aligned_depth_to_color/image_raw \
		/camera_front/aligned_depth_to_color/camera_info \
		/camera_back/color/image_raw \
		/camera_back/color/camera_info \
		/camera_back/aligned_depth_to_color/image_raw \
		/camera_back/aligned_depth_to_color/camera_info \
		/vrpn_client_node/GeminiMini2/pose & NODE_PID_BAG=$!
		echo "$NODE_PID_BAG"
		cd /mnt/robot_ws
		
}

rosbag_record_kill() {
    echo -e "${YELLOW}Killing rosbag recording...${RESET}"
    if [[ -n "$NODE_PID_BAG" ]]; then
        kill $NODE_PID_BAG
        wait $NODE_PID_BAG 2>/dev/null
        echo -e "${GREEN}Rosbag recording killed.${RESET}"
        NODE_PID_BAG=""
    else
        echo -e "${RED}No rosbag recording to kill.${RESET}"
    fi
}

#trap cleanup SIGINT


echo -e "${YELLOW}ROS node launched with PID $NODE_PID. Press 'q' to exit. Press 't' to call all rosservices. Press 'r' to record rosbag and 'e' to kill rosbag${RESET}"

while true; do

    read -n 1 -s key 
    # if [[ $key == "t" || $key == "T" ]]; then
    #     call_rosservice
    # elif [[ $key == "q" || $key == "Q" ]]; then  # Detect Ctrl+C
    #     cleanup
    #     break
    # fi
    # if [[ $key == "r" || $key == "R" ]]; then
    #     rosbag_record
    # elif [[ $key == "e" || $key == "E" ]]; then
	# rosbag_record_kill
    # fi
	case $key in
        t|T) call_rosservice ;;
        r|R) rosbag_record ;;
        e|E) rosbag_record_kill ;;
        q|Q) cleanup; break ;;
    esac

done


printf "\nDONE\n"
