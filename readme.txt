assignment running instructions:
1) run the following command in terminal .
	1.1) ssh komodo@192.168.0.100  #when 192.168.0.100 is the robot ip
	1.2) export ROS_IP="192.168.0.100" #define ROS_IP and check you defined it right with echo.
	1.3) run the following command in the ros.
		"roslaunch robotican_komodo komodo_nav.launch lidar:=true move_base:=true front_camera:=true robot_localization:=true asus_camera:=true have_map_file:=true map_file:=/home/komodo/ILO_map.yaml"
2)run the following command in terminal .
	source /opt/ros/indigo/setup.bash
    source $HOME/catkin_ws/devel/setup.bash
    source /usr/share/gazebo/setup.sh
	export ROS_MASTER_URI="http://192.168.0.100:11311"
	echo $ROS_MASTER_URI
	export ROS_IP="192.168.0.101"  #my computer ip. ifconfig
	echo $ROS_IP
	export PS1="[ROS-ROBOT]${PS1}"

	Another Option is to define the following in ~/.bashrc
	############################################################
"	# Setup ROS Lunar environment
	ros-env() {
	  export LD_LIBRARY_PATH="/usr/oldlib:${LD_LIBRARY_PATH}"
	    source /opt/ros/indigo/setup.bash
	      source $HOME/catkin_ws/devel/setup.bash
	        source /usr/share/gazebo/setup.sh
	          alias catkin_make="catkin_make -DPYTHON_EXECUTABLE=/usr/bin/python2"
	    	export PS1="[ROS-${ROS_DISTRO}]${PS1}"
	}
	ros-robot-ip(){
		source /opt/ros/indigo/setup.bash
	    source $HOME/catkin_ws/devel/setup.bash
	    source /usr/share/gazebo/setup.sh
		export ROS_MASTER_URI="http://192.168.0.100:11311"
		echo $ROS_MASTER_URI
		export ROS_IP="192.168.0.101"  #my computer ip. ifconfig
		echo $ROS_IP
		export PS1="[ROS-ROBOT]${PS1}"

	}"
	############################################################	
3) after we defined the env-variables . run in terminal the following " rosrun ass2 project_map.py ". 



writen by :
			Linoy Barel			206217382
			Or Naimark			305625014
			Inon Ben-David		206729543
