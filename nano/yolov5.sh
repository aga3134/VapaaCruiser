export ROS_MASTER_URI=http://vapaa_cruiser:11311
export ROS_HOSTNAME=vapaa_cruiser
source ~/VapaaCruiser/nano/py3_ros_ws/devel/setup.bash
roslaunch yolov5 yolov5.launch
