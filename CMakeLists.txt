cmake_minimum_required(VERSION 3.0.2)
project(radar_view)

find_package(catkin REQUIRED COMPONENTS
	rospy
	sensor_msgs
	std_msgs
	message_generation
)

add_message_files( FILES
	RadarObj.msg
	RadarObjArray.msg
)

generate_messages( DEPENDENCIES
	std_msgs
	geometry_msgs
)

catkin_package( CATKIN_DEPENDS
	message_runtime
	rospy
	sensor_msgs
	std_msgs
)

file(GLOB PYTHON_SCRIPTS_DIR
  "src/*.py"
)

catkin_install_python(
	PROGRAMS ${PYTHON_SCRIPTS_DIR}
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)