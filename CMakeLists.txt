cmake_minimum_required(VERSION 3.0.2)
project(hokuyo_control_server)

find_package(catkin REQUIRED COMPONENTS
  geometry_msgs
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

catkin_install_python(PROGRAMS
  scripts/robot_http_server_indigo.py
  scripts/robot_http_server_noetic_gazebo.py
  scripts/example_client_usage.py
  scripts/example_camera_viewer.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(FILES
  src/common_http_api.py
  src/http_api_client.py
  DESTINATION ${CATKIN_PACKAGE_PYTHON_DESTINATION}
)

install(DIRECTORY launch/
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
