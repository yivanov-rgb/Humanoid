cmake_minimum_required(VERSION 3.0.2)
project(robotrio_sim)
find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)
catkin_python_setup()
add_service_files(FILES ActionCommand.srv)
generate_messages(DEPENDENCIES std_msgs)
catkin_package(CATKIN_DEPENDS message_runtime std_msgs)
catkin_install_python(PROGRAMS scripts/smoke.py scripts/robot_description.py
  scripts/action_server.py scripts/action_cli.py scripts/prepare_start.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
install(DIRECTORY launch urdf worlds config
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
