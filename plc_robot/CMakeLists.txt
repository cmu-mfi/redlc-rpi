cmake_minimum_required(VERSION 3.1.3)
project(plc_robot)

find_package(catkin REQUIRED COMPONENTS message_generation std_msgs)

add_message_files(FILES StampedArray.msg)
generate_messages(DEPENDENCIES std_msgs)

catkin_package(CATKIN_DEPENDS message_runtime)


install(DIRECTORY launch DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  PATTERN "setup_assistant.launch" EXCLUDE)
install(DIRECTORY config DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
