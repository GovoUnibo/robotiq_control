cmake_minimum_required(VERSION 3.0.2)
project(robotiq_control)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

find_package(catkin 
  REQUIRED 
    genmsg 
    actionlib 
    actionlib_msgs
  COMPONENTS
    rospy 
    message_generation 
    std_msgs
)

add_action_files(
  DIRECTORY 
    action
  FILES 
    CommandRobotiqGripper.action
)

add_message_files(
  FILES
  GripperControl.msg
)

catkin_python_setup()

generate_messages(
        DEPENDENCIES 
        actionlib_msgs 
        std_msgs
)

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

include_directories(
  #include 
  ${catkin_INCLUDE_DIRS}
)


catkin_package(
    CATKIN_DEPENDS 
    rospy
    actionlib_msgs 
    std_msgs 
    message_runtime
)



catkin_install_python(PROGRAMS

                  # src/${PROJECT_NAME}/GripperActSrvModbusRtu.py
                  src/${PROJECT_NAME}/GripperActSrvTcp_Ip.py
                  src/${PROJECT_NAME}/GripperActSrvMdbsRs485.py
                  src/${PROJECT_NAME}/GripperTopicMdbsRs485.py
                  # src/${PROJECT_NAME}/include/GripperModbusRtu.py

                  src/${PROJECT_NAME}/test_client.py
                  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}

                )
