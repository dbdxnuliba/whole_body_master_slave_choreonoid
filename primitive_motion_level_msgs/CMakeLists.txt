cmake_minimum_required(VERSION 3.0.2)
project(primitive_motion_level_msgs)

## Compile as C++11, supported in ROS Kinetic and newer
add_compile_options(-std=c++11)

# rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  message_generation
  std_msgs
  geometry_msgs
  )

add_message_files(
  FILES
  PrimitiveState.msg
  PrimitiveStateArray.msg
  )

# initialize rtmbuild (generate_messages is called)
rtmbuild_init(geometry_msgs)

catkin_package(
#  INCLUDE_DIRS include
  LIBRARIES PrimitiveStateSkel PrimitiveStateStub
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

# generate idl
rtmbuild_genidl()

add_custom_target(genidl ALL DEPENDS RTMBUILD_${PROJECT_NAME}_genrpc)
