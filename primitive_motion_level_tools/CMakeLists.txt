cmake_minimum_required(VERSION 3.0.2)
project(primitive_motion_level_tools)

add_compile_options(-std=c++11)

# rtmbuild has non exist dependency
# (ROSBUILD_genmsg_cpp) and (RTMBUILD_${PROJECT_NAME}_genidl)
cmake_policy(SET CMP0046 OLD)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  roscpp
  cpp_filters
  )
find_package(choreonoid REQUIRED)

# initialize rtmbuild (generate_messages is called)
rtmbuild_init()

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES ${PROJECT_NAME}
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)

###########
## Build ##
###########

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

add_library(${PROJECT_NAME}
  src/PrimitiveState.cpp
  )

target_link_libraries(${PROJECT_NAME}
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  )

target_include_directories(${PROJECT_NAME} PUBLIC
  include
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
)

add_subdirectory(rtc)
