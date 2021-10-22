add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  cpp_filters
  joint_limit_table
  collision_checker_msgs
  primitive_motion_level_msgs
  primitive_motion_level_tools
  )
find_package(choreonoid REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  )

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

rtmbuild_add_library(WholeBodyTorqueController SHARED
  WholeBodyTorqueController.cpp
  WholeBodyTorqueControllerService_impl.cpp
  Collision.cpp
  TorqueController.cpp
  )
target_link_libraries(WholeBodyTorqueController
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  )
set_target_properties(WholeBodyTorqueController PROPERTIES PREFIX "")
add_dependencies(WholeBodyTorqueController RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(WholeBodyTorqueControllerComp WholeBodyTorqueControllerComp.cpp)
target_link_libraries(WholeBodyTorqueControllerComp WholeBodyTorqueController)
