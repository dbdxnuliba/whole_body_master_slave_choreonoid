add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

# openrtm has deprecated coding. This causes too many warnings.
#   warning: dynamic exception specifications are deprecated in C++11
add_definitions(-Wno-deprecated)

find_package(catkin REQUIRED COMPONENTS
  rtmbuild
  fullbody_inverse_kinematics_solver
  prioritized_inverse_kinematics_solver
  cpp_filters
  )
find_package(choreonoid REQUIRED)
find_package(OsqpEigen REQUIRED)

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

rtmbuild_add_library(PrimitiveMotionLevelController SHARED
  PrimitiveMotionLevelController.cpp
  PrimitiveMotionLevelControllerService_impl.cpp
  PrimitiveCommand.cpp
  PositionController.cpp)
target_link_libraries(PrimitiveMotionLevelController
  ${catkin_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS}
  OsqpEigen::OsqpEigen)
set_target_properties(PrimitiveMotionLevelController PROPERTIES PREFIX "")
add_dependencies(PrimitiveMotionLevelController RTMBUILD_${PROJECT_NAME}_genrpc) # wait for rtmbuild_genidl

rtmbuild_add_executable(PrimitiveMotionLevelControllerComp PrimitiveMotionLevelControllerComp.cpp)
target_link_libraries(PrimitiveMotionLevelControllerComp PrimitiveMotionLevelController)
