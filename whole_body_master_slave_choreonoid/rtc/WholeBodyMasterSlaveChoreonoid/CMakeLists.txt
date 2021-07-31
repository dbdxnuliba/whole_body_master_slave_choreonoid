add_compile_options(-std=c++11)
set(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED
  rtmbuild
  fullbody_inverse_kinematics_solver)
find_package(choreonoid REQUIRED)
find_package(OpenCV REQUIRED)

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CHOREONOID_INCLUDE_DIRS}
  ${OpenCV_INCLUDE_DIRS}
  )

# 相対パスを絶対パスに直す
set(CHOREONOID_BODY_LIBRARIES_ABS)
foreach(lib ${CHOREONOID_BODY_LIBRARIES})
  find_library(${lib}_abs NAMES ${lib} PATHS ${CHOREONOID_LIBRARY_DIRS})
  set(CHOREONOID_BODY_LIBRARIES_ABS ${CHOREONOID_BODY_LIBRARIES_ABS} ${${lib}_abs})
endforeach(lib)

rtmbuild_add_library(WholeBodyMasterSlaveChoreonoid SHARED
  WholeBodyMasterSlaveChoreonoid.cpp
  WholeBodyMasterSlaveChoreonoidService_impl.cpp)
target_link_libraries(WholeBodyMasterSlaveChoreonoid
  ${OpenCV_LIBRARIES}
  ${CHOREONOID_BODY_LIBRARIES_ABS})
set_target_properties(WholeBodyMasterSlaveChoreonoid PROPERTIES PREFIX "")

rtmbuild_add_executable(WholeBodyMasterSlaveChoreonoidComp WholeBodyMasterSlaveChoreonoidComp.cpp)
target_link_libraries(WholeBodyMasterSlaveChoreonoidComp WholeBodyMasterSlaveChoreonoid)
