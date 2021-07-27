find_package(catkin REQUIRED rtmbuild)
find_package(OpenCV REQUIRED)

set(comp_sources WholeBodyMasterSlaveChoreonoid.cpp WholeBodyMasterSlaveChoreonoidService_impl.cpp)
set(libs ${OpenCV_LIBRARIES})

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")
rtmbuild_add_library(WholeBodyMasterSlaveChoreonoid SHARED ${comp_sources})
target_link_libraries(WholeBodyMasterSlaveChoreonoid ${libs})
set_target_properties(WholeBodyMasterSlaveChoreonoid PROPERTIES PREFIX "")

rtmbuild_add_executable(WholeBodyMasterSlaveChoreonoidComp WholeBodyMasterSlaveChoreonoidComp.cpp)
target_link_libraries(WholeBodyMasterSlaveChoreonoidComp WholeBodyMasterSlaveChoreonoid)
