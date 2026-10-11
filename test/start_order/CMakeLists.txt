cmake_minimum_required(VERSION 3.16)
project(gazebo_parallel_start_tests LANGUAGES CXX)
find_package(catkin REQUIRED COMPONENTS roscpp geometry_msgs sensor_msgs std_msgs tf tf2 tf2_ros foxglove_msgs xgc_camera_msgs scout_msgs)
find_package(XgcXrpc 0.2.0 EXACT CONFIG REQUIRED COMPONENTS http)
find_package(PkgConfig REQUIRED)
pkg_check_modules(JSONCPP REQUIRED IMPORTED_TARGET jsoncpp>=1.7)
find_package(Threads REQUIRED)
set(XGC_SCENE_SOURCE "" CACHE PATH "Scene source containing the shared ROS master watcher")
if(NOT EXISTS "${XGC_SCENE_SOURCE}/src/ros_master_registration.cpp")
  message(FATAL_ERROR "The scene package's shared ROS master watcher is required")
endif()
add_library(xgc2_simulation_ros_master SHARED ${XGC_SCENE_SOURCE}/src/ros_master_registration.cpp)
target_compile_features(xgc2_simulation_ros_master PUBLIC cxx_std_17)
target_include_directories(xgc2_simulation_ros_master PUBLIC ${XGC_SCENE_SOURCE}/include ${catkin_INCLUDE_DIRS})
target_link_libraries(xgc2_simulation_ros_master PRIVATE ${catkin_LIBRARIES} Threads::Threads)
add_executable(ros_master_fixture ros_master_fixture.cpp)
target_link_libraries(ros_master_fixture PRIVATE xgc2_simulation_ros_master ${catkin_LIBRARIES})
add_executable(camera_source_control_test ../camera_source_control_test.cpp ../../src/camera_source_control.cpp)
target_compile_features(camera_source_control_test PRIVATE cxx_std_20)
target_include_directories(camera_source_control_test PRIVATE ../../src)
target_link_libraries(camera_source_control_test PRIVATE XgcXrpc::http PkgConfig::JSONCPP Threads::Threads)
enable_testing()
add_test(NAME camera_source_control COMMAND camera_source_control_test)

# Compile the changed plugin code without starting or linking a Gazebo server.
find_package(gazebo REQUIRED)
find_package(GLEW REQUIRED)
find_package(xgc2_math 0.5.8 REQUIRED CONFIG)
set(XGC_SCOUT_SOURCE "" CACHE PATH "Scout source root")
set(XGC_MECANUM_SOURCE "" CACHE PATH "Mecanum source root")
set(XGC_NV_CODEC_HEADERS "" CACHE PATH "Pinned NVENC header include directory")
add_library(camera_plugin_compile OBJECT ../../src/xgc_media_camera_plugin.cpp)
target_compile_features(camera_plugin_compile PRIVATE cxx_std_17)
add_library(scout_plugin_compile OBJECT
  ${XGC_SCOUT_SOURCE}/scout/src/scout_skid_steer_model_plugin.cpp
  ${XGC_SCOUT_SOURCE}/scout/src/scout_skid_steer.cpp
  ${XGC_SCOUT_SOURCE}/scout/src/scout_unicycle_plugin.cpp
  ${XGC_SCOUT_SOURCE}/scout/src/scout_implicit_wheel_plugin.cpp
  ${XGC_SCOUT_SOURCE}/scout/src/scout_wheel_diagnostics.cpp
  ${XGC_SCOUT_SOURCE}/scout/vendor/gazebo_ros_block_laser.cpp)
target_compile_features(scout_plugin_compile PRIVATE cxx_std_20)
target_link_libraries(scout_plugin_compile PRIVATE xgc2_math::control)
add_library(mecanum_plugin_compile OBJECT ${XGC_MECANUM_SOURCE}/src/mecanum_contract_plugin.cpp)
target_compile_features(mecanum_plugin_compile PRIVATE cxx_std_20)
foreach(target camera_plugin_compile scout_plugin_compile mecanum_plugin_compile)
  target_include_directories(${target} PRIVATE ${XGC_SCENE_SOURCE}/include ${XGC_SCOUT_SOURCE}/scout/include
    ${catkin_INCLUDE_DIRS} ${GAZEBO_INCLUDE_DIRS} ${GLEW_INCLUDE_DIRS} ${JSONCPP_INCLUDE_DIRS} ${XGC_NV_CODEC_HEADERS})
endforeach()
