cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_camera)

find_package(catkin REQUIRED)
find_package(gazebo REQUIRED)
find_package(GLEW REQUIRED)
find_package(JPEG REQUIRED)
find_package(OpenGL REQUIRED)

catkin_package(
  LIBRARIES xgc_gazebo_media_camera
)

add_library(xgc_gazebo_media_camera SHARED
  src/xgc_media_camera_plugin.cpp
)
target_compile_features(xgc_gazebo_media_camera PRIVATE cxx_std_17)
target_include_directories(xgc_gazebo_media_camera PRIVATE
  ${catkin_INCLUDE_DIRS}
  ${GAZEBO_INCLUDE_DIRS}
  ${GLEW_INCLUDE_DIRS}
)
target_link_libraries(xgc_gazebo_media_camera
  ${catkin_LIBRARIES}
  ${GAZEBO_LIBRARIES}
  ${GLEW_LIBRARIES}
  JPEG::JPEG
  OpenGL::GL
  dl
)

install(TARGETS xgc_gazebo_media_camera
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
)

catkin_install_python(PROGRAMS
  scripts/camera_lifecycle_keepalive.py
  scripts/camera_contract_publisher.py
  scripts/camera_contract_test.py
  scripts/drive_intrinsic_calibration.py
  scripts/keyboard_camera_teleop.py
  scripts/web_calibration.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

install(DIRECTORY
  config
  launch
  rviz
  src
  test
  urdf
  web
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
  USE_SOURCE_PERMISSIONS
)

if(CATKIN_ENABLE_TESTING)
  find_package(rostest REQUIRED)
  catkin_add_nosetests(test/test_intrinsic_sweep.py)
  catkin_add_nosetests(test/test_keyboard_camera_teleop.py)
  catkin_add_nosetests(test/test_web_calibration.py)
  catkin_add_nosetests(test/test_world_camera_profiles.py)
  add_rostest(test/static_camera_contract.test)
endif()
