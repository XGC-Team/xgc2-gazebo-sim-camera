cmake_minimum_required(VERSION 3.0.2)
project(gazebo_sim_camera)

find_package(catkin REQUIRED COMPONENTS
  foxglove_msgs
  roscpp
  xgc_camera_msgs
)
find_package(gazebo REQUIRED)
find_package(GLEW REQUIRED)
find_package(JPEG REQUIRED)
find_package(OpenGL REQUIRED)

option(XGC_BUILD_NVJPEG_BACKEND "Build the optional CUDA/nvJPEG snapshot module" OFF)

catkin_package(
  LIBRARIES xgc_gazebo_media_camera
  CATKIN_DEPENDS foxglove_msgs roscpp xgc_camera_msgs
)

add_library(xgc_gazebo_media_camera SHARED
  src/snapshot_jpeg_backend.cpp
  src/xgc_media_camera_plugin.cpp
)
target_compile_features(xgc_gazebo_media_camera PRIVATE cxx_std_17)
add_dependencies(xgc_gazebo_media_camera ${catkin_EXPORTED_TARGETS})
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

if(XGC_BUILD_NVJPEG_BACKEND)
  find_path(XGC_CUDA_INCLUDE_DIR cuda_runtime_api.h
    HINTS ${XGC_CUDA_ROOT} ENV CUDA_HOME
    PATH_SUFFIXES include
    PATHS /usr/local/cuda/include
  )
  find_path(XGC_NVJPEG_INCLUDE_DIR nvjpeg.h
    HINTS ${XGC_CUDA_ROOT} ENV CUDA_HOME
    PATH_SUFFIXES include
    PATHS /usr/local/cuda/include
  )
  find_library(XGC_CUDART_LIBRARY cudart
    HINTS ${XGC_CUDA_ROOT} ENV CUDA_HOME
    PATH_SUFFIXES lib64 lib
    PATHS /usr/local/cuda/lib64
  )
  find_library(XGC_NVJPEG_LIBRARY nvjpeg
    HINTS ${XGC_CUDA_ROOT} ENV CUDA_HOME
    PATH_SUFFIXES lib64 lib
    PATHS /usr/local/cuda/lib64
  )
  if(NOT XGC_CUDA_INCLUDE_DIR OR NOT XGC_NVJPEG_INCLUDE_DIR OR
     NOT XGC_CUDART_LIBRARY OR NOT XGC_NVJPEG_LIBRARY)
    message(FATAL_ERROR "XGC_BUILD_NVJPEG_BACKEND requires CUDA runtime and nvJPEG development files")
  endif()
  add_library(xgc_gazebo_snapshot_jpeg_hardware SHARED
    src/snapshot_jpeg_nvjpeg.cpp
  )
  target_compile_features(xgc_gazebo_snapshot_jpeg_hardware PRIVATE cxx_std_17)
  target_include_directories(xgc_gazebo_snapshot_jpeg_hardware PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
    ${XGC_CUDA_INCLUDE_DIR}
    ${XGC_NVJPEG_INCLUDE_DIR}
  )
  target_link_libraries(xgc_gazebo_snapshot_jpeg_hardware
    ${XGC_NVJPEG_LIBRARY}
    ${XGC_CUDART_LIBRARY}
  )
  set_target_properties(xgc_gazebo_snapshot_jpeg_hardware PROPERTIES
    BUILD_RPATH_USE_ORIGIN TRUE
    INSTALL_RPATH "$ORIGIN"
  )
  install(TARGETS xgc_gazebo_snapshot_jpeg_hardware
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  )
endif()

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
  docs
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
  add_library(test_snapshot_jpeg_hardware_plugin SHARED
    test/fake_snapshot_jpeg_hardware.cpp
  )
  target_compile_features(test_snapshot_jpeg_hardware_plugin PRIVATE cxx_std_17)
  target_include_directories(test_snapshot_jpeg_hardware_plugin PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  add_executable(snapshot_jpeg_hardware_smoke
    test/snapshot_jpeg_hardware_smoke.cpp
    src/snapshot_jpeg_backend.cpp
  )
  target_compile_features(snapshot_jpeg_hardware_smoke PRIVATE cxx_std_17)
  target_include_directories(snapshot_jpeg_hardware_smoke PRIVATE
    ${CMAKE_CURRENT_SOURCE_DIR}/src
  )
  target_link_libraries(snapshot_jpeg_hardware_smoke JPEG::JPEG dl)
  catkin_add_gtest(test_fresh_render_gate
    test/test_fresh_render_gate.cpp
  )
  if(TARGET test_fresh_render_gate)
    target_compile_features(test_fresh_render_gate PRIVATE cxx_std_17)
    target_include_directories(test_fresh_render_gate PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
  endif()
  catkin_add_gtest(test_snapshot_jpeg_backend
    test/test_snapshot_jpeg_backend.cpp
    src/snapshot_jpeg_backend.cpp
  )
  if(TARGET test_snapshot_jpeg_backend)
    target_compile_features(test_snapshot_jpeg_backend PRIVATE cxx_std_17)
    target_include_directories(test_snapshot_jpeg_backend PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
    target_compile_definitions(test_snapshot_jpeg_backend PRIVATE
      XGC_TEST_JPEG_HARDWARE_LIBRARY="$<TARGET_FILE:test_snapshot_jpeg_hardware_plugin>"
    )
    target_link_libraries(test_snapshot_jpeg_backend JPEG::JPEG dl)
    add_dependencies(test_snapshot_jpeg_backend test_snapshot_jpeg_hardware_plugin)
  endif()
  catkin_add_gtest(test_unix_control_socket
    test/test_unix_control_socket.cpp
  )
  if(TARGET test_unix_control_socket)
    target_compile_features(test_unix_control_socket PRIVATE cxx_std_17)
    target_include_directories(test_unix_control_socket PRIVATE
      ${CMAKE_CURRENT_SOURCE_DIR}/src
    )
  endif()
  catkin_add_nosetests(test/test_intrinsic_sweep.py)
  catkin_add_nosetests(test/test_keyboard_camera_teleop.py)
  catkin_add_nosetests(test/test_web_calibration.py)
  catkin_add_nosetests(test/test_world_camera_profiles.py)
  add_rostest(test/static_camera_contract.test)
endif()
