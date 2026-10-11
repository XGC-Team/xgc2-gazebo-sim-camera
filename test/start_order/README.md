These targets compile the changed plugin sources and test the camera control
host and shared ROS master watcher without starting Gazebo. The watcher comes
from the F10 scene package; its header and exported shared library must land
with the camera, Scout and Mecanum changes. No second watcher implementation is
included here.

From the camera worktree, source the underlay and foundation, then run:

```sh
nice -n 19 cmake -S test/start_order -B /home/node/workspace/build/f10-gzb/start-order \
  -DXGC_SCENE_SOURCE=/home/node/workspace/wt/f10-gz-scenes/xgc2_gazebo_scene \
  -DXGC_SCOUT_SOURCE=/home/node/workspace/wt/f10-gz-scout \
  -DXGC_MECANUM_SOURCE=/home/node/workspace/wt/f10-gz-mecanum \
  -DXGC_NV_CODEC_HEADERS=/home/node/workspace/build/f10-gzb/codec/include \
  -DCMAKE_BUILD_TYPE=Debug
nice -n 19 cmake --build /home/node/workspace/build/f10-gzb/start-order \
  --target ros_master_fixture camera_source_control_test camera_plugin_compile \
  scout_plugin_compile mecanum_plugin_compile -j2
```

The NVENC include directory must contain the same pinned headers as the main
camera build. Tests use the formal Python XRPC wheel, ROS Noetic Python modules
and private fake peer endpoints. The fake master uses a random port in
20000–40000 and its own ROS home. Every server, child process and worker is
stopped and joined by its owner.

Fast tests skip the long cases by default. A long test cell enables them with
`XGC_LONG_TESTS=1` and runs `test_peer_start_order.py`,
`test_ros_master_start_order.py`, `test_keyboard_camera_teleop.py` and the FS150 worktree's
`test_mavros_start_order.py`. Set `XGC_MASTER_FIXTURE` to the built
`ros_master_fixture` executable for the master test. Each outage lasts 60
seconds of wall time and checks process CPU below 1.2 seconds, bounded attempts
and logs, stable thread and descriptor counts, recovery and a single replay.
