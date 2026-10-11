# Base launch rendering reference

`base_launch.json` contains six normalized description parameters captured from
commit `14bfb28` with `roslaunch --dump-params`, without starting a ROS master or
Gazebo. The base launch and xacro were extracted with `git show` into an owned
temporary directory under `/home/node/workspace/build/w3-camera`; the original
launch was not retained as product code.

Each command was:

```sh
roslaunch --dump-params <BASE>/launch/static_camera.launch \
  world:=/granted/prepared-camera.world \
  simulation_endpoint:=/granted/aaaaaaaaaaaaaaaaaaaaaaaaaaaaaaaa.sock \
  target_id:=fixture python_executable:=/usr/bin/python3 \
  start_gazebo:=false gui:=false <case arguments>
```

The complete case arguments and full base commit SHA are recorded in the JSON.
`@profiles@` was replaced with the absolute path to `profiles.yaml` during
capture and is expanded the same way in the test. Its narrow and fast profiles
are test fixtures; the shipped profile catalog is unchanged. The six cases
cover default static/encoded, dynamic/unencoded, two fixture profiles, all
sensor/encoder overrides, and alternate identities/frames/topics/backend.
Endpoints and RTP allocations vary between cases.

Normalization removes comments, formatting whitespace and attribute ordering;
element order, attribute values and text content remain exact. The comparison
test obtains the CLI's authored entity by injecting a fake shared placement
module into `sys.modules`, so rendering is exercised through the actual CLI.
