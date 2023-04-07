# ROS Workspace setup

You need to build the workspace using the following command :
`catkin_make --cmake-args -DCMAKE_BUILD_TYPE=Release -DPYTHON_EXECUTABLE=/usr/bin/python3 -DPYTHON_INCLUDE_DIR=/usr/include/python3.6m -DPYTHON_LIBRARY=/usr/lib/x86_64-linux-gnu/libpython3.6m.so`

Don't forget to `source` the `setup.sh` file :
`source devel/setup.sh`

Run using `rosrun` :
`rosrun skeleton_converter skeleton_converter.py`
