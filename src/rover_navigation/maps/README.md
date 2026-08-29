# Maps

No map ships with this workspace — one can't be pre-built without the
actual rover driving around the actual space it'll operate in.

Build one:
1. `ros2 launch rover_bringup bringup.launch.py` (needs the LIDAR)
2. `ros2 launch rover_navigation slam.launch.py`
3. Drive the rover around (Xbox teleop or the web GUI) until the map
   looks complete in RViz.
4. `ros2 run nav2_map_server map_saver_cli -f ~/mars_rover_ws/src/rover_navigation/maps/<name>`

That produces `<name>.yaml` + `<name>.pgm` right here. Point
`navigation.launch.py`'s `map` argument at the `.yaml` file to
navigate against it.

See the "Navigation (SLAM + Nav2)" section of the top-level README for
the full walkthrough.
