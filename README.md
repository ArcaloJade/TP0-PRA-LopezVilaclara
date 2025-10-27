Para turtlebot3:
Terminal 1: ros2 launch turtlebot3_custom_simulation custom_room.launch.py
Terminal 2: ros2 run tpf_part0 part0_node --ros-args -p is_turtlebot3:=true

Para turtlebot4:
Terminal: ros2 run tpf_part0 part0_node --ros-args -p is_turtlebot3:=false