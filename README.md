# goal_mover

ros2 topic pub --once /goal_pose geometry_msgs/msg/PoseStamped   '{header: {frame_id: "odom"}, pose: {position: {x: -18.75, y: -34.8, z: 0.0}, orientation: {x: 0.0, y: 0.0, z: 1.0, w: 0.0}}}'

# 1. Start the Avular Simulation with the mecanum wheel configuration
ros2 launch origin_one_gazebo ty_test_area.launch.py use_cmd_vel_controller:=True drive_configuration:=mecanum_drive

# 2. Start RviZ
ros2 launch origin_one_description origin_one_rviz.launch.py

# Goal Mover Launch in der Simulation verwenden (/goal_pose über Terminal)
ros2 launch goal_mover goal_mover.launch.py

# Goal Mover Launch in der Simulation verwenden (/goal_pose über RviZ)
ros2 launch goal_mover goal_mover_rviz.launch.py



# With the real robot:
Roboterplattform Starten:
 ON-Knopf
 auf Gitlab bwr_documentation laden
 
 
 Im Container starten:
 (Für Lidar zuerst install/setup.bash sourcen)
 -lidar
 -nav2 stack
 
 Separat starten:
 -zenoh bridge
 -rviz2
 
 Node starten.


# Debugging: