# Hausarbeit-ROS2

```
ros2 security generate_artifacts \
  -k keystore \
  -p POLICYFILE.xml \
  -e /gazebo \
     /spawn_entity \
     /robot_controller \
     /robot_state_publisher \
     /turtlebot3_diff_drive \
     /turtlebot3_imu \
     /turtlebot3_joint_state \
     /turtlebot3_laserscan \
     /rviz
```
Policy base\
-> created by ros2 security generate policy
-> missing many nodes like spawn_entity

Policy simple
-> handadded some nodes
-> very permissive with namespaces 
-> 