# UR16e ROS PACKAGE V1.0.0
This package is used to control ur16e with ros messages and transform ur16e data into ros message type

---
## DEPENDECIES ##
1. UR_RTDE:Realtime UR series robot API   https://pypi.org/project/ur-rtde/

---

## HowToStart ##
1.UR16e controller(Control ur16e with ros messages)
 ```
 rosrun ur16e_controller controller
 ```
2.UR16e receiver(Transform ur16e data into ros messages)
 ```
 rosrun ur16e_controller receiver
 ```
3.Both nodes(Start both controller and receiver)
 ```
 roslaunch ur16e_controller ur16e_bringup.launch
 ```
---
## Functions ##
 ```
 1.Joints speed command(SpeedJ)
    Control ur16e joints speed with ros topic "/joint_twist_cmd"
 2.Joints pose command(MoveJ)
    Control ur16e tool speed with ros topic "/joint_pose_cmd"
 3.Tool speed command(SpeedL)
    Control ur16e tool speed with ros topic "/tool_twist_cmd"
 4.Tool pose command(MoveL)
    Control ur16e tool speed with ros topic "/tool_pose_cmd"
 ```
---
## Topics ##
 ```
 1.Tool twist info
    Subscribe "/ur16e_tool_twist" topic to get tool twist info 
 2.Tool pose info
    Subscribe "/ur16e_tool_pose" topic to get tool pose info 
 3.Joint states info
    Subscribe "/ur16e_joint_states" topic to get joint states info 
 ```
---
## Update ##
  ```
 V1.0.0 Released
 Complete the framework of ur16e ros controller and message processor
  ```
---
## Attention ##
  ```
  1.Publishing two kinds of command in the same time is not allowed
  ```
---
