# Omnidirectional AGV with integrated low level controlller                                                                                                                                                          

## Instructions

1. Clone the repository into local workspace
2. To launch the gazebo simulation of the agv :
<<<<<<< HEAD
<br><br>
```shell
$ roslaunch agv_low agv_low_youbot_collisionwheel.launch
<br><br>
```
3. To launch to position controller :
```shell
<br><br>
$ roslaunch position_controller start.launch
```
4. To publish the desired pose :
<br><br>
```shell
$ rostopic pub /agv_mecanum/sp_pose geometry_msgs/PoseStamped "header: 
=======
<br>
'''bash
roslaunch agv_low agv_low_youbot_collisionwheel.launch
'''
3. To launch to position controller :
'''bash
roslaunch position_controller start.launch
'''
4. To publish the desired pose :
'''bash
rostopic pub /agv_mecanum/sp_pose geometry_msgs/PoseStamped "header:                                                                                                                             mark_one@alfred
>>>>>>> b6459d8d2adaf8c9bfe8d4744b386f703c951ba5
  seq: 0
  stamp:
    secs: 0
    nsecs: 0
  frame_id: ''
pose:
  position:
    x: 1.0 
    y: 1.0 
    z: 0.0 
  orientation:
    x: 0.0 
    y: 0.0 
    z: 0.99749499
    w: 0.0707372"
```

## TO DO
Implement Velocity Controller
