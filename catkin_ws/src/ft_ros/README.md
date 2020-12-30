# Force Torque Sensor PACAKGE V1.00
This package is used to process force torque sensor data

---
## DEPENDECIES ##
1. Ati_netft_ros_driver:Driver for the ATI NET/FT box https://github.com/ros-drivers/ati_netft_ros_driver

---
## HowToStart ##
1. netft controller(process ft_sensor data)
 ``` 
   roslaunch netft_controller netft_controller.launch
 ```
---
## Functions ##
 ```
 1.Bias compensation
    Bias compensation for the bias of force torque sensor to get true force torque data
 2.Low pass filter
    Process sensor data with low pass filter
 3.Gravity compensation
    Gravity compensation for gripper to get true force torque data
 ```
---
## Topics ##
 ```
 1.Raw wrench data
    Subscribe "/netft_data" topic to get raw wrench data
 2.Transformed wrench data
    Publish "/transformed_wrench_data" topic with transformed data
 ```
---
## Update ##
```
 V1.0.0 Releases
 Complete the framework of force torque sensor processor
 V1.1.0
 Update the naming method
```
---
## Attention ##
  ```
  1.The wrench data published is relative to tool frame
  ```
---
