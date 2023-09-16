# The No Spill Robot Project
This package implements the slosh-free control approach for teleoperation as described on our [site](https://sites.google.com/view/thenospillproject?pli=1#h.3f4tphhd9pn8).


This implementation requires the [osqp-eigen](https://github.com/robotology/osqp-eigen) interface. 

## Build and Launch
Build with optimization  
`catkin build spillnot_mpc --cmake-args -DCMAKE_BUILD_TYPE=Release`  

Launch spillnot joint controller  
`roslaunch spillnot_mpc joint_mpc.launch robot_ip:=<...>`  

Launch the safety mapping in geometric_fixtures
`roslaunch geometric_fixtures asym_scalar.launch`  

In case of a tilted robot base (GARMI), set the parameter `y_up: true` in `spillnot_mpc.yaml`.

Collision Behavior
The standard robot collision reflex is changed in the `spillnot_mpc/config/collision_behavior.yaml` for the force admittance interface 

## Available Interfaces for Velocity Based Teleoperation
Android IMU data following this [tutorial](https://www.linkedin.com/pulse/using-your-smartphone-sensor-suite-playing-around-ros-satyajit-bagchi).
`rosrun android_sensor_remote remote_control.py`

Keyboard teleoperation
`rosrun teleop_twist_keyboard teleop_twist_keyboard.py cmd_vel:=pre_teleop_vel _speed:=0.2`

Admittance framework (ongoing research, disabled by default), the velocities are designed according to the measured external forces at the end effector
Enable with `admittance_active` parameter in the `geometric_fixtures/config/geometric_fixtures.yaml` config file.
