# CodeComments_franka

## How to run the controller to do the grasp task

```shell
roslaunch franka_example_controllers joint_velocity_example_controller.launch
```

Before run the code you need to set three target pose first:

1. First pose: where the object needs to be picked up.
   - If no trajectory planning, copy the target joint pose into [here](https://github.com/Fernweh-yang/CodeComments_franka/blob/c27909939de9a8c02eeb2f870bd57e6b8f6983aa/franka_example_controllers/config/franka_example_controllers.yaml#L14)
   - If with trajectory planning, copy the target EE‘s pose into [here](https://github.com/Fernweh-yang/CodeComments_franka/blob/c27909939de9a8c02eeb2f870bd57e6b8f6983aa/franka_example_controllers/config/franka_example_controllers.yaml#L19)
2. Second pose: as a midpoint between first and third pose
   - copy the pose [here](https://github.com/Fernweh-yang/CodeComments_franka/blob/c27909939de9a8c02eeb2f870bd57e6b8f6983aa/franka_example_controllers/src/joint_velocity_example_controller.cpp#L222)
3. Third pose: where the object needs to be put.
   - copy the pose [here](https://github.com/Fernweh-yang/CodeComments_franka/blob/c27909939de9a8c02eeb2f870bd57e6b8f6983aa/franka_example_controllers/src/joint_velocity_example_controller.cpp#L230)

## How to set different robot's IP

Change the robot_ip in the [launch](https://github.com/Fernweh-yang/CodeComments_franka/blob/c27909939de9a8c02eeb2f870bd57e6b8f6983aa/franka_example_controllers/launch/joint_velocity_example_controller.launch#L3) file.

## How to get the current pose

use the [code](https://github.com/Fernweh-yang/CodeComments_franka/blob/main/panda_tutorial/source/03_Panda_Axes_Motion/do_axes_motion.cpp) to read current joint state and current EE pose

1. Move the robot to the place, where the object stay. 

   This place will be used as target pose in the controller

2. compile the code 

   ```
   cd CodeComments_franka/panda_tutorial
   mkdir build 
   cd build 
   make
   ```

3. run the code 

   ```
   cd CodeComments_franka/panda_tutorial/build/source/03_Panda_Axes_Motion
   ./axes_motion
   ```

4. copy the pose to the code as showed in **How to run the controller to do the grasp task**