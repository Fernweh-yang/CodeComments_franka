#include "pinocchio/parsers/urdf.hpp"
 
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
 
#include <iostream>

// #include <spillnot_mpc/robot.h>

int main(int argc, char ** argv)
{ 

  const std::string arm_hand_urdf_filename = "/home/gari/spillnot_ws/src/pivot-control/src/spillnot_mpc/robots/panda_arm_hand_fixed_fingers.urdf";
  const std::string ee_frame_name = "panda_hand_tcp";

  Robot robot(arm_hand_urdf_filename, ee_frame_name);
  
  return 0;
}
