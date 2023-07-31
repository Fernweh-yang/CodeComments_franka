#include "Trajectory.h"

#include <franka/exception.h>
#include <franka/robot.h>
#include <string>
#include <iostream>

#include <iostream>

std::string robot_ip = "192.168.3.127";

void setDefaultBehaviour(franka::Robot &robot);

int main() {

  try {
    // Connect to robot
    franka::Robot panda(robot_ip);
    setDefaultBehaviour(panda);

    // 2. read current robot state
    franka::RobotState initial_state = panda.readOnce();
    // O_T_EE_c contains the homogeneous transformation matrix as array with 16 values. 
    // Convert it to a 6d vector (3 translations, 3 RPY rotations) 
    Eigen::Vector6d initial_pose = homogeneousTfArray2PoseVec(initial_state.O_T_EE_c);

    // 3. Calculate target pose
    Eigen::Vector6d targetPose = initial_pose;
    // Calculate a 6d goal pose by adding 0.1 m to each of the translational coordinates.
    targetPose.head<3>() += Eigen::Vector3d::Constant(0.1);

    // 4./5. LinearTrajectory and TrajectoryIteratorCartesianVelocity object creation
    // Create a LinearTrajectory between start an end pose. Use v_max = 0.05, a_max = 0.5 and j_max = 1e-3.
    auto traj = LinearTrajectory(initial_pose, targetPose, 0.05, 0.5, 1.e-3);
    std::cout << "t_E = " << traj.getTEnd() << " s" << std::endl;
    //TrajectoryIteratorCartesianVelocity： overloads the function call operator, such that it can directly be used in franka::Robot.control(...)
    auto motionIterator = std::make_unique<TrajectoryIteratorCartesianVelocity>(traj);  

    std::cout << "WARNING: The robot will move now. "
              << "Keep around the STOP button." << std::endl
              << "Press ENTER to continue." << std::endl;
    std::cin.ignore();

    // 6. Franka Robot Controller:
    // motionIterator：Callback function for motion generation。即定义的operator()函数
    // controller_mode：Controller to use to execute the motion.
    panda.control(*motionIterator,
                  /*controller_mode = */ franka::ControllerMode::kCartesianImpedance);

  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  } catch (const std::invalid_argument &e) {
    std::cout << e.what() << std::endl;
    return -2;
  } catch (const std::exception &e) {
    std::cout << e.what() << std::endl;
    return -10;
  }

  std::cout << "Motion finished regularly." << std::endl;
  return 0;
}

void setDefaultBehaviour(franka::Robot &robot) {
  robot.setCollisionBehavior(
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0, 10.0}},
      {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}}, {{20.0, 20.0, 20.0, 20.0, 20.0, 20.0}},
      {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}}, {{10.0, 10.0, 10.0, 10.0, 10.0, 10.0}});
  robot.setJointImpedance({{3000, 3000, 3000, 2500, 2500, 2000, 2000}});
  robot.setCartesianImpedance({{3000, 3000, 3000, 300, 300, 300}});
}