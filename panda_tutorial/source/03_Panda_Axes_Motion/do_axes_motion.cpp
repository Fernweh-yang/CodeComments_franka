#include "examples_common.h"
#include <franka/exception.h>
#include <franka/robot.h>
#include <string>
#include <iostream>

std::string pandaIP = "192.168.3.127";
int main(void) {
  try {
    franka::Robot panda(pandaIP);
    setDefaultBehavior(panda);

    // 2. Define goal position and construct MotionGenerator (defined in "examples_common.h")
    // std::array<double, 7> q_goal = {{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
    std::array<double, 7> q_goal = {{0.968844,  0.305047,   -0.452106,  -1.89069,   0.0577989,   2.24276,   1.39396}};
    // motiongenerator class: https://frankaemika.github.io/libfranka/classMotionGenerator.html
    MotionGenerator motion_generator(0.05, q_goal);

    std::cout << "WARNING: The robot will move now. "
              << "Keep around the STOP button." << std::endl
              << "Press ENTER to continue." << std::endl;
    std::cin.ignore();

    // 3. start the control
    // libfranka不同的control models: https://frankaemika.github.io/libfranka/classfranka_1_1Robot.html
    panda.control(motion_generator);

    std::cout << "Finished moving to joint configuration : [";
    for (auto e : q_goal) {
      std::cout << e << ",";
    }
    std::cout << "] ." << std::endl;

  } catch (const franka::Exception &e) {
    std::cout << e.what() << std::endl;
    return -1;
  }

  return 0;
}