#define _USE_MATH_DEFINES
#include <math.h>
#include "traj.h"
#include <thread>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>


int main(int argc, char *argv[])
{   
    std::string robot_ip = "192.168.3.127";
    franka::Robot panda(robot_ip);
    setDefaultBehaviour(panda);
    // RobotState current_state;
    franka::RobotState current_state = panda.readOnce();
    // current_state.q_d = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    double speed_factor = 0.5;
    std::array<double, 7> q_goal = { M_PI_4, M_PI_2, 0.0 , 0.0 , 0.0 , 0.0, 0.0 };

    TrajectoryGenerator traj_generator(speed_factor, q_goal);

    int count = 0;
    bool isfinished = false;
    while (!isfinished)
    {   
        // TrajectoryGenerator实现了函数调用运算符operator()，所以实例可以像函数一样调用
        // 所以每次迭代都是在调用operator()函数
        // 因为机械臂频率是1KHZ，所以count/1000?
        isfinished = traj_generator(current_state, count / 1000.0);

        // thread和chrono都是从c++11引入的
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        count++;
    }
    
    std::cout << "Motion finished" << std::endl;

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