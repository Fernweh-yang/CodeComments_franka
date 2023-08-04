#define _USE_MATH_DEFINES
#include <math.h>
#include "traj.h"
#include <thread>
#include <iostream>

#include <franka/exception.h>
#include <franka/robot.h>

std::string robot_ip = "192.168.3.127";
int main(int argc, char *argv[])
{   
    try{
        franka::Robot panda(robot_ip);
        setDefaultBehavior(panda);
        // ********** RobotState current_state **********
        franka::RobotState current_state = panda.readOnce();
        std::cout<< "current joint state:";
        for (const double& element : current_state.q ) {
            std::cout << element << ",";
        }
        std::cout << std::endl;

        // ********** desired state **********
        std::array<double, 7> q_goal;
        q_goal = {{-0.169099,-0.186942,-0.351694,-1.99691,-0.0261756,1.96934,0.213579}};
        q_goal = {-0.208641,0.0507064,-0.25078,-2.01031,0.0316269,2.09578,0.330997};
        // q_goal = {0.0120356, 0.0681359, -0.430619,  -1.91028,  0.123545,   2.05192,  0.296204};
        // ********** call trajectory generate function **********
        double speed_factor = 0.5;
        TrajectoryGenerator traj_generator(speed_factor, q_goal);

        // ********** control the robot **********
        std::cout << "WARNING: The robot will move now. "
              << "Keep around the STOP button." << std::endl
              << "Press ENTER to continue." << std::endl;
        std::cin.ignore();
        panda.control(traj_generator);

        // int count = 0;
        // bool isfinished = false;
        // while (!isfinished)
        // {   
        //     /TrajectoryGenerator实现了函数调用运算符operator()，所以实例可以像函数一样调用
        //     /所以每次迭代都是在调用operator()函数
        //     /因为机械臂频率是1KHZ，所以count/1000?
        //     isfinished = traj_generator(current_state, count / 1000.0);

        //     /thread和chrono都是从c++11引入的
        //     std::this_thread::sleep_for(std::chrono::milliseconds(1));
        //     count++;
        // }
        
        // ********** Finish **********
        std::cout << "Finished moving to joint configuration : [";
        for (auto e : q_goal) {
        std::cout << e << ",";
        }
        std::cout << "] ." << std::endl;
    }catch (const franka::Exception &e) {
        std::cout << e.what() << std::endl;
        return -1;
    }

    return 0;
}
