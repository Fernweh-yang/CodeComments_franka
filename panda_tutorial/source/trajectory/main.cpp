#define _USE_MATH_DEFINES
#include <math.h>
#include "traj.h"
#include <thread>
#include <iostream>


int main(int argc, char *argv[])
{
    RobotState current_state;
    current_state.q_d = { 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0 };

    double speed_factor = 0.5;
    std::array<double, 7> q_goal = { M_PI_4, M_PI_2, 0.0 , 0.0 , 0.0 , 0.0, 0.0 };

    TrajectoryGenerator traj_generator(speed_factor, q_goal);

    int count = 0;
    bool isfinished = false;
    while (!isfinished)
    {
        isfinished = traj_generator(current_state, count / 1000.0);

        // thread和chrono都是从c++11引入的
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
        count++;
    }
    
    std::cout << "Motion finished" << std::endl;

    return 0;
}