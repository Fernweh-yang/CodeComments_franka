#ifndef DQ_ROBOTS_FRANKAROBOT_H
#define DQ_ROBOTS_FRANKAROBOT_H

#include<dqrobotics/robot_modeling/DQ_SerialManipulatorDH.h>
#include<dqrobotics/utils/DQ_Constants.h>

namespace DQ_robotics
{

class FrankaRobot
{
public:
    static DQ_SerialManipulatorDH kinematics(bool load_gripper)
    {
        const double pi2 = pi/2.0;

        Matrix<double,5,7> franka_dh(5,7);
      
        franka_dh <<    0,      0,          0,          0,      0,      0,      0,
                    0.333,      0,      0.316,          0,  0.384,      0,     0.107,
                        0,      0,     0.0825,    -0.0825,      0,  0.088,  0.0003,
                     -pi2,    pi2,        pi2,       -pi2,    pi2,    pi2,           0,
                        0,      0,          0,          0,      0,      0,             0;

        if (load_gripper){
            franka_dh(1, 6) += 0.1034;
            franka_dh(3, 6) = pi/4.0;
        }

        DQ_SerialManipulatorDH franka(franka_dh,"standard");

        return franka;
    }
};

}

#endif
