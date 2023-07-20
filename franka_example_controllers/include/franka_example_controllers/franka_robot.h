#ifndef FRANKA_ROBOT_H
#define FRANKA_ROBOT_H

// MDH指的是 Modified Denavit-Hartenberg（改进的 Denavit-Hartenberg）参数化方法
#include <dqrobotics/robot_modeling/DQ_SerialManipulatorMDH.h>

namespace DQ_robotics{
    class FrankaRobot
        {
        public:
            static DQ_robotics::DQ_SerialManipulatorMDH kinematics();
            //static DQ_SerialManipulatorMDH dynamics(); To be implemented
        };

}

#endif // FRANKA_ROBOT_H