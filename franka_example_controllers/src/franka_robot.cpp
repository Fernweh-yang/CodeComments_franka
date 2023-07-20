#include <franka_example_controllers/franka_robot.h>

namespace DQ_robotics{
    DQ_SerialManipulatorMDH FrankaRobot::kinematics(){
        Matrix<double,5,7> FEp_DH_matrix(5,7);
        FEp_DH_matrix <<    0,      0,       0,       0,       0,      0,       0,      //FEp_DH_theta
                        0.333,      0,   0.316,       0,   0.384,      0,   0.107,      //FEp_DH_d
                            0,      0,  0.0825, -0.0825,       0,  0.088,  0.0003,      //FEp_DH_a
                    -M_PI/2, M_PI/2,  M_PI/2, -M_PI/2,  M_PI/2, M_PI/2,       0,      //FEp_DH_alpha
                            0,      0,       0,       0,       0,      0,       0;      //must be 5x7
        DQ_SerialManipulatorMDH kin(FEp_DH_matrix);
        
        // ? 这个坐标是否正确？
        DQ r = DQ(1.,0,0,0); 
        DQ p = DQ(0,0,0,0);
        kin.set_reference_frame(r+E_*p*r);
        kin.set_base_frame(r+E_*p*r);
        return kin;
    }
}