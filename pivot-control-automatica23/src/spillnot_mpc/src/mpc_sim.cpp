// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <iostream>
#include <fstream>

// #include "mpc.cpp"
#include <spillnot_mpc/mpc.cpp>

int main()
{
    const int stateDim = 10;
    const int outputDim = 6;

    // double l = 0.4;
    double T = 0.001; 
    // int mpcWindow = 40;
    // double input_cost = 0.001;
    // double state_diag_cost = 0.001;
    Eigen::Matrix<double, stateDim, 1> x0;
    x0 << 0, 0, 0, 0, 0, 0, 0, 0, 0, 0;
    

    // double inputBound = 1.;

    // bool verbose = true;
    // double timeLimit = 0.0005;


    // SpillnotMPC mpc(l, T, mpcWindow, input_cost, state_diag_cost, x0, inputBound, verbose, timeLimit);
    SpillnotMPC mpc;
    mpc = SpillnotMPC(0.001, mpc.x0, mpc.v_lim, mpc.tilt_lim, mpc.r_lim, mpc.input_bound);

    // controller  input and QPSolution vector
    Eigen::Vector3d ctr;
    Eigen::VectorXd QPSolution;

    // number of iteration steps
    // int numberOfSteps = 300*mpc.T_mpc/mpc.T_robot;
    // int numberOfSteps = 15000;
    int numberOfSteps = 10000;

    // file for output
    std::ofstream twist_file("/home/rafael/Documents/pivot_controller/catkin_ws/up/twist_sim_vel.csv");
    std::ofstream state_file("/home/rafael/Documents/pivot_controller/catkin_ws/up/state_sim_vel.csv");

    // desired position
    Eigen::Vector3d p_des(0.0, 0.4, 0);

    // desired velocity, testing modality
    Eigen::Vector3d v_des(0.12, 0.1, 0.5);

    // // populate reference position matrix to test update
    // Eigen::Matrix<double, mpc.outputDim, mpc.mpcWindow+1> yRef;
    // yRef.colwise() = p_des;


    Eigen::Matrix<double, 6, 1> twist;

    for (int i = 0; i < numberOfSteps; i++){

        // return to init pos
        // if(i==numberOfSteps/2) yRef(1) = 0; 
        // if(i>5000) p_des(1) = 0; 
        
        // // // reference update, trajectory
        // yRef.colwise() = p_des;
        // mpc.updateReferenceStateTraj(yRef);

        // reference update, position
        // mpc.updateReferenceStatePos(p_des);

        // reference update, velocity
        mpc.updateReferenceStateVel(v_des);

        // solve the QP problem
        if(mpc.solver.solveProblem() != OsqpEigen::ErrorExitFlag::NoError) return 1;

        // get the controller input
        QPSolution = mpc.solver.getSolution();
        ctr = QPSolution.block(10 * (mpc.mpcWindow + 1), 0, 3, 1);

        // // save data into file
        // auto x0Data = x0.data();
        twist = mpc.cTwist*mpc.x0;
        
        if (twist_file.is_open() && state_file.is_open())
        {
            // file << mpc.x0.transpose() << '\n';
            twist_file << twist.transpose() << '\n';
            state_file << x0.transpose() << '\n';
        }

        // propagate the model
        x0 = mpc.a_robot * mpc.x0 + mpc.b_robot * ctr;

        // update the constraint bound
        if (mpc.updateConstraintVectors(x0)==1) return 1;
        
      }
    twist_file.close();
    state_file.close();

    return 0;
}