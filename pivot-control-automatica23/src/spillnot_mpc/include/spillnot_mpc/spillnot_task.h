// separating trajectory computation from controller, goal: reusability in different control modalities and different manipulators

// set world frame 
// initial orientation (reference vertical)

// task space kinematic controller: ((pose, twist) -> safe twist command)
//     measure state:
//         pose, twist -> measured state
//     state kalman filter:
//         measured state -> state estimate
//     mpc|spillnot:
//         state estimate, reference velocity -> safe twist command

#pragma once

#include <Eigen/Dense>

#include "mpc.cpp"

class Spillnot {
    public:

        // constructor
        Spillnot(const Eigen::Matrix<double, 4, 4> &W_T_O, const Eigen::Matrix<double, 4, 4> &O_T_EE, double v_lim, double tilt_lim, double r_lim, double input_bound);

        // Base frame transformation
        Eigen::Matrix<double, 4, 4> W_T_O; // init

        // reference vertical orientation
        Eigen::Vector3d V0; // init

        // state variables
        Eigen::Matrix<double, 10, 1> state_predicted; // init set to measured

        // MPC object
        SpillnotMPC mpc;

        // Solution variables
        Eigen::VectorXd QPSolution;
        Eigen::Vector3d ctr;

        // Kalman Filter
        Eigen::Matrix<double, 10, 10> P;
        Eigen::Matrix<double, 10, 10> Q;
        Eigen::Matrix<double, 10, 10> R;

        Eigen::Matrix<double, 10, 1> measure_state(const Eigen::Matrix<double, 4, 4> &O_T_EE,
            const Eigen::Matrix<double, 6, 1> &O_dP_EE);

        Eigen::Matrix<double, 10, 1> filter_state(const Eigen::Matrix<double, 10, 1> &state_measured);

        Eigen::Matrix<double, 6, 1> get_twist_command(const Eigen::Vector3d &v_des, const Eigen::Matrix<double, 4, 4> &O_T_EE, const Eigen::Matrix<double, 6, 1> &O_dP_EE);

};