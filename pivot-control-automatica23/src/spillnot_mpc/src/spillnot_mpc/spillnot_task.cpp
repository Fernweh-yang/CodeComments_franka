#include <spillnot_mpc/spillnot_task.h>
#include <iostream>

Spillnot::Spillnot(const Eigen::Matrix<double, 4, 4> &W_T_O, const Eigen::Matrix<double, 4, 4> &O_T_EE, double v_lim, double tilt_lim, double r_lim, double input_bound): W_T_O(W_T_O){

    // set reference vertical direction 
    Eigen::Matrix<double, 4, 4> W_T_EE = W_T_O*O_T_EE;
    V0 = W_T_EE.block(0, 0, 3, 3).row(2);

    // zero initial velocity
    Eigen::Matrix<double, 6, 1> O_dP_EE;
    O_dP_EE.setZero();

    // initial state measurement for mpc
    Eigen::Matrix<double, 10, 1> state_measured = measure_state(O_T_EE, O_dP_EE);
    
    // set initial prediction to measured state
    state_predicted = state_measured;

    // initalize mpc and solve first iter 
    mpc = SpillnotMPC(0.001, state_measured, v_lim, tilt_lim, r_lim, input_bound);
    mpc.solver.solveProblem();
    QPSolution = mpc.solver.getSolution();
    ctr = QPSolution.block(10 * (mpc.mpcWindow + 1), 0, 3, 1);

    // Kalman filter initial values    
    Q.setIdentity();
    Q *= 1e-3;
    R.setIdentity();
    P.setIdentity();

    std::cout << "created spillnot obj" << std::endl;
}

Eigen::Matrix<double, 10, 1> Spillnot::measure_state(const Eigen::Matrix<double, 4, 4> &O_T_EE, const Eigen::Matrix<double, 6, 1> &O_dP_EE){        

  // EE pose
  Eigen::Matrix<double, 4, 4> W_T_EE = W_T_O * O_T_EE;
  
  // EE twist
  Eigen::Matrix<double, 6, 1> W_dP_EE;
  W_dP_EE.head(3) = W_T_O.block(0, 0, 3, 3) * O_dP_EE.head(3);
  W_dP_EE.tail(3) = W_T_O.block(0, 0, 3, 3) * O_dP_EE.tail(3);

  // initialize
  Eigen::Matrix<double, 10, 1> state_measured;
  state_measured.setZero();

  // get tilted vertical
  Eigen::Matrix<double, 3, 1> V;
  V = W_T_EE.block(0, 0, 3, 3) * V0;

  // calculate angles
  double x_rot = std::atan(-V(1) / V(2)); // phi
  double y_rot = std::atan(V(0) / V(2));  // theta

  // populate state_measured
  state_measured(0) = W_T_EE(0, 3) + y_rot * mpc.l; // xp
  state_measured(1) = W_T_EE(1, 3) - x_rot * mpc.l; // yp
  state_measured(2) = W_T_EE(2, 3); // zp
  state_measured(3) = y_rot; // ry
  state_measured(4) = x_rot; // rx
  state_measured(5) = W_dP_EE(0) + W_dP_EE(4) * mpc.l; // dxp
  state_measured(6) = W_dP_EE(1) - W_dP_EE(3) * mpc.l; // dyp
  state_measured(7) = W_dP_EE(2); // dzp
  state_measured(8) = W_dP_EE(4); // dry
  state_measured(9) = W_dP_EE(3); // drx
  return state_measured;
}

Eigen::Matrix<double, 10, 1> Spillnot::filter_state(const Eigen::Matrix<double, 10, 1> &state_measured){
  P = mpc.a_robot * P * mpc.a_robot.transpose() + Q;
  Eigen::MatrixXd K = P * (P + R).inverse();
  Eigen::Matrix<double, 10, 1> state_filtered = state_predicted + K * (state_measured - state_predicted);
  P = (Eigen::MatrixXd::Identity(10, 10) - K) * P;
  return state_filtered;
}

Eigen::Matrix<double, 6, 1> Spillnot::get_twist_command(const Eigen::Vector3d &v_des, const Eigen::Matrix<double, 4, 4> &O_T_EE, const Eigen::Matrix<double, 6, 1> &O_dP_EE){
  // 0. Measure
  Eigen::Matrix<double, 10, 1> state_measured = measure_state(O_T_EE, O_dP_EE);

  // 1. Filter (Kalman)
  Eigen::Matrix<double, 10, 1> state_filtered = filter_state(state_measured);

  // 2. Update MPC state and reference
  mpc.updateConstraintVectors(state_filtered);
  mpc.updateReferenceStateVel(v_des);
  
  // 3. Solve MPC
  mpc.solver.solveProblem();
  QPSolution = mpc.solver.getSolution();
  ctr = QPSolution.block(10 * (mpc.mpcWindow + 1), 0, 3, 1);

  // 4. Sate Prediction
  state_predicted = mpc.a_robot * mpc.x0 + mpc.b_robot * ctr;
  Eigen::Matrix<double, 6, 1> W_twist = mpc.cTwist * state_predicted;


  // 5. Rotate
  Eigen::Matrix<double, 6, 1> O_twist;
  O_twist.head(3) = W_T_O.block(0, 0, 3, 3).transpose() * W_twist.head(3);
  O_twist.tail(3) = W_T_O.block(0, 0, 3, 3).transpose() * W_twist.tail(3);




  return O_twist;
}