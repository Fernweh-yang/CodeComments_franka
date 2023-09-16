// implements the spillnot model predictive control for trajectory generation.
// in this case the tracking is only for position trajectories, i.e.,
// the desired velocity is implicitly obtained from the desired positions
// according to the state equations. Velocity tracking is implemented as an
// extrapolation of the current position in direction of the desired velocity.
// Author: Rafael Cabral

/**
 * adapted from
 * https://github.com/robotology/osqp-eigen/blob/master/example/src/MPCExample.cpp
 */

// osqp-eigen
#include "OsqpEigen/OsqpEigen.h"

// eigen
#include <Eigen/Dense>
#include <unsupported/Eigen/MatrixFunctions>

#include <fstream>
#include <iostream>

class SpillnotMPC {
public:
  static const int stateDim = 10;
  static const int controlDim = 3;
  static const int outputDim = 3;

  double l = 0.6;         // m, pendulum rod length
  double T_mpc = 0.03;    // s, discretization time step in mpc 0.05
  double T_robot = 0.001; // s, control time step
  static const int mpcWindow = 64;
  double inputCost = 0.;
  double stateDiagCost = 0.1;

  // allocate the system parameters
  double g = 9.81; // m/s2

  // allocate the dynamics matrices
  Eigen::Matrix<double, stateDim, stateDim> a;
  Eigen::Matrix<double, stateDim, controlDim> b;
  Eigen::Matrix<double, outputDim, stateDim> c;
  Eigen::Matrix<double, 6, stateDim> cTwist;

  // allocate the discrete dynamics matrices for mpc
  Eigen::Matrix<double, stateDim, stateDim> a_robot;
  Eigen::Matrix<double, stateDim, controlDim> b_robot;

  // allocate the discrete dynamics matrices for state propagation
  Eigen::Matrix<double, stateDim, stateDim> a_mpc;
  Eigen::Matrix<double, stateDim, controlDim> b_mpc;

  // allocate the constraints vector
  Eigen::Matrix<double, stateDim, 1> xMax;
  Eigen::Matrix<double, stateDim, 1> xMin;
  Eigen::Matrix<double, controlDim, 1> uMax;
  Eigen::Matrix<double, controlDim, 1> uMin;
  Eigen::Matrix<double, 2, 1> angleJerkMin;
  Eigen::Matrix<double, 2, 1> angleJerkMax;

  // allocate the weight matrices
  Eigen::Matrix<double, stateDim, stateDim> Q;
  Eigen::Matrix<double, stateDim, stateDim> Q_terminal;
  Eigen::DiagonalMatrix<double, controlDim> R;

  // allocate the initial and the reference state space
  Eigen::Matrix<double, stateDim, 1> x0;
  // x0.setZero();

  // state inequality constraints
  double v_lim = 0.85;
  double tilt_lim = 0.5;
  double r_lim = 1.75;
  double input_bound = 0.5; // param

  Eigen::Matrix<double, outputDim, mpcWindow + 1> yRef;
  // yRef.setZero();

  bool verbose = false;
  double timeLimit = 0.0002;

  // allocate QP problem matrices and vectores
  Eigen::SparseMatrix<double> hessian;
  Eigen::VectorXd gradient;
  Eigen::SparseMatrix<double> constraintMatrix;
  Eigen::VectorXd lowerBound;
  Eigen::VectorXd upperBound;

  OsqpEigen::Solver solver;
  SpillnotMPC() {
    setModel();
    x0.setZero();

    yRef.setZero();
    yRef.colwise() = c * x0;

    // setQP();
    // setSolver();
  };

  SpillnotMPC(double T_robot, Eigen::Matrix<double, stateDim, 1> x0,
              double v_lim, double tilt_lim, double r_lim, double input_bound)
      : T_robot(T_robot), x0(x0), v_lim(v_lim), tilt_lim(tilt_lim),
        r_lim(r_lim), input_bound(input_bound) {
    setModel();

    yRef.setZero();
    yRef.colwise() = c * x0;

    setQP();
    setSolver();
  }

  // SpillnotMPC(double l, double T_mpc, double T_robot, int mpcWindow,
  //             double inputCost, double stateDiagCost,
  //             Eigen::Matrix<double, stateDim, 1> x0, double input_bound,
  //             bool verbose, double timeLimit)
  //     : l(l), T_mpc(T_mpc), T_robot(T_robot), mpcWindow(mpcWindow),
  //       inputCost(inputCost), stateDiagCost(stateDiagCost), x0(x0),
  //       input_bound(input_bound), verbose(verbose), timeLimit(timeLimit) {
  //   setModel();
  //   yRef = c * x0;
  //   setQP();
  //   setSolver();
  // }

  void setModel() {
    // set model mpc
    setDynamicsMatrices();
    setInequalityConstraints();
    setWeightMatrices();
  }

  void setQP() {
    // cast the MPC problem as QP problem
    castMPCToQPHessian();
    castMPCToQPGradient();
    castMPCToQPConstraintMatrix();
    castMPCToQPConstraintVectors();
  }

  void setSolver() {
    setSolverSettings();
    // todo: error handling for solver init
    setInitialSolverData();
    initSolver();
  }

  void setSolverSettings() {
    // initial solver settings
    // https://osqp.org/docs/interfaces/solver_settings.html#solver-settings
    solver.settings()->setVerbosity(verbose);
    solver.settings()->setTimeLimit(timeLimit);
  }

  int setInitialSolverData() {
    // set the initial data of the QP solver
    solver.data()->setNumberOfVariables(10 * (mpcWindow + 1) + 3 * mpcWindow);
    solver.data()->setNumberOfConstraints(2 * 10 * (mpcWindow + 1) +
                                          3 * mpcWindow + 2 * (mpcWindow - 1));
    if (!solver.data()->setHessianMatrix(hessian))
      return 1;
    if (!solver.data()->setGradient(gradient))
      return 1;
    if (!solver.data()->setLinearConstraintsMatrix(constraintMatrix))
      return 1;
    if (!solver.data()->setLowerBound(lowerBound))
      return 1;
    if (!solver.data()->setUpperBound(upperBound))
      return 1;
    return 0;
  }

  int initSolver() {
    // instantiate the solver
    if (!solver.initSolver())
      return 1;
    return 0;
  }

  void setDynamicsMatrices() {
    // set continuous dynamic matrices
    // System Matrix
    a.setZero();
    a.block(0, 5, 5, 5).setIdentity();
    a(8, 3) = -g / l;
    a(9, 4) = -g / l;

    // Control Matrix
    b.setZero();
    b.block(5, 0, 3, 3).setIdentity();
    b(8, 0) = 1 / l;
    b(9, 1) = -1 / l;

    // Output Matrix
    c.setZero();
    c.block(0, 0, 3, 3).setIdentity();
    c(0, 3) = -l;
    c(1, 4) = l;
    // c.block(3, 5, 3, 5) = c.block(0, 0, 3, 5); to include velocity in output

    // Twist Output Matrix (for cartesian velocity control)
    cTwist.setZero();
    cTwist.block(0, 5, 3, 3).setIdentity();
    cTwist(0, 8) = -l;
    cTwist(1, 9) = l;
    cTwist(3, 9) = 1;
    cTwist(4, 8) = 1;

    // Zero-order hold discretization of continuous time state space
    // https://kyuhwanyeon.github.io/control/ZOHdecretization/
    a_mpc = (a * T_mpc).exp();
    b_mpc = discretizeControlMatrix(T_mpc);

    a_robot = (a * T_robot).exp();
    b_robot = discretizeControlMatrix(T_robot);
  }

  Eigen::Matrix<double, stateDim, controlDim>
  discretizeControlMatrix(double T) {
    Eigen::Matrix<double, 10, 10> mid; // helper matrix
    mid = mid.setIdentity() * T;
    double factorial = 1.0;
    for (int i = 1; i < 10; i++) {
      factorial *= (i + 1);
      mid += a.pow(i) * pow(T, i + 1) / factorial;
    }
    return mid * b;
  }

  void setInequalityConstraints() {
    // input inequality constraints
    uMin << -input_bound, -input_bound, -input_bound;

    uMax << input_bound, input_bound, input_bound;

    xMin << -OsqpEigen::INFTY, -OsqpEigen::INFTY, -OsqpEigen::INFTY, -tilt_lim,
        -tilt_lim, -v_lim, -v_lim, -v_lim, -r_lim, -r_lim;

    xMax << OsqpEigen::INFTY, OsqpEigen::INFTY, OsqpEigen::INFTY, tilt_lim,
        tilt_lim, v_lim, v_lim, v_lim, r_lim, r_lim;

    double angleJerkLim = 12500. * pow(T_mpc, 2);
    angleJerkMin << -angleJerkLim, -angleJerkLim;
    angleJerkMax << angleJerkLim, angleJerkLim;
  }

  void setWeightMatrices() {

    Eigen::Matrix<double, 10, 10> angles_selector;
    angles_selector.setZero();
    angles_selector.diagonal() << 0, 0, 0, 1, 1, 0, 0, 0, 1, 1;

    Q = c.transpose() * c + stateDiagCost * angles_selector;

    double r = inputCost;
    R.diagonal() << r, r, r;
  }

  void castMPCToQPHessian() {

    hessian.resize(10 * (mpcWindow + 1) + 3 * mpcWindow,
                   10 * (mpcWindow + 1) + 3 * mpcWindow);

    // populate hessian matrix
    for (int i = 0; i < 10 * (mpcWindow + 1) + 3 * mpcWindow; i++) {
      if (i < 10 * (mpcWindow + 1)) {
        int posQ = i % 10;
        if (posQ == 0) {
          // block write (not available for eigen sparse matrix)
          for (int j = 0; j < 10; j++) {
            for (int k = 0; k < 10; k++) {
              hessian.insert(i + j, i + k) = Q(j, k);
            }
          }
        }
      } else {
        int posR = i % 3;
        float value = R.diagonal()[posR];
        if (value != 0)
          hessian.insert(i, i) = value;
      }
    }
  }

  void castMPCToQPGradient() {

    // populate the gradient vector
    gradient = Eigen::VectorXd::Zero(10 * (mpcWindow + 1) + 3 * mpcWindow, 1);

    for (int wi = 0; wi < yRef.cols(); wi++) {
      Eigen::Matrix<double, 10, 1> Qx_ref;
      Eigen::Matrix<double, 10, 1> x0Ref;
      x0Ref.setZero();
      x0Ref.head(3) = yRef.block(0, wi, 3, 1);
      Qx_ref = Q * (-x0Ref);

      for (int i = 0; i < 10; i++) {
        gradient(10 * wi + i, 0) = Qx_ref(i, 0);
      }
    }
  }

  void castMPCToQPConstraintMatrix() {
    constraintMatrix.resize(10 * (mpcWindow + 1) + 10 * (mpcWindow + 1) +
                                3 * mpcWindow + 2 * (mpcWindow - 1),
                            10 * (mpcWindow + 1) + 3 * mpcWindow);

    // populate linear constraint matrix
    for (int i = 0; i < 10 * (mpcWindow + 1); i++) {
      constraintMatrix.insert(i, i) = -1;
    }

    for (int i = 0; i < mpcWindow; i++)
      for (int j = 0; j < 10; j++)
        for (int k = 0; k < 10; k++) {
          float value = a_mpc(j, k);
          if (value != 0) {
            constraintMatrix.insert(10 * (i + 1) + j, 10 * i + k) = value;
          }
        }

    for (int i = 0; i < mpcWindow; i++)
      for (int j = 0; j < 10; j++)
        for (int k = 0; k < 3; k++) {
          float value = b_mpc(j, k);
          if (value != 0) {
            constraintMatrix.insert(10 * (i + 1) + j,
                                    3 * i + k + 10 * (mpcWindow + 1)) = value;
          }
        }

    // state and input bounds
    for (int i = 0; i < 10 * (mpcWindow + 1) + 3 * mpcWindow; i++) {
      constraintMatrix.insert(i + (mpcWindow + 1) * 10, i) = 1;
    }

    // tilt acceleration bounds
    for (int i = 1; i < mpcWindow - 1; i++) {
      constraintMatrix.insert(2 * i + 2 * (mpcWindow + 1) * 10 + 3 * mpcWindow,
                              i * 10 + 8) = 1;
      constraintMatrix.insert(2 * i + 2 * (mpcWindow + 1) * 10 + 3 * mpcWindow,
                              i * (10 + 1) + 8) = -2;
      constraintMatrix.insert(2 * i + 2 * (mpcWindow + 1) * 10 + 3 * mpcWindow,
                              i * (10 + 2) + 8) = 1;

      constraintMatrix.insert(
          2 * i + 1 + 2 * (mpcWindow + 1) * 10 + 3 * mpcWindow, i * 10 + 9) = 1;
      constraintMatrix.insert(2 * i + 1 + 2 * (mpcWindow + 1) * 10 +
                                  3 * mpcWindow,
                              i * (10 + 1) + 9) = -2;
      constraintMatrix.insert(2 * i + 1 + 2 * (mpcWindow + 1) * 10 +
                                  3 * mpcWindow,
                              i * (10 + 2) + 9) = 1;
    }
  }

  void castMPCToQPConstraintVectors() {
    // evaluate the lower and the upper inequality vectors
    Eigen::VectorXd lowerInequality = Eigen::MatrixXd::Zero(
        10 * (mpcWindow + 1) + 3 * mpcWindow + 2 * (mpcWindow - 1), 1);
    Eigen::VectorXd upperInequality = Eigen::MatrixXd::Zero(
        10 * (mpcWindow + 1) + 3 * mpcWindow + 2 * (mpcWindow - 1), 1);

    for (int i = 0; i < mpcWindow + 1; i++) {
      lowerInequality.block(10 * i, 0, 10, 1) = xMin;
      upperInequality.block(10 * i, 0, 10, 1) = xMax;
    }
    for (int i = 0; i < mpcWindow; i++) {
      lowerInequality.block(3 * i + 10 * (mpcWindow + 1), 0, 3, 1) = uMin;
      upperInequality.block(3 * i + 10 * (mpcWindow + 1), 0, 3, 1) = uMax;
    }

    // tilt acceleration bounds
    for (int i = 0; i < mpcWindow - 1; i++) {
      lowerInequality.block(2 * i + (mpcWindow + 1) * 10 + 3 * mpcWindow, 0, 2,
                            1) = angleJerkMin;
      upperInequality.block(2 * i + (mpcWindow + 1) * 10 + 3 * mpcWindow, 0, 2,
                            1) = angleJerkMax;
    }

    // evaluate the lower and the upper equality vectors
    Eigen::VectorXd lowerEquality =
        Eigen::MatrixXd::Zero(10 * (mpcWindow + 1), 1);
    Eigen::VectorXd upperEquality;
    lowerEquality.block(0, 0, 10, 1) = -x0;

    upperEquality = lowerEquality;
    lowerEquality = lowerEquality;

    // merge inequality and equality vectors
    lowerBound = Eigen::MatrixXd::Zero(
        2 * 10 * (mpcWindow + 1) + 3 * mpcWindow + 2 * (mpcWindow - 1), 1);
    lowerBound << lowerEquality, lowerInequality;

    upperBound = Eigen::MatrixXd::Zero(
        2 * 10 * (mpcWindow + 1) + 3 * mpcWindow + 2 * (mpcWindow - 1), 1);
    upperBound << upperEquality, upperInequality;
  }

  // initial state equality constraint
  int updateConstraintVectors(Eigen::Matrix<double, stateDim, 1> x0_) {
    x0 = x0_;
    lowerBound.block(0, 0, 10, 1) = -x0_;
    upperBound.block(0, 0, 10, 1) = -x0_;
    if (!solver.updateBounds(lowerBound, upperBound))
      return 1;
    return 0;
  }

  // initial state equality constraint
  // and velocity limits through current J(q) of current q config
  int updateConstraintVectorsEI(Eigen::Matrix<double, stateDim, 1> x0_,
                                Eigen::Matrix<double, 6, 1> twist_lim) {
    // initial equality
    x0 = x0_;
    lowerBound.block(0, 0, 10, 1) = -x0_;
    upperBound.block(0, 0, 10, 1) = -x0_;

    // inequality update
    Eigen::Matrix<double, 10, 1> new_x_bound;
    new_x_bound.setZero();
    // same bound for tilt and position
    new_x_bound.head(5) = upperBound.block(10 * (mpcWindow + 1), 0, 5, 1);
    // new bounds for velocity and tilt rate
    // aware that it is pivot velocity and not eef velocity, inaccuracy
    new_x_bound(5) = twist_lim(0);
    new_x_bound(6) = twist_lim(1);
    new_x_bound(7) = twist_lim(2);
    new_x_bound(8) = twist_lim(4);
    new_x_bound(9) = twist_lim(3);

    Eigen::Matrix<double, 10 * (mpcWindow + 1), 1> nxb;
    nxb.setZero();
    for (int i = 0; i < (mpcWindow + 1); i++) {
      for (int j = 0; j < 10; j++) {
        nxb(10 * i + j) = new_x_bound(j);
      }
    }

    std::cout << "max nxb: " << nxb.maxCoeff() << std::endl;
    std::cout << "min nxb: " << nxb.minCoeff() << std::endl;

    upperBound.block(10 * (mpcWindow + 1), 0, 10 * (mpcWindow + 1), 1) = nxb;
    lowerBound.block(10 * (mpcWindow + 1), 0, 10 * (mpcWindow + 1), 1) = -nxb;

    if (!solver.updateBounds(lowerBound, upperBound))
      return 1;
    return 0;
  }

  int updateReferenceStatePos(Eigen::Matrix<double, outputDim, 1> yPosRef_) {
    // set all columns of yRef to the desired reference position
    yRef.setZero();
    yRef.colwise() = yPosRef_;
    castMPCToQPGradient();
    if (!solver.updateGradient(gradient))
      return 1;
    return 0;
  }

  int updateReferenceStateVel(Eigen::Matrix<double, outputDim, 1> yVelRef_) {
    // populate yRef_ from desired velocity by extrapolating the current
    // position in direction of des vel.
    Eigen::Matrix<double, outputDim, mpcWindow + 1> v_des;

    v_des.colwise() = yVelRef_;

    // create time vector
    Eigen::Matrix<double, mpcWindow + 1, mpcWindow + 1> t_linspace;
    t_linspace.setZero();
    t_linspace.diagonal() =
        Eigen::VectorXd::LinSpaced(mpcWindow + 1, 0, T_mpc * (mpcWindow));

    // populate yRef with current position
    yRef.colwise() = c * x0;

    // add displacement in v_des direction
    yRef += v_des * t_linspace;

    castMPCToQPGradient();
    if (!solver.updateGradient(gradient))
      return 1;
    return 0;
  }

  int updateReferenceStateTraj(
      Eigen::Matrix<double, outputDim, mpcWindow + 1> yRef_) {
    yRef = yRef_;
    castMPCToQPGradient();
    if (!solver.updateGradient(gradient))
      return 1;
    return 0;
  }
  double getErrorNorm(const Eigen::Matrix<double, 10, 1> &x) {
    // evaluate the error
    // todo: error dependent on control modality
    Eigen::Matrix<double, outputDim, 1> error = c * x - yRef.block(0, 0, 3, 1);

    // return the norm
    return error.norm();
  }
};