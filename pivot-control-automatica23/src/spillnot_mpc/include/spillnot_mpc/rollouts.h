// Online rollouts orch
// Author: Rafael Cabral

#pragma once

#include <Eigen/Dense>

#include "pinocchio/parsers/urdf.hpp"
#include <pinocchio/multibody/data.hpp>

// #include <pinocchio/multibody/model.hpp>
// #include <spillnot_mpc/spillnot_task.h>


class Rollouts {
  public:

    Rollouts(const std::string urdf_filename);

    void fkm();

    // // spillnot
    // Spillnot *sn;

    // robot model (create data object at each rollout loop, do not find how to declare without model or as pointer)
    pinocchio::Model model;
    pinocchio::DataTpl<double, 0, pinocchio::JointCollectionDefaultTpl> data;

    const std::string urdf_filename;

    Eigen::Vector3d v_des;
    double max_time;


    // // high level approach description
    // v_min=0, v_des, v in k*v_des, k in [0, 1]
    // binary search during N_bs steps to find largest k with valid rollout wrt. q, dq
    // get trajectory with terminal constraints (task space spillnot mpc)
    // rollout check (robot model pinocchio diff IK)


};


