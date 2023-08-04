// Copyright (c) 2017 Franka Emika GmbH
// Use of this source code is governed by the Apache-2.0 license, see LICENSE
#include <franka_example_controllers/joint_velocity_example_controller.h>

#include <cmath>

#include <controller_interface/controller_base.h>
#include <hardware_interface/hardware_interface.h>
#include <hardware_interface/joint_command_interface.h>
#include <pluginlib/class_list_macros.h>
#include <ros/ros.h>

// some parameters for the controller
double gain = -1.5;
double d = 0.1;
double k = 0.1;

template<typename _Matrix_Type_>
_Matrix_Type_ pseudoInverse(const _Matrix_Type_ &a, double epsilon = std::numeric_limits<double>::epsilon())
{
    // Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeFullU | Eigen::ComputeFullV); // For a square matrix
    Eigen::JacobiSVD< _Matrix_Type_ > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);    // For a non-square matrix
    double tolerance =epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
    return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}

namespace franka_example_controllers {
    bool JointVelocityExampleController::init(hardware_interface::RobotHW* robot_hardware,ros::NodeHandle& node_handle){
        velocity_joint_interface_ = robot_hardware->get<hardware_interface::VelocityJointInterface>();
        if (velocity_joint_interface_ == nullptr) {
            ROS_ERROR(
            "JointVelocityExampleController: Error getting velocity joint interface from hardware!");
            return false;
        }

        std::string arm_id;
        if (!node_handle.getParam("arm_id", arm_id)) {
            ROS_ERROR("JointVelocityExampleController: Could not get parameter arm_id");
            return false;
        }

        std::vector<std::string> joint_names;
        if (!node_handle.getParam("joint_names", joint_names)) {
            ROS_ERROR("JointVelocityExampleController: Could not parse joint names");
        }
        if (joint_names.size() != 7) {
            ROS_ERROR_STREAM("JointVelocityExampleController: Wrong number of joint names, got "<< joint_names.size() << " instead of 7 names!");
            return false;
        }
        velocity_joint_handles_.resize(7);
        for (size_t i = 0; i < 7; ++i) {
            try {
                velocity_joint_handles_[i] = velocity_joint_interface_->getHandle(joint_names[i]);
            }catch(const hardware_interface::HardwareInterfaceException& ex) {
                ROS_ERROR_STREAM("JointVelocityExampleController: Exception getting joint handles: " << ex.what());
                return false;
            }
        }

        auto state_interface = robot_hardware->get<franka_hw::FrankaStateInterface>();
        if (state_interface == nullptr) {
            ROS_ERROR("JointVelocityExampleController: Could not get state interface from hardware");
            return false;
        }

        try {
            //**************** edit start ****************
            state_handle_ = std::make_unique<franka_hw::FrankaStateHandle>(state_interface->getHandle(arm_id + "_robot"));
            //**************** edit end ****************
            // auto state_handle = state_interface->getHandle(arm_id + "_robot");
            // std::array<double, 7> q_start{{0, -M_PI_4, 0, -3 * M_PI_4, 0, M_PI_2, M_PI_4}};
            // for (size_t i = 0; i < q_start.size(); i++) {
            //   if (std::abs(state_handle.getRobotState().q_d[i] - q_start[i]) > 0.1) {
            //     ROS_ERROR_STREAM(
            //         "JointVelocityExampleController: Robot is not in the expected starting position for "
            //         "running this example. Run `roslaunch franka_example_controllers move_to_start.launch "
            //         "robot_ip:=<robot-ip> load_gripper:=<has-attached-gripper>` first.");
            //     return false;
            //   }
            // }
        } catch (const hardware_interface::HardwareInterfaceException& e) {
            ROS_ERROR_STREAM(
            "JointVelocityExampleController: Exception getting state handle: " << e.what());
            return false;
        }

        //**************** edit start ****************
        ROS_INFO_STREAM("xd before assigment:"<<xd);
        // goal <<  -M_PI/2.0,   0.004,       0.0,  -1.57156,       0.0,   1.57075,       0.0;
        // goal << 0.968844,  0.305047,   -0.452106,  -1.89069,   0.0577989,   2.24276,   1.39396;
        // goal << 0.0120356, 0.0681359, -0.430619,  -1.91028,  0.123545,   2.05192,  0.296204;
        std::vector<double> joint_goal;
        if(node_handle.getParam("joint_goal",joint_goal)||joint_goal.size()==7){
            goal = Map<RowVector7d>(joint_goal.data(),joint_goal.size());
            ROS_INFO_STREAM("goal:"<<goal);
        }else{
            ROS_ERROR("JointVelocityExampleController: Could not get parameter joint_goal");
            // ROS_INFO_STREAM("Jointgoal:"<<joint_goal);
            return false;
        }
        xd = fep.fkm(goal); //不能放到starting里，否则控制防盗器会挂掉
        ROS_INFO_STREAM("xd after assignment:"<<xd);

        // *generate the trajectory
        // 1.创建轨迹实例
        traj = new PolynomialTrajectory();
        // 2.计算整体所需要的时间
        robot_state = state_handle_->getRobotState();
        for(int i=0; i<7; i++){
            q_s.coeffRef(i)  = robot_state.q[i];
        }
        std::copy(goal.transpose().begin(),goal.transpose().end(),q_f.begin());
        traj->CalculateFinishedTime(q_f, q_s, q_f-q_s);
        // 3.设置约束   
        t_s = {0,0,0,0,0,0,0};                           
        v_s = {0,0,0,0,0,0,0};
        a_s = {0,0,0,0,0,0,0};
        t_f = traj->t_f_sync_;
        v_f = {0,0,0,0,0,0,0};
        a_f = {0,0,0,0,0,0,0};
        // 4.计算五次多项式的系数
        traj->CalculateCoefficient(t_s,q_s,v_s,a_s,t_f,q_f,v_f,a_f);
        // 5.计算每一迭代步的目标
        // traj.CalculateTrajectory(t,t_s);
        //**************** edit end ****************
        return true;
    }

    void JointVelocityExampleController::starting(const ros::Time& /* time */) {
        elapsed_time_ = ros::Duration(0.0);
        q_min << -2.8973,   -1.7628,   -2.8973,   -3.0718,   -2.8973,   -0.0175,   -2.8973;
        q_max <<  2.8973,    1.7628,    2.8973,   -0.0698,    2.8973,    3.7525,    2.8973;
        q_c = 0.5*(q_min+q_max);
        e << 1.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0; //error
        robot_state = state_handle_->getRobotState(); //get robotstate

        flag = 0;
        e_norm =0;
        e_norm_old=0;
    }

    void JointVelocityExampleController::update(const ros::Time&,const ros::Duration& period) {
        robot_state = state_handle_->getRobotState(); //get robotstate
        // 如果直接一串可以用流<<，否则用索引
        for(int i=0; i<7; i++){
            q.coeffRef(i)  = robot_state.q[i];
        }
        x=fep.fkm(q);
        traj->CalculateTrajectory(t,t_s);
        xd = fep.fkm(traj->q.transpose());
        e = DQ_robotics::vec8(x - xd);  // 注意e是8维的，对应2个四元数

        e_norm_old = e_norm;
        e_norm = e.norm();

        J=fep.pose_jacobian(q); // 8x7
        J_pinv = pseudoInverse(J); // 7x8
        JJ = J_pinv*J; // 7x7
        MatrixXd I = MatrixXd::Identity(JJ.rows(),JJ.cols()); //7X7
        MatrixXd N = I-JJ;  //7X7

        k=0.1;
        u = -J_pinv*k*e.transpose()+N*d*(q_c.transpose()-q.transpose()); //7x1
        if(e.norm()<0.01) u.setZero(); 
        // u = -J_pinv*k*e.transpose(); //7x1

        // gain = -0.01;
        // u = gain*J_pinv*e.transpose();

        for (int i=0; i<7; i++) {
            velocity_joint_handles_[i].setCommand(u[i]);
        }

        // *** debug info ***
        if(flag%10==0){
            ROS_INFO_STREAM("iteration:"<<flag );
            // ROS_INFO_STREAM("x:" <<x);
            // ROS_INFO_STREAM("xd:" <<xd);
            ROS_INFO_STREAM("goal:"<<goal);
            ROS_INFO_STREAM("q:"<<q);
            ROS_INFO_STREAM("e:"<<e);
            ROS_INFO_STREAM("e_norm:"<<e_norm);
            ROS_INFO_STREAM("e_norm - e_norm_old:"<<e_norm - e_norm_old);
            // ROS_INFO_STREAM("J:"<<J);
            // ROS_INFO_STREAM("J_pinv:"<<J_pinv);
            // ROS_INFO_STREAM("JJ:"<<JJ);
            // ROS_INFO_STREAM("I:"<<I);
            // ROS_INFO_STREAM("N:"<<N);
            // ROS_INFO_STREAM("u:"<<u);
            flag++;
        }
        t += period.toSec();
        // elapsed_time_ += period;
        // // ROS_INFO_STREAM("xd!!!!!!!!!!:"<<xd);
        // ros::Duration time_max(8.0);
        // double omega_max = 0.1;
        // double cycle = std::floor(
        //     std::pow(-1.0, (elapsed_time_.toSec() - std::fmod(elapsed_time_.toSec(), time_max.toSec())) /
        //                        time_max.toSec()));
        // double omega = cycle * omega_max / 2.0 *
        //                (1.0 - std::cos(2.0 * M_PI / time_max.toSec() * elapsed_time_.toSec()));

        // for (auto joint_handle : velocity_joint_handles_) {
        //   joint_handle.setCommand(omega);
        // }
    }

    void JointVelocityExampleController::stopping(const ros::Time& /*time*/) {
        // WARNING: DO NOT SEND ZERO VELOCITIES HERE AS IN CASE OF ABORTING DURING MOTION
        // A JUMP TO ZERO WILL BE COMMANDED PUTTING HIGH LOADS ON THE ROBOT. LET THE DEFAULT
        // BUILT-IN STOPPING BEHAVIOR SLOW DOWN THE ROBOT.
    }

}  // namespace franka_example_controllers

PLUGINLIB_EXPORT_CLASS(franka_example_controllers::JointVelocityExampleController,
          controller_interface::ControllerBase)
