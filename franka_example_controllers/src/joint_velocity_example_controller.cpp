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

// 变换矩阵转换为对偶四元数
DQ_robotics::DQ homogeneousTfArray2DQ(std::array<double,16> &pose){
    Eigen::Matrix3d rotationMatrixEigen;
    DQ_robotics::DQ r,p,x;
    rotationMatrixEigen << pose[0], pose[4], pose[8],
                           pose[1], pose[5], pose[9],
                           pose[2], pose[6], pose[10];    
    Eigen::Quaterniond quaternion(rotationMatrixEigen);
    r = DQ_robotics::DQ(quaternion.w(),quaternion.x(),quaternion.y(),quaternion.z());
    p = pose[12]*DQ_robotics::i_ + pose[13]*DQ_robotics::j_ + pose[14]*DQ_robotics::k_;
    x = r + DQ_robotics::E_*0.5*p*r;

    return x;
}

// 伪逆
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
        // *读取目标下的joint space
        ROS_INFO_STREAM("xd before assigment:"<<xd);
        std::vector<double> joint_goal;
        if(node_handle.getParam("joint_goal",joint_goal)||joint_goal.size()==7){
            goal = Map<RowVector7d>(joint_goal.data(),joint_goal.size());
            ROS_INFO_STREAM("goal:"<<goal);
        }else{
            ROS_ERROR("JointVelocityController: Could not get parameter joint_goal");
            // ROS_INFO_STREAM("Jointgoal:"<<joint_goal);
            return false;
        }
        xd = fep.fkm(goal); //不能放到starting里，否则控制防盗器会挂掉
        ROS_INFO_STREAM("xd after assignment:"<<xd);

        // ! 用于轨迹规划>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // // *创建轨迹(运用motion profile)
        // robot_state = state_handle_->getRobotState(); //get robotstate

        // std::array<double, 16> T_end_c;     //读取目标EE位姿
        // std::vector<double> target_pose;
        // if(node_handle.getParam("target_pose",target_pose) && target_pose.size()==16){
        //     for(int i=0; i<16; i++){
        //         T_end_c[i] = target_pose[i];
        //         // ROS_INFO_STREAM("T_end_c"<<i <<":"<<T_end_c[i]);
        //     }       
        // }else{
        //     ROS_ERROR("JointVelocityController: Could not get parameter T_end_c");
        //     return false;
        // }

        // std::array<double, 16> T_start_c = robot_state.O_T_EE_c;    //读取当前EE位姿

        // Vector6d initial_pose = homogeneousTfArray2PoseVec(T_start_c);  // 将位姿转为(3 translations, 3 RPY rotations) 
        // Vector6d end_pose = homogeneousTfArray2PoseVec(T_end_c);
        // traj = new LinearTrajectory(initial_pose, end_pose, 0.05,0.5,1.e-3);
        // traj_Car = new TrajectoryIteratorCartesian(*traj);
        // ! 用于轨迹规划<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<
        
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
        // ! 用于轨迹规划>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>>
        // // * calculate the error between EE's target pose and EE's current pose
        // robot_state = state_handle_->getRobotState(); //get robotstate
        // std::array<double, 16> targetPose = traj_Car->getCartesianPose();
        // std::array<double, 16> currentPose = robot_state.O_T_EE_c;    
        // x = homogeneousTfArray2DQ(currentPose);
        // xd = homogeneousTfArray2DQ(targetPose);
        // e = DQ_robotics::vec8(x - xd);  

        // // * calculate the norm of the error
        // e_norm_old = e_norm;
        // e_norm = e.norm();
        // ! 用于轨迹规划<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<

        // * calculate the jacobian
        robot_state = state_handle_->getRobotState(); //get robotstate
        for(int i=0; i<7; i++){
            q.coeffRef(i)  = robot_state.q[i];
        }
        // ! 不用轨迹**********
        x = fep.fkm(q);
        e = DQ_robotics::vec8(x-xd);
        e_norm = e.norm();
        // ! 不用轨迹**********
        J=fep.pose_jacobian(q);                                 // 8x7
        J_pinv = pseudoInverse(J);                              // 7x8
        JJ = J_pinv*J;                                          // 7x7
        MatrixXd I = MatrixXd::Identity(JJ.rows(),JJ.cols());   // 7X7
        MatrixXd N = I-JJ;                                      // 7X7


        if(e_norm > 0.001){
            // * calculatet the controller
            k=0.4;
            u = -J_pinv*k*e.transpose()+N*d*(q_c.transpose()-q.transpose()); //7x1
            // * output the command
            for (int i=0; i<7; i++) {
                velocity_joint_handles_[i].setCommand(u[i]);
            }
        }else{
            u.setZero();
            for (int i=0; i<7; i++) {
                velocity_joint_handles_[i].setCommand(u[i]);
            };
            ROS_INFO_STREAM("flag"<<flag);

            /* 
                ! 不加下面的完全没问题，否则会报错：
                [ERROR] [1693492510.001093386]: libfranka: Move command aborted: motion aborted by reflex! ["communication_constraints_violation"] 
                control_command_success_rate: -16.13 packets lost in a row in the last sample: 1713
            */ 
           
            // if(flag ==0){
            //     flag += 1;
            //     // actionlib::SimpleActionClient<franka_gripper::GraspAction> grasp_client("/franka_gripper/grasp",true);
            //     // actionlib::SimpleActionClient<franka_gripper::HomingAction> homing_client("/franka_gripper/homing",true);
            //     // actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("/franka_gripper/move",true);
            //     // actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client("/franka_gripper/stop",true);

            //     // *********************** grasp ***********************
            //     ROS_INFO("Waiting for action server to start.");
            //     grasp_client.waitForServer();
            //     stop_client.waitForServer();
            //     ROS_INFO("Action server started, sending goal.");
            //     // Grasp object
            //     franka_gripper::GraspGoal grasp_goal;
            //     // grap action的goal
            //     grasp_goal.force = 10;  //N
            //     grasp_goal.speed = 0.1; ///m/s
            //     grasp_goal.epsilon.inner = 0.005; //m
            //     grasp_goal.epsilon.outer = 0.005; //m

            //     grasp_client.sendGoal(grasp_goal);
            //     if (grasp_client.waitForResult(ros::Duration(5.0))) {
            //     ROS_INFO("teleop_gripper_node: GraspAction was successful.");
            //     } else {
            //     ROS_INFO("teleop_gripper_node: GraspAction was not successful.");
            //     stop_client.sendGoal(franka_gripper::StopGoal());
            //     }

            //     // new goal
            //     // RowVector7d new_goal;
            //     // new_goal << 0.000787047, -0.785144, -0.00124833, -2.35626, 0.00198855, 1.57248, 0.791138;
            //     // xd = fep.fkm(new_goal);
            // }
            // else if(flag == 1){
            if(flag==0){
                flag += 1;
                // *********************** move ***********************
                actionlib::SimpleActionClient<franka_gripper::MoveAction> move_client("/franka_gripper/move",true);
                actionlib::SimpleActionClient<franka_gripper::StopAction> stop_client("/franka_gripper/stop",true);
                ROS_INFO("Waiting for action server to start.");
                move_client.waitForServer();
                stop_client.waitForServer();
                ROS_INFO("Action server started, sending goal.");
                // Open gripper
                franka_gripper::MoveGoal move_goal;
                move_goal.speed = 0.1;  // m/s
                move_goal.width = 0.06; // m 最大0.07
        
                move_client.sendGoal(move_goal);
                if (move_client.waitForResult(ros::Duration(5.0))) {
                    ROS_INFO("teleop_gripper_node: MoveAction was successful.");
                } else {
                    ROS_ERROR("teleop_gripper_node: MoveAction was not successful.");
                    stop_client.sendGoal(franka_gripper::StopGoal());
                }
            }

            
        }

        // * iter step +1
        // traj_Car->step();

        // *** debug info ***
        // if(flag<100){
        //     // ROS_INFO_STREAM("iteration:"<<flag );
        //     // ROS_INFO_STREAM("x:" <<x);
        //     // ROS_INFO_STREAM("xd:" <<xd);
        //     // ROS_INFO_STREAM("q:"<<q);
        //     ROS_INFO_STREAM("e:"<<e);
        //     ROS_INFO_STREAM("e_norm:"<<e_norm);
        //     ROS_INFO_STREAM("e_norm - e_norm_old:"<<e_norm - e_norm_old);
        //     flag++;
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
