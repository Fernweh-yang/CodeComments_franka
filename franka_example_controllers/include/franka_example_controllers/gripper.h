#pragma once

#include "ros/duration.h"
#include "actionlib/client/simple_action_client.h"
#include <franka/gripper.h>
#include <franka_gripper/GraspAction.h>
#include <franka_gripper/HomingAction.h>
#include <franka_gripper/MoveAction.h>
#include <franka_gripper/StopAction.h>


class FrankaGripper {
    
    public:
        actionlib::SimpleActionClient<franka_gripper::HomingAction> *home_client;
        actionlib::SimpleActionClient<franka_gripper::GraspAction> *grasp_client;
        actionlib::SimpleActionClient<franka_gripper::StopAction> *stop_client;
        actionlib::SimpleActionClient<franka_gripper::MoveAction> *move_client;
        
        FrankaGripper() {
            grasp_client = new actionlib::SimpleActionClient<franka_gripper::GraspAction>("/franka_gripper/grasp", false);
            stop_client = new actionlib::SimpleActionClient<franka_gripper::StopAction>("/franka_gripper/stop", false);
            move_client = new actionlib::SimpleActionClient<franka_gripper::MoveAction>("/franka_gripper/move", false);
            home_client = new actionlib::SimpleActionClient<franka_gripper::HomingAction>("/franka_gripper/homing", false);
        }

        void grasp(actionlib::SimpleActionClient<franka_gripper::GraspAction> &grasp_client, const double width, const double grasp_force,
                const double move_speed, const double grasp_epsilon_inner, const double grasp_epsilon_outer)
        {
            // Grasp object
            franka_gripper::GraspGoal grasp_goal;
            grasp_goal.width         = width;
            grasp_goal.force         = grasp_force;
            grasp_goal.speed         = move_speed;
            grasp_goal.epsilon.inner = grasp_epsilon_inner;
            grasp_goal.epsilon.outer = grasp_epsilon_outer;

            grasp_client.sendGoal(grasp_goal);
            // return grasp_client.waitForResult(ros::Duration(5.0));
            // return
        }

        void open_gripper(actionlib::SimpleActionClient<franka_gripper::MoveAction> &move_client, const double width, const double move_speed)
        {
            franka_gripper::MoveGoal move_goal;
            move_goal.width = width;
            move_goal.speed = move_speed;

            move_client.sendGoal(move_goal);
            // return move_client.waitForResult(ros::Duration(5.0));
        }

        bool home_gripper(actionlib::SimpleActionClient<franka_gripper::HomingAction> &home_client)
        {
            if (home_client.waitForServer(ros::Duration(2.0)))
            {
                home_client.sendGoal(franka_gripper::HomingGoal());
                if (home_client.waitForResult(ros::Duration(10.0)))
                    return true;
                else
                    ROS_ERROR_STREAM(ros::this_node::getName() << ": homing gripper was not successful.");
            }
            else
                ROS_ERROR_STREAM(ros::this_node::getName() << ": cant connect to homing-gripper server.");
            return false;
        }

};