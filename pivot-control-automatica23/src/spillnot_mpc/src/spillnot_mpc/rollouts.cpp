// Online rollouts orch
// Author: Rafael Cabral

#include <spillnot_mpc/rollouts.h>

// #include "pinocchio/algorithm/kinematics.hpp"
// #include "pinocchio/algorithm/joint-configuration.hpp"

#include "pinocchio/parsers/sample-models.hpp"
#include "pinocchio/spatial/explog.hpp"
#include "pinocchio/algorithm/kinematics.hpp"
#include "pinocchio/algorithm/jacobian.hpp"
#include "pinocchio/algorithm/joint-configuration.hpp"
#include "pinocchio/algorithm/frames.hpp"
#include "pinocchio/algorithm/model.hpp"

namespace pin = pinocchio;

Rollouts::Rollouts(const std::string urdf_filename){

    pin::urdf::buildModel(urdf_filename, model);
    std::cout << "model name: " << model.name << std::endl;
    
    // Create data required by the algorithms
    data = pin::Data(model);

    std::cout << "created model and data" << std::endl;

}

void Rollouts::fkm(){
    // for (auto frame : model.frames){
    //     std::cout << frame.name << std::endl;
    // }

    // testing frame access
    std::string frame_ee_name = "panda_hand_tcp";
    if (model.existFrame(frame_ee_name, pin::FrameType::BODY)){
        pin::FrameIndex frame_ee = model.getFrameId(frame_ee_name);
        std::cout << "frame id: " << frame_ee << ", frame name: " << model.frames[28].name << ", frame type: " << model.frames[28].type << std::endl;
    } else {
        std::cout << "frame does not exist" << std::endl;
    }

    // fkm
    const pin::FrameIndex frame_ee = model.getFrameId(frame_ee_name);
    Eigen::VectorXd q = pin::neutral(model);
    pin::forwardKinematics(model, data,q);
    pin::updateFramePlacement(model, data, frame_ee);
    std::cout << q.transpose() << std::endl;
    std::cout << "O_T_EE: " << data.oMf[frame_ee] << std::endl;

    std::cout << "nv: " << model.nv << std::endl;

    // build reduced model with fixed fingers
    // Create a list of joint to lock
    std::vector<std::string> list_of_joints_to_lock_by_name;
    list_of_joints_to_lock_by_name.push_back("panda_finger_joint1");
    list_of_joints_to_lock_by_name.push_back("panda_finger_joint2");

    // Print the list of joints to remove + retrieve the joint id
  std::vector<pinocchio::JointIndex> list_of_joints_to_lock_by_id;
  for(std::vector<std::string>::const_iterator it = list_of_joints_to_lock_by_name.begin();
      it != list_of_joints_to_lock_by_name.end(); ++it)
  {
    const std::string & joint_name = *it;
    if(model.existJointName(joint_name)) // do not consider joint that are not in the model
      list_of_joints_to_lock_by_id.push_back(model.getJointId(joint_name));
    else
      std::cout << "joint: " << joint_name << " does not belong to the model" << std::endl;
  }

    std::cout << "\n\nFIRST CASE: BUILD A REDUCED MODEL FROM A LIST OF JOINT TO LOCK" << std::endl;
  // Build the reduced model from the list of lock joints
  Eigen::VectorXd q_neutral = neutral(model);
  pinocchio::Model reduced_model = pinocchio::buildReducedModel(model, list_of_joints_to_lock_by_id, q_neutral);
  
  // Print the list of joints in the original model
  std::cout << "List of joints in the original model:" << std::endl;
  for(pinocchio::JointIndex joint_id = 1; joint_id < model.joints.size(); ++joint_id)
    std::cout << "\t- " << model.names[joint_id] << std::endl;
    
  // Print the list of joints in the reduced model
  std::cout << "List of joints in the reduced model:" << std::endl;
  for(pinocchio::JointIndex joint_id = 1; joint_id < reduced_model.joints.size(); ++joint_id)
    std::cout << "\t- " << reduced_model.names[joint_id] << std::endl;


}