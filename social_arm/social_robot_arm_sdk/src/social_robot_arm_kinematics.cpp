#include <stdio.h>
#include "social_robot_arm_sdk/social_robot_arm_kinematics.h"

using namespace social_robot_arm;

SocialRobotArmKinematics::SocialRobotArmKinematics()
{
  rarm_joint_position_.resize(ARM_JOINT_NUM);
  larm_joint_position_.resize(ARM_JOINT_NUM);

  for (int i=0; i<ARM_JOINT_NUM; i++)
  {
    rarm_joint_position_(i) = 0.0;
    larm_joint_position_(i) = 0.0;
  }
}

SocialRobotArmKinematics::~SocialRobotArmKinematics()
{

}

void SocialRobotArmKinematics::initialize(Eigen::MatrixXd body_position, Eigen::MatrixXd body_orientation)
{
  larm_chain_ = new OPEN_MANIPULATOR::Manipulator();
  rarm_chain_ = new OPEN_MANIPULATOR::Manipulator();

  Eigen::Vector3d body_position_vector;
  Eigen::Matrix3d body_orientation_matrix;

  body_position_vector(0) = body_position.coeff(0,0);
  body_position_vector(1) = body_position.coeff(1,0);
  body_position_vector(2) = body_position.coeff(2,0);

//  body_orientation_matrix(0,0) = body_orientation.coeff(0,0);
//  body_orientation_matrix(0,1) = body_orientation.coeff(0,1);
//  body_orientation_matrix(0,2) = body_orientation.coeff(0,2);

//  body_orientation_matrix(1,0) = body_orientation.coeff(1,0);
//  body_orientation_matrix(1,1) = body_orientation.coeff(1,1);
//  body_orientation_matrix(1,2) = body_orientation.coeff(1,2);

//  body_orientation_matrix(2,0) = body_orientation.coeff(2,0);
//  body_orientation_matrix(2,1) = body_orientation.coeff(2,1);
//  body_orientation_matrix(2,2) = body_orientation.coeff(2,2);

  // Set Kinematics Tree
  // ROS_INFO("init");
  // Left Arm Chain
  larm_chain_->addWorld(BODY, SHOULDER_PITCH ,body_position_vector, body_orientation);
  larm_chain_->addComponent(SHOULDER_PITCH, BODY, SHOULDER_ROLL,
                          OM_MATH::makeVector3(0.0, 0.1065, 0.2815),
                          Eigen::Matrix3d::Identity(3, 3),
                          Y_AXIS,
                          1,
                          1.0f,
                          0.25106828,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(0.00045422009, 0.043392074, -0.00000052043389));
  larm_chain_->addComponent(SHOULDER_ROLL, SHOULDER_PITCH, ELBOW_PITCH,
                          OM_MATH::makeVector3(0.0, 0.05975, 0.0),
                          Eigen::Matrix3d::Identity(3, 3),
                          X_AXIS,
                          2,
                          1.0f,
                          0.26316311,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(-0.000022708624, 0.068183465, -0.000013842578));
  larm_chain_->addComponent(ELBOW_PITCH, SHOULDER_ROLL, ELBOW_YAW,
                          OM_MATH::makeVector3(0.0, 0.09525, 0.0),
                          Eigen::Matrix3d::Identity(3, 3),
                          Y_AXIS,
                          3,
                          1.0f,
                          0.22544242,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(0.00000029928508, 0.067483293, 0.00050643015));
  larm_chain_->addComponent(ELBOW_YAW, ELBOW_PITCH, WRIST_PITCH,
                          OM_MATH::makeVector3(0.0, 0.086, 0.0),
                          Eigen::Matrix3d::Identity(3, 3),
                          Z_AXIS,
                          4,
                          1.0f,
                          0.22625975,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(-0.00013358711, 0.058087493, -0.000096098218));
  larm_chain_->addComponent(WRIST_PITCH, ELBOW_YAW, FINGER,
                          OM_MATH::makeVector3(0.0, 0.08745, 0.0),
                          Eigen::Matrix3d::Identity(3, 3),
                          Y_AXIS,
                          5,
                          1.0f,
                          0.26008606,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(0.0013880465, 0.064104565, 0.0061201240));
  larm_chain_->addTool(FINGER, WRIST_PITCH,
                     OM_MATH::makeVector3(0.0212078, 0.107735, 0.0213208),
                     Eigen::Matrix3d::Identity(3, 3),
                     6);

  // Right Arm Chain
  rarm_chain_->addWorld(BODY, SHOULDER_PITCH ,body_position_vector, body_orientation);
  rarm_chain_->addComponent(SHOULDER_PITCH, BODY, SHOULDER_ROLL,
                          OM_MATH::makeVector3(0.0, -0.1065, 0.2815),
                          Eigen::Matrix3d::Identity(3, 3),
                          Y_AXIS,
                          7,
                          1.0f,
                          0.25106828,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(0.00045422009, -0.043392074, -0.00000052043389));
  rarm_chain_->addComponent(SHOULDER_ROLL, SHOULDER_PITCH, ELBOW_PITCH,
                          OM_MATH::makeVector3(0.0, -0.05975, 0.0),
                          Eigen::Matrix3d::Identity(3, 3),
                          X_AXIS,
                          8,
                          1.0f,
                          0.26316311,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(-0.000022708624, -0.068183465, -0.000013842578));
  rarm_chain_->addComponent(ELBOW_PITCH, SHOULDER_ROLL, ELBOW_YAW,
                          OM_MATH::makeVector3(0.0, -0.09525, 0.0),
                          Eigen::Matrix3d::Identity(3, 3),
                          Y_AXIS,
                          9,
                          1.0f,
                          0.22544242,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(0.00000029928508, -0.067483293, 0.00050643015));
  rarm_chain_->addComponent(ELBOW_YAW, ELBOW_PITCH, WRIST_PITCH,
                          OM_MATH::makeVector3(0.0, -0.086, 0.0),
                          Eigen::Matrix3d::Identity(3, 3),
                          Z_AXIS,
                          10,
                          1.0f,
                          0.22625975,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(-0.00013358711, -0.058087493, -0.000096098218));
  rarm_chain_->addComponent(WRIST_PITCH, ELBOW_YAW, FINGER,
                          OM_MATH::makeVector3(0.0, -0.08745, 0.0),
                          Eigen::Matrix3d::Identity(3, 3),
                          Y_AXIS,
                          11,
                          1.0f,
                          0.26008606,
                          Eigen::Matrix3d::Identity(3, 3),
                          OM_MATH::makeVector3(0.0013880465, -0.064104565, 0.0061201240));
  rarm_chain_->addTool(FINGER, WRIST_PITCH,
                     OM_MATH::makeVector3(0.0212078, -0.107735, 0.0213208),
                     Eigen::Matrix3d::Identity(3, 3),
                     12);

  rarm_kinematics_ = new OPEN_MANIPULATOR::Kinematics();
  larm_kinematics_ = new OPEN_MANIPULATOR::Kinematics();

}

void SocialRobotArmKinematics::setJointPosition(Eigen::VectorXd rarm_joint_position, Eigen::VectorXd larm_joint_position)
{
  rarm_joint_position_ = rarm_joint_position;
  larm_joint_position_ = larm_joint_position;
}

void SocialRobotArmKinematics::solveForwardKinematics(std::vector<double_t> &rarm_position, std::vector<double_t> &rarm_orientation,
                                                      std::vector<double_t> &larm_position, std::vector<double_t> &larm_orientation)
{

  // rarm
  std::vector<double_t> rarm_joint_position;
  rarm_joint_position.resize(5,0.0);
  for(int i=0; i<rarm_chain_->getDOF(); i++)
  {
    rarm_joint_position[i] = rarm_joint_position_(i);
  }

  rarm_chain_->setAllActiveJointAngle(rarm_joint_position);

  rarm_kinematics_->forward(rarm_chain_);
//  ROS_INFO("----------------rarm-------------------");
//  rarm_chain_->checkManipulatorSetting();
  Pose rarm_pose = rarm_chain_->getComponentPoseToWorld(FINGER);
  //position
  rarm_pose_.position.x = rarm_pose.position(0);
  rarm_pose_.position.y = rarm_pose.position(1);
  rarm_pose_.position.z = rarm_pose.position(2);

  rarm_position.resize(3,0.0);
  rarm_position[0] = rarm_pose_.position.x;
  rarm_position[1] = rarm_pose_.position.y;
  rarm_position[2] = rarm_pose_.position.z;

  //orientation

  rarm_orientation = OM_MATH::MakeQuaternion(rarm_pose.orientation);
  rarm_pose_.orientation.x = rarm_orientation[0];
  rarm_pose_.orientation.y = rarm_orientation[1];
  rarm_pose_.orientation.z = rarm_orientation[2];
  rarm_pose_.orientation.w = rarm_orientation[3];
  ROS_INFO("rarm position x : %f y: %f, z: %f", rarm_pose_.position.x, rarm_pose_.position.y, rarm_pose_.position.z);

  // larm
  std::vector<double_t> larm_joint_position;
  larm_joint_position.resize(5,0.0);
  for(int i=0; i<larm_chain_->getDOF();i++)
  {
    larm_joint_position[i] = larm_joint_position_(i);
  }

  larm_chain_->setAllActiveJointAngle(larm_joint_position);

  larm_kinematics_->forward(larm_chain_);
//  ROS_INFO("----------------larm-------------------");
//  larm_chain_->checkManipulatorSetting();
  Pose larm_pose = larm_chain_->getComponentPoseToWorld(FINGER);

  //position
  larm_pose_.position.x = larm_pose.position(0);
  larm_pose_.position.y = larm_pose.position(1);
  larm_pose_.position.z = larm_pose.position(2);

  larm_position.resize(3,0.0);
  larm_position[0] = larm_pose_.position.x;
  larm_position[1] = larm_pose_.position.y;
  larm_position[2] = larm_pose_.position.z;

  //orientation

  larm_orientation = OM_MATH::MakeQuaternion(larm_pose.orientation);

  larm_pose_.orientation.x = larm_orientation[0];
  larm_pose_.orientation.y = larm_orientation[1];
  larm_pose_.orientation.z = larm_orientation[2];
  larm_pose_.orientation.w = larm_orientation[3];


  ROS_INFO("larm position x : %f y: %f, z: %f", larm_pose_.position.x, larm_pose_.position.y, larm_pose_.position.z);

}

bool SocialRobotArmKinematics::solveInverseKinematics(std::vector<double_t> &rarm_output,
                                                      Eigen::MatrixXd rarm_target_position, Eigen::Quaterniond rarm_target_orientation,
                                                      std::vector<double_t> &larm_output,
                                                      Eigen::MatrixXd larm_target_position, Eigen::Quaterniond larm_target_orientation)
{
//  ROS_INFO("[KD] right x: %f, y: %f, z: %f", rarm_target_position(0), rarm_target_position(1), rarm_target_position(2));
//  ROS_INFO("[KD] left x: %f, y: %f, z: %f", larm_target_position(0), larm_target_position(1), larm_target_position(2));

  // rarm
  Pose rarm_target_pose;

  rarm_target_pose.position(0) = rarm_target_position.coeff(0,0);
  rarm_target_pose.position(1) = rarm_target_position.coeff(1,0);
  rarm_target_pose.position(2) = rarm_target_position.coeff(2,0);
  rarm_target_pose.orientation = rarm_target_orientation.toRotationMatrix();
  rarm_output = rarm_kinematics_->inverse(rarm_chain_, FINGER, rarm_target_pose);

  // larm
  Pose larm_target_pose;

  larm_target_pose.position(0) = larm_target_position.coeff(0,0);
  larm_target_pose.position(1) = larm_target_position.coeff(1,0);
  larm_target_pose.position(2) = larm_target_position.coeff(2,0);
  larm_target_pose.orientation = larm_target_orientation.toRotationMatrix();
  larm_output = larm_kinematics_->inverse(larm_chain_, FINGER, larm_target_pose);

  return true;
}

void SocialRobotArmKinematics::finalize()
{
  delete rarm_chain_;
  delete larm_chain_;
  delete rarm_kinematics_;
  delete larm_kinematics_;
}



////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Manipulator/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

using namespace Eigen;

///////////////////////////*initialize function*/////////////////////////////
void OPEN_MANIPULATOR::Manipulator::addWorld(Name world_name,
                           Name child_name,
                           Vector3d world_position,
                           Matrix3d world_orientation)
{
  world_.name = world_name;
  world_.child = child_name;
  world_.pose.position = world_position;
  world_.pose.orientation = world_orientation;
  world_.origin.velocity = VectorXd::Zero(3);
  world_.origin.acceleration = VectorXd::Zero(3);
}

void OPEN_MANIPULATOR::Manipulator::addComponent(Name my_name,
                               Name parent_name,
                               Name child_name,
                               Vector3d relative_position,
                               Matrix3d relative_orientation,
                               Vector3d axis_of_rotation,
                               int8_t joint_actuator_id,
                               double_t coefficient,
                               double_t mass,
                               Matrix3d inertia_tensor,
                               Vector3d center_of_mass)
{
  if (joint_actuator_id != -1)
    dof_++;

  Component temp_component;

  temp_component.parent = parent_name;
  temp_component.child.push_back(child_name);
  temp_component.relative_to_parent.position = relative_position;
  temp_component.relative_to_parent.orientation = relative_orientation;
  temp_component.pose_to_world.position = Vector3d::Zero();
  temp_component.pose_to_world.orientation = Matrix3d::Identity(3, 3);
  temp_component.origin.velocity = VectorXd::Zero(3);
  temp_component.origin.acceleration = VectorXd::Zero(3);
  temp_component.joint.id = joint_actuator_id;
  temp_component.joint.coefficient = coefficient;
  temp_component.joint.axis = axis_of_rotation;
  temp_component.joint.angle = 0.0;
  temp_component.joint.velocity = 0.0;
  temp_component.joint.acceleration = 0.0;
  temp_component.tool.id = -1;
  temp_component.tool.coefficient = 0;
  temp_component.tool.on_off = false;
  temp_component.tool.value = 0.0;
  temp_component.inertia.mass = mass;
  temp_component.inertia.inertia_tensor = inertia_tensor;
  temp_component.inertia.center_of_mass = center_of_mass;

  component_.insert(std::make_pair(my_name, temp_component));
}

void OPEN_MANIPULATOR::Manipulator::addComponentChild(Name my_name, Name child_name)
{
  component_.at(my_name).child.push_back(child_name);
}

void OPEN_MANIPULATOR::Manipulator::addTool(Name my_name,
                          Name parent_name,
                          Vector3d relative_position,
                          Matrix3d relative_orientation,
                          int8_t tool_id,
                          double_t coefficient,
                          double_t mass,
                          Matrix3d inertia_tensor,
                          Vector3d center_of_mass)
{
  Component temp_component;

  temp_component.parent = parent_name;
  temp_component.relative_to_parent.position = relative_position;
  temp_component.relative_to_parent.orientation = relative_orientation;
  temp_component.pose_to_world.position = Vector3d::Zero();
  temp_component.pose_to_world.orientation = Matrix3d::Identity(3, 3);
  temp_component.origin.velocity = VectorXd::Zero(3);
  temp_component.origin.acceleration = VectorXd::Zero(3);
  temp_component.joint.id = -1;
  temp_component.joint.coefficient = 0;
  temp_component.joint.axis = OM_MATH::makeVector3(1.0, 1.0, 1.0);
  temp_component.joint.angle = 0.0;
  temp_component.joint.velocity = 0.0;
  temp_component.joint.acceleration = 0.0;
  temp_component.tool.id = tool_id;
  temp_component.tool.coefficient = coefficient;
  temp_component.tool.on_off = false;
  temp_component.tool.value = 0.0;
  temp_component.inertia.mass = mass;
  temp_component.inertia.inertia_tensor = inertia_tensor;
  temp_component.inertia.center_of_mass = center_of_mass;

  component_.insert(std::make_pair(my_name, temp_component));
}

void OPEN_MANIPULATOR::Manipulator::checkManipulatorSetting()
{
  ROS_INFO("------------------------------------");
  ROS_INFO("Degree of freedom : %d", dof_);
//  ROS_INFO("Size of Component : %d", component_.size());

  ROS_INFO("<Configuration of world>");
  ROS_INFO("Name : %d", world_.name);
//  ROS_INFO("Child name : %d", world_.child);
  ROS_INFO("Position : (%lf, %lf, %lf)", world_.pose.position(0), world_.pose.position(1), world_.pose.position(2));
  ROS_INFO("Orientation : (%lf, %lf, %lf  /  %lf, %lf, %lf  /  %lf, %lf, %lf)", world_.pose.orientation(0,0), world_.pose.orientation(0,1), world_.pose.orientation(0,2),
                                                                                                  world_.pose.orientation(1,0), world_.pose.orientation(1,1), world_.pose.orientation(1,2),
                                                                                                  world_.pose.orientation(2,0), world_.pose.orientation(2,1), world_.pose.orientation(2,2));
  ROS_INFO("<Configuration of components>");
  for (std::map<Name, Component>::iterator it = component_.begin(); it != component_.end(); it++)
  {
    ROS_INFO("Name : %d -----------------------------", it->first);
//    ROS_INFO("Parent : %d", component_[it->first].parent);
//    for (std::vector<Name>::size_type index = 0; index < component_[it->first].child.size(); index++)
//      ROS_INFO("Child  : %d", component_[it->first].child[index]);

    ROS_INFO("Relative to parent");
    ROS_INFO("Position : (%lf, %lf, %lf)", component_[it->first].relative_to_parent.position(0), component_[it->first].relative_to_parent.position(1), component_[it->first].relative_to_parent.position(2));
    ROS_INFO("Orientation : (%lf, %lf, %lf  /  %lf, %lf, %lf  /  %lf, %lf, %lf)", component_[it->first].relative_to_parent.orientation(0,0), component_[it->first].relative_to_parent.orientation(0,1), component_[it->first].relative_to_parent.orientation(0,2),
                                                                                                    component_[it->first].relative_to_parent.orientation(1,0), component_[it->first].relative_to_parent.orientation(1,1), component_[it->first].relative_to_parent.orientation(1,2),
                                                                                                    component_[it->first].relative_to_parent.orientation(2,0), component_[it->first].relative_to_parent.orientation(2,1), component_[it->first].relative_to_parent.orientation(2,2));
    ROS_INFO("Pose to world");
    ROS_INFO("Position : (%lf, %lf, %lf)", component_[it->first].pose_to_world.position(0), component_[it->first].pose_to_world.position(1), component_[it->first].pose_to_world.position(2));
    ROS_INFO("Orientation : (%lf, %lf, %lf  /  %lf, %lf, %lf  /  %lf, %lf, %lf)", component_[it->first].pose_to_world.orientation(0,0), component_[it->first].pose_to_world.orientation(0,1), component_[it->first].pose_to_world.orientation(0,2),
                                                                                                    component_[it->first].pose_to_world.orientation(1,0), component_[it->first].pose_to_world.orientation(1,1), component_[it->first].pose_to_world.orientation(1,2),
                                                                                                    component_[it->first].pose_to_world.orientation(2,0), component_[it->first].pose_to_world.orientation(2,1), component_[it->first].pose_to_world.orientation(2,2));
    ROS_INFO("Joint");
//    ROS_INFO(" ID : %d", component_[it->first].joint.id);
//    ROS_INFO(" Coefficient : %lf", component_[it->first].joint.coefficient);
//    ROS_INFO(" Axis : (%lf, %lf, %lf)", component_[it->first].joint.axis(0), component_[it->first].joint.axis(1), component_[it->first].joint.axis(2));
    ROS_INFO(" Angle : %lf", component_[it->first].joint.angle);
//    ROS_INFO(" Velocity : %lf", component_[it->first].joint.velocity);
//    ROS_INFO(" Acceleration : %lf", component_[it->first].joint.acceleration);

//    ROS_INFO("Tool");
//    ROS_INFO(" ID : %d", component_[it->first].tool.id);
//    ROS_INFO(" Coefficient : %lf", component_[it->first].tool.coefficient);
//    ROS_INFO(" OnOff : %d", component_[it->first].tool.on_off);
//    ROS_INFO(" Value : %lf", component_[it->first].tool.value);

//    ROS_INFO("Inertia");
//    ROS_INFO(" Mass : %lf", component_[it->first].inertia.mass);
//    ROS_INFO(" Inertia tensor : (%lf, %lf, %lf  /  %lf, %lf, %lf  /  %lf, %lf, %lf)", component_[it->first].inertia.inertia_tensor(0,0), component_[it->first].inertia.inertia_tensor(0,1), component_[it->first].inertia.inertia_tensor(0,2),
//                                                                                                        component_[it->first].inertia.inertia_tensor(1,0), component_[it->first].inertia.inertia_tensor(1,1), component_[it->first].inertia.inertia_tensor(1,2),
//                                                                                                        component_[it->first].inertia.inertia_tensor(2,0), component_[it->first].inertia.inertia_tensor(2,1), component_[it->first].inertia.inertia_tensor(2,2));
//    ROS_INFO(" Center of mass : (%lf, %lf, %lf)", component_[it->first].inertia.center_of_mass(0), component_[it->first].inertia.center_of_mass(1), component_[it->first].inertia.center_of_mass(2));
  }
}
/////////////////////////////////////////////////////////////////////////////

///////////////////////////////Set function//////////////////////////////////
void OPEN_MANIPULATOR::Manipulator::setWorldPose(Pose world_pose)
{
  world_.pose = world_pose;
}

void OPEN_MANIPULATOR::Manipulator::setWorldPosition(Vector3d world_position)
{
  world_.pose.position = world_position;
}

void OPEN_MANIPULATOR::Manipulator::setWorldOrientation(Matrix3d world_orientation)
{
  world_.pose.orientation = world_orientation;
}

void OPEN_MANIPULATOR::Manipulator::setWorldState(State world_state)
{
  world_.origin = world_state;
}

void OPEN_MANIPULATOR::Manipulator::setWorldVelocity(VectorXd world_velocity)
{
  world_.origin.velocity = world_velocity;
}

void OPEN_MANIPULATOR::Manipulator::setWorldAcceleration(VectorXd world_acceleration)
{
  world_.origin.acceleration = world_acceleration;
}

void OPEN_MANIPULATOR::Manipulator::setComponent(Name name, Component component, bool *error)
{
  if (component_.find(name) != component_.end())
  {
    component_.insert(std::make_pair(name, component));
    *error = false;
  }
  else
  {
    *error = true;
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentPoseToWorld(Name name, Pose pose_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world = pose_to_world;
  }
  else
  {
    //error
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentPositionToWorld(Name name, Vector3d position_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world.position = position_to_world;
  }
  else
  {
    //error
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentOrientationToWorld(Name name, Matrix3d orientation_to_wolrd)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).pose_to_world.orientation = orientation_to_wolrd;
  }
  else
  {
    //error
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentStateToWorld(Name name, State state_to_world)
{
  if (component_.find(name) != component_.end())
  {
    component_.at(name).origin = state_to_world;
  }
  else
  {
    //error
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentVelocityToWorld(Name name, VectorXd velocity)
{
  if (velocity.size() != 6)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).origin.velocity = velocity;
    }
    else
    {
      //error
    }
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentAccelerationToWorld(Name name, VectorXd acceleration)
{
  if (acceleration.size() != 6)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).origin.acceleration = acceleration;
    }
    else
    {
      //error
    }
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentJointAngle(Name name, double_t angle)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
//      component_.at(name).joint.angle = angle;
      component_.at(name).joint.angle = component_.at(name).joint.coefficient* angle;
    }
    else
    {
      //error
    }
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentJointVelocity(Name name, double_t angular_velocity)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.velocity = angular_velocity;
    }
    else
    {
      //error
    }
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentJointAcceleration(Name name, double_t angular_acceleration)
{
  if (component_.at(name).tool.id > 0)
  {
    //error
  }
  else
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).joint.acceleration = angular_acceleration;
    }
    else
    {
      //error
    }
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentToolOnOff(Name name, bool on_off)
{
  if (component_.at(name).tool.id > 0)
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).tool.on_off = on_off;
    }
    else
    {
      //error
    }
  }
  else
  {
    //error
  }
}

void OPEN_MANIPULATOR::Manipulator::setComponentToolValue(Name name, double_t value)
{
  if (component_.at(name).tool.id > 0)
  {
    if (component_.find(name) != component_.end())
    {
      component_.at(name).tool.value = value;
    }
    else
    {
      //error
    }
  }
  else
  {
    //error
  }
}

void OPEN_MANIPULATOR::Manipulator::setAllActiveJointAngle(std::vector<double_t> angle_vector)
{
  std::map<Name, Component>::iterator it;
  int8_t index = 0;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1)
    {
      component_.at(it->first).joint.angle =  component_.at(it->first).joint.coefficient * angle_vector.at(index);
      index++;
    }
  }
}

///////////////////////////////Get function//////////////////////////////////

int8_t OPEN_MANIPULATOR::Manipulator::getDOF()
{
  return dof_;
}

int8_t OPEN_MANIPULATOR::Manipulator::getComponentSize()
{
  return component_.size();
}

Name OPEN_MANIPULATOR::Manipulator::getWorldName()
{
  return world_.name;
}

Name OPEN_MANIPULATOR::Manipulator::getWorldChildName()
{
  return world_.child;
}

Pose OPEN_MANIPULATOR::Manipulator::getWorldPose()
{
  return world_.pose;
}

Vector3d OPEN_MANIPULATOR::Manipulator::getWorldPosition()
{
  return world_.pose.position;
}

Matrix3d OPEN_MANIPULATOR::Manipulator::getWorldOrientation()
{
  return world_.pose.orientation;
}

State OPEN_MANIPULATOR::Manipulator::getWorldState()
{
  return world_.origin;
}

VectorXd OPEN_MANIPULATOR::Manipulator::getWorldVelocity()
{
  return world_.origin.velocity;
}

VectorXd OPEN_MANIPULATOR::Manipulator::getWorldAcceleration()
{
  return world_.origin.acceleration;
}

std::map<Name, Component> OPEN_MANIPULATOR::Manipulator::getAllComponent()
{
  return component_;
}

std::map<Name, Component>::iterator OPEN_MANIPULATOR::Manipulator::getIteratorBegin()
{
  return component_.begin();
}

std::map<Name, Component>::iterator OPEN_MANIPULATOR::Manipulator::getIteratorEnd()
{
  return component_.end();;
}

Component OPEN_MANIPULATOR::Manipulator::getComponent(Name name)
{
  return component_.at(name);
}

Name OPEN_MANIPULATOR::Manipulator::getComponentParentName(Name name)
{
  return component_.at(name).parent;
}

std::vector<Name> OPEN_MANIPULATOR::Manipulator::getComponentChildName(Name name)
{
  return component_.at(name).child;
}

Pose OPEN_MANIPULATOR::Manipulator::getComponentPoseToWorld(Name name)
{
  return component_.at(name).pose_to_world;
}

Vector3d OPEN_MANIPULATOR::Manipulator::getComponentPositionToWorld(Name name)
{
  return component_.at(name).pose_to_world.position;
}

Matrix3d OPEN_MANIPULATOR::Manipulator::getComponentOrientationToWorld(Name name)
{
  return component_.at(name).pose_to_world.orientation;
}

State OPEN_MANIPULATOR::Manipulator::getComponentStateToWorld(Name name)
{
  return component_.at(name).origin;
}

VectorXd OPEN_MANIPULATOR::Manipulator::getComponentVelocityToWorld(Name name)
{
  return component_.at(name).origin.velocity;
}

VectorXd OPEN_MANIPULATOR::Manipulator::getComponentAccelerationToWorld(Name name)
{
  return component_.at(name).origin.acceleration;
}

Pose OPEN_MANIPULATOR::Manipulator::getComponentRelativePoseToParent(Name name)
{
  return component_.at(name).relative_to_parent;
}

Vector3d OPEN_MANIPULATOR::Manipulator::getComponentRelativePositionToParent(Name name)
{
  return component_.at(name).relative_to_parent.position;
}

Matrix3d OPEN_MANIPULATOR::Manipulator::getComponentRelativeOrientationToParent(Name name)
{
  return component_.at(name).relative_to_parent.orientation;
}

Joint OPEN_MANIPULATOR::Manipulator::getComponentJoint(Name name)
{
  return component_.at(name).joint;
}

int8_t OPEN_MANIPULATOR::Manipulator::getComponentJointId(Name name)
{
  return component_.at(name).joint.id;
}

double_t OPEN_MANIPULATOR::Manipulator::getComponentJointCoefficient(Name name)
{
  return component_.at(name).joint.coefficient;
}

Vector3d OPEN_MANIPULATOR::Manipulator::getComponentJointAxis(Name name)
{
  return component_.at(name).joint.axis;
}

double_t OPEN_MANIPULATOR::Manipulator::getComponentJointAngle(Name name)
{
  return component_.at(name).joint.angle/component_.at(name).joint.coefficient;
}

double_t OPEN_MANIPULATOR::Manipulator::getComponentJointVelocity(Name name)
{
  return component_.at(name).joint.velocity;
}

double_t OPEN_MANIPULATOR::Manipulator::getComponentJointAcceleration(Name name)
{
  return component_.at(name).joint.acceleration;
}

Tool OPEN_MANIPULATOR::Manipulator::getComponentTool(Name name)
{
  return component_.at(name).tool;
}

int8_t OPEN_MANIPULATOR::Manipulator::getComponentToolId(Name name)
{
  return component_.at(name).tool.id;
}

double_t OPEN_MANIPULATOR::Manipulator::getComponentToolCoefficient(Name name)
{
  return component_.at(name).tool.coefficient;
}

bool OPEN_MANIPULATOR::Manipulator::getComponentToolOnOff(Name name)
{
  return component_.at(name).tool.on_off;
}

double_t OPEN_MANIPULATOR::Manipulator::getComponentToolValue(Name name)
{
  return component_.at(name).tool.value;
}

double_t OPEN_MANIPULATOR::Manipulator::getComponentMass(Name name)
{
  return component_.at(name).inertia.mass;
}

Matrix3d OPEN_MANIPULATOR::Manipulator::getComponentInertiaTensor(Name name)
{
  return component_.at(name).inertia.inertia_tensor;
}

Vector3d OPEN_MANIPULATOR::Manipulator::getComponentCenterOfMass(Name name)
{
  return component_.at(name).inertia.center_of_mass;
}

std::vector<double_t> OPEN_MANIPULATOR::Manipulator::getAllJointAngle()
{
  std::vector<double_t> result_vector;
  std::map<Name, Component>::iterator it;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).tool.id == -1) // Check whether Tool or not
    {
      // This is not Tool -> This is Joint
      result_vector.push_back(component_.at(it->first).joint.angle/component_.at(it->first).joint.coefficient);
    }
  }
  return result_vector;
}

std::vector<double_t> OPEN_MANIPULATOR::Manipulator::getAllActiveJointAngle()
{
  std::vector<double_t> result_vector;
  std::map<Name, Component>::iterator it;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1) // Check whether Active or Passive
    {
      // Active
      result_vector.push_back(component_.at(it->first).joint.angle/component_.at(it->first).joint.coefficient);
    }
  }
  return result_vector;
}

std::vector<uint8_t> OPEN_MANIPULATOR::Manipulator::getAllActiveJointID()
{
  std::vector<uint8_t> active_joint_id;
  std::map<Name, Component>::iterator it;

  for (it = component_.begin(); it != component_.end(); it++)
  {
    if (component_.at(it->first).joint.id != -1)
    {
      active_joint_id.push_back(component_.at(it->first).joint.id);
    }
  }
  return active_joint_id;
}

MatrixXd OPEN_MANIPULATOR::Kinematics::jacobian(Manipulator *manipulator, Name tool_name)
{
  MatrixXd jacobian = MatrixXd::Identity(6, manipulator->getDOF());

  Vector3d joint_axis = ZERO_VECTOR;

  Vector3d position_changed = ZERO_VECTOR;
  Vector3d orientation_changed = ZERO_VECTOR;
  VectorXd pose_changed = VectorXd::Zero(6);

  int8_t index = 0;
  Name my_name = manipulator->getIteratorBegin()->first;

  for (int8_t size = 0; size < manipulator->getDOF(); size++)
  {
    Name parent_name = manipulator->getComponentParentName(my_name);
    if (parent_name == manipulator->getWorldName())
    {
      joint_axis = manipulator->getWorldOrientation() * manipulator->getComponentJointAxis(my_name);
    }
    else
    {
      joint_axis = manipulator->getComponentOrientationToWorld(parent_name) * manipulator->getComponentJointAxis(my_name);
    }

    position_changed = OM_MATH::skewSymmetricMatrix(joint_axis) *
                       (manipulator->getComponentPositionToWorld(tool_name) - manipulator->getComponentPositionToWorld(my_name));
    orientation_changed = joint_axis;

    pose_changed << position_changed(0),
        position_changed(1),
        position_changed(2),
        orientation_changed(0),
        orientation_changed(1),
        orientation_changed(2);

    jacobian.col(index) = pose_changed;
    index++;
    my_name = manipulator->getComponentChildName(my_name).at(0); // Get Child name which has active joint
  }
  return jacobian;
}

void OPEN_MANIPULATOR::Kinematics::forward(Manipulator *manipulator)
{
  forward(manipulator, manipulator->getWorldChildName());
}

void OPEN_MANIPULATOR::Kinematics::forward(Manipulator *manipulator, Name component_name)
{

  Name my_name = component_name;
  Name parent_name = manipulator->getComponentParentName(my_name);
  int8_t number_of_child = manipulator->getComponentChildName(my_name).size();

  Vector3d parent_position_to_world;
  Vector3d my_position_to_world;
  Matrix3d parent_orientation_to_world;
  Matrix3d my_orientation_to_world;

  if (parent_name == manipulator->getWorldName())
  {
    parent_position_to_world = manipulator->getWorldPosition();
    parent_orientation_to_world = manipulator->getWorldOrientation();
  }
  else
  {
    parent_position_to_world = manipulator->getComponentPositionToWorld(parent_name);
    parent_orientation_to_world = manipulator->getComponentOrientationToWorld(parent_name);
  }

  my_position_to_world = parent_orientation_to_world * manipulator->getComponentRelativePositionToParent(my_name) + parent_position_to_world;
  if(manipulator->getComponentJointId(my_name) != -1)
    my_orientation_to_world = parent_orientation_to_world * OM_MATH::rodriguesRotationMatrix(manipulator->getComponentJointAxis(my_name), manipulator->getComponentJointAngle(my_name));
  else
    my_orientation_to_world = parent_orientation_to_world;

  manipulator->setComponentPositionToWorld(my_name, my_position_to_world);
  manipulator->setComponentOrientationToWorld(my_name, my_orientation_to_world);

  for (int8_t index = 0; index < number_of_child; index++)
  {
    Name child_name = manipulator->getComponentChildName(my_name).at(index);
    forward(manipulator, child_name);
  }
}

std::vector<double_t> OPEN_MANIPULATOR::Kinematics::inverse(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
//  return positionOnlyInverseKinematics(manipulator, tool_name, target_pose);
  return srInverseKinematics(manipulator, tool_name, target_pose);
//  return InverseKinematicsUsing3DOF(manipulator, tool_name, target_pose);
}

std::vector<double_t> OPEN_MANIPULATOR::Kinematics::InverseKinematicsUsing3DOF(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  Manipulator _manipulator = *manipulator;

  MatrixXd jacobian = MatrixXd::Identity(6, _manipulator.getDOF());
  MatrixXd position_jacobian = MatrixXd::Identity(3, _manipulator.getDOF());
  MatrixXd only3dof_jacobian = MatrixXd::Identity(3, 3);
  MatrixXd updated_jacobian = MatrixXd::Identity(3, 3);

  VectorXd position_changed = VectorXd::Zero(3);
  VectorXd angle_changed = VectorXd::Zero(3);
  VectorXd gerr(3);

  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double Ek = 0.0;
  double Ek2 = 0.0;

  MatrixXd We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  MatrixXd Wn = MatrixXd::Identity(3, 3);

  forward(&_manipulator, _manipulator.getIteratorBegin()->first);
  position_changed = OM_MATH::positionDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name));
  Ek = position_changed.transpose() * We * position_changed;

  for (int8_t count = 0; count < iteration; count++)
  {

    jacobian = this->jacobian(&_manipulator, tool_name);
    position_jacobian.row(0) = jacobian.row(0);
    position_jacobian.row(1) = jacobian.row(1);
    position_jacobian.row(2) = jacobian.row(2);

    only3dof_jacobian.col(0) = position_jacobian.col(0);
    only3dof_jacobian.col(1) = position_jacobian.col(1);
    only3dof_jacobian.col(2) = position_jacobian.col(2);
    lambda = Ek + param;

    updated_jacobian = (only3dof_jacobian.transpose() * We * only3dof_jacobian) + (lambda * Wn);
    gerr = only3dof_jacobian.transpose() * We * position_changed;

    ColPivHouseholderQR<MatrixXd> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
    {
      if(index<3)
        set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) + angle_changed(index));
      else
        set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index));
    }

    _manipulator.setAllActiveJointAngle(set_angle_changed);

    forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    position_changed = OM_MATH::positionDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name));

    Ek2 = position_changed.transpose() * We * position_changed;

    if (Ek2 < 1E-12)
    {
      return _manipulator.getAllActiveJointAngle();
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<double> set_angle_changed;
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      {
        if(index<3)
          set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) - angle_changed(index));
        else
          set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index));
      }

      _manipulator.setAllActiveJointAngle(set_angle_changed);
      forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    }
  }
  return _manipulator.getAllActiveJointAngle();
}

std::vector<double_t> OPEN_MANIPULATOR::Kinematics::srInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 50;

  Manipulator _manipulator = *manipulator;

  MatrixXd jacobian = MatrixXd::Identity(6, _manipulator.getDOF());
  MatrixXd updated_jacobian = MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());
  VectorXd pose_changed = VectorXd::Zero(_manipulator.getDOF());
  VectorXd angle_changed = VectorXd::Zero(_manipulator.getDOF());
  VectorXd gerr(_manipulator.getDOF());

  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double Ek = 0.0;
  double Ek2 = 0.0;

  MatrixXd We(6, 6);
  We << wn_pos, 0, 0, 0, 0, 0,
      0, wn_pos, 0, 0, 0, 0,
      0, 0, wn_pos, 0, 0, 0,
      0, 0, 0, wn_ang, 0, 0,
      0, 0, 0, 0, wn_ang, 0,
      0, 0, 0, 0, 0, wn_ang;

  MatrixXd Wn = MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  forward(&_manipulator, _manipulator.getIteratorBegin()->first);
  pose_changed = OM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name),
                                         target_pose.orientation, _manipulator.getComponentOrientationToWorld(tool_name));
  Ek = pose_changed.transpose() * We * pose_changed;

  for (int8_t count = 0; count < iteration; count++)
  {
    jacobian = this->jacobian(&_manipulator, tool_name);
    lambda = Ek + param;

    updated_jacobian = (jacobian.transpose() * We * jacobian) + (lambda * Wn);
    gerr = jacobian.transpose() * We * pose_changed;

    ColPivHouseholderQR<MatrixXd> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<double_t> set_angle_changed;
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) + angle_changed(index));

    _manipulator.setAllActiveJointAngle(set_angle_changed);

    forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    pose_changed = OM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name),
                                           target_pose.orientation, _manipulator.getComponentOrientationToWorld(tool_name));

    Ek2 = pose_changed.transpose() * We * pose_changed;

    if (Ek2 < 1E-12)
    {
      return _manipulator.getAllActiveJointAngle();
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<double_t> set_angle_changed;
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) - angle_changed(index));

      _manipulator.setAllActiveJointAngle(set_angle_changed);

      forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    }
  }

  return _manipulator.getAllActiveJointAngle();
}

std::vector<double_t> OPEN_MANIPULATOR::Kinematics::positionOnlyInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  double lambda = 0.0;
  const double param = 0.002;
  const int8_t iteration = 10;

  Manipulator _manipulator = *manipulator;

  MatrixXd jacobian = MatrixXd::Identity(6, _manipulator.getDOF());
  MatrixXd position_jacobian = MatrixXd::Identity(3, _manipulator.getDOF());
  MatrixXd updated_jacobian = MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());
  VectorXd position_changed = VectorXd::Zero(3);
  VectorXd angle_changed = VectorXd::Zero(_manipulator.getDOF());
  VectorXd gerr(_manipulator.getDOF());

  double wn_pos = 1 / 0.3;
  double wn_ang = 1 / (2 * M_PI);
  double Ek = 0.0;
  double Ek2 = 0.0;

  MatrixXd We(3, 3);
  We << wn_pos, 0, 0,
      0, wn_pos, 0,
      0, 0, wn_pos;

  MatrixXd Wn = MatrixXd::Identity(_manipulator.getDOF(), _manipulator.getDOF());

  forward(&_manipulator, _manipulator.getIteratorBegin()->first);
  position_changed = OM_MATH::positionDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name));
  Ek = position_changed.transpose() * We * position_changed;

  for (int8_t count = 0; count < iteration; count++)
  {

    jacobian = this->jacobian(&_manipulator, tool_name);
    position_jacobian.row(0) = jacobian.row(0);
    position_jacobian.row(1) = jacobian.row(1);
    position_jacobian.row(2) = jacobian.row(2);
    lambda = Ek + param;

    updated_jacobian = (position_jacobian.transpose() * We * position_jacobian) + (lambda * Wn);
    gerr = position_jacobian.transpose() * We * position_changed;

    ColPivHouseholderQR<MatrixXd> dec(updated_jacobian);
    angle_changed = dec.solve(gerr);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < _manipulator.getDOF(); index++)
      set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) + angle_changed(index));

    _manipulator.setAllActiveJointAngle(set_angle_changed);

    forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    position_changed = OM_MATH::positionDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name));

    Ek2 = position_changed.transpose() * We * position_changed;

    if (Ek2 < 1E-12)
    {
      return _manipulator.getAllActiveJointAngle();
    }
    else if (Ek2 < Ek)
    {
      Ek = Ek2;
    }
    else
    {
      std::vector<double> set_angle_changed;
      for (int8_t index = 0; index < _manipulator.getDOF(); index++)
        set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) - angle_changed(index));

      _manipulator.setAllActiveJointAngle(set_angle_changed);
      forward(&_manipulator, _manipulator.getIteratorBegin()->first);
    }
  }
  return _manipulator.getAllActiveJointAngle();
}

std::vector<double_t> OPEN_MANIPULATOR::Kinematics::inverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose)
{
  const double lambda = 0.7;
  const int8_t iteration = 10;

  Manipulator _manipulator = *manipulator;

  MatrixXd jacobian = MatrixXd::Identity(6, _manipulator.getDOF());

  VectorXd pose_changed = VectorXd::Zero(6);
  VectorXd angle_changed = VectorXd::Zero(_manipulator.getDOF());

  for (int8_t count = 0; count < iteration; count++)
  {
    forward(&_manipulator, _manipulator.getIteratorBegin()->first);

    jacobian = this->jacobian(&_manipulator, tool_name);

    pose_changed = OM_MATH::poseDifference(target_pose.position, _manipulator.getComponentPositionToWorld(tool_name),
                                           target_pose.orientation, _manipulator.getComponentOrientationToWorld(tool_name));
    if (pose_changed.norm() < 1E-6)
      return _manipulator.getAllActiveJointAngle();

    ColPivHouseholderQR<MatrixXd> dec(jacobian);
    angle_changed = lambda * dec.solve(pose_changed);

    std::vector<double> set_angle_changed;
    for (int8_t index = 0; index < getDOF(); index++)
      set_angle_changed.push_back(_manipulator.getAllActiveJointAngle().at(index) + angle_changed(index));

    _manipulator.setAllActiveJointAngle(set_angle_changed);
  }

  return _manipulator.getAllActiveJointAngle();
}


double_t OM_MATH::sign(double_t number)
{
  if (number >= 0.0)
  {
    return 1.0;
  }
  else
  {
    return -1.0;
  }
}

Vector3d OM_MATH::makeVector3(double_t v1, double_t v2, double_t v3)
{
  Vector3d temp;
  temp << v1, v2, v3;
  return temp;
}

Matrix3d OM_MATH::makeMatrix3(double_t m11, double_t m12, double_t m13,
                           double_t m21, double_t m22, double_t m23,
                           double_t m31, double_t m32, double_t m33)
{
  Matrix3d temp;
  temp << m11, m12, m13, m21, m22, m23, m31, m32, m33;
  return temp;
}

Vector3d OM_MATH::matrixLogarithm(Matrix3d rotation_matrix)
{
  Matrix3d R = rotation_matrix;
  Vector3d l = Vector3d::Zero();
  Vector3d rotation_vector = Vector3d::Zero();

  double_t theta = 0.0;
  double_t diag = 0.0;
  bool diagonal_matrix = R.isDiagonal();

  l << R(2, 1) - R(1, 2),
      R(0, 2) - R(2, 0),
      R(1, 0) - R(0, 1);
  theta = atan2(l.norm(), R(0, 0) + R(1, 1) + R(2, 2) - 1);
  diag = R.determinant();

  if (R.isIdentity())
  {
    rotation_vector.setZero();
    return rotation_vector;
  }

  if (diagonal_matrix == true)
  {
    rotation_vector << R(0, 0) + 1, R(1, 1) + 1, R(2, 2) + 1;
    rotation_vector = rotation_vector * M_PI_2;
  }
  else
  {
    rotation_vector = theta * (l / l.norm());
  }
  return rotation_vector;
}

Matrix3d OM_MATH::skewSymmetricMatrix(Vector3d v)
{
  Matrix3d skew_symmetric_matrix = Matrix3d::Zero();
  skew_symmetric_matrix << 0, -v(2), v(1),
      v(2), 0, -v(0),
      -v(1), v(0), 0;
  return skew_symmetric_matrix;
}

Matrix3d OM_MATH::rodriguesRotationMatrix(Vector3d axis, double_t angle)
{
  Matrix3d skew_symmetric_matrix = Matrix3d::Zero();
  Matrix3d rotation_matrix = Matrix3d::Zero();
  Matrix3d Identity_matrix = Matrix3d::Identity();

  skew_symmetric_matrix = skewSymmetricMatrix(axis);
  rotation_matrix = Identity_matrix +
                    skew_symmetric_matrix * sin(angle) +
                    skew_symmetric_matrix * skew_symmetric_matrix * (1 - cos(angle));
  return rotation_matrix;
}

Matrix3d OM_MATH::makeRotationMatrix(double_t roll, double_t pitch, double_t yaw)
{
#if 0 // Euler angle
  Eigen::Matrix3d rotation_matrix = Eigen::Matrix3d::Identity();

  rotation_matrix << cos(yaw) * cos(pitch), (-1.0f) * sin(yaw) * cos(roll) + cos(yaw) * sin(pitch) * sin(roll), sin(yaw) * sin(roll) + cos(yaw) * sin(pitch) * cos(roll),
      sin(yaw) * cos(pitch), cos(yaw) * cos(roll) + sin(yaw) * sin(pitch) * sin(roll), (-1.0f) * cos(yaw) * sin(roll) + sin(yaw) * sin(pitch) * cos(roll),
      (-1.0f) * sin(pitch), cos(pitch) * sin(roll), cos(pitch) * cos(roll);

  return rotation_matrix;
#endif

  Vector3d rotation_vector;

  rotation_vector(0) = roll;
  rotation_vector(1) = pitch;
  rotation_vector(2) = yaw;

  return makeRotationMatrix(rotation_vector);
}

Matrix3d OM_MATH::makeRotationMatrix(Vector3d rotation_vector)
{
  Matrix3d rotation_matrix;
  Vector3d axis;
  double_t angle;

  angle = rotation_vector.norm();
  axis(0) = rotation_vector(0) / angle;
  axis(1) = rotation_vector(1) / angle;
  axis(2) = rotation_vector(2) / angle;

  rotation_matrix = rodriguesRotationMatrix(axis, angle);
  return rotation_matrix;
}

Vector3d OM_MATH::makeRotationVector(Matrix3d rotation_matrix)
{
  return matrixLogarithm(rotation_matrix);
}

std::vector<double_t> OM_MATH::MakeQuaternion(Matrix3d rotation_matrix)
{
  double trace = rotation_matrix(0,0) + rotation_matrix(1,1) + rotation_matrix(2,2);
  double epsilon=1E-12;
  double w,x,y,z;
  std::vector<double_t> result;

  if( trace > epsilon )
  {
    double s = 0.5 / sqrt(trace + 1.0);
    w = 0.25 / s;
    x = ( rotation_matrix(2,1) - rotation_matrix(1,2) ) * s;
    y = ( rotation_matrix(0,2) - rotation_matrix(2,0) ) * s;
    z = ( rotation_matrix(1,0) - rotation_matrix(0,1) ) * s;
  }
  else
  {
    if ( rotation_matrix(0,0) > rotation_matrix(1,1) && rotation_matrix(0,0) > rotation_matrix(2,2) )
    {
      double s = 2.0 * sqrt( 1.0 + rotation_matrix(0,0) - rotation_matrix(1,1) - rotation_matrix(2,2));
      w = (rotation_matrix(2,1) - rotation_matrix(1,2) ) / s;
      x = 0.25 * s;
      y = (rotation_matrix(0,1) + rotation_matrix(1,0) ) / s;
      z = (rotation_matrix(0,2) + rotation_matrix(2,0) ) / s;
    }
    else if (rotation_matrix(1,1) > rotation_matrix(2,2))
    {
      double s = 2.0 * sqrt( 1.0 + rotation_matrix(1,1) - rotation_matrix(0,0) - rotation_matrix(2,2));
      w = (rotation_matrix(0,2) - rotation_matrix(2,0) ) / s;
      x = (rotation_matrix(0,1) + rotation_matrix(1,0) ) / s;
      y = 0.25 * s;
      z = (rotation_matrix(1,2) + rotation_matrix(2,1) ) / s;
    }
    else
    {
      double s = 2.0 * sqrt( 1.0 + rotation_matrix(2,2) - rotation_matrix(0,0) - rotation_matrix(1,1) );
      w = (rotation_matrix(1,0) - rotation_matrix(0,1) ) / s;
      x = (rotation_matrix(0,2) + rotation_matrix(2,0) ) / s;
      y = (rotation_matrix(1,2) + rotation_matrix(2,1) ) / s;
      z = 0.25 * s;
    }
  }
  result.push_back(x);
  result.push_back(y);
  result.push_back(z);
  result.push_back(w);
  return result;
}



Vector3d OM_MATH::positionDifference(Vector3d desired_position, Vector3d present_position)
{
  Vector3d position_difference;
  position_difference = desired_position - present_position;

  return position_difference;
}

Vector3d OM_MATH::orientationDifference(Matrix3d desired_orientation, Matrix3d present_orientation)
{
  Vector3d orientation_difference;
  orientation_difference = present_orientation * makeRotationVector(present_orientation.transpose() * desired_orientation);

  return orientation_difference;
}

VectorXd OM_MATH::poseDifference(Vector3d desired_position, Vector3d present_position,
                              Matrix3d desired_orientation, Matrix3d present_orientation)
{
  Vector3d position_difference;
  Vector3d orientation_difference;
  VectorXd pose_difference(6);

  position_difference = positionDifference(desired_position, present_position);
  orientation_difference = orientationDifference(desired_orientation, present_orientation);
  pose_difference << position_difference(0), position_difference(1), position_difference(2),
      orientation_difference(0), orientation_difference(1), orientation_difference(2);

  return pose_difference;
}
