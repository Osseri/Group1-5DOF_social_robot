
#ifndef SOCIAL_ROBOT_ARM_SDK_SOCIAL_ROBOT_ARM_KINEMATICS_H_
#define SOCIAL_ROBOT_ARM_SDK_SOCIAL_ROBOT_ARM_KINEMATICS_H_

#pragma once

#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <ros/package.h>

#include <math.h>
#include <stdint.h>
#include <string>
#include <vector>
#include <map>

#include <eigen3/Eigen/Eigen>
#include <Eigen/Dense>
#include <Eigen/LU> // Calls inverse, determinant, LU decomp., etc.
#include <unistd.h>

#include <geometry_msgs/Pose.h>

//#include <kdl/joint.hpp>
//#include <kdl/chain.hpp>
//#include <kdl/chaindynparam.hpp>
//#include <kdl/jacobian.hpp>
//#include <kdl/chainjnttojacsolver.hpp>
//#include <kdl/chainfksolver.hpp>
//#include <kdl/chainfksolverpos_recursive.hpp>
//#include <kdl/chainiksolvervel_pinv.hpp>
//#include <kdl/chainiksolverpos_nr_jl.hpp>
//#include <kdl/chainiksolverpos_lma.hpp>

#define ARM_JOINT_NUM   (5)
#define D2R             (M_PI/180.0)

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Manipulator/////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////

#define BODY 0
#define SHOULDER_PITCH 1
#define SHOULDER_ROLL 2
#define ELBOW_PITCH 3
#define ELBOW_YAW 4
#define WRIST_PITCH 5
#define FINGER 6

#define X_AXIS OM_MATH::makeVector3(1.0, 0.0, 0.0)
#define Y_AXIS OM_MATH::makeVector3(0.0, 1.0, 0.0)
#define Z_AXIS OM_MATH::makeVector3(0.0, 0.0, 1.0)

#define M_X_AXIS OM_MATH::makeVector3(-1.0, 0.0, 0.0)
#define M_Y_AXIS OM_MATH::makeVector3(0.0, -1.0, 0.0)
#define M_Z_AXIS OM_MATH::makeVector3(0.0, 0.0, -1.0)

using namespace Eigen;

typedef int8_t Name;

typedef struct
{
  Vector3d position;
  Matrix3d orientation;
} Pose;

typedef struct
{
  VectorXd velocity;
  VectorXd acceleration;
} State;

typedef struct
{
  int8_t id;
  Vector3d axis;
  double_t coefficient; //actuator angle to joint angle
  double_t angle;
  double_t velocity;
  double_t acceleration;
} Joint;

typedef struct
{
  int8_t id;
  bool on_off;
  double_t coefficient; //actuator value to tool value
  double_t value;       //m or rad
} Tool;

typedef struct
{
  double_t mass;
  Matrix3d inertia_tensor;
  Vector3d center_of_mass;
} Inertia;

typedef struct
{
  Name name;
  Name child;
  Pose pose;
  State origin;
} World;

typedef struct
{
  Name parent;
  std::vector<Name> child;
  Pose relative_to_parent;
  Pose pose_to_world;
  State origin;
  Joint joint;  Tool tool;
  Inertia inertia;
} Component;

namespace OPEN_MANIPULATOR
{
class Manipulator
{
private:
  int8_t dof_;
  World world_;
  std::map<Name, Component> component_;
  std::map<Name, Component>::iterator it_component_;

  /////////////////////////////Parameter list///////////////////////////////
  /*
  dof_
  world_.name
  world_.child
  world_.pose.position
  world_.pose.orientation
  world_.origin.velocity
  world_.origin.acceleration
  component_.at(name).parent
  component_.at(name).child.at(i)
  component_.at(name).relative_to_parent.position
  component_.at(name).relative_to_parent.orientation
  component_.at(name).pose_to_world.position
  component_.at(name).pose_to_world.orientation
  component_.at(name).origin.velocity
  component_.at(name).origin.acceleration
  component_.at(name).joint.id
  component_.at(name).joint.coefficient
  component_.at(name).joint.axis
  component_.at(name).joint.angle
  component_.at(name).joint.velocity
  component_.at(name).joint.acceleration
  component_.at(name).tool.id
  component_.at(name).tool.coefficient
  component_.at(name).tool.on_off
  component_.at(name).tool.value
  component_.at(name).inertia.mass
  component_.at(name).inertia.inertia_tensor
  component_.at(name).inertia.center_of_mass
  */
  /////////////////////////////////////////////////////////////////////////////

public:
  Manipulator() : dof_(0){};
  ~Manipulator(){};

  ///////////////////////////initialize function/////////////////////////////

  void addWorld(Name world_name,
                Name child_name,
                Vector3d world_position = Vector3d::Zero(),
                Matrix3d world_orientation = Matrix3d::Identity(3, 3));

  void addComponent(Name my_name,
                    Name parent_name,
                    Name child_name,
                    Vector3d relative_position,
                    Matrix3d relative_orientation,
                    Vector3d axis_of_rotation = Vector3d::Zero(),
                    int8_t joint_actuator_id = -1,
                    double_t coefficient = 1.0f,
                    double_t mass = 0.0f,
                    Matrix3d inertia_tensor = Matrix3d::Identity(3, 3),
                    Vector3d center_of_mass = Vector3d::Zero());

  void addTool(Name my_name,
               Name parent_name,
               Vector3d relative_position,
               Matrix3d relative_orientation,
               int8_t tool_id = -1,
               double_t coefficient = 1.0f,
               double_t mass = 0.0f,
               Matrix3d inertia_tensor = Matrix3d::Identity(3, 3),
               Vector3d center_of_mass = Vector3d::Zero());

  void addComponentChild(Name my_name, Name child_name);
  void checkManipulatorSetting();

  ///////////////////////////////Set function//////////////////////////////////

  void setWorldPose(Pose world_pose);
  void setWorldPosition(Vector3d world_position);
  void setWorldOrientation(Matrix3d world_orientation);
  void setWorldState(State world_state);
  void setWorldVelocity(VectorXd world_velocity);
  void setWorldAcceleration(VectorXd world_acceleration);

  void setComponent(Name name, Component component, bool *error = NULL);
  void setComponentPoseToWorld(Name name, Pose pose_to_world);
  void setComponentPositionToWorld(Name name, Vector3d position_to_world);
  void setComponentOrientationToWorld(Name name, Matrix3d orientation_to_wolrd);
  void setComponentStateToWorld(Name name, State state_to_world);
  void setComponentVelocityToWorld(Name name, VectorXd velocity);
  void setComponentAccelerationToWorld(Name name, VectorXd accelaration);
  void setComponentJointAngle(Name name, double_t angle);
  void setComponentJointVelocity(Name name, double_t angular_velocity);
  void setComponentJointAcceleration(Name name, double_t angular_acceleration);
  void setComponentToolOnOff(Name name, bool on_off);
  void setComponentToolValue(Name name, double_t value);

  void setAllActiveJointAngle(std::vector<double_t> angle_vector);

  ///////////////////////////////Get function//////////////////////////////////

  int8_t getDOF();

  Name getWorldName();
  Name getWorldChildName();
  Pose getWorldPose();
  Vector3d getWorldPosition();
  Matrix3d getWorldOrientation();
  State getWorldState();
  VectorXd getWorldVelocity();
  VectorXd getWorldAcceleration();

  int8_t getComponentSize();
  std::map<Name, Component> getAllComponent();
  std::map<Name, Component>::iterator getIteratorBegin();
  std::map<Name, Component>::iterator getIteratorEnd();
  Component getComponent(Name name);
  Name getComponentParentName(Name name);
  std::vector<Name> getComponentChildName(Name name);
  Pose getComponentPoseToWorld(Name name);
  Vector3d getComponentPositionToWorld(Name name);
  Matrix3d getComponentOrientationToWorld(Name name);
  State getComponentStateToWorld(Name name);
  VectorXd getComponentVelocityToWorld(Name name);
  VectorXd getComponentAccelerationToWorld(Name name);
  Pose getComponentRelativePoseToParent(Name name);
  Vector3d getComponentRelativePositionToParent(Name name);
  Matrix3d getComponentRelativeOrientationToParent(Name name);
  Joint getComponentJoint(Name name);
  int8_t getComponentJointId(Name name);
  double_t getComponentJointCoefficient(Name name);
  Vector3d getComponentJointAxis(Name name);
  double_t getComponentJointAngle(Name name);
  double_t getComponentJointVelocity(Name name);
  double_t getComponentJointAcceleration(Name name);
  Tool getComponentTool(Name name);
  int8_t getComponentToolId(Name name);
  double_t getComponentToolCoefficient(Name name);
  bool getComponentToolOnOff(Name name);
  double_t getComponentToolValue(Name name);
  double_t getComponentMass(Name name);
  Matrix3d getComponentInertiaTensor(Name name);
  Vector3d getComponentCenterOfMass(Name name);

  std::vector<double_t> getAllJointAngle();
  std::vector<double_t> getAllActiveJointAngle();
  std::vector<uint8_t> getAllActiveJointID();
};

class Kinematics : public Manipulator
{
public:
  Kinematics(){};
  ~Kinematics(){};

  MatrixXd jacobian(Manipulator *manipulator, Name tool_name);
  void forward(Manipulator *manipulator);
  void forward(Manipulator *manipulator, Name component_name);
  std::vector<double_t> inverse(Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<double_t> InverseKinematicsUsing3DOF(Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<double_t> srInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<double_t> positionOnlyInverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose);
  std::vector<double_t> inverseKinematics(Manipulator *manipulator, Name tool_name, Pose target_pose);

};

}

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////OMMath//////////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
#define DEG2RAD 0.01745329252f //(M_PI / 180.0)
#define RAD2DEG 57.2957795131f //(180.0 / M_PI)

#define ZERO_VECTOR Vector3d::Zero()
#define IDENTITY_MATRIX Matrix3d::Identity(3, 3)

namespace OM_MATH
{
double_t sign(double_t number);

Vector3d makeVector3(double_t v1, double_t v2, double_t v3);
Matrix3d makeMatrix3(double_t m11, double_t m12, double_t m13,
                     double_t m21, double_t m22, double_t m23,
                     double_t m31, double_t m32, double_t m33);

Vector3d matrixLogarithm(Matrix3d rotation_matrix);
Matrix3d skewSymmetricMatrix(Vector3d v);
Matrix3d rodriguesRotationMatrix(Vector3d axis, double_t angle);

Matrix3d makeRotationMatrix(double_t roll, double_t pitch, double_t yaw);
Matrix3d makeRotationMatrix(Vector3d rotation_vector);
Vector3d makeRotationVector(Matrix3d rotation_matrix);
std::vector<double_t> MakeQuaternion(Matrix3d rotation_matrix);


Vector3d positionDifference(Vector3d desired_position, Vector3d present_position);
Vector3d orientationDifference(Matrix3d desired_orientation, Matrix3d present_orientation);
VectorXd poseDifference(Vector3d desired_position, Vector3d present_position,
                        Matrix3d desired_orientation, Matrix3d present_orientation);




template <typename T>
T map(T x, T in_min, T in_max, T out_min, T out_max)
{
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}
} // namespace MATH

////////////////////////////////////////////////////////////////////////////////////
////////////////////////////////////Social_robot////////////////////////////////////
////////////////////////////////////////////////////////////////////////////////////
namespace social_robot_arm
{

class SocialRobotArmKinematics
{
public:
  SocialRobotArmKinematics();
  virtual ~SocialRobotArmKinematics();

  void initialize(Eigen::MatrixXd body_position, Eigen::MatrixXd body_orientation);
  void setJointPosition(Eigen::VectorXd rarm_joint_position, Eigen::VectorXd larm_joint_position);
  void solveForwardKinematics(std::vector<double_t> &rarm_position, std::vector<double_t> &rarm_orientation,
                              std::vector<double_t> &larm_position, std::vector<double_t> &larm_orientation);
  bool solveInverseKinematics(std::vector<double_t> &rarm_output,
                              Eigen::MatrixXd rarm_target_position, Eigen::Quaterniond rarm_target_orientation,
                              std::vector<double_t> &larm_output,
                              Eigen::MatrixXd larm_target_position, Eigen::Quaterniond larm_target_orientation);
  void finalize();

protected:

  OPEN_MANIPULATOR::Manipulator *rarm_chain_;
  OPEN_MANIPULATOR::Manipulator *larm_chain_;
  OPEN_MANIPULATOR::Kinematics *rarm_kinematics_;
  OPEN_MANIPULATOR::Kinematics *larm_kinematics_;

  Eigen::VectorXd rarm_joint_position_, larm_joint_position_;
  geometry_msgs::Pose rarm_pose_, larm_pose_;
};

}

#endif
