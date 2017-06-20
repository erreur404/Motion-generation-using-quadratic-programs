/* Author: William Attié
 * Date:   24/04/20017
 *
 * Description: This is a simple orocos/rtt component to control a robot
 *              using a cascade of quadratic programs
 */

#pragma once

// RTT header files. Might missing some or some be unused
#include <rtt/Port.hpp>
#include <rtt/TaskContext.hpp>
#include <string>
#include <vector>
#include <Eigen/Dense>
#include "QuadProgpp/QuadProg++.hh"
#include "QuadProgpp/Array.hh"

// Joint value datatype:
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/robot/JointState.hpp>
#include "QuaternionHelper.hpp"

/* =======================================================
             MotionGenerationQuadraticProgram
========================================================== */

struct {
  Eigen::MatrixXf conditions;
  Eigen::VectorXf goal;
  Eigen::MatrixXf constraints;
  Eigen::VectorXf limits;
  int pbDOF;

  int nbConditions () {return conditions.rows();};
  int rows() {return conditions.rows();};
  int cols() {return conditions.cols();};
  int init(int DOFsize) {
      pbDOF = DOFsize;
      conditions = Eigen::MatrixXf(0, DOFsize);
      goal = Eigen::VectorXf(0);
      return 0;
  };
  int dof() {return pbDOF;};
  void dof(int dof) {init(dof);};
} typedef QuadraticProblem;

struct {
  int stackSize;
  std::vector<QuadraticProblem> qps;
  std::map<std::string, int> level;

  int init(int nbOfLevels) {stackSize = nbOfLevels; qps.resize(stackSize);};
  QuadraticProblem * getQP(int level) {return &(qps[level]);}; // has to be addresses because otherwise the return will only send a copy
  //int getLevel(std::string task) {int l = -1; for (int i=0; i<level.size(); i++){for(int j=0; j<level[i].size(); j++) {if (level[i][j] == task){l = i;}}} return l;};
  //int getAddress(std::string task) {int l = -1; for (int i=0; i<level.size(); i++){for(int j=0; j<level[i].size(); j++) {if (level[i][j] == task){l = j;}}} return l;};
  int getLevel(std::string task) {if(level.find(task) == level.end()){return -1;} return level[task];}
  bool setPriority(std::string task, int priorityLevel) {if (priorityLevel >= stackSize) {return false;} level[task] = priorityLevel;};
} typedef StackOfTasks;

/* =======================================================
             MotionGenerationQuadraticProgram
========================================================== */

class MotionGenerationQuadraticProgram: public RTT::TaskContext {
public:
    MotionGenerationQuadraticProgram(std::string const & name);

    // RTT::TaskContext methods that are needed for any standard component and
    // should be implemented by user
    bool configureHook();
    bool startHook();
    void updateHook();
    void stopHook();
    void cleanupHook();

    void setGains(float kp, float kd);
    void setGainsOrientation(float kp, float kd);

    bool setTorqueLimitsE(Eigen::VectorXf torquesP, Eigen::VectorXf torquesN);
    bool setAccelerationLimitsE(Eigen::VectorXf accelerationsP, Eigen::VectorXf accelerationsN);
    bool setAngularLimitsE(Eigen::VectorXf jointsP, Eigen::VectorXf jointsN);
    //*
    bool setTorqueLimits(std::vector<double> torquesP, std::vector<double> torquesN);
    bool setAccelerationLimits(std::vector<double> accelerationsP, std::vector<double> accelerationsN);
    bool setAngularLimits(std::vector<double> limitSup, std::vector<double> limitInf);
    // */

    bool setPriorityLevel(std::string task, int level);

    void preparePorts();

private:
    // Declare ports and their datatypes
    std::vector<RTT::InputPort<Eigen::VectorXf>* > in_desiredTaskSpacePosition_port;
    std::vector<RTT::InputPort<Eigen::VectorXf>* > in_desiredTaskSpaceVelocity_port;
    std::vector<RTT::InputPort<Eigen::VectorXf>* > in_desiredTaskSpaceAcceleration_port;

    std::vector<RTT::InputPort<float>* > in_desiredJointSpacePosition_port;
    std::vector<RTT::InputPort<float>* > in_desiredJointSpaceVelocity_port;
    std::vector<RTT::InputPort<float>* > in_desiredJointSpaceAcceleration_port;

    std::vector<RTT::InputPort<Eigen::VectorXf>* > in_currentTaskSpacePosition_port;
    std::vector<RTT::InputPort<Eigen::VectorXf>* > in_currentTaskSpaceVelocity_port;

    std::vector<RTT::InputPort<Eigen::MatrixXf>* > in_jacobian_port;
    std::vector<RTT::InputPort<Eigen::MatrixXf>* > in_jacobianDot_port;

    RTT::InputPort<Eigen::VectorXf> in_h_port;
    RTT::InputPort<Eigen::MatrixXf> in_inertia_port;
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;
    RTT::OutputPort<Eigen::VectorXf> out_force_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;
    RTT::FlowStatus in_h_flow;
    RTT::FlowStatus in_inertia_flow;

    std::vector<RTT::FlowStatus> in_desiredTaskSpacePosition_flow;
    std::vector<RTT::FlowStatus> in_desiredTaskSpaceVelocity_flow;
    std::vector<RTT::FlowStatus> in_desiredTaskSpaceAcceleration_flow;

    std::vector<RTT::FlowStatus> in_desiredJointSpacePosition_flow;
    std::vector<RTT::FlowStatus> in_desiredJointSpaceVelocity_flow;
    std::vector<RTT::FlowStatus> in_desiredJointSpaceAcceleration_flow;

    std::vector<RTT::FlowStatus> in_currentTaskSpacePosition_flow;
    std::vector<RTT::FlowStatus> in_currentTaskSpaceVelocity_flow;

    std::vector<RTT::FlowStatus> in_jacobian_flow;
    std::vector<RTT::FlowStatus> in_jacobianDot_flow;


    // Actuall joint command to be sent over port:
    rstrt::kinematics::JointAngles q_des;
    rstrt::kinematics::JointVelocities qDot_des;

    // variables
    bool portsPrepared;
    // to handle task space conditions
    Eigen::VectorXf in_desiredTaskSpacePosition_var;
    Eigen::VectorXf in_desiredTaskSpaceVelocity_var;
    Eigen::VectorXf in_desiredTaskSpaceAcceleration_var;

    // to handle joint space condition
    float in_desiredJointSpacePosition_var;
    float in_desiredJointSpaceVelocity_var;
    float in_desiredJointSpaceAcceleration_var;

    // feedback from the real world
    Eigen::VectorXf in_currentTaskSpacePosition_var;
    Eigen::VectorXf in_currentTaskSpaceVelocity_var;
    rstrt::robot::JointState in_robotstatus_var;


    Eigen::MatrixXf in_jacobian_var;
    Eigen::MatrixXf in_jacobianDot_var;
    Eigen::VectorXf in_h_var;
    Eigen::MatrixXf in_inertia_var;

    // commands sent to the robot
    rstrt::dynamics::JointTorques out_torques_var;
    Eigen::VectorXf out_force_var;

    Eigen::Vector4f quaternion_desired, quaternion_current, quaternion_current_conj, quaternion_diff;
    Eigen::Quaternionf quat_target,quat_current,quat_diff;
    Eigen::Vector3f errorTranslationPosition, errorTranslationVelocity;
    Eigen::Vector3f errorOrientationPosition, errorOrientationVelocity;
    Eigen::VectorXf errorPosition, errorVelocity;
    Eigen::Vector3f desiredPosition, currentPosition, desiredVelocity, currentVelocity, desiredAcceleration, currentAcceleration;
    float desiredJointPosition, desiredJointVelocity, desiredJointAcceleration;

    unsigned int DOFsize;
    StackOfTasks stack_of_tasks;
    QuadraticProblem * prob;

    // Joints limits
    Eigen::VectorXf JointTorquesLimitsP;
    Eigen::VectorXf JointTorquesLimitsN;
    Eigen::VectorXf JointAccelerationLimitsP;
    Eigen::VectorXf JointAccelerationLimitsN;
    Eigen::VectorXf JointLimitsSup;
    Eigen::VectorXf JointLimitsInf;

    bool receiveTranslationOnly;
    unsigned int TaskSpaceDimension;
    float gainTranslationP, gainTranslationD, gainOrientationP, gainOrientationD;
    unsigned int numEndEffectors;
    unsigned int WorkspaceDimension;
    float velocityLimit;
    Eigen::MatrixXf identityTSdimTSdim, tmpeyeTSdimTSdim, tmpeye33;
    Eigen::MatrixXf lambdaTranslation, lambdaOrientation;
    Eigen::VectorXf pseudoVelocity;
    Eigen::MatrixXf limiter;
    Eigen::VectorXf ref_Acceleration, constraintForce;
    QuaternionHelper qh;
    Eigen::Vector4f quaternion_desired1, quaternion_current1, quaternion_desired2, quaternion_current2;




    // methods
    void setDOFsize(unsigned int DOFsize);
    bool solveNextStep(const Eigen::MatrixXf A, const Eigen::VectorXf a, const Eigen::MatrixXf B, const Eigen::VectorXf b, Eigen::VectorXf * res);
    Eigen::VectorXf solveNextHierarchy();
    void printCurrentState();

    // helpers:
    double getSimulationTime();
};
