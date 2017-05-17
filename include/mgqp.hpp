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
#include <QuadProg++.hh>
#include <Array.hh>

// Joint value datatype:
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/robot/JointState.hpp>
#include "QuaternionHelper.hpp"


/* =======================================================
                    ROBOT STRUCTURE
========================================================== */
typedef struct {
  int dof;
  /*
  jointStatus
  Array of jacobians pos
  Array of Jacobians Dot
  */
  float * jointPositions;
  std::string toString();
} Robot;

/* =======================================================
                    CONSTRAINT DEFINITION
========================================================== */
class taskMask {
  public:
      static const int posX=1<<0;
      static const int posY=1<<1;
      static const int posZ=1<<2;
      static const int oriX=1<<3;
      static const int oriY=1<<4;
      static const int oriZ=1<<5;
      static const int speX=1<<6;
      static const int speY=1<<7;
      static const int speZ=1<<8;
      static const int angX=1<<9;
      static const int angY=1<<10;
      static const int angZ=1<<11;
      static const int accX=1<<12;
      static const int accY=1<<13;
      static const int accZ=1<<14;
      static const int aacX=1<<15;
      static const int aacY=1<<16;
      static const int aacZ=1<<17;
      static const int torX=1<<18;
      static const int torY=1<<19;
      static const int torZ=1<<20;
      static const int momX=1<<21;
      static const int momY=1<<22;
      static const int momZ=1<<23;
      static const int cartP=1<<0 | 1<<1 | 1<<2;
      static const int cartS=7<<6;
};

class Constraint {
  public:
    Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> A;
    Eigen::VectorXf a;
    int task;

  public:
    Constraint(int tasks, Eigen::VectorXf goal);
    Constraint(Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> JacobianMask, Eigen::VectorXf goal);
    void createMask(int nbRobotDoF, int jointNumber);
    void createTaskMask(int nbTaskDoF, int task);
    std::string toString();
};

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

    void preparePorts();

private:
    // Declare ports and their datatypes
    RTT::InputPort<Eigen::VectorXf> in_desiredTaskSpacePosition_port;
    RTT::InputPort<Eigen::VectorXf> in_desiredTaskSpaceVelocity_port;
    RTT::InputPort<Eigen::VectorXf> in_desiredTaskSpaceAcceleration_port;

    RTT::InputPort<Eigen::VectorXf> in_currentTaskSpacePosition_port;
    RTT::InputPort<Eigen::VectorXf> in_currentTaskSpaceVelocity_port;
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;

    RTT::InputPort<Eigen::MatrixXf> in_jacobian_port;
    RTT::InputPort<Eigen::MatrixXf> in_jacobianDot_port;
    RTT::InputPort<Eigen::VectorXf> in_h_port;
    RTT::InputPort<Eigen::MatrixXf> in_constraintMinvP_port;
    RTT::InputPort<Eigen::MatrixXf> in_constraintC_port;

    // Declare output ports and their datatypes
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;
    RTT::OutputPort<Eigen::VectorXf> out_force_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;
    RTT::FlowStatus in_desiredTaskSpacePosition_flow;
    RTT::FlowStatus in_desiredTaskSpaceVelocity_flow;
    RTT::FlowStatus in_desiredTaskSpaceAcceleration_flow;

    RTT::FlowStatus in_currentTaskSpacePosition_flow;
    RTT::FlowStatus in_currentTaskSpaceVelocity_flow;

    RTT::FlowStatus in_jacobian_flow;
    RTT::FlowStatus in_jacobianDot_flow;
    RTT::FlowStatus in_h_flow;

    // Actuall joint command to be sent over port:
    rstrt::kinematics::JointAngles q_des;
    rstrt::kinematics::JointVelocities qDot_des;

    // variables
    bool portsPrepared;
    std::vector<Constraint> constraints;
      // for the purpose of debug, will use later only the inner constraints
      Eigen::VectorXf in_desiredTaskSpacePosition_var;
      Eigen::VectorXf in_desiredTaskSpaceVelocity_var;
      Eigen::VectorXf in_desiredTaskSpaceAcceleration_var;

    // for the sake of receiving feedback !
    Eigen::VectorXf in_currentTaskSpacePosition_var;
    Eigen::VectorXf in_currentTaskSpaceVelocity_var;
    rstrt::robot::JointState in_robotstatus_var;

    Eigen::MatrixXf in_jacobian_var;
    Eigen::MatrixXf in_jacobianDot_var;
    Eigen::VectorXf in_h_var;

    rstrt::dynamics::JointTorques out_torques_var;
    Eigen::VectorXf out_force_var;

    Eigen::Vector4f quaternion_desired, quaternion_current, quaternion_current_conj, quaternion_diff;
    Eigen::Quaternionf quat_target,quat_current,quat_diff;
    Eigen::Vector3f errorTranslationPosition, errorTranslationVelocity;
    Eigen::Vector3f errorOrientationPosition, errorOrientationVelocity;
    Eigen::VectorXf errorPosition, errorVelocity;
    Eigen::Vector3f desiredPosition, currentPosition, desiredVelocity, currentVelocity, desiredAcceleration, currentAcceleration;

    unsigned int DOFsize;
    bool receiveTranslationOnly;
    bool add_TSgravitycompensation;
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

    bool ranOnce;





    // methods
    void setDOFsize(unsigned int DOFsize);
    Eigen::VectorXf solveNextStep(const Eigen::MatrixXf A, const Eigen::VectorXf a, const Eigen::MatrixXf B, const Eigen::VectorXf b);
    void printCurrentState();
    void desiredPositionToConstraint(Eigen::Vector3f position);

    // helpers:
    double getSimulationTime();
};
