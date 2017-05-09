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

// Joint value datatype:
#include <rst-rt/kinematics/JointAngles.hpp>
#include <rst-rt/kinematics/JointVelocities.hpp>
#include <rst-rt/dynamics/JointTorques.hpp>
#include <rst-rt/robot/JointState.hpp>


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
} Robot;

/* =======================================================
                    CONSTRAINT DEFINITION
========================================================== */

enum ConstraintIty { EQ=1, SUP=2, INF=3, _SUP=4, _INF=5 };
enum ConstraintType { POSITION=1, SPEED=2, TORQUE=3, ACCELERATION=3};

class Constraint {
  private:
    ConstraintType type;
    ConstraintIty ity;
    float refValue; // for simple variables
    Eigen::Vector3f refValueP;// for position vector
    Eigen::Matrix3f refValueO;// for orientation Matrix
    int target;

  public:
    static Constraint newConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, float valueOfReference);
    static Constraint newConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, Eigen::Vector3f valueOfReference);
    static Constraint newConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, Eigen::Matrix3f valueOfReference);
    float getError(Robot* robot);

  private:
    Constraint (ConstraintType type, ConstraintIty type2, int jointNumber);
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
    void computeTranslationError(
            // executed by each constraint
            Eigen::Vector3f const & cart_desiredPosition,
            Eigen::Vector3f const & cart_currentPosition,
            Eigen::Vector3f const & desiredVelocity,
            Eigen::Vector3f const & cart_cart_currentVelocity,
            Eigen::Vector3f & cart_errorPosition,
            Eigen::Vector3f & cart_errorVelocity);
    void computeOrientationError(
            Eigen::Vector3f const & axisangle_desiredPosition,
            Eigen::Vector3f const & axisangle_currentPosition,
            Eigen::Vector3f const & axisangle_desiredVelocity,
            Eigen::Vector3f const & axisangle_currentVelocity,
            Eigen::Vector3f & axisangle_errorPosition,
            Eigen::Vector3f & axisangle_errorVelocity);
    void preparePorts();

private:
    // Declare ports and their datatypes
    RTT::InputPort<rstrt::robot::JointState> in_robotstatus_port;
    RTT::OutputPort<rstrt::dynamics::JointTorques> out_torques_port;

    // Data flow:
    RTT::FlowStatus in_robotstatus_flow;

    // Actuall joint command to be sent over port:
    rstrt::robot::JointState in_robotstatus_var;
    rstrt::dynamics::JointTorques out_torques_var;
    rstrt::kinematics::JointAngles q_des;
    rstrt::kinematics::JointVelocities qDot_des;

    unsigned int DOFsize;
    double magnitude;
    bool portsPrepared;
    std::vector<Constraint> constraints;

    void setDOFsize(unsigned int DOFsize);
    void printCurrentState();
    void desiredPositionToConstraint(Eigen::Vector3f position);
    //void addConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, float valueOfReference);
    void addConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, Eigen::Vector3f valueOfReference);
    //void addConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, Eigen::Matrix3f valueOfReference);

    // helpers:
    double getSimulationTime();
};
