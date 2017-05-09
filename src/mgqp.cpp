/* Author: William Attié
 * Date:   24/04/20017
 *
 * Description: This is a simple orocos/rtt component to control a robot
 *              using a cascade of quadratic programs
 */

#include "mgqp.hpp"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>

/* =======================================================
                    CONSTRAINT DEFINITION
========================================================== */

Constraint::Constraint (ConstraintType type, ConstraintIty type2, int jointNumber, float valueOfReference)
{
  this->type = type;
  this->ity = type2;
  this->refValue = valueOfReference;
  this->target = jointNumber;
}

/*
  returns the error made to a constraint.
  for not violated inequalities, the error is zero
*/
float Constraint::getError(Robot* robot)
{
  if (this->ity == EQ || this->ity == _SUP || this->ity == _INF)
  {
    if (this->type == POSITION)
    {
      return robot->jointPositions[this->target]-this->refValue;
    }
  }
  if (this->ity == SUP)
  {
    if (this->type == POSITION)
    {
      // if a constraint is violated, we change its type to violated which will make it being evaluater as an equality.
      if (robot->jointPositions[this->target] > this->refValue)
      {
        this->ity = _SUP;
        return this->getError(robot);
      }
    }
  }
  else
  {
    return 0;
  }
}

/* =======================================================
             MotionGenerationQuadraticProgram
========================================================== */
MotionGenerationQuadraticProgram::MotionGenerationQuadraticProgram(std::string const & name) : RTT::TaskContext(name) {
    // constructor
    addOperation("setDOFsize", &MotionGenerationQuadraticProgram::setDOFsize, this, RTT::ClientThread).doc("set DOF size");
    addOperation("printCurrentState", &MotionGenerationQuadraticProgram::printCurrentState, this, RTT::ClientThread).doc("print current state");

    magnitude = 1.0;
    addProperty("trajectory_magnitude", magnitude).doc("Magnitude of sinusoidal trajectory");
    portsPrepared = false;

    constraints = std::vector<Constraint>();
}

bool MotionGenerationQuadraticProgram::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    // setting gains for controller:


    if (!(out_torques_port.connected() && (in_robotstatus_port.connected())))
        return false;
    else
        return true;
}

bool MotionGenerationQuadraticProgram::startHook() {
    // this method starts the component
    return true;
}

Constraint MotionGenerationQuadraticProgram::desiredPositionToConstraint(Eigen::Vector3f position)
{


}

void MotionGenerationQuadraticProgram::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    if (in_robotstatus_port.connected()) {
        // read into "currJntPos" and save state of data into "currJntPos_Flow", which can be "NewData", "OldData" or "NoData".
        in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    } else {
        // handle the situation
    }

    // you can handle cases when there is no new data.
    if (in_robotstatus_flow == RTT::NewData){
        // do next :)
    } else if (in_robotstatus_flow == RTT::OldData) {
        return;
    } else if (in_robotstatus_flow == RTT::NoData){
        return;
    } else {
        // there should be something really wrong!
    }

    // actual controller!
    for(int i=0; i<DOFsize; ++i){
        q_des.angles(i)        = magnitude*sin(getSimulationTime());
        qDot_des.velocities(i) = magnitude*cos(getSimulationTime());
    }

    out_torques_var.torques = 1*(q_des.angles - in_robotstatus_var.angles) + 100*(qDot_des.velocities - in_robotstatus_var.velocities);

    // write it to port
    out_torques_port.write(out_torques_var);
}

void MotionGenerationQuadraticProgram::stopHook() {
    // stops the component (update hook wont be  called anymore)
}

void MotionGenerationQuadraticProgram::cleanupHook() {
    // cleaning the component data
}

double MotionGenerationQuadraticProgram::getSimulationTime() {
    return 1E-9 * RTT::os::TimeService::ticks2nsecs(RTT::os::TimeService::Instance()->getTicks());
}

void MotionGenerationQuadraticProgram::setDOFsize(unsigned int DOFsize){
    this->DOFsize = DOFsize;

    if(portsPrepared){
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("out_torques_port");
    }

    // prepare input ports:
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    //prepare output ports:
    out_torques_var = rstrt::dynamics::JointTorques(DOFsize);
    out_torques_var.torques.setZero();
    out_torques_port.setName("out_torques_port");
    out_torques_port.doc("Output port for sending torque values");
    out_torques_port.setDataSample(out_torques_var);
    ports()->addPort(out_torques_port);

    portsPrepared = true;

    // initializing data
    q_des = rstrt::kinematics::JointAngles(DOFsize);
    q_des.angles.setZero();

    qDot_des = rstrt::kinematics::JointVelocities(DOFsize);
    qDot_des.velocities.setZero();
}

void MotionGenerationQuadraticProgram::printCurrentState(){
    std::cout << "############## MotionGenerationQuadraticProgram State begin " << std::endl;
    std::cout << " feedback angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " feedback velocities " << in_robotstatus_var.velocities << std::endl;
    std::cout << " feedback torques " << in_robotstatus_var.torques << std::endl;
    std::cout << " command torques " << out_torques_var.torques << std::endl;
    std::cout << "############## MotionGenerationQuadraticProgram State end " << std::endl;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(MotionGenerationQuadraticProgram)
