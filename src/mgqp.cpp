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
                    ROBOT STRUCTURE
========================================================== */
std::string Robot::toString ()
{
  std::ostringstream stringStream;
  stringStream << "Je suis un RRRRobot !";
  return stringStream.str();
}

/* =======================================================
                    CONSTRAINT DEFINITION
========================================================== */

Constraint::Constraint (ConstraintType type, ConstraintIty type2, int jointNumber)
{
  this->type = type;
  this->ity = type2;
  this->target = jointNumber;
}

Constraint Constraint::newConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, float valueOfReference)
{
  Constraint c = Constraint(type, type2, jointNumber);
  c.refValue = valueOfReference;
  return c;
}

Constraint Constraint::newConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, Eigen::Vector3f valueOfReference)
{
  Constraint c = Constraint(type, type2, jointNumber);
  *c.refValueP = valueOfReference;
  return c;
}

Constraint Constraint::newConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, Eigen::Matrix3f valueOfReference)
{
  Constraint c = Constraint(type, type2, jointNumber);
  *c.refValueO = valueOfReference;
  return c;
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

std::string Constraint::toString()
{
  // type EQ|INEQ ----- Activated*|Inactivated ----- target ----- type POS|OR ----- value
  std::ostringstream stringStream;
  stringStream  << "Constraint["
                << (this->ity == EQ ? "Equality  " : "Inequality")<<"\t"
                << (this->ity == EQ ||this->ity == _SUP || this->ity == _INF ? "*" : " ")<<"\t"
                << (this->target)<<"\t"
                << (this->refValueP != 0 ? "Position   " :
                    (this->refValueO != 0 ?"Orientation" :
                                              "Value      "))<<"\t"
                /*<< (this->refValueP != 0 ?  *this->refValueP :
                    (this->refValueO != 0 ? *this->refValueO :
                                               this->refValue))*/
                << this->refValue
                <<"]"<<std::endl;
  return stringStream.str(); //"%s \t %c \t %d \t %s"
}

/* =======================================================
             MotionGenerationQuadraticProgram
========================================================== */
MotionGenerationQuadraticProgram::MotionGenerationQuadraticProgram(std::string const & name) : RTT::TaskContext(name) {
    // constructor
    addOperation("setDOFsize", &MotionGenerationQuadraticProgram::setDOFsize, this, RTT::ClientThread).doc("set DOF size");
    addOperation("printCurrentState", &MotionGenerationQuadraticProgram::printCurrentState, this, RTT::ClientThread).doc("print current state");
    addOperation("addConstraint",
    &MotionGenerationQuadraticProgram::addConstraint, this, RTT::ClientThread).doc("addConstraint(ConstraintType:POSITION=1|SPEED=2|TORQUE=3|ACCELERATION=3, ConstraintIty:EQ=1|SUP=2|INF=3, targetJoint:int, targetValue: float");
    addOperation("addConstraintP",
    &MotionGenerationQuadraticProgram::addConstraintP, this, RTT::ClientThread).doc("addConstraint(ConstraintType:POSITION=1|SPEED=2|TORQUE=3|ACCELERATION=3, ConstraintIty:EQ=1|SUP=2|INF=3, targetJoint:int, targetValue: Vector3f");
    addOperation("addConstraintO",
    &MotionGenerationQuadraticProgram::addConstraintO, this, RTT::ClientThread).doc("addConstraint(ConstraintType:POSITION=1|SPEED=2|TORQUE=3|ACCELERATION=3, ConstraintIty:EQ=1|SUP=2|INF=3, targetJoint:int, targetValue: Matrix3f");

    portsPrepared = false;

    constraints = std::vector<Constraint>();

    velocityLimit = 0.2;

    gainTranslationP = 100;
    gainTranslationD = 20;

    quaternion_desired = Eigen::Vector4f::Zero();
    quaternion_current = Eigen::Vector4f::Zero();
    quaternion_current_conj = Eigen::Vector4f::Zero();
    quaternion_diff = Eigen::Vector4f::Zero();

    quat_target = Eigen::Quaternionf();
    quat_current = Eigen::Quaternionf();
    quat_diff = Eigen::Quaternionf();

    desiredPosition = Eigen::Vector3f::Zero();
    currentPosition = Eigen::Vector3f::Zero();
    desiredVelocity = Eigen::Vector3f::Zero();
    currentVelocity = Eigen::Vector3f::Zero();

    errorTranslationPosition = Eigen::Vector3f::Zero();
    errorTranslationVelocity = Eigen::Vector3f::Zero();
    errorOrientationPosition = Eigen::Vector3f::Zero();
    errorOrientationVelocity = Eigen::Vector3f::Zero();

    qh = QuaternionHelper();
    quaternion_desired1 = Eigen::Vector4f::Zero();
    quaternion_current1 = Eigen::Vector4f::Zero();
    quaternion_desired2 = Eigen::Vector4f::Zero();
    quaternion_current2 = Eigen::Vector4f::Zero();
}

bool MotionGenerationQuadraticProgram::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    // setting gains for controller:

    if (!in_robotstatus_port.connected()) {
      RTT::log(RTT::Info) << "in_robotstatus_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_desiredTaskSpacePosition_port.connected()) {
      RTT::log(RTT::Info) << "in_desiredTaskSpacePosition_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_desiredTaskSpaceVelocity_port.connected()) {
      RTT::log(RTT::Info) << "in_desiredTaskSpaceVelocity_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_desiredTaskSpaceAcceleration_port.connected()) {
      RTT::log(RTT::Info) << "in_desiredTaskSpaceAcceleration_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_currentTaskSpacePosition_port.connected()) {
      RTT::log(RTT::Info) << "in_currentTaskSpacePosition_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_currentTaskSpaceVelocity_port.connected()) {
      RTT::log(RTT::Info) << "in_currentTaskSpaceVelocity_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_jacobian_port.connected()) {
      RTT::log(RTT::Info) << "in_jacobian_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_jacobianDot_port.connected()) {
      RTT::log(RTT::Info) << "in_jacobianDot_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_h_port.connected()) {
      RTT::log(RTT::Info) << "in_h_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!in_constraintMinvP_port.connected()) {
      RTT::log(RTT::Info) << "in_constraintMinvP_port not connected"
      << RTT::endlog();
      return false;
    }
    if (!(out_torques_port.connected() && (in_robotstatus_port.connected())))
        return false;
    else
        return true;
}

bool MotionGenerationQuadraticProgram::startHook() {
    // this method starts the component
    return true;
}

void MotionGenerationQuadraticProgram::desiredPositionToConstraint(Eigen::Vector3f position)
{
      if (this->constraints.size() > 1)
      {
        this->constraints.pop_back();
      }
      this->addConstraintP(POSITION, EQ, this->DOFsize-1, position);
}
//*
void MotionGenerationQuadraticProgram::addConstraint (ConstraintType type, ConstraintIty type2, int jointNumber, float valueOfReference)
{
  this->constraints.push_back(Constraint::newConstraint(type, type2, jointNumber, valueOfReference));
} // */

void MotionGenerationQuadraticProgram::addConstraintP (ConstraintType type, ConstraintIty type2, int jointNumber, Eigen::Vector3f valueOfReference)
{
  this->constraints.push_back(Constraint::newConstraint(type, type2, jointNumber, valueOfReference));
}
//*
void MotionGenerationQuadraticProgram::addConstraintO (ConstraintType type, ConstraintIty type2, int jointNumber, Eigen::Matrix3f valueOfReference)
{
  this->constraints.push_back(Constraint::newConstraint(type, type2, jointNumber, valueOfReference));
} // */

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

void MotionGenerationQuadraticProgram::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    in_desiredTaskSpacePosition_flow = in_desiredTaskSpacePosition_port.read(in_desiredTaskSpacePosition_var);
    in_desiredTaskSpaceVelocity_flow = in_desiredTaskSpaceVelocity_port.read(in_desiredTaskSpaceVelocity_var);
    in_desiredTaskSpaceAcceleration_flow = in_desiredTaskSpaceAcceleration_port.read(in_desiredTaskSpaceAcceleration_var);
    in_currentTaskSpacePosition_flow = in_currentTaskSpacePosition_port.read(in_currentTaskSpacePosition_var);
    in_currentTaskSpaceVelocity_flow = in_currentTaskSpaceVelocity_port.read(in_currentTaskSpaceVelocity_var);
    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);

    in_jacobian_flow = in_jacobian_port.read(in_jacobian_var);
    in_jacobianDot_flow = in_jacobianDot_port.read(in_jacobianDot_var);
    in_h_flow = in_h_port.read(in_h_var);

    if (in_desiredTaskSpacePosition_flow == RTT::NoData
      || in_desiredTaskSpaceVelocity_flow == RTT::NoData
      || in_desiredTaskSpaceAcceleration_flow == RTT::NoData
      || in_currentTaskSpacePosition_flow == RTT::NoData
      || in_currentTaskSpaceVelocity_flow == RTT::NoData
      || in_robotstatus_flow == RTT::NoData
      || in_jacobian_flow == RTT::NoData
      || in_jacobianDot_flow == RTT::NoData
      || in_h_flow == RTT::NoData)
    {
        return;
    }

    // reading the variables from the flows
    desiredPosition = in_desiredTaskSpacePosition_var.segment<3>(WorkspaceDimension);
    currentPosition = in_currentTaskSpacePosition_var.segment<3>(WorkspaceDimension);
    desiredVelocity = in_desiredTaskSpaceVelocity_var.segment<3>(WorkspaceDimension);
    currentVelocity = in_currentTaskSpaceVelocity_var.segment<3>(WorkspaceDimension);
    // Filling the robot structure

    // Dev : setting the EE desired position as a constraint
    desiredPositionToConstraint(desiredPosition);
    // actual controller!
    for(int i=0; i<DOFsize; ++i){
      q_des.angles(i)        = 1*sin(getSimulationTime());
      qDot_des.velocities(i) = 1*cos(getSimulationTime());
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

void MotionGenerationQuadraticProgram::printCurrentState(){
    std::cout << "############## MotionGenerationQuadraticProgram State begin " << std::endl;
    /*
    std::cout << " feedback angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " feedback velocities " << in_robotstatus_var.velocities << std::endl;
    std::cout << " feedback torques " << in_robotstatus_var.torques << std::endl;
    std::cout << " command torques " << out_torques_var.torques << std::endl;
    */
    for (int i=0; i<this->constraints.size(); ++i)
    {
      std::cout << this->constraints[i].toString() << '\n';
    }
    std::cout << "############## MotionGenerationQuadraticProgram State end " << std::endl;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(MotionGenerationQuadraticProgram)
