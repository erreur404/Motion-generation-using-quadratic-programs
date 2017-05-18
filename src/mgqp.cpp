/* Author: William Attié
 * Date:   24/04/20017
 *
 * Description: This is a simple orocos/rtt component to control a robot
 *              using a cascade of quadratic programs
 */

#include "mgqp.hpp"
#include "Array.hh"
// needed for the macro at the end of this file:
#include <rtt/Component.hpp>

#define PRINT(txt) RTT::log(RTT::Info) << txt << RTT::endlog()


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

Constraint::Constraint(int tasks, Eigen::VectorXf goal)
{
  this->task = tasks;
  this->a = goal;
}

Constraint::Constraint(Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic> JacobianMask, Eigen::VectorXf goal)
{
  this->A = JacobianMask;
  this->a = goal;
}

void Constraint::createMask(int nbRobotDoF, int jointNumber)
{

}

void Constraint::createTaskMask(int nbTaskDoF, int task)
{
  this->A = Eigen::Matrix<bool, Eigen::Dynamic, Eigen::Dynamic>(nbTaskDoF, nbTaskDoF);
  this->task = task;
  //this->A.resize(nbTaskDoF, nbTaskDoF);
  this->A.setZero();
  for (int i=0; i<nbTaskDoF; ++i)
  {
    if (task & 1<<i)
    {
      this->A(i,i) = true;
    }
    else
    {
      this->a(i) = 0; // so that there won't be a goal impossible to reach because of the "deficiency of the masked matrix"
    }
  }
}

std::string Constraint::toString()
{
  // type EQ|INEQ ----- Activated*|Inactivated ----- target ----- type POS|OR ----- value
  std::ostringstream stringStream;
  stringStream  << "Constraint["
                << this->a
                <<"]"<<std::endl;
  return stringStream.str(); //"%s \t %c \t %d \t %s"
}

/* =======================================================
             MotionGenerationQuadraticProgram
========================================================== */
MotionGenerationQuadraticProgram::MotionGenerationQuadraticProgram(std::string const & name) : RTT::TaskContext(name) {
    // constructor
    addOperation("setDOFsize", &MotionGenerationQuadraticProgram::setDOFsize, this, RTT::ClientThread).doc("set DOF size");
    addOperation("setGains", &MotionGenerationQuadraticProgram::setGains, this, RTT::ClientThread).doc("set gains setGains(int kp, int kd)");
    addOperation("printCurrentState", &MotionGenerationQuadraticProgram::printCurrentState, this, RTT::ClientThread).doc("print current state");

    portsPrepared = false;

    constraints = std::vector<Constraint>();

    velocityLimit = 0.2;

    gainTranslationP = 100;
    gainTranslationD = 5;

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

    ranOnce = false;
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
    }/*
    if (!in_constraintMinvP_port.connected()) {
      RTT::log(RTT::Info) << "in_constraintMinvP_port not connected"
      << RTT::endlog();
      return false;
    }//*/
    if (!out_torques_port.connected())
    {
        RTT::log(RTT::Info) << "out_torques_port not connected"
        << RTT::endlog();
        return false;
    }
    RTT::log(RTT::Info) << "??????????????? MotionGenerationQuadraticProgram configure success"
    << RTT::endlog();
}

bool MotionGenerationQuadraticProgram::startHook() {
    // this method starts the component
    return true;
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

    in_jacobian_var = Eigen::MatrixXf();
    in_jacobian_port.setName("in_jacobian_port");
    in_jacobian_port.doc("Input port for receiving the EE Jacobian from fkin");
    ports()->addPort(in_jacobian_port);
    in_jacobian_flow = RTT::NoData;

    in_jacobianDot_var = Eigen::MatrixXf();
    in_jacobianDot_port.setName("in_jacobianDot_port");
    in_jacobianDot_port.doc("Input port for receiving the EE JacobianDot from fkin");
    ports()->addPort(in_jacobianDot_port);
    in_jacobianDot_flow = RTT::NoData;

    in_currentTaskSpacePosition_var = Eigen::VectorXf();
    in_currentTaskSpacePosition_port.setName("in_currentTaskSpacePosition_port");
    in_currentTaskSpacePosition_port.doc("Input port for receiving the current task space position of the robot");
    ports()->addPort(in_currentTaskSpacePosition_port);
    in_currentTaskSpacePosition_flow = RTT::NoData;

    in_currentTaskSpaceVelocity_var = Eigen::VectorXf();
    in_currentTaskSpaceVelocity_port.setName("in_currentTaskSpaceVelocity_port");
    in_currentTaskSpaceVelocity_port.doc("Input port for receiving the current task space velocity of the robot");
    ports()->addPort(in_currentTaskSpaceVelocity_port);
    in_currentTaskSpaceVelocity_flow = RTT::NoData;

    in_desiredTaskSpacePosition_var = Eigen::VectorXf();
    in_desiredTaskSpacePosition_port.setName("in_desiredTaskSpacePosition_port");
    in_desiredTaskSpacePosition_port.doc("to receive the position to track from a trajectory generator");
    ports()->addPort(in_desiredTaskSpacePosition_port);
    in_desiredTaskSpacePosition_flow = RTT::NoData;

    in_desiredTaskSpaceVelocity_var = Eigen::VectorXf();
    in_desiredTaskSpaceVelocity_port.setName("in_desiredTaskSpaceVelocity_port");
    in_desiredTaskSpaceVelocity_port.doc("to receive the Velocity to track from a trajectory generator");
    ports()->addPort(in_desiredTaskSpaceVelocity_port);
    in_desiredTaskSpaceVelocity_flow = RTT::NoData;

    in_desiredTaskSpaceAcceleration_var = Eigen::VectorXf();
    in_desiredTaskSpaceAcceleration_port.setName("in_desiredTaskSpaceAcceleration_port");
    in_desiredTaskSpaceAcceleration_port.doc("to receive the Acceleration to track from a trajectory generator");
    ports()->addPort(in_desiredTaskSpaceAcceleration_port);
    in_desiredTaskSpaceAcceleration_flow = RTT::NoData;

    in_h_var = Eigen::VectorXf();
    in_h_port.setName("in_h_port");
    in_h_port.doc("Input port to receive the weight, inertia and coriolis matrix from fkin");
    ports()->addPort(in_h_port);
    in_h_flow = RTT::NoData;


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

    WorkspaceDimension = 3;
}

#define COPYMAT(a, toB) for(int i=0; i<a.rows(); i++) {for (int j=0; j<a.cols(); j++) {toB[i][j] = a(i,j);}}
#define COPYVEC(a, toB) for(int i=0; i<a.rows(); i++) {toB[i] = a(i);}

Eigen::VectorXf MotionGenerationQuadraticProgram::solveNextStep(const Eigen::MatrixXf A, const Eigen::VectorXf a, const Eigen::MatrixXf B, const Eigen::VectorXf b)
{
  // From Eigen MAtrix to Matrix for QuadProgpp :
  // Matrix(EigenMatrix.data(), m, n) // This is a COPY constructor !! :D
  // By analysing some methods of the Matrixs provided by QuadProg++ I determined
  // that they used the same order for the data. We can then use the constructor
  // by giving the address of the array. This is a huge gain of performance over
  // copying each data with a loop.


  Eigen::MatrixXf JG = Eigen::MatrixXf::Identity(this->DOFsize*2, this->DOFsize*2);
  ArrayHH::Matrix<double> G, CE, CI;
  ArrayHH::Vector<double> g0, ce0, ci0, x;
	int n, m, p;
	double sum = 0.0;
  Eigen::VectorXf res;
  G = ArrayHH::Matrix<double>();G.resize((int)JG.rows(), (int)JG.cols());COPYMAT(JG, G);
  CE = ArrayHH::Matrix<double>();CE.resize((int)A.rows(), (int)A.cols());COPYMAT(A, CE);
  CI = ArrayHH::Matrix<double>(1, 1); CI.resize((int)B.rows(), (int)B.cols());COPYMAT(B, CI); // solving now just equalities
  ce0 = ArrayHH::Vector<double>();ce0.resize((int)a.rows());COPYVEC(a, ce0);
  ci0 = ArrayHH::Vector<double>(1); ci0[0] = 0; // solving now just equalities
  x = ArrayHH::Vector<double>();x.resize((int)JG.cols());
  g0 = ArrayHH::Vector<double>();g0.resize((int)JG.cols());//g0*=0;
  res = Eigen::VectorXf(JG.cols());
  CE = ArrayHH::t(CE);
  CI = ArrayHH::t(CI);
  sum = solve_quadprog(G, g0, CE,  ce0, CI, ci0, x);
  for (int i=0; i<JG.cols(); i++)
  {
    res[i] = x[i];
  }
  return res;
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
        PRINT("FAILED, NO DATA, RETURN");
        return;
    }


    // reading the variables from the flows

    //PRINT(WorkspaceDimension);3
    //PRINT(desiredPosition); [0 0 0]
    //PRINT(in_desiredTaskSpacePosition_var); [0.7 0 0.7]
    //PRINT(in_desiredTaskSpacePosition_var.head(WorkspaceDimension)); [0.7 0 0.7]
    desiredPosition = in_desiredTaskSpacePosition_var.head(WorkspaceDimension);
    currentPosition = in_currentTaskSpacePosition_var.head(WorkspaceDimension);
    desiredVelocity = in_desiredTaskSpaceVelocity_var.head(WorkspaceDimension);
    currentVelocity = in_currentTaskSpaceVelocity_var.head(WorkspaceDimension);


    Eigen::VectorXf rDot, rDotDot, q, qDot, qDotDot, a, b;
    // good up to here
    rDot = in_jacobian_var.transpose()*in_desiredTaskSpaceVelocity_var;
    rDotDot = in_jacobian_var.transpose()*in_desiredTaskSpaceAcceleration_var+in_jacobianDot_var.transpose()*in_desiredTaskSpaceVelocity_var;
    q = in_robotstatus_var.angles;
    qDot = in_robotstatus_var.velocities;
    //qDotDot =

    int resultVectorSize = in_jacobian_var.cols()*2;
    int nbEquality = in_jacobian_var.rows();
    int nbInequality = 1;
    Eigen::MatrixXf A(nbEquality, resultVectorSize);A.setZero();
    A.block(0,0, nbEquality, resultVectorSize/2) = in_jacobian_var; // setting acceleration equality constraint
    A.block(0, resultVectorSize/2, nbEquality, resultVectorSize/2) = Eigen::MatrixXf::Zero(nbEquality, resultVectorSize/2); // A = [J, 0]
    a = -(this->gainTranslationP*(in_desiredTaskSpacePosition_var - in_currentTaskSpacePosition_var) +
        this->gainTranslationD*(in_desiredTaskSpaceVelocity_var - in_currentTaskSpaceVelocity_var)  -
        in_jacobianDot_var*qDot);

    Eigen::VectorXf sol = this->solveNextStep(A,a, Eigen::MatrixXf::Zero (nbInequality, resultVectorSize), Eigen::VectorXf::Zero(nbInequality));
    out_torques_var.torques = sol.block(0, 0, this->DOFsize, 1);


    // write it to port
    out_torques_port.write(out_torques_var);
    this->ranOnce = true;
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

void MotionGenerationQuadraticProgram::setGains(float kp, float kd)
{
  this->gainTranslationP = kp;
  this->gainTranslationD = kd;
}

void MotionGenerationQuadraticProgram::printCurrentState(){
    std::cout << "############## MotionGenerationQuadraticProgram State begin " << std::endl;

    std::cout << " feedback angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " feedback velocities " << in_robotstatus_var.velocities << std::endl;
    std::cout << " feedback torques " << in_robotstatus_var.torques << std::endl;
    std::cout << " command torques " << out_torques_var.torques << std::endl;
    //*/
    for (int i=0; i<this->constraints.size(); ++i)
    {
      std::cout << this->constraints[i].toString() << '\n';
    }

    std::cout << "############## MotionGenerationQuadraticProgram State end " << std::endl;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(MotionGenerationQuadraticProgram)
