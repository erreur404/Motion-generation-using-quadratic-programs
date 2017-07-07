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

#define PRINT(txt) RTT::log(RTT::Info) << txt
#define PRINTNL(txt) RTT::log(RTT::Info) << txt << RTT::endlog()

std::string cat (std::string str, int i)
{
  std::stringstream sstm;
  sstm << str << i;
  return sstm.str();
}

std::string cat (std::string str, float i)
{
  std::stringstream sstm;
  sstm << str << i;
  return sstm.str();
}

std::string cat (std::string str, std::string i)
{
  std::stringstream sstm;
  sstm << str << i;
  return sstm.str();
}

/**
    see http://eigen.tuxfamily.org/bz/show_bug.cgi?id=257#c14
**/
/*
Eigen::MatrixXf pseudoInverse(const Eigen::MatrixXf &a, double epsilon = std::numeric_limits<double>::epsilon())
{
	Eigen::JacobiSVD< Eigen::MatrixXf > svd(a ,Eigen::ComputeThinU | Eigen::ComputeThinV);
	double tolerance = epsilon * std::max(a.cols(), a.rows()) *svd.singularValues().array().abs()(0);
	return svd.matrixV() *  (svd.singularValues().array().abs() > tolerance).select(svd.singularValues().array().inverse(), 0).matrix().asDiagonal() * svd.matrixU().adjoint();
}//*/

void displayLimits (Eigen::MatrixXf limitMatrix, Eigen::VectorXf limits)
{
    // convention : limitMatrix * x >= limits
    //              limitMatrix * x - limits >= 0
    Eigen::VectorXf sup = Eigen::VectorXf::Constant(limitMatrix.cols(),+std::numeric_limits<float>::infinity());
    Eigen::VectorXf inf = Eigen::VectorXf::Constant(limitMatrix.cols(),-std::numeric_limits<float>::infinity());
    limits = -limits;
    for (int i=0; i<limitMatrix.rows(); i++)
    {
        for (int j=0; j<limitMatrix.cols(); j++)
        {
            if(limitMatrix(i,j) != 0)
            {
                if (limitMatrix(i,j) > 0)
                {
                    inf[j] = std::max(limits[i]/limitMatrix(i,j), inf[j]);
                }
                if (limitMatrix(i,j) < 0)
                {
                    sup[j] = std::min(sup[j],limits[i]/limitMatrix(i,j));
                }
            }
        }
    }
    for (int k=0; k<sup.rows(); k++)
    {
        PRINT(inf[k]); PRINT(" < "); PRINT(cat("x", k+1)); PRINT(" < "); PRINTNL(sup[k]);
    }
}

/* =======================================================
             MotionGenerationQuadraticProgram
========================================================== */
MotionGenerationQuadraticProgram::MotionGenerationQuadraticProgram(std::string const & name) : RTT::TaskContext(name) {
    // constructor
    addOperation("setDOFsize", &MotionGenerationQuadraticProgram::setDOFsize, this, RTT::ClientThread).doc("set DOF size");
    addOperation("setGains", &MotionGenerationQuadraticProgram::setGains, this, RTT::ClientThread).doc("set gains setGains(int kp, int kd)");
    addOperation("printCurrentState", &MotionGenerationQuadraticProgram::printCurrentState, this, RTT::ClientThread).doc("print current state");
    addOperation("setAccelerationLimits", &MotionGenerationQuadraticProgram::setAccelerationLimits, this, RTT::ClientThread).doc("set acceleration limits setAccelerationLimits(Eigen::VectorXf limitPositiv, Eigen::VectorXf limitNegativ)");
    addOperation("setTorqueLimits", &MotionGenerationQuadraticProgram::setTorqueLimits, this, RTT::ClientThread).doc("set torque limits setTorqueLimits(Eigen::VectorXf limitPositiv, Eigen::VectorXf limitNegativ)");
    addOperation("setAngularLimits", &MotionGenerationQuadraticProgram::setAngularLimits, this, RTT::ClientThread).doc("set angular limits setAngularLimits(Eigen::VectorXf limitSup, Eigen::VectorXf limitInf)");
    addOperation("setPriorityLevel", &MotionGenerationQuadraticProgram::setPriorityLevel, this, RTT::ClientThread).doc("set priority level of a task or it will be ignored");

    stack_of_tasks = StackOfTasks(); stack_of_tasks.init(3);

    in_jacobian_port;
    in_jacobianDot_port;

    portsPrepared = false;

    velocityLimit = 0.2;

    gainTranslationP = 10;
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
}

bool MotionGenerationQuadraticProgram::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    // setting gains for controller:

    if (!in_robotstatus_port.connected()) {
      RTT::log(RTT::Info) << "in_robotstatus_port not connected"
      << RTT::endlog();
      return false;
    }
    /*
        loop made useless since the joint conditions can now dynamically be added during execution
        the availability of information is therefore checked in the updateHook
    */
    if (!in_h_port.connected()) {
      RTT::log(RTT::Info) << "in_h_port not connected"
      << RTT::endlog();
      return false;
    }
    /*
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
    PRINTNL("Controller configured SUCCESS !");
}

bool MotionGenerationQuadraticProgram::startHook() {
    // this method starts the component
    return true;
}

void MotionGenerationQuadraticProgram::setDOFsize(unsigned int DOFsize){

    int i;
    if(portsPrepared){
        // removign the old ports
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("out_torques_port");
        PRINTNL("removing iterative ports");

        for (i=1; i<=this->DOFsize; i++)
        {
            PRINT(i);
            ports()->removePort(cat("in_jacobian_port_", i));
            free(in_jacobian_port[i]);
            ports()->removePort(cat("in_currentTaskSpacePosition_port_", i));
            free(in_currentTaskSpacePosition_port[i]);
            ports()->removePort(cat("in_currentTaskSpaceVelocity_port_", i));
            free(in_currentTaskSpaceVelocity_port[i]);
            ports()->removePort(cat("in_desiredTaskSpacePosition_port_", i));
            free(in_desiredTaskSpacePosition_port[i]);
            ports()->removePort(cat("in_desiredTaskSpaceVelocity_port_", i));
            free(in_desiredTaskSpaceVelocity_port[i]);
            ports()->removePort(cat("in_desiredTaskSpaceAcceleration_port_", i));
            free(in_desiredTaskSpaceAcceleration_port[i]);
            ports()->removePort(cat("in_desiredJointSpacePosition_port_", i));
            free(in_desiredJointSpacePosition_port[i]);
            ports()->removePort(cat("in_desiredJointSpaceVelocity_port_", i));
            free(in_desiredJointSpaceVelocity_port[i]);
            ports()->removePort(cat("in_desiredJointSpaceAcceleration_port_", i));
            free(in_desiredJointSpaceAcceleration_port[i]);
            //delete s;
            PRINTNL("-OK");
        }
        PRINTNL("iterative ports removed");
        in_desiredTaskSpacePosition_port.clear();
        in_desiredTaskSpaceVelocity_port.clear();
        in_desiredTaskSpaceAcceleration_port.clear();
        in_desiredJointSpacePosition_port.clear();
        in_desiredJointSpaceVelocity_port.clear();
        in_desiredJointSpaceAcceleration_port.clear();
        in_currentTaskSpacePosition_port.clear();
        in_currentTaskSpaceVelocity_port.clear();
        in_jacobian_port.clear();
        in_jacobianDot_port.clear();

        in_desiredTaskSpacePosition_flow.clear();
        in_desiredTaskSpaceVelocity_flow.clear();
        in_desiredTaskSpaceAcceleration_flow.clear();
        in_desiredJointSpacePosition_flow.clear();
        in_desiredJointSpaceVelocity_flow.clear();
        in_desiredJointSpaceAcceleration_flow.clear();
        in_currentTaskSpacePosition_flow.clear();
        in_currentTaskSpaceVelocity_flow.clear();
        in_jacobian_flow.clear();
        in_jacobianDot_flow.clear();
        PRINTNL("port & flow vector freed");
        ports()->removePort("in_h_port");
        ports()->removePort("in_inertia_port");
        PRINTNL("all ports removed");
    }

    this->DOFsize = DOFsize;

    for (i=1; i<=this->DOFsize; i++)
    {

      in_desiredTaskSpacePosition_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_desiredTaskSpacePosition_port_",i), RTT::ConnPolicy()));
      in_desiredTaskSpaceVelocity_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_desiredTaskSpaceVelocity_port_",i), RTT::ConnPolicy()));
      in_desiredTaskSpaceAcceleration_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_desiredTaskSpaceAcceleration_port_",i), RTT::ConnPolicy()));

      in_desiredJointSpacePosition_port.push_back(new RTT::InputPort<float>(cat("in_desiredJointSpacePosition_port_",i), RTT::ConnPolicy()));
      in_desiredJointSpaceVelocity_port.push_back(new RTT::InputPort<float>(cat("in_desiredJointSpaceVelocity_port_",i), RTT::ConnPolicy()));
      in_desiredJointSpaceAcceleration_port.push_back(new RTT::InputPort<float>(cat("in_desiredJointSpaceAcceleration_port_",i), RTT::ConnPolicy()));

      in_currentTaskSpacePosition_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_currentTaskSpacePosition_port_",i), RTT::ConnPolicy()));
      in_currentTaskSpaceVelocity_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_currentTaskSpaceVelocity_port_",i), RTT::ConnPolicy()));

      in_jacobian_port.push_back(new RTT::InputPort<Eigen::MatrixXf>(cat("in_jacobian_port_",i), RTT::ConnPolicy()));
      in_jacobianDot_port.push_back(new RTT::InputPort<Eigen::MatrixXf>(cat("in_jacobianDot_port_",i), RTT::ConnPolicy()));

      in_desiredTaskSpacePosition_flow.push_back(RTT::FlowStatus());
      in_desiredTaskSpaceVelocity_flow.push_back(RTT::FlowStatus());
      in_desiredTaskSpaceAcceleration_flow.push_back(RTT::FlowStatus());
      in_desiredJointSpacePosition_flow.push_back(RTT::FlowStatus());
      in_desiredJointSpaceVelocity_flow.push_back(RTT::FlowStatus());
      in_desiredJointSpaceAcceleration_flow.push_back(RTT::FlowStatus());
      in_currentTaskSpacePosition_flow.push_back(RTT::FlowStatus());
      in_currentTaskSpaceVelocity_flow.push_back(RTT::FlowStatus());
      in_jacobian_flow.push_back(RTT::FlowStatus());
      in_jacobianDot_flow.push_back(RTT::FlowStatus());
    }

    // prepare input ports:
    in_robotstatus_var = rstrt::robot::JointState(DOFsize);
    in_robotstatus_port.setName("in_robotstatus_port");
    in_robotstatus_port.doc("Input port for reading robotstatus values");
    ports()->addPort(in_robotstatus_port);
    in_robotstatus_flow = RTT::NoData;

    PRINTNL("Adding the number ports");
    //*
    for (i=1; i<=this->DOFsize; i++)
    {
      PRINT("mgqp ================================================ Joint ");PRINTNL(i);
      PRINT("Jacobian ...");
      in_jacobian_var = Eigen::MatrixXf();
      in_jacobian_port[i-1]->setName(cat("in_jacobian_port_",i));
      in_jacobian_port[i-1]->doc(cat(cat("Input port for receiving the Jacobian for joint", i), "from fkin"));
      ports()->addPort(*in_jacobian_port[i-1]);
      in_jacobian_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("JacobianDot ...");
      in_jacobianDot_var = Eigen::MatrixXf();
      in_jacobianDot_port[i-1]->setName(cat("in_jacobianDot_port_", i));
      in_jacobianDot_port[i-1]->doc("Input port for receiving the EE JacobianDot from fkin");
      ports()->addPort(*in_jacobianDot_port[i-1]);
      in_jacobianDot_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("CurPos ...");
      in_currentTaskSpacePosition_var = Eigen::VectorXf();
      in_currentTaskSpacePosition_port[i-1]->setName(cat("in_currentTaskSpacePosition_port_", i));
      in_currentTaskSpacePosition_port[i-1]->doc("Input port for receiving the current task space position of the robot");
      ports()->addPort(*in_currentTaskSpacePosition_port[i-1]);
      in_currentTaskSpacePosition_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("CurVel ...");
      in_currentTaskSpaceVelocity_var = Eigen::VectorXf();
      in_currentTaskSpaceVelocity_port[i-1]->setName(cat("in_currentTaskSpaceVelocity_port_", i));
      in_currentTaskSpaceVelocity_port[i-1]->doc("Input port for receiving the current task space velocity of the robot");
      ports()->addPort(*in_currentTaskSpaceVelocity_port[i-1]);
      in_currentTaskSpaceVelocity_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DesPos ...");
      in_desiredTaskSpacePosition_var = Eigen::VectorXf();
      in_desiredTaskSpacePosition_port[i-1]->setName(cat("in_desiredTaskSpacePosition_port_", i));
      in_desiredTaskSpacePosition_port[i-1]->doc("to receive the position to track from a trajectory generator");
      ports()->addPort(*in_desiredTaskSpacePosition_port[i-1]);
      in_desiredTaskSpacePosition_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DesVel ...");
      in_desiredTaskSpaceVelocity_var = Eigen::VectorXf();
      in_desiredTaskSpaceVelocity_port[i-1]->setName(cat("in_desiredTaskSpaceVelocity_port_", i));
      in_desiredTaskSpaceVelocity_port[i-1]->doc("to receive the Velocity to track from a trajectory generator");
      ports()->addPort(*in_desiredTaskSpaceVelocity_port[i-1]);
      in_desiredTaskSpaceVelocity_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DesAcc ...");
      in_desiredTaskSpaceAcceleration_var = Eigen::VectorXf();
      in_desiredTaskSpaceAcceleration_port[i-1]->setName(cat("in_desiredTaskSpaceAcceleration_port_",i));
      in_desiredTaskSpaceAcceleration_port[i-1]->doc("to receive the Acceleration to track from a trajectory generator");
      ports()->addPort(*in_desiredTaskSpaceAcceleration_port[i-1]);
      in_desiredTaskSpaceAcceleration_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DesJointPos ...");
      in_desiredJointSpacePosition_var = 0.0;
      in_desiredJointSpacePosition_port[i-1]->setName(cat("in_desiredJointSpacePosition_port_", i));
      in_desiredJointSpacePosition_port[i-1]->doc("to receive the angle to track from a trajectory generator");
      ports()->addPort(*in_desiredJointSpacePosition_port[i-1]);
      in_desiredJointSpacePosition_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DesJointVel ...");
      in_desiredJointSpaceVelocity_var = 0.0;
      in_desiredJointSpaceVelocity_port[i-1]->setName(cat("in_desiredJointSpaceVelocity_port_", i));
      in_desiredJointSpaceVelocity_port[i-1]->doc("to receive the Velocity to track from a trajectory generator");
      ports()->addPort(*in_desiredJointSpaceVelocity_port[i-1]);
      in_desiredJointSpaceVelocity_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DesJointAcc ...");
      in_desiredJointSpaceAcceleration_var = 0.0;
      in_desiredJointSpaceAcceleration_port[i-1]->setName(cat("in_desiredJointSpaceAcceleration_port_",i));
      in_desiredJointSpaceAcceleration_port[i-1]->doc("to receive the Acceleration to track from a trajectory generator");
      ports()->addPort(*in_desiredJointSpaceAcceleration_port[i-1]);
      in_desiredJointSpaceAcceleration_flow[i-1] = RTT::NoData;
      PRINTNL(" OK");
    }
    // */
    in_h_var = Eigen::VectorXf();
    in_h_port.setName("in_h_port");
    in_h_port.doc("Input port to receive the weight, and coriolis matrix from fkin");
    ports()->addPort(in_h_port);
    in_h_flow = RTT::NoData;

    in_inertia_var = Eigen::MatrixXf();
    in_inertia_port.setName("in_inertia_port");
    in_inertia_port.doc("Input port for the inertia Matrix");
    ports()->addPort(in_inertia_port);
    in_inertia_flow = RTT::NoData;


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

//*
void doubleVToEigenV(std::vector<double> v, Eigen::VectorXf &e)
{
  e = Eigen::VectorXf(v.size());
  for (int i=0; i<v.size(); i++)
  {
    e[i] = v[i];
  }
}

bool MotionGenerationQuadraticProgram::setTorqueLimits(std::vector<double> torquesP, std::vector<double> torquesN)
{
  Eigen::VectorXf p, n;
  assert(torquesP.size()==torquesN.size());
  doubleVToEigenV(torquesP, p);
  doubleVToEigenV(torquesN, n);
  return setTorqueLimitsE(p, n);
}

bool MotionGenerationQuadraticProgram::setAccelerationLimits(std::vector<double> accelerationsP, std::vector<double> accelerationsN)
{
  Eigen::VectorXf p, n;
  assert(accelerationsP.size()==accelerationsN.size());
  doubleVToEigenV(accelerationsP, p);
  doubleVToEigenV(accelerationsN, n);
  return setAccelerationLimitsE(p, n);
}

bool MotionGenerationQuadraticProgram::setAngularLimits(std::vector<double> limitSup, std::vector<double> limitInf)
{
  Eigen::VectorXf s, i;
  assert(limitSup.size()==limitInf.size());
  doubleVToEigenV(limitSup, s);
  doubleVToEigenV(limitInf, i);
  return setAccelerationLimitsE(s, i);
}
// */

//*
bool MotionGenerationQuadraticProgram::setTorqueLimitsE(Eigen::VectorXf torquesP, Eigen::VectorXf torquesN)
{
  if (torquesP.rows() != this->DOFsize || torquesN.rows() != this->DOFsize)
  {
    PRINTNL("Can't assign ");PRINT(torquesP.rows());PRINT(" torque limits to ");PRINT(this->DOFsize);PRINT(" joints robot");
    return false;
  }
  this->JointTorquesLimitsP = torquesP;
  this->JointTorquesLimitsN = torquesN;
  return true;
}

bool MotionGenerationQuadraticProgram::setAccelerationLimitsE(Eigen::VectorXf accelerationsP, Eigen::VectorXf accelerationsN)
{
  if (accelerationsP.rows() != this->DOFsize || accelerationsN.rows() != this->DOFsize)
  {
    PRINTNL("Can't assign ");PRINT(accelerationsP.rows());PRINT(" acceleration limits to ");PRINT(this->DOFsize);PRINT(" joints robot");
    return false;
  }
  this->JointAccelerationLimitsP = accelerationsP;
  this->JointAccelerationLimitsN = accelerationsN;
  return true;
}

bool MotionGenerationQuadraticProgram::setAngularLimitsE(Eigen::VectorXf jointsP, Eigen::VectorXf jointsN)
{
  if (jointsP.rows() != this->DOFsize || jointsN.rows() != this->DOFsize)
  {
    PRINTNL("Can't assign ");PRINT(jointsP.rows());PRINT(" joint limits to ");PRINT(this->DOFsize);PRINT(" joints robot");
    return false;
  }
  this->JointLimitsSup = jointsP;
  this->JointLimitsInf = jointsN;
  return true;
}
// */

bool MotionGenerationQuadraticProgram::setPriorityLevel(std::string task, int level)
{
  if (level > this->stack_of_tasks.stackSize)
  {
    std::cerr << "priority level greater than the priority task size" << '\n';
    return false;
  }
  this->stack_of_tasks.setPriority(task, level);
  return true;
}

void matrixAppend(Eigen::VectorXf * a, const Eigen::VectorXf * b)
{
  /* puts b after a */
  int oldRows = a->rows();
  if (a->rows() == 0)
  {
      *a = *b; // copy, so that b won't be affected
      return;
  }
  if (b->rows() == 0)
  {
      return;
  }
  if (a->cols() != b->cols())
  {
      PRINTNL("DIM ERR - ");
      PRINT("A.size() : (");PRINT(a->rows());PRINT("x");PRINT(a->cols());PRINTNL(")");
      PRINT("B.size() : (");PRINT(b->rows());PRINT("x");PRINT(b->cols());PRINTNL(")");
      return;
  }
  a->conservativeResize((Eigen::Index)(a->rows()+b->rows()), (Eigen::NoChange_t) a->cols());
  a->block(oldRows, 0, b->rows(), a->cols()) = *b;
}

void matrixAppend(Eigen::MatrixXf * a, Eigen::MatrixXf * b)
{
  /* puts b after a */
  int oldRows = a->rows();
  if (a->rows() == 0)
  {
      *a = *b; // copy, so that b won't be affected
      return;
  }
  if (b->rows() == 0)
  {
      return;
  }
  if (a->cols() != b->cols())
  {
      PRINTNL("DIM ERR - ");
      PRINT("A.size() : (");PRINT(a->rows());PRINT("x");PRINT(a->cols());PRINTNL(")");
      PRINT("B.size() : (");PRINT(b->rows());PRINT("x");PRINT(b->cols());PRINTNL(")");
      return;
  }
  a->conservativeResize((Eigen::Index)(a->rows()+b->rows()), (Eigen::NoChange_t) a->cols());
  a->block(oldRows, 0, b->rows(), a->cols()) = *b;
}

void addToProblem(Eigen::MatrixXf conditions, Eigen::VectorXf goal, QuadraticProblem &problem)
{
  if (conditions.cols() != problem.dof())
  {
    throw std::length_error("condition matrix number of columns does not match the problem's number of columns");
  }
  if (conditions.rows() != goal.rows())
  {
    throw std::length_error("condition matrix number of rows is different than the goals number of rows");
  }
  /* displaying the dimensions for debugging
  PRINTNL("addToProblem");
  Eigen::MatrixXf a;
  a=problem.conditions;
  PRINT("prob.cond : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
  a=problem.goal;
  PRINT("prob.goal : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
  a=conditions;
  PRINT("cond : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
  a=goal;
  PRINT("goal : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")"); // */
  matrixAppend(&problem.conditions, &conditions);
  matrixAppend(&problem.goal, &goal);
  /*
  problem.conditions.conservativeResize((Eigen::Index) (problem.rows()+conditions.rows()), (Eigen::NoChange_t) problem.cols());
  problem.conditions.block(problem.rows()-conditions.rows(), 0, conditions.rows(), problem.cols()) = conditions;
  problem.goal.conservativeResize((Eigen::Index) (problem.rows()), (Eigen::NoChange_t) 1);
  problem.goal.block(problem.rows()-conditions.rows(), 0, conditions.rows(), 1) = goal;
  */
}

#define COPYMAT(a, toB) for(int i=0; i<a.rows(); i++) {for (int j=0; j<a.cols(); j++) {toB[i][j] = a(i,j);}}
#define COPYVEC(a, toB) for(int i=0; i<a.rows(); i++) {toB[i] = a(i);}
#define COPYMATE(a, toB) for(int i=0; i<a.rows(); i++) {for (int j=0; j<a.cols(); j++) {toB(i,j) = a(i,j);}}
#define COPYVECE(a, toB) for(int i=0; i<a.rows(); i++) {toB[i] = a(i);}

bool MotionGenerationQuadraticProgram::solveNextStep(const Eigen::MatrixXf A, const Eigen::VectorXf a, const Eigen::MatrixXf B, const Eigen::VectorXf b, Eigen::VectorXf * res)
{
  // From Eigen MAtrix to Matrix for QuadProgpp :
  // Matrix(EigenMatrix.data(), m, n) // This is a COPY constructor !! :D
  // By analysing some methods of the Matrixs provided by QuadProg++ I determined
  // that they used the same order for the data. We can then use the constructor
  // by giving the address of the array. This is a huge gain of performance over
  // copying each data with a loop.

  if (A.cols() != B.cols())
  {
      PRINTNL("A and B matrix don't have the same DOF");
      PRINT("A.cols()=");PRINT(A.cols());PRINT(" ; B.cols()=");PRINTNL(B.cols());
      assert(A.cols() == B.cols());
  }

  int pbDOF = A.cols();
  Eigen::MatrixXf JG = Eigen::MatrixXf::Identity(pbDOF, pbDOF); // the size of the problem now depends on the level in the hierarchy
  Eigen::VectorXf jg = Eigen::VectorXf::Zero(pbDOF);
  QPPP_MATRIX(double) G, CE, CI;
  QPPP_VECTOR(double) g0, ce0, ci0, x;
	int n, m, p;
	double sum;

  G.resize((int)JG.rows(), (int)JG.cols());COPYMATE(JG, G);// = QuadProgpp::Matrix<double>();G.resize((int)JG.rows(), (int)JG.cols());COPYMAT(JG, G);
  CE.resize((int)A.rows(), (int)A.cols());COPYMATE(A, CE);// = ;QuadProgpp::Matrix<double>();CE.resize((int)A.rows(), (int)A.cols());COPYMAT(A, CE);
  CI.resize((int)B.rows(), (int)B.cols());COPYMATE(B, CI);// = QuadProgpp::Matrix<double>(); CI.resize((int)B.rows(), (int)B.cols());COPYMAT(B, CI); // solving now just equalities
  ce0.resize((int)a.rows());COPYVECE(a, ce0);// = QuadProgpp::Vector<double>();ce0.resize((int)a.rows());COPYVEC(a, ce0);
  ci0.resize((int)b.rows());COPYVECE(b, ci0);// = QuadProgpp::Vector<double>(); ci0.resize((int)b.rows());COPYVEC(b, ci0);
  x = Eigen::VectorXd::Zero(JG.rows()); //x.resize((int)JG.cols());
  g0 = Eigen::VectorXd::Zero(JG.rows());// = QuadProgpp::Vector<double>();g0.resize((int)JG.cols());COPYVEC(Eigen::VectorXf::Zero(JG.cols()),g0)//g0*=0;
  *res = Eigen::VectorXf(JG.cols());

  PRINTNL("x prob ?");
  QuadProgpp::Solver quadprog;
  sum = quadprog.solve(G, g0, CE.transpose(),  ce0, CI.transpose(), ci0, x);

  for (int i=0; i<JG.cols(); i++)
  {
    (*res)[i] = x[i];
  }
  //* Here the problem seems never feasible but still returns a good result ...
  if (std::isnan(sum) || sum == std::numeric_limits<double>::infinity())
  {
      PRINTNL("unsolvable");
      *res =  Eigen::VectorXf::Zero(res->rows());
      return false;
  }
  else
  {
      PRINTNL("solvable");
      return true;
  } // */
  return true;
}

Eigen::VectorXf MotionGenerationQuadraticProgram::solveNextHierarchy()
{
    QuadraticProblem * p;

    Eigen::VectorXf res, last_res, u, acumul, bcumul;
    bool stepSuccess;
    Eigen::MatrixXf Acumul, Bcumul, Z, Ztemp;

    Acumul = Bcumul = Eigen::MatrixXf::Zero(0, 2*this->DOFsize);

    last_res = Eigen::VectorXf::Zero(2*this->DOFsize);
    res = Eigen::VectorXf::Zero(this->DOFsize*2);
    u = Eigen::VectorXf::Zero(this->DOFsize*2);
    Z = Eigen::MatrixXf::Identity(this->DOFsize*2, this->DOFsize*2);
    Eigen::MatrixXf Z_pseudo, Z_fullpivlu, Z_svd;
    //PRINTNL("Solving hierarchy ===========================================");
    for (int lvl = 0; lvl < this->stack_of_tasks.stackSize; lvl++)
    {
        PRINT("lvl ");PRINTNL(lvl);
        p = this->stack_of_tasks.getQP(lvl);

        matrixAppend(&Bcumul, &p->constraints);
        matrixAppend(&bcumul, &p->limits);
        // in hierarchy solving, the constraints are simply stacked upon each other

        last_res = res;
        Eigen::MatrixXf a1, a2, a3, a4, a5, a6, a;
        /*
        a1 = p->constraints;
        a = a1;
        PRINT("p->constraints : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        a1 = p->conditions;
        a = a1;
        PRINT("p->conditions : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        PRINTNL(p->conditions);
        a1 = p->conditions * Z;
        a = a1;
        PRINT("p->conditions * Z : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        a2 = p->goal + p->conditions * res;
        a = a2;
        PRINT("p->goal + p->conditions * res; : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        a3 = Bcumul;
        a = a3;
        PRINT("Bcumul : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        a4 = bcumul;
        a = a4;
        PRINT("bcumul : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        a5 = Acumul;
        a = a5;
        PRINT("Acumul : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        a6 = acumul;
        a = a6;
        PRINT("acumul : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        a = Z;
        PRINT("Z : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");

        if (p->constraints.rows() > 0)
        {
            a = p->constraints;
            PRINT("p->constraints : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
            a = Bcumul * Z;
            PRINT("Bcumul * Z : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
            a = bcumul - p->constraints * res;
            PRINT("bcumul - p->constraints * res : (");PRINT(a.rows());PRINT("x");PRINT(a.cols());PRINTNL(")");
        }

        a = res.transpose();
        PRINT("res : ");PRINTNL(a); // */


        // protection against empty problems
        //try{
        if (p->conditions.rows() == 0 && p->constraints.rows() == 0)
        {
            continue;
        }
        else if (p->conditions.rows() > 0 && p->constraints.rows() > 0 && Bcumul.rows() > 0)
        {
            PRINTNL("Pb1");
            // u_1                     A_1*Z_0         a_1 + A_1*y_0
            stepSuccess = solveNextStep(p->conditions * Z, p->goal - p->conditions * last_res, Bcumul * Z, bcumul - p->constraints * last_res, &u);
        }
        else if (p->conditions.rows() > 0 && p->constraints.rows() == 0 && Bcumul.rows() > 0)
        {
            PRINTNL("Pb2");
            //PRINT("p->conditions");PRINTNL(p->conditions);
            //PRINT("Z");PRINTNL(Z);
            //PRINT("p->conditions * Z");PRINTNL(p->conditions * Z);
            //PRINT("p->goal - p->conditions * last_res");PRINTNL(p->goal - p->conditions * last_res);
            //PRINT("Bcumul * Z  --  ");PRINTNL(Bcumul * Z);
            //PRINT("bcumul  --  ");PRINTNL(bcumul);
            stepSuccess = solveNextStep(p->conditions * Z, p->goal - p->conditions * last_res, Bcumul * Z, bcumul, &u);
            //PRINTNL("Solved pb 2");
        }
        else if (p->conditions.rows() > 0 && p->constraints.rows() == 0 && Bcumul.rows() == 0)
        {
            stepSuccess = solveNextStep(p->conditions * Z, p->goal - p->conditions * last_res,  Eigen::MatrixXf::Zero(1, Z.cols()), Eigen::VectorXf::Zero(1), &u);
        }
        /*
        }
        catch (...)
        {
        PRINT("p->conditions * Z");PRINTNL(p->conditions * Z);
        PRINT("p->goal - p->conditions * last_res");PRINTNL(p->goal - p->conditions * last_res);
        PRINT("Bcumul * Z");PRINTNL(Bcumul * Z);
        PRINT(" bcumul - p->constraints * last_res");PRINTNL( bcumul - p->constraints * last_res);
        PRINT("u <-- ");PRINTNL(u);
      } // */

        if (!stepSuccess)
        {
            // if this problem is not solvable, return the solved first levels
            return last_res;
        }
        /*
        else if (p->conditions.rows() == 0 && p->constraints.rows() > 0)
        {
            // just stack and solve nothing. the inequalities will be used on the next equality level.
        }
        else
        {
          // well, no input ? do nothing
        }
      }
      }
        */

        // y =  y*_p    +   Z_p . u_p+1
        res = last_res + Z*u;

        //PRINTNL(res);


        // protection against empty A matrice
        if(p->conditions.rows() > 0)
        {
            matrixAppend(&Acumul, &p->conditions);
            matrixAppend(&acumul, &p->goal);
            //Eigen::FullPivLU <Eigen::MatrixXf> lu (Acumul);
            //Z = lu.kernel();
            Eigen::JacobiSVD<Eigen::MatrixXf> svd(Acumul, Eigen::ComputeFullV); // in Acumul = U S V* we need only V
            Z = Eigen::MatrixXf::Zero(0, 2*this->DOFsize);
            /*
                null space classic formula for robotic is

                y = y_i + (I - J'(J'⁺))u_null

                J times its pseudo inverse will be 1 on the diagonal in the null space and the substraction will yield zero.
                In the SVD decomposition, a vector of V is from the nullspace if the singular value is zero.
                Here we replicate the behavior of the first equation with the help of the SVD decomposition.
            */
            int k;
            for (k=0; k<svd.singularValues().rows(); k++)
            {
                if (svd.singularValues()(k) < 0.0000000000000001)
                {
                    // if the eigen value is null, the column of V is vector of the nullspace
                    // we use a transposed Z to use the simple append helper function
                    Ztemp = svd.matrixV().block(0, k, 2*this->DOFsize, 1).transpose();
                    matrixAppend(&Z, &Ztemp);
                }
                else
                {
                    //Ztemp = Eigen::MatrixXf::Zero(2*this->DOFsize, 1).transpose();
                    Ztemp = Eigen::MatrixXf::Zero(1, 2*this->DOFsize);
                    matrixAppend(&Z, &Ztemp);
                }
            }
            for (;k<2*this->DOFsize; k++)
            {
                // for the extra degrees of freedom which weren't subject to conditions, obviously they are null space
                Ztemp = svd.matrixV().block(0, k, 2*this->DOFsize, 1).transpose();
                matrixAppend(&Z, &Ztemp);
            }
            if (Z.rows() == 0)
            {
              PRINTNL("no null space");
              Z = Eigen::MatrixXf::Zero(1, 2*this->DOFsize);
              return last_res;
            }
            Ztemp = Z.transpose();
            Z = Ztemp;
            //PRINTNL(Z);
        }


    }
    return res;
}


void MotionGenerationQuadraticProgram::updateHook() {
    // this is the actual body of a component. it is called on each cycle
    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    in_h_flow = in_h_port.read(in_h_var);
    in_inertia_flow = in_inertia_port.read(in_inertia_var);
    if (in_h_flow == RTT::NoData
        || in_inertia_flow == RTT::NoData
        || in_robotstatus_flow == RTT::NoData)
    {
        PRINTNL("FAILED, NO DATA, RETURN");
        return;
    }


    for (int i=0; i<stack_of_tasks.stackSize; i++)
    {
        stack_of_tasks.getQP(i)->init(2*this->DOFsize);
    }

    // looping in the joints to create the quadratic problem's Matrix
    for (int j=0; j<this->DOFsize; j++)
    {
        int jointN = j;//this->constrainedJoints[j];

        in_desiredTaskSpacePosition_flow[jointN] = in_desiredTaskSpacePosition_port[jointN]->read(in_desiredTaskSpacePosition_var);
        in_desiredTaskSpaceVelocity_flow[jointN] = in_desiredTaskSpaceVelocity_port[jointN]->read(in_desiredTaskSpaceVelocity_var);
        in_desiredTaskSpaceAcceleration_flow[jointN] = in_desiredTaskSpaceAcceleration_port[jointN]->read(in_desiredTaskSpaceAcceleration_var);

        in_desiredJointSpacePosition_flow[jointN] = in_desiredJointSpacePosition_port[jointN]->read(in_desiredJointSpacePosition_var);
        in_desiredJointSpaceVelocity_flow[jointN] = in_desiredJointSpaceVelocity_port[jointN]->read(in_desiredJointSpaceVelocity_var);
        in_desiredJointSpaceAcceleration_flow[jointN] = in_desiredJointSpaceAcceleration_port[jointN]->read(in_desiredJointSpaceAcceleration_var);

        in_currentTaskSpacePosition_flow[jointN] = in_currentTaskSpacePosition_port[jointN]->read(in_currentTaskSpacePosition_var);
        in_currentTaskSpaceVelocity_flow[jointN] = in_currentTaskSpaceVelocity_port[jointN]->read(in_currentTaskSpaceVelocity_var);

        in_jacobian_flow[jointN] = in_jacobian_port[jointN]->read(in_jacobian_var);
        in_jacobianDot_flow[jointN] = in_jacobianDot_port[jointN]->read(in_jacobianDot_var);

        // looping among the QP problem's priority levels to create each QP's equality Matrix
        for (int lvl=0; lvl < this->stack_of_tasks.stackSize; lvl ++)
        {
            prob = this->stack_of_tasks.getQP(lvl);

            bool taskSpaceOperation = false;
            bool jointSpaceOperation = false;
            /**
            *** The shape of a task ***
            if (the flows required have no data, or the task is not in this prio level)
            {
                set the values to default
            }
            else
            {
                use the received values
            }

            *** The name of a task
            naming convention :
            joint : 5
            port : in_desiredTaskName_port_[5] || in_desiredTaskName_port_5 (in deployer)
            flow : in_desiredTaskName_flow[5]
            var  : in_desiredTaskName_var
            name : in_desiredTaskName_5
            **/

            /*
            std::vector<std::string> v1;
            v1.push_back("in_desiredTaskSpacePosition");
            v1.push_back("in_desiredTaskSpaceVelocity");
            v1.push_back("in_desiredTaskSpaceAcceleration");
            v1.push_back("in_desiredJointSpacePosition");
            v1.push_back("in_desiredJointSpaceVelocity");
            v1.push_back("in_desiredJointSpaceAcceleration");

            for (int lmfao=0; lmfao<v1.size(); lmfao ++)
            {
                if (this->stack_of_tasks.getLevel(cat(cat(v1[lmfao], "_"), jointN)) == lvl)
                {
                  PRINT(cat(cat(v1[lmfao], "_"), jointN)); PRINT(" in level ");PRINT(lvl);PRINT(" but in level ");PRINTNL(this->stack_of_tasks.getLevel(cat(cat(v1[lmfao], "_"), jointN)));
                }
            }
            //  the priorities may not appear from the very beginning in the program
            */



            if (in_desiredTaskSpacePosition_flow[jointN] == RTT::NoData ||
                in_currentTaskSpacePosition_flow[jointN] == RTT::NoData ||
                this->stack_of_tasks.getLevel(cat("in_desiredTaskSpacePosition_", jointN+1)) != lvl)
            {
              desiredPosition = Eigen::VectorXf::Zero(3);
              currentPosition = Eigen::VectorXf::Zero(3);
            }
            else
            {
              // ending up here only if it is the right level and data is there. Otherwise, default values
              desiredPosition = in_desiredTaskSpacePosition_var.head(WorkspaceDimension);
              currentPosition = in_currentTaskSpacePosition_var.head(WorkspaceDimension);
              taskSpaceOperation |= true;
            }

            if (in_desiredTaskSpaceVelocity_flow[jointN] == RTT::NoData ||
                in_currentTaskSpaceVelocity_flow[jointN] == RTT::NoData ||
                this->stack_of_tasks.getLevel(cat("in_desiredTaskSpaceVelocity_", jointN+1)) != lvl)
            {
              desiredVelocity = Eigen::VectorXf::Zero(3);
              currentVelocity = Eigen::VectorXf::Zero(3);
            }
            else
            {
              desiredVelocity = in_desiredTaskSpaceVelocity_var.head(WorkspaceDimension);
              currentVelocity = in_currentTaskSpaceVelocity_var.head(WorkspaceDimension);
              taskSpaceOperation |= true;
            }

            if (in_desiredTaskSpaceAcceleration_flow[jointN] == RTT::NoData ||
              this->stack_of_tasks.getLevel(cat("in_desiredTaskSpaceAcceleration_", jointN+1)) != lvl)
            {
              desiredAcceleration = Eigen::VectorXf::Zero(3);
            }
            else
            {
              desiredAcceleration = in_desiredTaskSpaceAcceleration_var;
              taskSpaceOperation |= true;
            }

            if (taskSpaceOperation && (in_jacobian_flow[jointN] == RTT::NoData ||
                in_jacobianDot_flow[jointN] == RTT::NoData))
            {
                PRINT("FAILED, NO JACOBIAN FOR JOINT ");PRINT(jointN+1);PRINTNL(" RETURN");
                return;
            }

            if (in_desiredJointSpacePosition_flow[jointN] == RTT::NoData ||
                this->stack_of_tasks.getLevel(cat("in_desiredJointSpacePosition_", jointN+1)) != lvl)
            {
              //desiredJointPosition = Eigen::VectorXf::Zero(jointN); // targets the 0
              desiredJointPosition = in_robotstatus_var.angles[jointN];
            }
            else
            {
              desiredJointPosition = in_desiredJointSpacePosition_var;
              jointSpaceOperation |= true;
            }

            if (in_desiredJointSpaceVelocity_flow[jointN] == RTT::NoData ||
                this->stack_of_tasks.getLevel(cat("in_desiredJointSpaceVelocity_", jointN+1)) != lvl)
            {
              desiredJointVelocity = in_robotstatus_var.velocities[jointN];
            }
            else
            {
              desiredJointVelocity = in_desiredJointSpaceVelocity_var;
              jointSpaceOperation |= true;
            }

            if (in_desiredJointSpaceAcceleration_flow[jointN] == RTT::NoData ||
                this->stack_of_tasks.getLevel(cat("in_desiredJointSpaceAcceleration_", jointN+1)) != lvl)
            {
              desiredJointAcceleration = in_robotstatus_var.accelerations[jointN];
            }
            else
            {
              desiredJointAcceleration = in_desiredJointSpaceAcceleration_var;
              jointSpaceOperation |= true;
            }

            Eigen::VectorXf rDot, rDotDot, q, qDot, qDotDot, a, b;
            Eigen::MatrixXf A;

            q = in_robotstatus_var.angles;
            qDot = in_robotstatus_var.velocities;
            q = q.block(0, 0, in_jacobian_var.cols(), 1);
            qDot = qDot.block(0, 0, in_jacobian_var.cols(), 1);


            if (taskSpaceOperation)
            {
                rDot = in_jacobian_var.transpose()*desiredVelocity;
                rDotDot = in_jacobian_var.transpose()*desiredAcceleration+in_jacobianDot_var.transpose()*desiredVelocity;

                int A_rows = in_jacobian_var.rows();
                int A_cols = this->DOFsize * 2;

                A = Eigen::MatrixXf::Zero(A_rows, A_cols);
                // Position tracking in Task space
                A.block(0,0, A_rows, in_jacobian_var.cols()) = in_jacobian_var; // setting acceleration equality constraint // jacobian may be smaller as the space, the rest is 0 filled
                A.block(0, A_cols/2, A_rows, A_cols/2) = Eigen::MatrixXf::Zero(A_rows, A_cols/2); // A = [J, 0]
                a = -(this->gainTranslationP*(desiredPosition - currentPosition) +
                    this->gainTranslationD*(desiredVelocity - currentVelocity)  -
                    in_jacobianDot_var*qDot);
                /*
                    NOTE : the goal matches the dimension of the result vector. When we work on an inferior degree of freedom,
                    the jacobian is null on these joints, and so are the associated goals
                */
                addToProblem(A, a, *prob);
            }
            if (jointSpaceOperation)
            {
                int A_rows = 1;
                int A_cols = this->DOFsize * 2;

                A = Eigen::MatrixXf::Zero(A_rows, A_cols);
                // Position tracking in Task space
                A(0, jointN) = 1;
                a = Eigen::VectorXf(1);
                a(0) = -(this->gainTranslationP*(desiredJointPosition - in_robotstatus_var.angles[jointN]) +
                    this->gainTranslationD*(desiredJointVelocity - in_robotstatus_var.velocities[jointN]));
                addToProblem(A, a, *prob);
            }
        }
    }
    //addToProblem(A, a, pb1);
    //*/
    // Energy economy, minimize torque
    /*
    A = Eigen::MatrixXf(resultVectorSize/2, resultVectorSize); A.setZero();
    A.block(0, resultVectorSize/2, resultVectorSize/2, resultVectorSize/2) = (1/100)*Eigen::MatrixXf::Identity(resultVectorSize/2, resultVectorSize/2);
    a = Eigen::VectorXf(resultVectorSize/2);a.setZero();

    addToProblem(A, a, pb1);
    //*/

    int nbEquality = prob->rows();
    int nbInequality = 4*this->DOFsize;
    int resultVectorSize = 2*this->DOFsize;

    // adding inequalities
    Eigen::MatrixXf limitsMatrix(nbInequality, resultVectorSize);
    limitsMatrix.setZero();
    Eigen::VectorXf limits(nbInequality);
    limits.setZero();

    /*
        NOTE : the QuadProgpp library solves inequality as Ax+a >= 0. Not exceeding a torque limit like Ax <= a would then be
                    -Ax >= -a --> -Ax + a >= 0

                    limitsMatrix :
                    (for a 4 DOF robot in plan)
                    -1 0 0 0   0 0 0 0                      AccelLimit+
                    0 -1 0 0   0 0 0 0                      AccelLimit+
                    0 0 -1 0   0 0 0 0                      AccelLimit+
                    0 0 0 -1   0 0 0 0                      AccelLimit+
                                          X   Solution  -                 >= 0
                    0 0 0 0   -1 0 0 0                      TorqueLimit+
                    0 0 0 0   0 -1 0 0                      TorqueLimit+
                    0 0 0 0   0 0 -1 0                      TorqueLimit+
                    0 0 0 0   0 0 0 -1                      TorqueLimit+

                    +1 0 0 0   0 0 0 0                      AccelLimit-
                    0 +1 0 0   0 0 0 0                      AccelLimit-
                    0 0 +1 0   0 0 0 0                      AccelLimit-
                    0 0 0 +1   0 0 0 0                      AccelLimit-

                    0 0 0 0   +1 0 0 0                      TorqueLimit-
                    0 0 0 0   0 +1 0 0                      TorqueLimit-
                    0 0 0 0   0 0 +1 0                      TorqueLimit-
                    0 0 0 0   0 0 0 +1                      TorqueLimit-
    */
    limitsMatrix.block(0, 0, nbInequality/2, nbInequality/2) = -Eigen::MatrixXf::Identity(nbInequality/2, nbInequality/2); // acceleration limit + and torque limit +
    limitsMatrix.block(nbInequality/2, 0, nbInequality/2, nbInequality/2) = Eigen::MatrixXf::Identity(nbInequality/2,nbInequality/2); // acceleration limit - and torque limit -

    Eigen::VectorXf jointAccelAndAngleLimitP = (this->JointAccelerationLimitsP.rows() != this->DOFsize ? Eigen::VectorXf::Zero(nbInequality/4) : this->JointAccelerationLimitsP);
    Eigen::VectorXf jointAccelAndAngleLimitN = (this->JointAccelerationLimitsN.rows() != this->DOFsize ? Eigen::VectorXf::Zero(nbInequality/4) : this->JointAccelerationLimitsN);
    JointLimitsSup = (JointLimitsSup.rows() == this->DOFsize ? JointLimitsSup : Eigen::VectorXf::Zero(this->DOFsize));
    JointLimitsInf = (JointLimitsInf.rows() == this->DOFsize ? JointLimitsInf : Eigen::VectorXf::Zero(this->DOFsize));

    for (int i=0; i<jointAccelAndAngleLimitP.rows(); i++)
    {
        jointAccelAndAngleLimitP[i] = std::min ((double) jointAccelAndAngleLimitP[i], log(JointLimitsSup[i] - in_robotstatus_var.angles[i]));
        jointAccelAndAngleLimitN[i] = std::max ((double) jointAccelAndAngleLimitN[i], -log(in_robotstatus_var.angles[i] - JointLimitsInf[i]));
    }

    limits.block(0*nbInequality/4, 0, nbInequality/4, 1) = jointAccelAndAngleLimitP;
    limits.block(1*nbInequality/4, 0, nbInequality/4, 1) = (this->JointTorquesLimitsP.rows() != this->DOFsize ? Eigen::VectorXf::Zero(nbInequality/4) : this->JointTorquesLimitsP);
    limits.block(2*nbInequality/4, 0, nbInequality/4, 1) = -jointAccelAndAngleLimitN;
    limits.block(3*nbInequality/4, 0, nbInequality/4, 1) = -(this->JointTorquesLimitsN.rows() != this->DOFsize ? Eigen::VectorXf::Zero(nbInequality/4) : this->JointTorquesLimitsN);

    //displayLimits(limitsMatrix, limits, this->DOFsize);

    //PRINT(JointLimitsSup.transpose());PRINTNL(" -- ");
    //PRINT(" -- ");PRINTNL(JointLimitsInf.transpose());

    Eigen::VectorXf tracking = Eigen::VectorXf(this->DOFsize * 2);
    tracking.setZero();
    // setting these inequalities as part of the top priority
    this->stack_of_tasks.getQP(0)->constraints = limitsMatrix.block(0,0,1,2*this->DOFsize);
    this->stack_of_tasks.getQP(0)->limits = limits.block(0, 0, 1, 1);
    // and adding the relation between acceleration and torques inside the QP, so that all constraints shall be respected
    Eigen::MatrixXf A; Eigen::VectorXf a;
    A = Eigen::MatrixXf(this->DOFsize, 2*this->DOFsize); // acceleration is identity. Matrix*Accel = torques <=> + accel -
    A.block(0, 0, this->DOFsize, this->DOFsize) = in_inertia_var;
    A.block(0, this->DOFsize, this->DOFsize, this->DOFsize) = -Eigen::MatrixXf::Identity(this->DOFsize, this->DOFsize);
    a = Eigen::VectorXf::Zero(this->DOFsize);
    addToProblem(A, a, *(this->stack_of_tasks.getQP(0)));

    //try
    //{
        tracking = this->solveNextHierarchy();
        // sum of all problems as command
        Eigen::VectorXf acceleration = (tracking).block(0, 0, this->DOFsize, 1);
        Eigen::VectorXf torques = (tracking).block(this->DOFsize, 0, this->DOFsize, 1);
        tracking.setZero();
    // setting these inequalities as part of the top priority->DOFsize, 1);
        //out_torques_var.torques = torques+in_inertia_var*acceleration+in_h_var;
        out_torques_var.torques = torques+in_h_var; // +in_inertia_var*acceleration+ now taken into account in QP

        //PRINT("torques : ");PRINTNL(out_torques_var.torques.transpose());
        // write it to port
        out_torques_port.write(out_torques_var);
        /*
    }
    catch (...)
    {
        tracking = Eigen::VectorXf::Zero(this->DOFsize);

        //tracking[0] = 100;
        out_torques_var.torques = tracking;
        out_torques_port.write(out_torques_var);
    }// */



    /*
    try {
      //tracking = this->solveNextStep(jointTrackPos.conditions, jointTrackPos.goal, limitsMatrix, limits);
      tracking = this->solveNextStep(jointTrackPos.conditions, jointTrackPos.goal, Eigen::MatrixXf::Zero(1, 1), Eigen::VectorXf::Zero(1));
    }
    catch (...)
    {
      // catch all
      tracking.setZero();
      PRINTNL("Exeption in looking for a solution");
      PRINT("matrix size : (");PRINT(jointTrackPos.conditions.rows());PRINT(", ");PRINT(jointTrackPos.conditions.cols());PRINTNL(")");
      PRINTNL(jointTrackPos.conditions);
      return;
    }
    PRINTNL("Succeed indeed !");
    */


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
    std::cout << "############## MotionGenerationQuadraticProgram State begin " << std::endl << std::endl;

    std::vector<std::string> v1;
    v1.push_back("in_desiredTaskSpacePosition");
    v1.push_back("in_desiredTaskSpaceVelocity");
    v1.push_back("in_desiredTaskSpaceAcceleration");
    v1.push_back("in_desiredJointSpacePosition");
    v1.push_back("in_desiredJointSpaceVelocity");
    v1.push_back("in_desiredJointSpaceAcceleration");

    std::vector<void*> v2;
    v2.push_back(&in_desiredTaskSpacePosition_port);
    v2.push_back(&in_desiredTaskSpaceVelocity_port);
    v2.push_back(&in_desiredTaskSpaceAcceleration_port);
    v2.push_back(&in_desiredJointSpacePosition_port);
    v2.push_back(&in_desiredJointSpaceVelocity_port);
    v2.push_back(&in_desiredJointSpaceAcceleration_port);

    for (int i=1; i <= this->DOFsize; i++)
    {
      for (int j=0; j<v1.size(); j++)
      {
        if (((std::vector<RTT::InputPort<float>*> *) v2.at(j))->at(i-1)->connected())
        {
          std::cout << cat(v1[j], "_port_") << i << " connected -- ";
          int lvl = stack_of_tasks.getLevel(cat(v1[j], cat("_", i))); // in_taskName + _ + i
          if (lvl != -1)
          {
            std::cout << " with priority " << lvl;
          }
          else
          {
            std::cout << " no priority // not considered ";
          }
          std::cout << std::endl;
        }
      }
    }
    std::cout << std::endl << std::endl;

    std::cout << " degrees of freedom " << DOFsize << std::endl;
    std::cout << " torque limits+ " << JointTorquesLimitsP.transpose() << std::endl;
    std::cout << " torque limits- " << JointTorquesLimitsN.transpose() << std::endl;
    std::cout << " acceleration limits+ " << JointAccelerationLimitsP.transpose() << std::endl;
    std::cout << " acceleration limits- " << JointAccelerationLimitsN.transpose() << std::endl;

    std::cout << " feedback angles " << in_robotstatus_var.angles << std::endl;
    std::cout << " feedback velocities " << in_robotstatus_var.velocities << std::endl;
    std::cout << " feedback torques " << in_robotstatus_var.torques << std::endl;
    std::cout << " command torques " << out_torques_var.torques << std::endl;

    std::cout << "############## MotionGenerationQuadraticProgram State end " << std::endl;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(MotionGenerationQuadraticProgram)
