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

/* =======================================================
             MotionGenerationQuadraticProgram
========================================================== */
MotionGenerationQuadraticProgram::MotionGenerationQuadraticProgram(std::string const & name) : RTT::TaskContext(name) {
    // constructor
    addOperation("setDOFsize", &MotionGenerationQuadraticProgram::setDOFsize, this, RTT::ClientThread).doc("set DOF size");
    addOperation("setGains", &MotionGenerationQuadraticProgram::setGains, this, RTT::ClientThread).doc("set gains setGains(int kp, int kd)");
    addOperation("printCurrentState", &MotionGenerationQuadraticProgram::printCurrentState, this, RTT::ClientThread).doc("print current state");

    in_jacobian_port;
    in_jacobianDot_port;

    portsPrepared = false;

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
}

bool MotionGenerationQuadraticProgram::configureHook() {
    // intializations and object creations go here. Each component should run this before being able to run
    // setting gains for controller:

    if (!in_robotstatus_port.connected()) {
      RTT::log(RTT::Info) << "in_robotstatus_port not connected"
      << RTT::endlog();
      return false;
    }
    for (int i=0; i<this->DOFsize; i++)
    {
        if (!in_desiredTaskSpacePosition_port[i]->connected()) {
          RTT::log(RTT::Info) << "in_desiredTaskSpacePosition_port_"<<i<<" not connected"
          << RTT::endlog();
          continue;
        }
        if (!in_desiredTaskSpaceVelocity_port[i]->connected()) {
          RTT::log(RTT::Info) << "in_desiredTaskSpaceVelocity_port_"<<i<<" not connected"
          << RTT::endlog();
          continue;
        }
        if (!in_desiredTaskSpaceAcceleration_port[i]->connected()) {
          RTT::log(RTT::Info) << "in_desiredTaskSpaceAcceleration_port_"<<i<<" not connected"
          << RTT::endlog();
          continue;
        }
        if (!in_currentTaskSpacePosition_port[i]->connected()) {
          RTT::log(RTT::Info) << "in_currentTaskSpacePosition_port_"<<i<<" not connected"
          << RTT::endlog();
          continue;
        }
        if (!in_currentTaskSpaceVelocity_port[i]->connected()) {
          RTT::log(RTT::Info) << "in_currentTaskSpaceVelocity_port_"<<i<<" not connected"
          << RTT::endlog();
          continue;
        }
        if (!in_jacobian_port[i]->connected()) {
          RTT::log(RTT::Info) << "in_jacobian_port_"<<i<<" not connected"
          << RTT::endlog();
          continue;
        }
        if (!in_jacobianDot_port[i]->connected()) {
          RTT::log(RTT::Info) << "in_jacobianDot_port_"<<i<<" not connected"
          << RTT::endlog();
          continue;
        }
        // if all the ports are connected for a joint, the joint is added to the "constrainedjoints" vector
        this->constrainedJoints.push_back(i);
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

    int i;
    if(portsPrepared){
        // removign the old ports
        ports()->removePort("in_robotstatus_port");
        ports()->removePort("out_torques_port");
        PRINTNL("removing iterative ports");

        for (i=0; i<this->DOFsize; i++)
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
            //delete s;
            PRINTNL("-OK");
        }
        PRINTNL("iterative ports removed");
        in_desiredTaskSpacePosition_port.clear();
        in_desiredTaskSpaceVelocity_port.clear();
        in_desiredTaskSpaceAcceleration_port.clear();
        in_currentTaskSpacePosition_port.clear();
        in_currentTaskSpaceVelocity_port.clear();
        in_jacobian_port.clear();
        in_jacobianDot_port.clear();

        in_desiredTaskSpacePosition_flow.clear();
        in_desiredTaskSpaceVelocity_flow.clear();
        in_desiredTaskSpaceAcceleration_flow.clear();
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

    for (i=0; i<this->DOFsize; i++)
    {

      in_desiredTaskSpacePosition_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_desiredTaskSpacePosition_port_",i), RTT::ConnPolicy()));
      in_desiredTaskSpaceVelocity_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_desiredTaskSpaceVelocity_port_",i), RTT::ConnPolicy()));
      in_desiredTaskSpaceAcceleration_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_desiredTaskSpacePosition_port_",i), RTT::ConnPolicy()));

      in_currentTaskSpacePosition_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_desiredTaskSpaceAcceleration_port_",i), RTT::ConnPolicy()));
      in_currentTaskSpaceVelocity_port.push_back(new RTT::InputPort<Eigen::VectorXf>(cat("in_currentTaskSpaceVelocity_port_",i), RTT::ConnPolicy()));

      in_jacobian_port.push_back(new RTT::InputPort<Eigen::MatrixXf>(cat("in_jacobian_port_",i), RTT::ConnPolicy()));
      in_jacobianDot_port.push_back(new RTT::InputPort<Eigen::MatrixXf>(cat("in_jacobianDot_port_",i), RTT::ConnPolicy()));

      in_desiredTaskSpacePosition_flow.push_back(RTT::FlowStatus());
      in_desiredTaskSpaceVelocity_flow.push_back(RTT::FlowStatus());
      in_desiredTaskSpaceAcceleration_flow.push_back(RTT::FlowStatus());
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
    for (i=0; i<this->DOFsize; i++)
    {
      PRINT("mgqp ================================================ Joint ");PRINTNL(i);
      PRINT("Jacobian ...");
      in_jacobian_var = Eigen::MatrixXf();
      in_jacobian_port[i]->setName(cat("in_jacobian_port_",i));
      in_jacobian_port[i]->doc(cat(cat("Input port for receiving the Jacobian for joint", i), "from fkin"));
      ports()->addPort(*in_jacobian_port[i]);
      in_jacobian_flow[i] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("JacobianDot ...");
      in_jacobianDot_var = Eigen::MatrixXf();
      in_jacobianDot_port[i]->setName(cat("in_jacobianDot_port_", i));
      in_jacobianDot_port[i]->doc("Input port for receiving the EE JacobianDot from fkin");
      ports()->addPort(*in_jacobianDot_port[i]);
      in_jacobianDot_flow[i] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("CurPos ...");
      in_currentTaskSpacePosition_var = Eigen::VectorXf();
      in_currentTaskSpacePosition_port[i]->setName(cat("in_currentTaskSpacePosition_port", i));
      in_currentTaskSpacePosition_port[i]->doc("Input port for receiving the current task space position of the robot");
      ports()->addPort(*in_currentTaskSpacePosition_port[i]);
      in_currentTaskSpacePosition_flow[i] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("CurVel ...");
      in_currentTaskSpaceVelocity_var = Eigen::VectorXf();
      in_currentTaskSpaceVelocity_port[i]->setName(cat("in_currentTaskSpaceVelocity_port", i));
      in_currentTaskSpaceVelocity_port[i]->doc("Input port for receiving the current task space velocity of the robot");
      ports()->addPort(*in_currentTaskSpaceVelocity_port[i]);
      in_currentTaskSpaceVelocity_flow[i] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DesPos ...");
      in_desiredTaskSpacePosition_var = Eigen::VectorXf();
      in_desiredTaskSpacePosition_port[i]->setName(cat("in_desiredTaskSpacePosition_port", i));
      in_desiredTaskSpacePosition_port[i]->doc("to receive the position to track from a trajectory generator");
      ports()->addPort(*in_desiredTaskSpacePosition_port[i]);
      in_desiredTaskSpacePosition_flow[i] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DesVel ...");
      in_desiredTaskSpaceVelocity_var = Eigen::VectorXf();
      in_desiredTaskSpaceVelocity_port[i]->setName(cat("in_desiredTaskSpaceVelocity_port", i));
      in_desiredTaskSpaceVelocity_port[i]->doc("to receive the Velocity to track from a trajectory generator");
      ports()->addPort(*in_desiredTaskSpaceVelocity_port[i]);
      in_desiredTaskSpaceVelocity_flow[i] = RTT::NoData;
      PRINTNL(" OK");

      PRINT("DevAcc ...");
      in_desiredTaskSpaceAcceleration_var = Eigen::VectorXf();
      in_desiredTaskSpaceAcceleration_port[i]->setName(cat("in_desiredTaskSpaceAcceleration_port",i));
      in_desiredTaskSpaceAcceleration_port[i]->doc("to receive the Acceleration to track from a trajectory generator");
      ports()->addPort(*in_desiredTaskSpaceAcceleration_port[i]);
      in_desiredTaskSpaceAcceleration_flow[i] = RTT::NoData;
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
  problem.conditions.conservativeResize((Eigen::Index) (problem.rows()+conditions.rows()), (Eigen::NoChange_t) problem.cols());
  problem.conditions.block(problem.rows()-conditions.rows(), 0, conditions.rows(), problem.cols()) = conditions;
  problem.goal.conservativeResize((Eigen::Index) (problem.rows()), (Eigen::NoChange_t) problem.cols());
  problem.conditions.block(problem.rows()-conditions.rows(), 0, conditions.rows(), 1) = goal;
}

void addToProblem(Eigen::VectorXf condition, float goal, Eigen::MatrixXf &problem)
{

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
    in_robotstatus_flow = in_robotstatus_port.read(in_robotstatus_var);
    in_h_flow = in_h_port.read(in_h_var);
    in_inertia_flow = in_inertia_port.read(in_inertia_var);
    if (in_h_flow == RTT::NoData
        || in_inertia_flow == RTT::NoData
        || in_robotstatus_flow == RTT::NoData)
    {
        PRINT("FAILED, NO DATA, RETURN");
        return;
    }

    QuadraticProblem jointTrackPos; jointTrackPos.init(2*this->DOFsize);


    for (int j=0; j<this->constrainedJoints.size(); j++)
    {
        int jointN = this->constrainedJoints[j];

        in_desiredTaskSpacePosition_flow[jointN] = in_desiredTaskSpacePosition_port[jointN]->read(in_desiredTaskSpacePosition_var);
        in_desiredTaskSpaceVelocity_flow[jointN] = in_desiredTaskSpaceVelocity_port[jointN]->read(in_desiredTaskSpaceVelocity_var);
        in_desiredTaskSpaceAcceleration_flow[jointN] = in_desiredTaskSpaceAcceleration_port[jointN]->read(in_desiredTaskSpaceAcceleration_var);
        in_currentTaskSpacePosition_flow[jointN] = in_currentTaskSpacePosition_port[jointN]->read(in_currentTaskSpacePosition_var);
        in_currentTaskSpaceVelocity_flow[jointN] = in_currentTaskSpaceVelocity_port[jointN]->read(in_currentTaskSpaceVelocity_var);

        in_jacobian_flow[jointN] = in_jacobian_port[jointN]->read(in_jacobian_var);
        in_jacobianDot_flow[jointN] = in_jacobianDot_port[jointN]->read(in_jacobianDot_var);


        if (in_desiredTaskSpacePosition_flow[jointN] == RTT::NoData
          || in_desiredTaskSpaceVelocity_flow[jointN] == RTT::NoData
          || in_desiredTaskSpaceAcceleration_flow[jointN] == RTT::NoData
          || in_currentTaskSpacePosition_flow[jointN] == RTT::NoData
          || in_currentTaskSpaceVelocity_flow[jointN] == RTT::NoData
          || in_jacobian_flow[jointN] == RTT::NoData
          || in_jacobianDot_flow[jointN] == RTT::NoData)
        {
            PRINT("FAILED, NO DATA FOR JOINT ");PRINT(jointN);PRINTNL(" RETURN");
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
        rDot = in_jacobian_var.transpose()*in_desiredTaskSpaceVelocity_var;
        rDotDot = in_jacobian_var.transpose()*in_desiredTaskSpaceAcceleration_var+in_jacobianDot_var.transpose()*in_desiredTaskSpaceVelocity_var;
        q = in_robotstatus_var.angles;
        qDot = in_robotstatus_var.velocities;
        //qDotDot =

        int A_rows = in_jacobian_var.rows();
        int A_cols = this->DOFsize * 2;

        Eigen::MatrixXf A = Eigen::MatrixXf(A_rows, A_cols);A.setZero();

        // Position tracking in Task space
        A.block(0,0, A_rows, A_cols/2) = in_jacobian_var; // setting acceleration equality constraint // jacobian may be smaller as the space
        A.block(0, A_cols/2, A_rows, A_cols/2) = Eigen::MatrixXf::Zero(A_rows, A_cols/2); // A = [J, 0]
        a = -(this->gainTranslationP*(in_desiredTaskSpacePosition_var - in_currentTaskSpacePosition_var) +
            this->gainTranslationD*(in_desiredTaskSpaceVelocity_var - in_currentTaskSpaceVelocity_var)  -
            in_jacobianDot_var*qDot);
        addToProblem(A, a, jointTrackPos);
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

    int nbEquality = jointTrackPos.rows();
    int nbInequality = 1;
    int resultVectorSize = jointTrackPos.dof();

    Eigen::VectorXf tracking = Eigen::VectorXf(this->DOFsize * 2);
    tracking.setZero();
    try {
      tracking = this->solveNextStep(jointTrackPos.conditions, jointTrackPos.goal, Eigen::MatrixXf::Zero (nbInequality, resultVectorSize), Eigen::VectorXf::Zero(nbInequality));

      //Eigen::VectorXf compensation = this->solveNextStep(pb2.conditions, pb2.goal, Eigen::MatrixXf::Zero (nbInequality, resultVectorSize), Eigen::VectorXf::Zero(nbInequality));
    }
    catch (...)
    {
      // catch all
      tracking.setZero();
      PRINTNL("Exeption in looking for a solution");
      PRINT("matrix size : (");PRINT(jointTrackPos.conditions.rows());PRINT(", ");PRINT(jointTrackPos.conditions.cols());PRINTNL(")");
      PRINTNL(jointTrackPos.conditions);
    }
    // sum of all problems as command
    Eigen::VectorXf acceleration = (tracking).block(0, 0, this->DOFsize, 1);
    Eigen::VectorXf torques = (tracking).block(this->DOFsize, 0, this->DOFsize, 1);
    out_torques_var.torques = torques+in_inertia_var*acceleration+in_h_var;


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

    std::cout << "############## MotionGenerationQuadraticProgram State end " << std::endl;
}

// This macro, as you can see, creates the component. Every component should have this!
ORO_CREATE_COMPONENT_LIBRARY()ORO_LIST_COMPONENT_TYPE(MotionGenerationQuadraticProgram)
