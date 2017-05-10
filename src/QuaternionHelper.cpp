/* Author: Niels Dehio
 * Date:   30 August 2016
 *
 * Description:
 */

#include "QuaternionHelper.hpp"

void  QuaternionHelper::EigenVector2KDLVector(Eigen::Vector3f const & eigen_vec, KDL::Vector & kdl_vec){
    kdl_vec = KDL::Vector(eigen_vec(0), eigen_vec(1), eigen_vec(2));
}

void  QuaternionHelper::KDLVector2EigenVector(KDL::Vector const & kdl_vec, Eigen::Vector3f & eigen_vec){
    eigen_vec(0) = kdl_vec.x();
    eigen_vec(1) = kdl_vec.y();
    eigen_vec(2) = kdl_vec.z();
}

void  QuaternionHelper::AxisAngle2Quaternion(Eigen::Vector3f const & axisangle, Eigen::Vector4f & quaternion){
    KDL::Vector kdl_axisangle;
    kdl_axisangle = KDL::Vector();
    EigenVector2KDLVector(axisangle, kdl_axisangle);
    kdl_axisangle.Normalize(); //TODO: correct? unecessary?
    double angle = axisangle.norm();
    //double angle = kdl_axisangle.Norm();
    KDL::Rotation r;
    r = KDL::Rotation::Rot(kdl_axisangle, angle);
    double x, y, z, w;
    r.GetQuaternion(x,y,z,w);
    quaternion(1) = x;
    quaternion(2) = y;
    quaternion(3) = z;
    quaternion(0) = w;


//    double angle = axisangle.norm();
//    quaternion(0) = cos(angle * 0.5);
//    quaternion(1) = axisangle(0) * sin(angle * 0.5);
//    quaternion(2) = axisangle(1) * sin(angle * 0.5);
//    quaternion(3) = axisangle(2) * sin(angle * 0.5);
//    quaternion.tail<3>() = axisangle * sin(angle * 0.5);

}

void  QuaternionHelper::Quaternion2AxisAngle(Eigen::Vector4f const & quaternion, Eigen::Vector3f & axisangle){
    double x = quaternion(1);
    double y = quaternion(2);
    double z = quaternion(3);
    double w = quaternion(0);
    KDL::Rotation r;
    r = KDL::Rotation::Quaternion(x,y,z,w);
    KDL::Vector kdl_axisangle;
    kdl_axisangle = r.GetRot();
    KDLVector2EigenVector(kdl_axisangle, axisangle);
}



void QuaternionHelper::QuaternionProduct(
        Eigen::Vector4f const & quat1,
        Eigen::Vector4f const & quat2,
        Eigen::Vector4f& quatResult) {

    //version 1
//    float quaternionV1 = quat1(0);
//    Eigen::Vector3f quaternionU1 = Eigen::Vector3f();
//    quaternionU1(0) = quat1(1);
//    quaternionU1(1) = quat1(2);
//    quaternionU1(2) = quat1(3);

//    float quaternionV2 = quat2(0);
//    Eigen::Vector3f quaternionU2 = Eigen::Vector3f();
//    quaternionU2(0) = quat2(1);
//    quaternionU2(1) = quat2(2);
//    quaternionU2(2) = quat2(3);

//    float quaternionVResult;
//    Eigen::Vector3f quaternionUResult = Eigen::Vector3f();

//    this->QuaternionProduct(quaternionV1, quaternionU1, quaternionV2,
//            quaternionU2, quaternionVResult, quaternionUResult);

//    quatResult(0) = quaternionVResult;
//    quatResult(1) = quaternionUResult(0);
//    quatResult(2) = quaternionUResult(1);
//    quatResult(3) = quaternionUResult(2);

    //version 2
    Eigen::MatrixXf quat1Matrix;
    quat1Matrix = Eigen::MatrixXf::Zero(4,4);
    quat1Matrix(0,0) =  quat1(0);
    quat1Matrix(0,1) = -quat1(1);
    quat1Matrix(0,2) = -quat1(2);
    quat1Matrix(0,3) = -quat1(3);

    quat1Matrix(1,0) =  quat1(1);
    quat1Matrix(1,1) =  quat1(0);
    quat1Matrix(1,2) = -quat1(3);
    quat1Matrix(1,3) =  quat1(2);

    quat1Matrix(2,0) =  quat1(2);
    quat1Matrix(2,1) =  quat1(3);
    quat1Matrix(2,2) =  quat1(0);
    quat1Matrix(2,3) = -quat1(1);

    quat1Matrix(3,0) =  quat1(3);
    quat1Matrix(3,1) = -quat1(2);
    quat1Matrix(3,2) =  quat1(1);
    quat1Matrix(3,3) =  quat1(0);

    quatResult = quat1Matrix * quat2;
}

void QuaternionHelper::QuaternionProduct(
        float const & quaternionV1,
        Eigen::Vector3f const & quaternionU1,
        float const & quaternionV2,
        Eigen::Vector3f const & quaternionU2,
        float & resultV,
        Eigen::Vector3f & resultU) {

    resultV = quaternionV1 * quaternionV2
            - quaternionU1.transpose() * quaternionU2;
    resultU = quaternionV1 * quaternionU2 + quaternionV2 * quaternionU1
            + quaternionU1.cross(quaternionU2);
}

void QuaternionHelper::ExpAxisAngle2Quaternion(
        Eigen::Vector3f const & axisangle,
        Eigen::Vector4f & quaternion) {
    // equals quaternion = exp(axisangle)

    float quaternionV;
    Eigen::Vector3f quaternionU = Eigen::Vector3f();

    this->ExpAxisAngle2Quaternion(axisangle, quaternionV, quaternionU);

    quaternion(0) = quaternionV;
    quaternion(1) = quaternionU(0);
    quaternion(2) = quaternionU(1);
    quaternion(3) = quaternionU(2);

    float n=pow(quaternion(0),2) + pow(quaternion(1),2) + pow(quaternion(2),2) + pow(quaternion(3),2);
    if (n > 1.1){
        std::cout << " QuaternionHelper -> norm(quaternion) = " << n << std::endl;
    }
    if (n < 0.9){
        std::cout << " QuaternionHelper -> norm(quaternion) = " << n << std::endl;
    }
}

void QuaternionHelper::ExpAxisAngle2Quaternion(
        Eigen::Vector3f const & axisangle,
        float & quaternionV,
        Eigen::Vector3f & quaternionU) {
    // equals quaternion = exp(axisangle)

    if (axisangle.norm() > 3.15) {
        std::cout << " pi < norm(axisangle) = " << axisangle.norm() << std::endl;
    }

    if (axisangle(0) == 0.0 && axisangle(1) == 0.0 && axisangle(2) == 0.0) {
        quaternionV = 1.0;
        quaternionU.setZero();
    } else {
        quaternionV = cos(axisangle.norm());
        quaternionU = ( sin(axisangle.norm()) / axisangle.norm() ) * axisangle;
    }
}

void QuaternionHelper::LogQuaternion2AxisAngle(
        Eigen::Vector4f const & quaternion,
        Eigen::Vector3f & axisangle) {
    // equals axisangle = log(quaternion)

    float quaternionV = quaternion(0);
    Eigen::Vector3f quaternionU = Eigen::Vector3f();
    quaternionU(0) = quaternion(1);
    quaternionU(1) = quaternion(2);
    quaternionU(2) = quaternion(3);

    this->LogQuaternion2AxisAngle(quaternionV, quaternionU, axisangle);
}

void QuaternionHelper::LogQuaternion2AxisAngle(
        float const & quaternionV,
        Eigen::Vector3f const & quaternionU,
        Eigen::Vector3f & axisangle) {
    // equals axisangle = log(quaternion)

    if (quaternionU(0) == 0.0 && quaternionU(1) == 0.0
            && quaternionU(2) == 0.0) {
    //if(quaternionU.norm() == 0.0){
        axisangle.setZero();
    } else {
        axisangle = acos(quaternionV) * quaternionU / quaternionU.norm();
    }
}

void QuaternionHelper::ConjugateQuaternion(
        Eigen::Vector4f const & quaternion,
        Eigen::Vector4f & conjugateQuaternion) {
    conjugateQuaternion(0) = + quaternion(0);
    conjugateQuaternion(1) = - quaternion(1);
    conjugateQuaternion(2) = - quaternion(2);
    conjugateQuaternion(3) = - quaternion(3);
}
