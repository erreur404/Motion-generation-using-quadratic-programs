/* Author: Niels Dehio
 * Date:   30 August 2016
 *
 * Description:
 */

#pragma once

#include <Eigen/Dense>
#include <kdl/frames.hpp>
#include <iostream>

class QuaternionHelper {
public:

    void  EigenVector2KDLVector(Eigen::Vector3f const & eigen_vec, KDL::Vector & kdl_vec);
    void  KDLVector2EigenVector(KDL::Vector const & kdl_vec, Eigen::Vector3f & eigen_vec);

    void AxisAngle2Quaternion(Eigen::Vector3f const & axisangle, Eigen::Vector4f & quaternion);
    void Quaternion2AxisAngle(Eigen::Vector4f const & quaternion, Eigen::Vector3f & axisangle);

    void QuaternionProduct(
            Eigen::Vector4f const & quat1,
            Eigen::Vector4f const & quat2,
            Eigen::Vector4f& quatResult);
    void QuaternionProduct(
            float const & quaternionV1,
            Eigen::Vector3f const & quaternionU1,
            float const & quaternionV2,
            Eigen::Vector3f const & quaternionU2,
            float & resultV,
            Eigen::Vector3f & resultU);

    void ExpAxisAngle2Quaternion(Eigen::Vector3f const & axisangle, Eigen::Vector4f & quaternion);
    void ExpAxisAngle2Quaternion(Eigen::Vector3f const & axisangle, float & quaternionV, Eigen::Vector3f & quaternionU);
    void LogQuaternion2AxisAngle(Eigen::Vector4f const & quaternion, Eigen::Vector3f & axisangle);
    void LogQuaternion2AxisAngle(float const & quaternionV, Eigen::Vector3f const & quaternionU, Eigen::Vector3f & axisangle);
    void ConjugateQuaternion(Eigen::Vector4f const & quaternion, Eigen::Vector4f & conjugateQuaternion);
};

