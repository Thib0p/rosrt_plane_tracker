/*!=============================================================================
  ==============================================================================

  \file    geometry_utils.cpp

  \author  righetti
  \date    Oct 17, 2012

  ==============================================================================
  \remarks


  ============================================================================*/



#include "GeometryUtils.h"


namespace floating_base_utilities
{
//computations taken from Murray94

Eigen::Matrix4d GeometryUtils::log_map_se3(const Eigen::Matrix4d& T)
{
  Eigen::Matrix4d twist = Eigen::Matrix4d::Zero();

  twist.topLeftCorner<3,3>() = log_map_so3(T.topLeftCorner<3,3>());

  Eigen::Vector3d w(twist(2,1), twist(0,2), twist(1,0));

  double norm_w = w.norm();
  if(norm_w == 0)
  {
    twist.topRightCorner<3,1>() = T.topRightCorner<3,1>();
  }
  else
  {
    twist.topRightCorner<3,1>() = (Eigen::Matrix3d::Identity() - 0.5 * twist.topLeftCorner<3,3>() +
        (2 * sin(norm_w) - norm_w * (1 + cos(norm_w)) ) / (2 * norm_w * norm_w * sin(norm_w)) *
        twist.topLeftCorner<3,3>() * twist.topLeftCorner<3,3>() ) * T.topRightCorner<3,1>();
  }

  return twist;
}

Eigen::Matrix3d GeometryUtils::log_map_so3(const Eigen::Matrix3d& R)
{
  double theta = acos( (R.trace() - 1.0)/2.0);

  if(fabs(theta) < 0.00001)
    return Eigen::Matrix3d::Zero();
  else
    return theta/(2*sin(theta)) * (R - R.transpose());
}


Eigen::Matrix3d GeometryUtils::exp_map_SO3(const Eigen::Vector3d& axis, double angle)
{
	return exp_map_SO3(vector_to_skew_matrix(axis), angle);
}

Eigen::Matrix3d GeometryUtils::exp_map_SO3(const Eigen::Matrix3d& Axis, double angle)
{
	return Eigen::Matrix3d::Identity() + Axis*sin(angle) + Axis*Axis*(1-cos(angle));
}

Eigen::Matrix3d GeometryUtils::exp_map_SO3(const Eigen::Matrix3d& W)
{
	const double theta = get_angle_from_so3(W);
	Eigen::Matrix3d W_normed = 1/theta*W;
	return exp_map_SO3(W_normed, theta);
}

Eigen::Matrix3d GeometryUtils::exp_map_SO3(const Eigen::Vector3d& w)
{
	return exp_map_SO3(vector_to_skew_matrix(w));
}


double GeometryUtils::get_angle_from_so3(const Eigen::Matrix3d& W)
{
	return std::sqrt(W(1,0)*W(1,0) + W(2,0)*W(2,0) + W(2,1)*W(2,1));
}

Eigen::Matrix3d GeometryUtils::vector_to_skew_matrix(const Eigen::Vector3d& v)
{
	Eigen::Matrix3d S;
	S << 0.0, -v.z(), v.y(),
						v.z(), 0.0, -v.x(),
						-v.y(), v.x(), 0.0;
	return S;
}

Eigen::Vector3d GeometryUtils::skew_matrix_to_vector(const Eigen::Matrix3d& s)
{
	return Eigen::Vector3d(s(2,1), s(0,2), s(1,0));
}


void GeometryUtils::invertEigenTransform(Eigen::Matrix4d& T)
{
	T.block<3,3>(0,0).transposeInPlace();
	T.block<3,1>(0,3) = -T.block<3,3>(0,0)*T.block<3,1>(0,3);
}

void GeometryUtils::SLQuaternionToEigen(const Vector sl_quat, Eigen::Quaterniond& eig_quat)
{
  eig_quat.w() = sl_quat[_QW_];
  eig_quat.x() = sl_quat[_QX_];
  eig_quat.y() = sl_quat[_QY_];
  eig_quat.z() = sl_quat[_QZ_];
}

void GeometryUtils::EigQuaternionToSL(const Eigen::Quaterniond& eig_quat, Vector sl_quat)
{
  sl_quat[_QW_] = eig_quat.w();
  sl_quat[_QX_] = eig_quat.x();
  sl_quat[_QY_] = eig_quat.y();
  sl_quat[_QZ_] = eig_quat.z();
}

}
