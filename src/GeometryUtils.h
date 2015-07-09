/*!=============================================================================
  ==============================================================================

  \file    geometry_utils.h

  \author  righetti
  \date    Oct 17, 2012

  ==============================================================================
  \remarks
  
  
  ============================================================================*/

#ifndef GEOMETRY_UTILS_H_
#define GEOMETRY_UTILS_H_

#include <iostream>
#include <eigen3/Eigen/Eigen>

#include <SL.h>
#include <utility.h>
#include <utility_macros.h>
#include <SL_common.h>

namespace floating_base_utilities
{

class GeometryUtils
{
public:

//computes the logarithm map on SE(3)
static Eigen::Matrix4d log_map_se3(const Eigen::Matrix4d& T);

//computes the logarithm map on SO(3)
static Eigen::Matrix3d log_map_so3(const Eigen::Matrix3d& R);



/* convenient functions that do all compute so(3) -> SO(3)*/

//axis needs to have unit length
static Eigen::Matrix3d exp_map_SO3(const Eigen::Vector3d& axis, double angle);
//w has length of rotation per second
static Eigen::Matrix3d exp_map_SO3(const Eigen::Vector3d& w);
//Axis is skew matrix from unit length axis
static Eigen::Matrix3d exp_map_SO3(const Eigen::Matrix3d& Axis, double angle);
//W is skew matrix from angular velocity
static Eigen::Matrix3d exp_map_SO3(const Eigen::Matrix3d& W);


/* some geometrical transformations for convenience */
static double get_angle_from_so3(const Eigen::Matrix3d& W);
static Eigen::Matrix3d vector_to_skew_matrix(const Eigen::Vector3d& v);
static Eigen::Vector3d skew_matrix_to_vector(const Eigen::Matrix3d& s);
static void invertEigenTransform(Eigen::Matrix4d& T);

template<typename Mat_Type, typename Vec_Type>
static void transformWrench(Eigen::MatrixBase<Mat_Type>& T, Eigen::MatrixBase<Vec_Type>& wrench){
  assert(T.rows() == 4 && T.cols() == 4 && wrench.rows() == 6 && wrench.cols() == 1);
  wrench.template head<3>() = T.template topLeftCorner<3,3>()*wrench.template head<3>();
  wrench.template tail<3>() = T.template topLeftCorner<3,3>()*wrench.template tail<3>();
  wrench.template tail<3>() += T.template topRightCorner<3,1>().cross(wrench.template head<3>());
}

static void transformWrench(Eigen::Matrix4d T, Eigen::Matrix<double,6,1>& wrench){
  transformWrench<Eigen::Matrix4d, Eigen::Matrix<double,6,1> >(T, wrench);
}

static void SLQuaternionToEigen(const Vector sl_quat, Eigen::Quaterniond& eig_quat);
static void EigQuaternionToSL(const Eigen::Quaterniond& eig_quat, Vector sl_quat);

template<typename RotMat>
static void rotationMatrixToQuaternion(const Eigen::MatrixBase<RotMat>& des_rot,
                                     Eigen::Quaterniond& quaternion);
template<typename RotMat>
static void rotationMatrixToQuaternion(const Eigen::MatrixBase<RotMat>& des_rot,
                                     SL_quat& sl_quaternion);

/** be carefull: if you use this for orientation control then you want to use -err_vec
 * as a command: u = - GAIN * err_vec
 * @param des_rot
 * @param cur_rot
 * @param err_vec
 */
template<typename DesRotDer, typename CurRotDer, typename errVecDer>
static void computeOrientationError(const Eigen::MatrixBase<DesRotDer>& des_rot,
      const Eigen::MatrixBase<CurRotDer>& cur_rot, Eigen::MatrixBase<errVecDer>& err_vec);

template<typename errVecDer>
static void computeOrientationError(const Eigen::Quaterniond& des_rot,
      const Eigen::Quaterniond& cur_rot, Eigen::MatrixBase<errVecDer>& err_vec);

private:
  GeometryUtils(){};
 virtual ~GeometryUtils(){};
};

template<typename DesRotDer, typename CurRotDer, typename errVecDer>
void GeometryUtils::computeOrientationError(const Eigen::MatrixBase<DesRotDer>& des_rot,
      const Eigen::MatrixBase<CurRotDer>& cur_rot, Eigen::MatrixBase<errVecDer>& err_vec)
{
  Eigen::Quaterniond des_rot_quat(des_rot);
  Eigen::Quaterniond cur_rot_quat(cur_rot);

  computeOrientationError(des_rot_quat, cur_rot_quat, err_vec);
}


template<typename errVecDer>
void GeometryUtils::computeOrientationError(const Eigen::Quaterniond& des_rot,
      const Eigen::Quaterniond& cur_rot, Eigen::MatrixBase<errVecDer>& err_vec)
{
  Eigen::Quaterniond q1 = des_rot;
  Eigen::Quaterniond q2 = cur_rot;

  q1.normalize();
  q2.normalize();

  // fix sign
  if(q1.dot(q2)<0.0)
  {
    q1.vec() = -q1.vec();
    q1.w() = -q1.w();
  }

  Eigen::Quaterniond delta_q = q1 * q2.inverse();

  //make sure the q0 > 0 (to make the sine approx correct)
  if(delta_q.w()<0)
  {
    delta_q.w() = -delta_q.w();
    delta_q.x() = -delta_q.x();
    delta_q.y() = -delta_q.y();
    delta_q.z() = -delta_q.z();
  }

  //use a linear approximation
  if(fabs(1.0-delta_q.w()) < 0.01)
  {
    err_vec = 2.0 * delta_q.vec();
  }
  else //or the sine formulae
  {
    double alpha_norm = 2.0 * acos(delta_q.w());
    err_vec = alpha_norm/sin(alpha_norm/2) * delta_q.vec();
  }

  //TODO fix this to get the opposite sign (needs to check backwards compatibility
  //we change the sign because of SL old convention
  err_vec = -err_vec;
}


template<typename RotMat>
void GeometryUtils::rotationMatrixToQuaternion(const Eigen::MatrixBase<RotMat>& des_rot,
                                    Eigen::Quaterniond& quaternion)
{
  MY_MATRIX(sl_mat, 1, 3, 1, 3);
  SL_quat sl_quaternion;
  for(int r=1; r<=3; ++r)
    for(int c=1; c<=3; ++c)
      sl_mat[r][c] = des_rot(r-1, c-1);
  linkQuat(sl_mat, &sl_quaternion);
  quaternion.w() = sl_quaternion.q[_QW_];
  quaternion.x() = sl_quaternion.q[_QX_];
  quaternion.y() = sl_quaternion.q[_QY_];
  quaternion.z() = sl_quaternion.q[_QZ_];
}

template<typename RotMat>
void GeometryUtils::rotationMatrixToQuaternion(const Eigen::MatrixBase<RotMat>& des_rot,
                                     SL_quat& sl_quaternion)
{
  MY_MATRIX(sl_mat, 1, 3, 1, 3);
  for(int r=1; r<=3; ++r)
    for(int c=1; c<=3; ++c)
      sl_mat[r][c] = des_rot(r-1, c-1);
  linkQuat(sl_mat, &sl_quaternion);
}

}

#endif /* GEOMETRY_UTILS_H_ */
