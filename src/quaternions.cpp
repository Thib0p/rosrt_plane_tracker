/*
 * quaternions.cpp
 *
 *  Created on: Nov 15, 2010
 *      Author: kalakris
 */

#include "GeometryUtils.h"
#include "quaternions.h"
#include "SL_common.h"

namespace inverse_kinematics
{

bool normalizeQuaternion(SL_quat& q)
{
  double denom = 0.0;
  for (int i=1; i<=N_QUAT; ++i)
    denom += sqr(q.q[i]);

  //check that the denom is close to one
  if( (denom < 0.5) || (denom > 1.5) )
  {
    printf("\n");
    printf("Quaternion denom = %f\n", denom);
    return false;
  }

  double mult = 1.0 / denom;

  for (int i=1; i<=N_QUAT; ++i)
  {
    q.q[i] *= mult;
    q.qd[i] *= mult;
    q.qdd[i] *= mult;
  }

  return true;
}

void quatToAngularAcceleration(SL_quat& q)
{
  double Q[4+1][3+1];
  double Qd[4+1][3+1];
  double halfQdW[4+1];

  Q[1][1] = -q.q[_Q1_];
  Q[1][2] = -q.q[_Q2_];
  Q[1][3] = -q.q[_Q3_];

  Q[2][1] =  q.q[_Q0_];
  Q[2][2] =  q.q[_Q3_];
  Q[2][3] = -q.q[_Q2_];

  Q[3][1] = -q.q[_Q3_];
  Q[3][2] =  q.q[_Q0_];
  Q[3][3] =  q.q[_Q1_];

  Q[4][1] =  q.q[_Q2_];
  Q[4][2] = -q.q[_Q1_];
  Q[4][3] =  q.q[_Q0_];

  Qd[1][1] = -q.qd[_Q1_];
  Qd[1][2] = -q.qd[_Q2_];
  Qd[1][3] = -q.qd[_Q3_];

  Qd[2][1] =  q.qd[_Q0_];
  Qd[2][2] =  q.qd[_Q3_];
  Qd[2][3] = -q.qd[_Q2_];

  Qd[3][1] = -q.qd[_Q3_];
  Qd[3][2] =  q.qd[_Q0_];
  Qd[3][3] =  q.qd[_Q1_];

  Qd[4][1] =  q.qd[_Q2_];
  Qd[4][2] = -q.qd[_Q1_];
  Qd[4][3] =  q.qd[_Q0_];

  for (int i=1; i<=N_QUAT; i++)
  {
    halfQdW[i]=0.0;
    for (int j=1; j<=N_CART; j++)
      halfQdW[i] += 0.5*Qd[i][j]*q.ad[j];
  }

  for (int i=1; i<=N_CART; i++)
  {
    q.add[i]=0.0;
    for (int j=1; j<=N_QUAT; j++)
      q.add[i] += 2.0*Q[j][i]*(q.qdd[j] - halfQdW[j]);
  }

}

void quatToAngularVelAcc(SL_quat& q)
{
  quatToAngularVelocity(&q);
  quatToAngularAcceleration(q);
}

void fixQuaternionSign(const SL_quat &q1, SL_quat &q2)
{
  double dot=0.0;
  for (int j=1; j<=N_QUAT; j++)
  {
    dot+=q1.q[j]*q2.q[j];
  }
  if (dot<0)
  {
    for (int j=1; j<=N_QUAT; j++)
    {
      q2.q[j]=-q2.q[j];
      q2.qd[j]=-q2.qd[j];
      q2.qdd[j]=-q2.qdd[j];
    }
    quatToAngularVelAcc(q2);
  }
}

void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2)
{
  Eigen::Vector3d w1,w2;
  w1.setZero();
  w2.setZero();
  fixQuaternionSign(q1,q2,w1,w2);
}

void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2,
                       const Eigen::Vector3d& w1, Eigen::Vector3d& w2)
{
  Eigen::Vector3d a1,a2;
  a1.setZero();
  a2.setZero();
  fixQuaternionSign(q1,q2,w1,w2,a1,a2);
}



void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2,
                       const Eigen::Vector3d& w1, Eigen::Vector3d& w2,
                       const Eigen::Vector3d& a1, Eigen::Vector3d& a2)
{
  SL_quat sl_q1, sl_q2;
  floating_base_utilities::GeometryUtils::EigQuaternionToSL(q1, sl_q1.q);
  floating_base_utilities::GeometryUtils::EigQuaternionToSL(q2, sl_q2.q);

  Eigen::Map<Eigen::Vector3d>(&(sl_q1.ad[1])) = w1;
  Eigen::Map<Eigen::Vector3d>(&(sl_q2.ad[1])) = w2;
  Eigen::Map<Eigen::Vector3d>(&(sl_q1.add[1])) = a1;
  Eigen::Map<Eigen::Vector3d>(&(sl_q2.add[1])) = a2;

  quatDerivatives(&sl_q1);
  quatDerivatives(&sl_q2);

  fixQuaternionSign(sl_q1, sl_q2);

  //map back the values
  floating_base_utilities::GeometryUtils::SLQuaternionToEigen(sl_q2.q, q2);
  w2 = Eigen::Map<Eigen::Vector3d>(&(sl_q2.ad[1]));
  a2 = Eigen::Map<Eigen::Vector3d>(&(sl_q2.add[1]));
}

void quatLogError(const Eigen::Quaterniond& q_desired, const Eigen::Quaterniond& q_actual, Eigen::Vector3d& error)
{
  Eigen::Quaterniond q1 = q_desired;
  Eigen::Quaterniond q2 = q_actual;

  q1.normalize();
  q2.normalize();
  inverse_kinematics::fixQuaternionSign(q1, q2);

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
    error = 2.0 * delta_q.vec();
  }
  else //or the sine formulae
  {
    double alpha_norm = 2.0 * acos(delta_q.w());
    error = alpha_norm/sin(alpha_norm/2) * delta_q.vec();
  }
}


}
