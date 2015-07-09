/*
 * quaternions.h
 *
 *  Created on: Nov 15, 2010
 *      Author: kalakris
 */

#ifndef QUATERNIONS_H_
#define QUATERNIONS_H_

#include <eigen3/Eigen/Eigen>
#include "SL.h"

namespace inverse_kinematics
{

/**
 * Normalizes the input quaternion, such that its norm is 1
 * You will probably need to call quatToAngularVelAcc after this, because
 * the quaternion derivatives are also rescaled appropriately, but are inconsistent
 * with the angular derivatives.
 * @param q
 * @return false is the denominator is too small
 */
bool normalizeQuaternion(SL_quat& q);

/**
 * Converts quaternion accelerations to angular accelerations
 * Assumes that the quaternion, its velocity, and angular velocity are correct
 * @param q
 */
void quatToAngularAcceleration(SL_quat& q);

/**
 * Converts quaternion derivatives to angular velocity and acceleration
 * @param q
 */
void quatToAngularVelAcc(SL_quat& q);

/**
 * Fix the sign of q2 according to q1
 * @param q1
 * @param q2
 */
void fixQuaternionSign(const SL_quat &q1, SL_quat &q2);
void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2);
void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2,
                       const Eigen::Vector3d& w1, Eigen::Vector3d& w2);
void fixQuaternionSign(const Eigen::Quaterniond& q1, Eigen::Quaterniond& q2,
                       const Eigen::Vector3d& w1, Eigen::Vector3d& w2,
                       const Eigen::Vector3d& a1, Eigen::Vector3d& a2);

/** compute the error between two quaternions as q_desired -q_actual = log(q_desired * q_actual^-1)
 * This function is meant to replace the quatError function of SL (note that the error has the correct
 * sign in our case, as opposed to the SL fun).
 *
 * @param q_desired
 * @param q_actual
 * @param error
 */
void quatLogError(const Eigen::Quaterniond& q_desired, const Eigen::Quaterniond& q_actual, Eigen::Vector3d& error);

}


#endif /* QUATERNIONS_H_ */
