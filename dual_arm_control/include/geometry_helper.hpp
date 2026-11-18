#pragma once

#include <Eigen/Dense>
#include <iostream>
namespace uclv::geometry_helper
{

template <typename Scalar_t = double>
void skew(const Eigen::Vector<Scalar_t, 3>& v, Eigen::Matrix<Scalar_t, 3, 3>& M)
{
  M << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

template <typename Scalar_t = double>
void quaternion_propagation(const Eigen::Quaternion<Scalar_t>& q, const Eigen::Vector<Scalar_t, 3>& omega,
                            Eigen::Quaternion<Scalar_t>& qdot)
{
  Eigen::Vector<Scalar_t, 3> epsilon_q = q.vec();
  Scalar_t eta = q.w();
  Scalar_t eta_dot = -0.5 * epsilon_q.dot(omega);
  Eigen::Matrix<Scalar_t, 3, 3> skew_epsilon_q;
  skew(epsilon_q, skew_epsilon_q);
  Eigen::Vector<Scalar_t, 3> epsilon_dot =
      0.5 * (eta * Eigen::Matrix<Scalar_t, 3, 3>::Identity() - skew_epsilon_q) * omega;
  qdot.w() = eta_dot;
  qdot.vec() = epsilon_dot;
}

template <typename Scalar_t = double>
void pose_to_matrix(const Eigen::Matrix<Scalar_t, 7, 1>& pose, Eigen::Matrix<Scalar_t, 4, 4>& T)
{
  T = Eigen::Matrix<Scalar_t, 4, 4>::Zero();  // Ensure T is initialized as a 4x4 matrix
  Eigen::Quaternion<Scalar_t> q(pose(3), pose(4), pose(5), pose(6));
  q.normalize();
  T.template block<3, 3>(0, 0) = q.toRotationMatrix();
  T.template block<3, 1>(0, 3) = pose.template block<3, 1>(0, 0);
  T.template block<1, 4>(3, 0) << 0, 0, 0, 1;
}

template <typename Scalar_t = double>
void pose_to_matrix(const Eigen::Matrix<Scalar_t, 3, 1>& position, const Eigen::Quaternion<Scalar_t>& q,
                    Eigen::Matrix<Scalar_t, 4, 4>& T)
{
  T = Eigen::Matrix<Scalar_t, 4, 4>::Zero();  // Ensure T is initialized as a 4x4 matrix
  T.template block<3, 3>(0, 0) = q.toRotationMatrix();
  T.template block<3, 1>(0, 3) = position.template block<3, 1>(0, 0);
  T.template block<1, 4>(3, 0) << 0, 0, 0, 1;
}

template <typename Scalar_t = double>
void quaternion_continuity(const Eigen::Matrix<Scalar_t, 4, 1>& qnew, const Eigen::Matrix<Scalar_t, 4, 1>& qold,
                           Eigen::Matrix<Scalar_t, 4, 1>& q)
{
  q = qnew;
  Scalar_t tmp = qnew.template block<3, 1>(1, 0).transpose() * qold.template block<3, 1>(1, 0);
  if (tmp < -0.01)
  {
    q = -q;
  }
  // if(qnew(0)<0)
  // {
  //   q = -q;
  // }
}

template <typename Scalar_t = double>
Eigen::Matrix<Scalar_t, 4, 1> normalize_quaternion(const Eigen::Matrix<Scalar_t, 4, 1>& q_in)
{
  Scalar_t norm = q_in.norm();
  Eigen::Matrix<Scalar_t, 4, 1> q_out;
  if (norm > 0)
  {
    q_out = q_in / norm;
  }
  else
  {
    std::cerr << "Warning: Quaternion has zero norm and cannot be normalized." << std::endl;
    q_out = q_in;  // Return the input as-is if normalization is not possible
  }
  return q_out;
}

template <typename Scalar_t = double>
Eigen::Matrix<Scalar_t, 3,  3> quat2rotm(const Eigen::Quaternion<Scalar_t>& q_in)
{
  Eigen::Matrix<Scalar_t, 4, 1> q = normalize_quaternion(Eigen::Matrix<Scalar_t, 4, 1>(q_in.w(), q_in.x(), q_in.y(), q_in.z()));

  Scalar_t qw = q(0);
  Scalar_t qx = q(1);
  Scalar_t qy = q(2);
  Scalar_t qz = q(3);

  Eigen::Matrix<Scalar_t, 3, 3> R;
  R(0, 0) = 2 * (qw * qw + qx * qx) - 1;
  R(0, 1) = 2 * qx * qy - 2 * qz * qw;
  R(0, 2) = 2 * qx * qz + 2 * qy * qw;

  R(1, 0) = 2 * qx * qy + 2 * qz * qw;
  R(1, 1) = 2 * (qw * qw + qy * qy) - 1;
  R(1, 2) = 2 * qy * qz - 2 * qx * qw;

  R(2, 0) = 2 * qx * qz - 2 * qy * qw;
  R(2, 1) = 2 * qy * qz + 2 * qx * qw;
  R(2, 2) = 2 * (qw * qw + qz * qz) - 1;

  return R;
}

template <typename Scalar_t = double>
void quaternion_continuity(const Eigen::Quaternion<Scalar_t>& qnew, const Eigen::Quaternion<Scalar_t>& qold,
                           Eigen::Quaternion<Scalar_t>& q)
{
  Eigen::Matrix<Scalar_t, 4, 1> qnew_mat;
  qnew_mat << qnew.w(), qnew.x(), qnew.y(), qnew.z();

  Eigen::Matrix<Scalar_t, 4, 1> qold_mat;
  qold_mat << qold.w(), qold.x(), qold.y(), qold.z();

  Eigen::Matrix<Scalar_t, 4, 1> q_mat;

  quaternion_continuity(qnew_mat, qold_mat, q_mat);

  Eigen::Quaternion<Scalar_t> qtmp(q_mat[0], q_mat[1], q_mat[2], q_mat[3]);
  qtmp.normalize();
  q = qtmp;
}

}  // namespace uclv::geometry_helper
