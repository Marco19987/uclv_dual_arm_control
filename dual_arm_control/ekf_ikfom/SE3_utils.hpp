#ifndef SE3_UTILS_HPP
#define SE3_UTILS_HPP

#include <Eigen/Dense>
#include "utils.hpp"

#include <IKFoM_toolkit/mtk/src/mtkmath.hpp>
#include <IKFoM_toolkit/mtk/types/vect.hpp>
#include <IKFoM_toolkit/mtk/types/SOn.hpp>
#include <IKFoM_toolkit/mtk/build_manifold.hpp>

inline void adjoint_SE3(const Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 6, 6>& adjoint)
{
  // | R skew(p)*R |
  // | 0       R   |

  adjoint.setZero();
  adjoint.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
  adjoint.block<3, 3>(0, 3) = skew_matrix(T.block<3, 1>(0, 3)) * T.block<3, 3>(0, 0);
  adjoint.block<3, 3>(3, 3) = T.block<3, 3>(0, 0);
}

inline Eigen::Matrix<double, 6, 6> adjoint_SE3(const Eigen::Matrix<double, 4, 4>& T)
{
  Eigen::Matrix<double, 6, 6> adjoint;
  adjoint_SE3(T, adjoint);
  return adjoint;
}

inline void transpose_adjoint_invT_SE3(const Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 6, 6>& adjoint)
{
  // | R            0   |
  // | skew(p)*R    R   |

  adjoint.setZero();
  adjoint.block<3, 3>(0, 0) = T.block<3, 3>(0, 0);
  adjoint.block<3, 3>(3, 0) = skew_matrix(T.block<3, 1>(0, 3)) * T.block<3, 3>(0, 0);
  adjoint.block<3, 3>(3, 3) = T.block<3, 3>(0, 0);
}

inline Eigen::Matrix<double, 6, 6> transpose_adjoint_invT_SE3(const Eigen::Matrix<double, 4, 4>& T)
{
  Eigen::Matrix<double, 6, 6> adjoint;
  transpose_adjoint_invT_SE3(T, adjoint);
  return adjoint;
}

inline void jacobian_invT_T(const Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 6, 6>& jacobian)
{
  // J_invT_T = -adjoint(T)
  adjoint_SE3(T, jacobian);
  jacobian = -jacobian;
}
inline void jacobian_Ta_Tb_Ta(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& Tb,
                              Eigen::Matrix<double, 6, 6>& jacobian)
{
  // jacobian of Ta*Tb wrt Ta => J = adjoint(inv(Tb))
  adjoint_SE3(Tb.inverse(), jacobian);
}
inline void jacobian_Ta_Tb_Tb(Eigen::Matrix<double, 6, 6>& jacobian)
{
  // jacobian of Ta*Tb wrt Tb => J = eye(6)
  jacobian = Eigen::Matrix<double, 6, 6>::Identity();
}

inline void jacobian_gamma_R_T(const Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 3, 6>& jacobian)
{
  // jacobian of the gamma_R(T) function wrt T
  // gamma_R(T) = EXP_SO3([zeros(3,3) eye(3)] * LOG_SE3(T)) = R

  MTK::SO3<double> SO3;
  MTK::SEN<double> SEN;

  Eigen::VectorXd tau = SEN::log(T);
  Eigen::VectorXd theta;
  theta = tau.block(3, 0, 3, 1);

  Eigen::MatrixXd jac_theta;
  SO3.Jacob_right(theta, jac_theta);

  Eigen::MatrixXd jac_tau;
  SEN.Jacob_right_inv(tau, jac_tau);

  jacobian = jac_theta * jac_tau.block(3, 0, 3, 6);
}

inline void jacobian_gamma_R_T_SE3(const Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 6, 6>& jacobian)
{
  // jacobian of the gamma_R(T) function wrt T
  // gamma_R(T) = EXP_SE3([zeros(3,6);zeros(3,3) eye(3)] * LOG_SE3(T)) = [R 0; 0 1]

  MTK::SEN<double> SEN;

  Eigen::VectorXd tau = SEN::log(T);
  Eigen::VectorXd tau_theta;
  tau_theta.resize(6);
  tau_theta.setZero();
  tau_theta.block(3, 0, 3, 1) = tau.block(3, 0, 3, 1);

  Eigen::MatrixXd jac_theta;
  SEN.Jacob_right(tau_theta, jac_theta);

  Eigen::MatrixXd jac_tau;
  SEN.Jacob_right_inv(tau, jac_tau);
  jac_tau.block(0, 0, 3, 6).setZero();  // zero the first 3 rows of jac_tau

  jacobian = jac_theta * jac_tau;
}

inline void jacobian_gamma_t_T_SE3(const Eigen::Matrix<double, 4, 4>& T, Eigen::Matrix<double, 3, 6>& jacobian)
{
  // jacobian of the gamma_t(T) function wrt T
  // gamma_t(T) = [eye(3,3) zeros(3,3)] * LOG_SE3(T * gamma_R(inv(T))) = t

  MTK::SEN<double> SEN;
  Eigen::VectorXd tau_tmp(6);
  Eigen::MatrixXd dyn_jac_tmp(6, 6);
  Eigen::Matrix<double, 6, 6> tmp_jacobian, tmp_jacobian2, tmp_jacobian4;
  tau_tmp << T.block<3, 1>(0, 3), 0, 0, 0;
  SEN.Jacob_right_inv(tau_tmp, dyn_jac_tmp);

  SEN.mat.setIdentity();
  SEN.mat.block<3, 3>(0, 0) = T.block<3, 3>(0, 0).transpose();

  jacobian_Ta_Tb_Ta(SEN.mat, tmp_jacobian2);

  SEN.mat = T.inverse();
  jacobian_gamma_R_T_SE3(SEN.mat, tmp_jacobian);
  jacobian_invT_T(T, tmp_jacobian4);

  jacobian = (dyn_jac_tmp * (tmp_jacobian2 + tmp_jacobian * tmp_jacobian4)).block<3, 6>(0, 0);
}

inline void jacobian_gamma_R_vec_T(const Eigen::Matrix<double, 4, 4>& T, const Eigen::Vector<double, 3>& vec,
                                   Eigen::Matrix<double, 3, 6>& jacobian)
{
  // jacobian of the gamma_R(T)*vec function wrt T
  Eigen::Matrix<double, 3, 3> R, jacobian_tmp2;
  R = T.block<3, 3>(0, 0);
  jacobian_Rv_R(R, vec, jacobian_tmp2);

  Eigen::Matrix<double, 3, 6> jacobian_tmp;
  jacobian_gamma_R_T(T, jacobian_tmp);

  jacobian = jacobian_tmp2 * jacobian_tmp;
}

inline void jacobian_adjoint_T_twist_T(const Eigen::Ref<const Eigen::Matrix<double, 4, 4>>& T,
                                       const Eigen::Ref<const Eigen::Vector<double, 6>>& twist,
                                       Eigen::Matrix<double, 6, 6>& jacobian)
{
  // jacobian of the adjoint(T)*[v;omega] function wrt T

  Eigen::Matrix<double, 3, 6> jacobian_tmp, jacobian_tmp2, jacobian_tmp4, jacobian_tmp5;
  Eigen::Matrix<double, 6, 6> jacobian_tmp3;

  // jacob gamma_R_T * v
  Eigen::Vector3d v = twist.block<3, 1>(0, 0);
  jacobian_gamma_R_vec_T(T, v, jacobian_tmp);
  jacobian.block<3, 6>(0, 0) = jacobian_tmp;

  // jacob gamma_R_T * omega
  Eigen::Vector3d omega = twist.block<3, 1>(3, 0);
  jacobian_gamma_R_vec_T(T, omega, jacobian_tmp);
  jacobian.block<3, 6>(3, 0) = jacobian_tmp;

  // jacob -R * skew(omega) * R' * p
  Eigen::Matrix<double, 3, 3> R = T.block<3, 3>(0, 0);
  Eigen::Vector3d p = T.block<3, 1>(0, 3);
  jacobian_gamma_R_T(T, jacobian_tmp);
  jacobian_invT_T(T, jacobian_tmp3);
  jacobian_gamma_t_T_SE3(T, jacobian_tmp4);
  jacobian_gamma_R_T(T.inverse(), jacobian_tmp5);

  jacobian_tmp2 = skew_matrix(omega) *
                  (-R.transpose() * skew_matrix(p) * jacobian_tmp5 * jacobian_tmp3 + R.transpose() * jacobian_tmp4);

  jacobian.block<3, 6>(0, 0) +=
      -(R * jacobian_tmp2 - (R * skew_matrix(skew_matrix(omega) * R.transpose() * p)) * jacobian_tmp);
}

inline void jacobian_transpose_adjoint_invT_wrench_T(const Eigen::Matrix<double, 4, 4>& T,
                                                     const Eigen::Vector<double, 6>& wrench,
                                                     Eigen::Matrix<double, 6, 6>& jacobian)
{
  // jacobian of the adjoint(T)*[f;tau] function wrt T

  Eigen::Matrix<double, 3, 6> jacobian_tmp, jacobian_tmp2, jacobian_tmp4, jacobian_tmp5;
  Eigen::Matrix<double, 6, 6> jacobian_tmp3;

  // jacob gamma_R_T * f
  Eigen::Vector3d f = wrench.block<3, 1>(0, 0);
  jacobian_gamma_R_vec_T(T, f, jacobian_tmp);
  jacobian.block<3, 6>(0, 0) = jacobian_tmp;

  // jacob gamma_R_T * tau
  Eigen::Vector3d tau = wrench.block<3, 1>(3, 0);
  jacobian_gamma_R_vec_T(T, tau, jacobian_tmp);
  jacobian.block<3, 6>(3, 0) = jacobian_tmp;

  // jacob -R * skew(f) * R' * p
  Eigen::Matrix<double, 3, 3> R = T.block<3, 3>(0, 0);
  Eigen::Vector3d p = T.block<3, 1>(0, 3);
  jacobian_gamma_R_T(T, jacobian_tmp);
  jacobian_invT_T(T, jacobian_tmp3);
  jacobian_gamma_t_T_SE3(T, jacobian_tmp4);
  jacobian_gamma_R_T(T.inverse(), jacobian_tmp5);

  jacobian_tmp2 = skew_matrix(f) *
                  (-R.transpose() * skew_matrix(p) * jacobian_tmp5 * jacobian_tmp3 + R.transpose() * jacobian_tmp4);

  jacobian.block<3, 6>(3, 0) +=
      -(R * jacobian_tmp2 - (R * skew_matrix(skew_matrix(f) * R.transpose() * p)) * jacobian_tmp);
}

//##########################################
template <int dim_x>
inline void jacobian_gamma_R_Tx_vecx(const Eigen::Ref<const Eigen::Matrix<double, 3, 3>>& R,
                                     const Eigen::Ref<const Eigen::Vector<double, 3>>& vec,
                                     const Eigen::Ref<const Eigen::Matrix<double, 3, dim_x>>& jacobian_Rx_x,
                                     const Eigen::Ref<const Eigen::Matrix<double, 3, dim_x>>& jacobian_vecx_x,
                                     Eigen::Matrix<double, 3, dim_x>& jacobian)
{
  // jacobian of the gamma_R(T(x)) * vec(x) wrt x
  jacobian.setZero();

  Eigen::Matrix3d jacob_tmp;
  jacobian_Rv_R(R, vec, jacob_tmp);

  jacobian = jacob_tmp * jacobian_Rx_x + R * jacobian_vecx_x;
}

template <int dim_x>
inline void jacobian_transpose_adjoint_invTx_wrenchx(const Eigen::Matrix<double, 4, 4>& T,
                                                     const Eigen::Vector<double, 6>& wrench,
                                                     const Eigen::Matrix<double, 6, dim_x>& jacobian_T_x,
                                                     const Eigen::Matrix<double, 6, dim_x>& jacobian_wrench_x,
                                                     Eigen::Matrix<double, 6, dim_x>& jacobian)
{
  // jacobian of the adjoint(inv(T(x)))' * wrench(x) function wrt x
  jacobian.setZero();
  Eigen::Vector3d f = wrench.block<3, 1>(0, 0);
  Eigen::Vector3d tau = wrench.block<3, 1>(3, 0);
  Eigen::Matrix<double, 3, 6> jacobian_gamma_R_wrt_T;
  Eigen::Matrix<double, 3, 6> jacobian_gamma_t_wrt_T;
  Eigen::Matrix3d tmp_jacob;

  Eigen::Matrix<double, 3, dim_x> jacobian_tmp2;

  // jacobian gamma_R_T(x) * f(x)
  jacobian_gamma_R_T(T, jacobian_gamma_R_wrt_T);
  jacobian_gamma_R_Tx_vecx<dim_x>(T.block<3, 3>(0, 0), f, jacobian_gamma_R_wrt_T * jacobian_T_x,
                                  jacobian_wrench_x.template block<3, dim_x>(0, 0), jacobian_tmp2);

  jacobian.template block<3, dim_x>(0, 0) = jacobian_tmp2;

  // jacobian gamma_R_T(x) * tau
  jacobian_gamma_R_Tx_vecx<dim_x>(T.block<3, 3>(0, 0), tau, jacobian_gamma_R_wrt_T * jacobian_T_x,
                                  jacobian_wrench_x.template block<3, dim_x>(3, 0), jacobian_tmp2);

  jacobian.template block<3, dim_x>(3, 0) = jacobian_tmp2;

  // jacobian skew(gamma_p_T(x)) * R * f
  jacobian_gamma_t_T_SE3(T, jacobian_gamma_t_wrt_T);
  jacobian_Rv_R(T.block<3, 3>(0, 0), f, tmp_jacob);

  jacobian.template block<3, dim_x>(3, 0) +=
      -skew_matrix(T.block<3, 3>(0, 0) * f) * jacobian_gamma_t_wrt_T * jacobian_T_x +
      skew_matrix(T.block<3, 1>(0, 3)) * (tmp_jacob * jacobian_gamma_R_wrt_T * jacobian_T_x +
                                          T.block<3, 3>(0, 0) * jacobian_wrench_x.template block<3, dim_x>(0, 0));
}

#endif  // SE3_UTILS_HPP
