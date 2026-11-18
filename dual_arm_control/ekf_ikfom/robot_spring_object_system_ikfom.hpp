
#ifndef USE_IKFOM_H
#define USE_IKFOM_H

#include <IKFoM_toolkit/esekfom/esekfom.hpp>
#include "my_manifold.hpp"
#include "utils.hpp"
#include "SO3_utils.hpp"
#include "SE3_utils.hpp"

typedef MTK::vect<3, double> vect3;
typedef MTK::SO3<double> SO3;
typedef MTK::SEN<double> SEN;

MTK_BUILD_MANIFOLD(state_ikfom, ((SEN, bTo))((vect3, ovo))((vect3, oomegao))((SEN, b1Te1))((SEN, b2Te2))((SEN, b2Tb1))(
                                    (SEN, oTg1))((SEN, oTg2)));  //((vect3, f_v))((vect3, tau_v)));

MTK_BUILD_MANIFOLD(input_ikfom, ((vect3, e1ve1))((vect3, e1omegae1))((vect3, e2ve2))((vect3, e2omegae2)));

// MTK_BUILD_MANIFOLD(process_noise_ikfom,
//                    ((vect3, w_ovo))((vect3, w_oomegao))((vect3, w_linear_acc))((vect3, w_angular_acc))((
//                        vect3, w_e1ve1))((vect3, w_e1omegae1))((vect3, w_e2ve2))((vect3, w_e2omegae2))((vect3,
//                        w_b2vb1))( (vect3, w_b2omegab1))((vect3, w_ovg1))((vect3, w_oomegag1))((vect3,
//                        w_ovg2))((vect3,
//                                                                                                      w_oomegag2)));

#define state_dof 42          // 48
#define process_noise_dof 42  // 48
#define measurement_dof(num_pose_measure)                                                                              \
  (12 + 12 + 6 * (num_pose_measure))  //! zk = h(xk,vk) the dof is not the dim of zk

struct ProcessParams
{
  vect3 bg;                          // gravity vector in base frame
  SEN bTb1;                          // transofrm between robot1 base frame and common base frame
  Eigen::Matrix<double, 6, 6> M;     // inertia matrix
  Eigen::Matrix<double, 6, 6> K1;    // robot1-object elastic stiffness
  Eigen::Matrix<double, 6, 6> B1;    // robot1-object damping springs
  Eigen::Matrix<double, 6, 6> K2;    // robot2-object elastic stiffness
  Eigen::Matrix<double, 6, 6> B2;    // robot2-object damping stiffness
  Eigen::Matrix<double, 6, 6> Bair;  // air damping matrix
  int num_pose_measure_robot1;       //!   (num_pose_measure_robot1 + num_pose_measure_robot2) = num_pose_measure
  int num_pose_measure_robot2;  // num measure aruco from robot-i range from 0 to N (with N number arucos on the object)
  Eigen::Matrix<double, state_dof, state_dof> W_noise;  // state covariance matrix
  Eigen::Matrix<double, 6, 6> V_wrench_measure_rb1;     // measure noise covariance matrix measured wrench from robot1
  Eigen::Matrix<double, 6, 6> V_wrench_measure_rb2;     // measure noise covariance matrix measured wrench from robot2
  Eigen::Matrix<double, 6, 6> V_fkine_measure_rb1;      // measure noise covariance matrix measured fkine robot1
  Eigen::Matrix<double, 6, 6> V_fkine_measure_rb2;      // measure noise covariance matrix measured fkine robot2
  Eigen::Matrix<double, 6, 6> V_pose_measure_rb1;       // measure noise covariance matrix measured poses from robot1
  Eigen::Matrix<double, 6, 6> V_pose_measure_rb2;       // measure noise covariance matrix measured poses from robot2
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H_sel;  // selection matrix grasp (define the directions along
                                                                // which the wrench is transmitted)
  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic> H_sel_neg;  // selection matrix grasp negation (for transmitting
                                                                    // the end effector twist to the grasp frame along
                                                                    // the directions where the wrench is not
                                                                    // transmitted)
  Eigen::Matrix<double, 6, 6> K_grasp;                              // robot2-object damping stiffness
};

// Helper functions
inline void printProcessParams(const ProcessParams& process_params)
{
  std::cout << "Process Params:" << std::endl;
  std::cout << "bg: " << process_params.bg.transpose() << std::endl;
  std::cout << "bTb1:\n" << process_params.bTb1.mat << std::endl;
  std::cout << "M:\n" << process_params.M << std::endl;
  std::cout << "K1:\n" << process_params.K1 << std::endl;
  std::cout << "B1:\n" << process_params.B1 << std::endl;
  std::cout << "K2:\n" << process_params.K2 << std::endl;
  std::cout << "B2:\n" << process_params.B2 << std::endl;
  std::cout << "Bair:\n" << process_params.Bair << std::endl;
  std::cout << "num_pose_measure_robot1: " << process_params.num_pose_measure_robot1 << std::endl;
  std::cout << "num_pose_measure_robot2: " << process_params.num_pose_measure_robot2 << std::endl;
  std::cout << "W_noise:\n" << process_params.W_noise << std::endl;
  std::cout << "V_wrench_measure_rb1:\n" << process_params.V_wrench_measure_rb1 << std::endl;
  std::cout << "V_wrench_measure_rb2:\n" << process_params.V_wrench_measure_rb2 << std::endl;
  std::cout << "V_fkine_measure_rb1:\n" << process_params.V_fkine_measure_rb1 << std::endl;
  std::cout << "V_fkine_measure_rb2:\n" << process_params.V_fkine_measure_rb2 << std::endl;
  std::cout << "V_pose_measure_rb1:\n" << process_params.V_pose_measure_rb1 << std::endl;
  std::cout << "V_pose_measure_rb2:\n" << process_params.V_pose_measure_rb2 << std::endl;
  std::cout << "H selection :\n" << process_params.H_sel << std::endl;
  std::cout << "H selection_negate :\n" << process_params.H_sel_neg << std::endl;
}

inline void spring_model(state_ikfom& s, ProcessParams& process_params, Eigen::Matrix<double, 12, 1>& out,
                         const input_ikfom& in = input_ikfom(), bool multiply_K = true)
{
  Eigen::Matrix<double, 4, 4> g1Te1 =
      s.oTg1.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  Eigen::Matrix<double, 4, 4> g2Te2 =
      s.oTg2.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;

  Eigen::Matrix<double, 4, 4> e1Tg1 = g1Te1.inverse();
  Eigen::Matrix<double, 6, 1> e1he1;

  e1he1 = SEN::log(e1Tg1);
  if (multiply_K)
    e1he1 = process_params.K1 * e1he1;

  // if (compute_viscous)
  // {
  //   Eigen::Matrix3d eiRb =
  //       s.b1Te1.mat.block<3, 3>(0, 0).transpose() * process_params.bTb1.mat.block<3, 3>(0, 0).transpose();

  //   Eigen::Vector<double, 6> e1_twist_e1;
  //   e1_twist_e1 << in.e1ve1, in.e1omegae1;
  //   Eigen::Vector<double, 6> o_twist_o;
  //   o_twist_o << s.ovo, s.oomegao;
  //   Eigen::Vector<double, 6> e1_twist_g1;
  //   e1_twist_g1 << -e1_twist_e1 + get_Rbar(eiRb) * get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) *
  //                                     get_Rbar(s.oTg1.mat.block<3, 3>(0, 0)) * adjoint_SE3(s.oTg1.mat.inverse()) *
  //                                     o_twist_o;
  //   e1he1 = e1he1 + process_params.B1 * e1_twist_g1;
  // }

  Eigen::Matrix<double, 6, 1> e2he2;
  Eigen::Matrix<double, 4, 4> e2Tg2 = g2Te2.inverse();

  e2he2 = SEN::log(e2Tg2);
  if (multiply_K)
    e2he2 = process_params.K2 * e2he2;

  // if (compute_viscous)
  // {
  //   Eigen::Matrix3d eiRb = s.b2Te2.mat.block<3, 3>(0, 0).transpose() * s.b2Tb1.mat.block<3, 3>(0, 0) *
  //                          process_params.bTb1.mat.block<3, 3>(0, 0).transpose();
  //   Eigen::Vector<double, 6> e2_twist_e2;
  //   e2_twist_e2 << in.e2ve2, in.e2omegae2;
  //   Eigen::Vector<double, 6> o_twist_o;
  //   o_twist_o << s.ovo, s.oomegao;
  //   Eigen::Vector<double, 6> e2_twist_g2;
  //   e2_twist_g2 << -e2_twist_e2 + get_Rbar(eiRb) * get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) *
  //                                     get_Rbar(s.oTg2.mat.block<3, 3>(0, 0)) * adjoint_SE3(s.oTg2.mat.inverse()) *
  //                                     o_twist_o;
  //   e2he2 = e2he2 + process_params.B2 * e2_twist_g2;
  // }

  out << e1he1, e2he2;
}

inline void viscous_spring(state_ikfom& s, ProcessParams& process_params, Eigen::Matrix<double, 12, 1>& out,
                           const input_ikfom& in = input_ikfom(), bool multiply_B = true)
{
  Eigen::Matrix<double, 6, 1> e1he1;

  Eigen::Matrix3d eiRb =
      s.b1Te1.mat.block<3, 3>(0, 0).transpose() * process_params.bTb1.mat.block<3, 3>(0, 0).transpose();

  Eigen::Vector<double, 6> e1_twist_e1;
  e1_twist_e1 << in.e1ve1, in.e1omegae1;
  Eigen::Vector<double, 6> o_twist_o;
  o_twist_o << s.ovo, s.oomegao;
  Eigen::Vector<double, 6> e1_twist_g1;
  e1_twist_g1 << -e1_twist_e1 + get_Rbar(eiRb) * get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) *
                                    get_Rbar(s.oTg1.mat.block<3, 3>(0, 0)) * adjoint_SE3(s.oTg1.mat.inverse()) *
                                    o_twist_o;

  e1he1 = e1_twist_g1;
  if (multiply_B)
    e1he1 = process_params.B2 * e1he1;

  Eigen::Matrix<double, 6, 1> e2he2;

  eiRb = s.b2Te2.mat.block<3, 3>(0, 0).transpose() * s.b2Tb1.mat.block<3, 3>(0, 0) *
         process_params.bTb1.mat.block<3, 3>(0, 0).transpose();
  Eigen::Vector<double, 6> e2_twist_e2;
  e2_twist_e2 << in.e2ve2, in.e2omegae2;
  Eigen::Vector<double, 6> e2_twist_g2;
  e2_twist_g2 << -e2_twist_e2 + get_Rbar(eiRb) * get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) *
                                    get_Rbar(s.oTg2.mat.block<3, 3>(0, 0)) * adjoint_SE3(s.oTg2.mat.inverse()) *
                                    o_twist_o;

  e2he2 = e2_twist_g2;
  if (multiply_B)
    e2he2 = process_params.B2 * e2he2;

  out << e1he1, e2he2;
}

inline void compute_external_internal_wrench(state_ikfom& s, input_ikfom& in, ProcessParams& process_params,
                                             Eigen::Matrix<double, 6, 1>& external_wrench,
                                             Eigen::Matrix<double, 12, 1>& internal_wrench)
{
  // Eigen::Matrix<double, 12, 1> ghg = Eigen::Matrix<double, 12, 1>::Zero();  // spring model output in the grasp frame
  // spring_model(s, process_params, ghg, true, true, in);                     // spring model

  // Eigen::Matrix<double, 6, 12> oWRg = Eigen::Matrix<double, 6, 12>::Zero();
  // oWRg.block<6, 6>(0, 0) = transpose_adjoint_invT_SE3(s.oTg1.mat);
  // oWRg.block<6, 6>(0, 6) = transpose_adjoint_invT_SE3(s.oTg2.mat);

  // external_wrench = oWRg * ghg;

  // Eigen::Matrix<double, 12, 6> W_pinv = oWRg.completeOrthogonalDecomposition().pseudoInverse();
  // Eigen::Matrix<double, 12, 12> oRg_bar;
  // oRg_bar.setZero();
  // oRg_bar.block<6, 6>(0, 0) = get_Rbar(s.oTg1.mat.block<3, 3>(0, 0));
  // oRg_bar.block<6, 6>(6, 6) = get_Rbar(s.oTg2.mat.block<3, 3>(0, 0));

  // internal_wrench = oRg_bar * (Eigen::Matrix<double, 12, 12>::Identity() - W_pinv * oWRg) * ghg;  // internal wrenchs
  // rotated in the object frame

  //! different
  // Eigen::Matrix<double, 12, 1> ehe = Eigen::Matrix<double, 12, 1>::Zero();  // spring model output in the ee frame
  // spring_model(s, process_params, ehe, false, false, in);                   // spring model

  // ehe = -ehe;

  // Eigen::Matrix<double, 4, 4> g1Te1 =
  //     s.oTg1.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  // Eigen::Matrix<double, 4, 4> g2Te2 =
  //     s.oTg2.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;

  // Eigen::Matrix<double, 6, 12> oWRe = Eigen::Matrix<double, 6, 12>::Zero();
  // oWRe.block<6, 6>(0, 0) = transpose_adjoint_invT_SE3(s.oTg1.mat) * transpose_adjoint_invT_SE3(g1Te1);
  // oWRe.block<6, 6>(0, 6) = transpose_adjoint_invT_SE3(s.oTg2.mat) * transpose_adjoint_invT_SE3(g2Te2);

  // external_wrench = oWRe * ehe;

  // Eigen::Matrix<double, 12, 6> W_pinv = oWRe.completeOrthogonalDecomposition().pseudoInverse();

  // internal_wrench =
  //     (Eigen::Matrix<double, 12, 12>::Identity() - W_pinv * oWRe) * ehe;  // internal wrenchs in the end effector
  //     frames

  //! different 2
  // Eigen::Matrix<double, 12, 1> ghg = Eigen::Matrix<double, 12, 1>::Zero();  // spring model output in the grasp frame
  // spring_model(s, process_params, ghg, false, true, in);                     // spring model

  // Eigen::Matrix<double, 12, 12> oRg_bar;
  // oRg_bar.setZero();
  // oRg_bar.block<6, 6>(0, 0) = get_Rbar(s.oTg1.mat.block<3, 3>(0, 0));
  // oRg_bar.block<6, 6>(6, 6) = get_Rbar(s.oTg2.mat.block<3, 3>(0, 0));

  // // compute the grasp matrix
  // Eigen::Matrix<double, 6, 6> Wg1;
  // Wg1.setZero();
  // Wg1.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  // Wg1.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  // Wg1.block<3, 3>(3, 0) = skew_matrix(s.oTg1.mat.block<3, 1>(0, 3));
  // Eigen::Matrix<double, 6, 6> Wg2;
  // Wg2.setZero();
  // Wg2.block<3, 3>(0, 0) = Eigen::Matrix3d::Identity();
  // Wg2.block<3, 3>(3, 3) = Eigen::Matrix3d::Identity();
  // Wg2.block<3, 3>(3, 0) = skew_matrix(s.oTg2.mat.block<3, 1>(0, 3));

  // Eigen::Matrix<double, 6, 12> W;
  // W.setZero();
  // W.block<6, 6>(0, 0) = Wg1;
  // W.block<6, 6>(0, 6) = Wg2;

  // external_wrench = W * oRg_bar * ghg;

  // // compute the internal wrench
  // Eigen::Matrix<double, 12, 6> W_pinv = W.completeOrthogonalDecomposition().pseudoInverse();

  // internal_wrench = (Eigen::Matrix<double, 12, 12>::Identity() - W_pinv * W) * oRg_bar * ghg;

  //! different 4
  Eigen::Matrix<double, 12, 1> ehe = Eigen::Matrix<double, 12, 1>::Zero();  // spring model output in the ee frame
  spring_model(s, process_params, ehe, in);                                 // spring model

  ehe = -ehe;

  Eigen::Matrix<double, 4, 4> g1Te1 =
      s.oTg1.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  Eigen::Matrix<double, 4, 4> g2Te2 =
      s.oTg2.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;

  Eigen::Matrix<double, 6, 12> oWRe = Eigen::Matrix<double, 6, 12>::Zero();
  oWRe.block<6, 6>(0, 0) = transpose_adjoint_invT_SE3(s.oTg1.mat) * transpose_adjoint_invT_SE3(g1Te1);
  oWRe.block<6, 6>(0, 6) = transpose_adjoint_invT_SE3(s.oTg2.mat) * transpose_adjoint_invT_SE3(g2Te2);

  external_wrench = oWRe * process_params.H_sel.transpose() * process_params.H_sel * ehe;

  Eigen::MatrixXd W_pinv = (oWRe * process_params.H_sel.transpose()).completeOrthogonalDecomposition().pseudoInverse();

  // Eigen::Vector<double, 10> internal_wrench_t;
  // internal_wrench_t = (Eigen::Matrix<double, 10, 10>::Identity() - W_pinv * oWRe * process_params.H_sel.transpose())
  // *
  //                     process_params.H_sel * ehe;  // internal wrenchs in the end effector frames

  // internal_wrench.setZero();
  // internal_wrench.block<5, 1>(0, 0) = internal_wrench_t.block<5, 1>(0, 0);
  // internal_wrench.block<5, 1>(6, 0) = internal_wrench_t.block<5, 1>(5, 0);

  Eigen::MatrixXd eye = Eigen::MatrixXd::Identity(process_params.H_sel.rows(), process_params.H_sel.rows());
  internal_wrench = process_params.H_sel.transpose() * (eye - W_pinv * oWRe * process_params.H_sel.transpose()) *
                    process_params.H_sel * ehe;  // internal wrenchs in the end effector frames
}

inline void jacobian_spring_elastic(state_ikfom& s, ProcessParams& process_params,
                                    Eigen::Matrix<double, 12, state_dof>& out, const input_ikfom& in = input_ikfom(),
                                    const bool multiply_K = true)
{
  out.setZero();

  Eigen::Matrix<double, 6, 6> K1 = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> K2 = Eigen::Matrix<double, 6, 6>::Identity();

  if (multiply_K)
  {
    K1 = process_params.K1;
    K2 = process_params.K2;
  }

  Eigen::Matrix<double, 6, 6> tmp_jacobian = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> tmp_jacobian2 = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 3, 6> tmp_jacobian3 = Eigen::Matrix<double, 3, 6>::Zero();
  Eigen::Matrix<double, 6, 6> tmp_jacobian4 = Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Matrix<double, 4, 4> g1Te1 =
      s.oTg1.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  Eigen::Matrix<double, 4, 4> g2Te2 =
      s.oTg2.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;

  Eigen::Matrix<double, 4, 4> e1Tg1 = g1Te1.inverse();

  Eigen::Vector<double, 6> o_twist_o;
  o_twist_o << s.ovo, s.oomegao;

  //* jacobian spring 1 - elastic part - tau = K * Log(T)
  SEN temp_SE3;
  Eigen::VectorXd ei_tau_gi;
  ei_tau_gi = SEN::log(e1Tg1);
  Eigen::MatrixXd jacob_right_inv_ei_tau_gi;
  temp_SE3.Jacob_right_inv(ei_tau_gi, jacob_right_inv_ei_tau_gi);

  // wrt bTo
  jacobian_Ta_Tb_Ta(s.oTg1.mat, tmp_jacobian);
  out.template block<6, 6>(0, 0) = K1 * jacob_right_inv_ei_tau_gi * tmp_jacobian;

  // wrt b1Te1
  jacobian_Ta_Tb_Ta(process_params.bTb1.mat.inverse() * s.bTo.mat * s.oTg1.mat, tmp_jacobian);
  jacobian_invT_T(s.b1Te1.mat, tmp_jacobian2);
  out.template block<6, 6>(0, 12) = K1 * jacob_right_inv_ei_tau_gi * tmp_jacobian * tmp_jacobian2;

  // wrt oTg1
  out.template block<6, 6>(0, 30) = K1 * jacob_right_inv_ei_tau_gi;

  //* jacobian spring 2 - elastic part - tau = K * Log(T)
  Eigen::Matrix<double, 4, 4> e2Tg2 = g2Te2.inverse();
  ei_tau_gi = SEN::log(e2Tg2);
  temp_SE3.Jacob_right_inv(ei_tau_gi, jacob_right_inv_ei_tau_gi);

  // wrt bTo
  jacobian_Ta_Tb_Ta(s.oTg2.mat, tmp_jacobian);
  out.template block<6, 6>(6, 0) = K2 * jacob_right_inv_ei_tau_gi * tmp_jacobian;

  // wrt b2Te2
  jacobian_Ta_Tb_Ta(s.b2Tb1.mat * process_params.bTb1.mat.inverse() * s.bTo.mat * s.oTg2.mat, tmp_jacobian);
  jacobian_invT_T(s.b2Te2.mat, tmp_jacobian2);
  out.template block<6, 6>(6, 18) = K2 * jacob_right_inv_ei_tau_gi * tmp_jacobian * tmp_jacobian2;

  // wrt b2Tb1
  jacobian_Ta_Tb_Ta(process_params.bTb1.mat.inverse() * s.bTo.mat * s.oTg2.mat, tmp_jacobian);
  out.template block<6, 6>(6, 24) = K2 * jacob_right_inv_ei_tau_gi * tmp_jacobian;

  // wrt oTg2
  out.template block<6, 6>(6, 36) = K2 * jacob_right_inv_ei_tau_gi;
}

inline void jacobian_spring_viscous(state_ikfom& s, ProcessParams& process_params,
                                    Eigen::Matrix<double, 12, state_dof>& out, const input_ikfom& in = input_ikfom(),
                                    const bool multiply_beta = true)
{
  out.setZero();

  Eigen::Matrix<double, 6, 6> B1 = Eigen::Matrix<double, 6, 6>::Identity();
  Eigen::Matrix<double, 6, 6> B2 = Eigen::Matrix<double, 6, 6>::Identity();

  if (multiply_beta)
  {
    B1 = process_params.B1;
    B2 = process_params.B2;
  }

  Eigen::Matrix<double, 6, 6> tmp_jacobian = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> tmp_jacobian2 = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 3, 6> tmp_jacobian3 = Eigen::Matrix<double, 3, 6>::Zero();
  Eigen::Matrix<double, 6, 6> tmp_jacobian4 = Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Matrix<double, 4, 4> g1Te1 =
      s.oTg1.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  Eigen::Matrix<double, 4, 4> g2Te2 =
      s.oTg2.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;

  Eigen::Matrix<double, 4, 4> e1Tg1 = g1Te1.inverse();

  Eigen::Vector<double, 6> o_twist_o;
  o_twist_o << s.ovo, s.oomegao;

  SEN temp_SE3;

  {
    Eigen::Matrix3d eiRb =
        s.b1Te1.mat.block<3, 3>(0, 0).transpose() * process_params.bTb1.mat.block<3, 3>(0, 0).transpose();

    Eigen::Matrix<double, 6, state_dof> viscous_jacobian = Eigen::Matrix<double, 6, state_dof>::Zero();
    Eigen::Vector<double, 6> tmp_twist = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 3> tmp_vec = Eigen::Vector<double, 3>::Zero();

    // bTo
    tmp_twist = get_Rbar(s.oTg1.mat.block<3, 3>(0, 0)) * adjoint_SE3(s.oTg1.mat.inverse()) * o_twist_o;
    tmp_vec = tmp_twist.block<3, 1>(0, 0);
    jacobian_gamma_R_T(s.bTo.mat, tmp_jacobian3);
    viscous_jacobian.template block<3, 6>(0, 0) =
        -B1.block<3, 3>(0, 0) * eiRb * s.bTo.mat.block<3, 3>(0, 0) * skew_matrix(tmp_vec) * tmp_jacobian3;
    tmp_vec = tmp_twist.block<3, 1>(3, 0);
    viscous_jacobian.template block<3, 6>(3, 0) =
        -B1.block<3, 3>(3, 3) * eiRb * s.bTo.mat.block<3, 3>(0, 0) * skew_matrix(tmp_vec) * tmp_jacobian3;

    // otwisto
    viscous_jacobian.template block<6, 6>(0, 6) = B1 * get_Rbar(eiRb) * get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) *
                                                  get_Rbar(s.oTg1.mat.block<3, 3>(0, 0)) *
                                                  adjoint_SE3(s.oTg1.mat.inverse());

    // b1Te1
    tmp_twist = get_Rbar(process_params.bTb1.mat.block<3, 3>(0, 0).transpose()) *
                get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) * get_Rbar(s.oTg1.mat.block<3, 3>(0, 0)) *
                adjoint_SE3(s.oTg1.mat.inverse()) * o_twist_o;
    temp_SE3.mat = s.b1Te1.mat.inverse();
    jacobian_gamma_R_T(temp_SE3.mat, tmp_jacobian3);
    jacobian_invT_T(s.b1Te1.mat, tmp_jacobian2);
    tmp_vec = tmp_twist.block<3, 1>(0, 0);
    viscous_jacobian.template block<3, 6>(0, 12) = -B1.block<3, 3>(0, 0) * s.b1Te1.mat.block<3, 3>(0, 0).transpose() *
                                                   skew_matrix(tmp_vec) * tmp_jacobian3 * tmp_jacobian2;
    tmp_vec = tmp_twist.block<3, 1>(3, 0);
    viscous_jacobian.template block<3, 6>(3, 12) = -B1.block<3, 3>(3, 3) * s.b1Te1.mat.block<3, 3>(0, 0).transpose() *
                                                   skew_matrix(tmp_vec) * tmp_jacobian3 * tmp_jacobian2;

    // oTg1
    jacobian_gamma_t_T_SE3(s.oTg1.mat, tmp_jacobian3);

    viscous_jacobian.template block<3, 6>(0, 30) =
        B1.block<3, 3>(0, 0) * eiRb * s.bTo.mat.block<3, 3>(0, 0) * skew_matrix(s.oomegao) * tmp_jacobian3;

    out.template block<6, state_dof>(0, 0) = viscous_jacobian;
  }

  {
    Eigen::Matrix3d eiRb = s.b2Te2.mat.block<3, 3>(0, 0).transpose() * s.b2Tb1.mat.block<3, 3>(0, 0) *
                           process_params.bTb1.mat.block<3, 3>(0, 0).transpose();

    Eigen::Matrix<double, 6, state_dof> viscous_jacobian = Eigen::Matrix<double, 6, state_dof>::Zero();
    Eigen::Vector<double, 6> tmp_twist = Eigen::Vector<double, 6>::Zero();
    Eigen::Vector<double, 3> tmp_vec = Eigen::Vector<double, 3>::Zero();

    // bTo
    tmp_twist = get_Rbar(s.oTg2.mat.block<3, 3>(0, 0)) * adjoint_SE3(s.oTg2.mat.inverse()) * o_twist_o;
    tmp_vec = tmp_twist.block<3, 1>(0, 0);
    jacobian_gamma_R_T(s.bTo.mat, tmp_jacobian3);
    viscous_jacobian.template block<3, 6>(0, 0) =
        -B2.block<3, 3>(0, 0) * eiRb * s.bTo.mat.block<3, 3>(0, 0) * skew_matrix(tmp_vec) * tmp_jacobian3;
    tmp_vec = tmp_twist.block<3, 1>(3, 0);
    viscous_jacobian.template block<3, 6>(3, 0) =
        -B2.block<3, 3>(3, 3) * eiRb * s.bTo.mat.block<3, 3>(0, 0) * skew_matrix(tmp_vec) * tmp_jacobian3;

    // otwisto
    viscous_jacobian.template block<6, 6>(0, 6) = B2 * get_Rbar(eiRb) * get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) *
                                                  get_Rbar(s.oTg2.mat.block<3, 3>(0, 0)) *
                                                  adjoint_SE3(s.oTg2.mat.inverse());

    // b2Te2
    tmp_twist = get_Rbar(s.b2Tb1.mat.block<3, 3>(0, 0)) *
                get_Rbar(process_params.bTb1.mat.block<3, 3>(0, 0).transpose()) *
                get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) * get_Rbar(s.oTg2.mat.block<3, 3>(0, 0)) *
                adjoint_SE3(s.oTg2.mat.inverse()) * o_twist_o;
    tmp_vec = tmp_twist.block<3, 1>(0, 0);
    temp_SE3.mat = s.b2Te2.mat.inverse();
    jacobian_gamma_R_T(temp_SE3.mat, tmp_jacobian3);
    jacobian_invT_T(s.b2Te2.mat, tmp_jacobian2);
    viscous_jacobian.template block<3, 6>(0, 18) = -B2.block<3, 3>(0, 0) * s.b2Te2.mat.block<3, 3>(0, 0).transpose() *
                                                   skew_matrix(tmp_vec) * tmp_jacobian3 * tmp_jacobian2;
    tmp_vec = tmp_twist.block<3, 1>(3, 0);
    viscous_jacobian.template block<3, 6>(3, 18) = -B2.block<3, 3>(3, 3) * s.b2Te2.mat.block<3, 3>(0, 0).transpose() *
                                                   skew_matrix(tmp_vec) * tmp_jacobian3 * tmp_jacobian2;

    // b2Tb1
    tmp_twist = get_Rbar(process_params.bTb1.mat.block<3, 3>(0, 0).transpose()) *
                get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) * get_Rbar(s.oTg2.mat.block<3, 3>(0, 0)) *
                adjoint_SE3(s.oTg2.mat.inverse()) * o_twist_o;
    tmp_vec = tmp_twist.block<3, 1>(0, 0);
    jacobian_gamma_R_T(s.b2Tb1.mat, tmp_jacobian3);
    viscous_jacobian.template block<3, 6>(0, 24) = -B2.block<3, 3>(0, 0) * s.b2Te2.mat.block<3, 3>(0, 0).transpose() *
                                                   s.b2Tb1.mat.block<3, 3>(0, 0) * skew_matrix(tmp_vec) * tmp_jacobian3;
    tmp_vec = tmp_twist.block<3, 1>(3, 0);
    viscous_jacobian.template block<3, 6>(3, 24) = -B2.block<3, 3>(0, 0) * s.b2Te2.mat.block<3, 3>(0, 0).transpose() *
                                                   s.b2Tb1.mat.block<3, 3>(0, 0) * skew_matrix(tmp_vec) * tmp_jacobian3;

    // oTg2
    jacobian_gamma_t_T_SE3(s.oTg2.mat, tmp_jacobian3);

    viscous_jacobian.template block<3, 6>(0, 36) =
        B2.block<3, 3>(0, 0) * eiRb * s.bTo.mat.block<3, 3>(0, 0) * skew_matrix(s.oomegao) * tmp_jacobian3;

    out.template block<6, state_dof>(6, 0) = viscous_jacobian;
  }
}

Eigen::Matrix<double, state_dof, 1> get_f(state_ikfom& s, const input_ikfom& in, ProcessParams& process_params)
{
  Eigen::Matrix<double, state_dof, 1> res = Eigen::Matrix<double, state_dof, 1>::Zero();

  Eigen::Matrix<double, 12, 1> twist_grasp_control =
      Eigen::Matrix<double, 12, 1>::Zero();  // spring model output in the ee frames
  Eigen::Matrix<double, 12, 1> ehe_e =
      Eigen::Matrix<double, 12, 1>::Zero();  // spring model output in the ee frames elastic term
  Eigen::Matrix<double, 12, 1> ehe_b =
      Eigen::Matrix<double, 12, 1>::Zero();  // spring model output in the ee frames viscous term

  spring_model(s, process_params, ehe_e, in, false);  // spring model - elastic term
  ehe_e = -ehe_e;  //! the wrench computed right now is the wrench measured by the tactile sensors which measures the
                   //! wrench exerted by the object on the sensors. So we need to change the sign.

  viscous_spring(s, process_params, ehe_b, in, false);  // spring model - viscous term
  ehe_b = -ehe_b;  //! the wrench computed right now is the wrench measured by the tactile sensors which measures the
  //! wrench exerted by the object on the sensors. So we need to change the sign.

  Eigen::Matrix<double, 12, 12> K_grasp;
  K_grasp.setIdentity();
  // double kl = process_params.gain_linear_grasp_control;
  // double komega = process_params.gain_angular_grasp_control;

  // K_grasp.setZero();
  // K_grasp.diagonal() << Eigen::Vector<double, 12>(kl, kl, kl, komega, komega, komega, kl, kl, kl, komega, komega,
  //                                                 komega);
  K_grasp.block<6, 6>(0, 0) = process_params.K_grasp;
  K_grasp.block<6, 6>(6, 6) = process_params.K_grasp;

  // K_grasp = 1E-3 * K_grasp;
  twist_grasp_control = K_grasp * ehe_e + ehe_b;

  ehe_e.block<6, 1>(0, 0) = process_params.K1 * ehe_e.block<6, 1>(0, 0);
  ehe_e.block<6, 1>(6, 0) = process_params.K2 * ehe_e.block<6, 1>(6, 0);
  ehe_b.block<6, 1>(0, 0) = process_params.B1 * ehe_b.block<6, 1>(0, 0);
  ehe_b.block<6, 1>(6, 0) = process_params.B2 * ehe_b.block<6, 1>(6, 0);

  Eigen::Matrix<double, 4, 4> g1Te1 =
      s.oTg1.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  Eigen::Matrix<double, 4, 4> g2Te2 =
      s.oTg2.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;

  Eigen::Matrix<double, 12, 12> WR_grasp = Eigen::Matrix<double, 12, 12>::Zero();
  WR_grasp.template block<6, 6>(0, 0) = transpose_adjoint_invT_SE3(g1Te1);
  WR_grasp.template block<6, 6>(6, 6) = transpose_adjoint_invT_SE3(g2Te2);

  Eigen::Matrix<double, 6, 12> oWRg = Eigen::Matrix<double, 6, 12>::Zero();
  oWRg.block<6, 6>(0, 0) = adjoint_SE3(s.oTg1.mat.inverse()).transpose();
  oWRg.block<6, 6>(0, 6) = adjoint_SE3(s.oTg2.mat.inverse()).transpose();

  Eigen::Matrix<double, 6, 1> o_twist_o;
  o_twist_o << s.ovo, s.oomegao;  // object twist in the object frame

  Eigen::Matrix<double, 3, 3> skew_oomegao;
  Eigen::Matrix<double, 3, 3> skew_ovo;
  skew_matrix(s.oomegao, skew_oomegao);
  skew_matrix(s.ovo, skew_ovo);
  Eigen::Matrix<double, 6, 6> double_skew = Eigen::Matrix<double, 6, 6>::Zero();
  double_skew.template block<3, 3>(0, 0) = skew_oomegao;
  double_skew.template block<3, 3>(3, 0) = skew_ovo;
  double_skew.template block<3, 3>(3, 3) = skew_oomegao;

  Eigen::Matrix<double, 3, 1> og =
      s.bTo.mat.block<3, 3>(0, 0).transpose() * process_params.bg;  // gravity vector in object frame

  // Eigen::Matrix<double, 6, 1> h_v;
  // h_v << s.f_v, s.tau_v;

  Eigen::Matrix<double, 6, 1> o_twist_dot_o =
      process_params.M.inverse() *
          (oWRg * WR_grasp * process_params.H_sel.transpose() * process_params.H_sel * ehe_e + oWRg * WR_grasp * ehe_b -
           process_params.Bair * o_twist_o - double_skew * process_params.M * o_twist_o) +
      Eigen::Vector<double, 6>(og(0), og(1), og(2), 0, 0, 0);

  res.template block<3, 1>(0, 0) = s.ovo;
  res.template block<3, 1>(3, 0) = s.oomegao;
  res.template block<6, 1>(6, 0) = o_twist_dot_o;
  res.template block<3, 1>(12, 0) = in.e1ve1;
  res.template block<3, 1>(15, 0) = in.e1omegae1;
  res.template block<3, 1>(18, 0) = in.e2ve2;
  res.template block<3, 1>(21, 0) = in.e2omegae2;

  // Eigen::Vector<double, 6> e1_twist_e1;
  // e1_twist_e1 << in.e1ve1, in.e1omegae1;
  // Eigen::Vector<double, 6> e2_twist_e2;
  // e2_twist_e2 << in.e2ve2, in.e2omegae2;

  // Eigen::Matrix3d eiRb =
  //     s.b1Te1.mat.block<3, 3>(0, 0).transpose() * process_params.bTb1.mat.block<3, 3>(0, 0).transpose();

  // Eigen::Vector<double, 6> e1_twist_g1;
  // e1_twist_g1 << -e1_twist_e1 + get_Rbar(eiRb) * get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) *
  //                                   get_Rbar(s.oTg1.mat.block<3, 3>(0, 0)) * adjoint_SE3(s.oTg1.mat.inverse()) *
  //                                   o_twist_o;

  // eiRb = s.b2Te2.mat.block<3, 3>(0, 0).transpose() * s.b2Tb1.mat.block<3, 3>(0, 0) *
  //        process_params.bTb1.mat.block<3, 3>(0, 0).transpose();
  // Eigen::Vector<double, 6> e2_twist_g2;
  // e2_twist_g2 << -e2_twist_e2 + get_Rbar(eiRb) * get_Rbar(s.bTo.mat.block<3, 3>(0, 0)) *
  //                                   get_Rbar(s.oTg2.mat.block<3, 3>(0, 0)) * adjoint_SE3(s.oTg2.mat.inverse()) *
  //                                   o_twist_o;

  // Eigen::Vector<double, 12> g_twist_g;  // stacked combination of end effector velocities reported in the grasp
  // frames g_twist_g.setZero(); g_twist_g.block<6, 1>(0, 0) = adjoint_SE3(g1Te1) * (-e1_twist_g1); g_twist_g.block<6,
  // 1>(6, 0) = adjoint_SE3(g2Te2) * (-e2_twist_g2);
  Eigen::Matrix<double, 12, 12> gAdje = Eigen::Matrix<double, 12, 12>::Zero();
  gAdje.template block<6, 6>(0, 0) = adjoint_SE3(g1Te1);
  gAdje.template block<6, 6>(6, 6) = adjoint_SE3(g2Te2);

  if (process_params.H_sel_neg.rows() != 0)
  {
    res.template block<12, 1>(30, 0) =
        process_params.H_sel_neg.transpose() * process_params.H_sel_neg * gAdje * twist_grasp_control;  // g_twist_g;
  }

  // res.template block<6, 1>(30, 0) =
  //     process_params.H_sel_neg.transpose() * process_params.H_sel_neg * adjoint_SE3(g1Te1) * e1_twist_e1;

  // res.template block<6, 1>(36, 0) =
  //     process_params.H_sel_neg.transpose() * process_params.H_sel_neg * adjoint_SE3(g2Te2) * e2_twist_e2;

  // std::cout << "res: " << res.transpose() << std::endl;

  return res;
}

Eigen::Matrix<double, state_dof, state_dof> df_dx(state_ikfom& s, const input_ikfom& in, ProcessParams& process_params)
{
  Eigen::Matrix<double, state_dof, state_dof> cov = Eigen::Matrix<double, state_dof, state_dof>::Zero();
  Eigen::Matrix<double, 3, 3> temp_jacob = Eigen::Matrix<double, 3, 3>::Zero();   // temporary jacobian
  Eigen::Matrix<double, 6, 6> temp_jacob3 = Eigen::Matrix<double, 6, 6>::Zero();  // temporary jacobian
  Eigen::Matrix<double, 3, 6> temp_jacob4 = Eigen::Matrix<double, 3, 6>::Zero();  // temporary jacobian
  Eigen::Matrix<double, 6, 6> temp_jacob5 = Eigen::Matrix<double, 6, 6>::Zero();  // temporary jacobian
  Eigen::Matrix<double, 6, 6> temp_jacob6 = Eigen::Matrix<double, 6, 6>::Zero();  // temporary jacobian

  Eigen::Matrix<double, 3, 3> temp_skew = Eigen::Matrix<double, 3, 3>::Zero();  // temporary skew
  Eigen::Matrix<double, 6, 6> M_inv = process_params.M.inverse();

  //* jacobians bTo "=" ovo;
  cov.template block<6, 6>(0, 6) = Eigen::Matrix<double, 6, 6>::Identity();

  //* jacobians twist_dot = M^-1 * (oh - Bair * o_twist_o - double_skew * M * o_twist_o) + bRo_bar^T * bg_ext;

  // gravity term bRo' * bg
  jacobian_Rv_R(s.bTo.mat.template block<3, 3>(0, 0).transpose(), process_params.bg, temp_jacob);
  Eigen::Matrix4d oTb = s.bTo.mat.inverse();
  jacobian_gamma_R_T(oTb, temp_jacob4);
  jacobian_invT_T(s.bTo.mat, temp_jacob3);

  cov.template block<3, 6>(6, 0) = temp_jacob * temp_jacob4 * temp_jacob3;

  // air viscous term  - M_inv * Bair * o_twist_o
  cov.template block<6, 6>(6, 6) = -M_inv * process_params.Bair;

  // double skew term - M_inv * double_skew * M * o_twist_o
  skew_matrix(s.oomegao, temp_skew);
  cov.template block<3, 3>(6, 6) += -M_inv.block<3, 3>(0, 0) * temp_skew * process_params.M.block<3, 3>(0, 0);

  jacobian_skew_w_v_w(-process_params.M.block<3, 3>(0, 0) * s.ovo, temp_jacob);
  cov.template block<3, 3>(6, 9) += -M_inv.block<3, 3>(0, 0) * temp_jacob;

  jacobian_skew_w_Aw_w(s.ovo, process_params.M.block<3, 3>(0, 0), temp_jacob);
  cov.template block<3, 3>(9, 6) += -M_inv.block<3, 3>(0, 0) * temp_jacob;

  jacobian_skew_w_Aw_w(s.oomegao, process_params.M.block<3, 3>(3, 3), temp_jacob);
  cov.template block<3, 3>(9, 9) += -M_inv.block<3, 3>(3, 3) * temp_jacob;

  //! spring term
  Eigen::Matrix<double, 12, state_dof> jacob_spring;
  Eigen::Matrix<double, 12, state_dof> jacob_spring_elastic;
  Eigen::Matrix<double, 12, state_dof> jacob_spring_viscous;

  jacobian_spring_elastic(s, process_params, jacob_spring_elastic);
  jacobian_spring_viscous(s, process_params, jacob_spring_viscous);
  jacob_spring_elastic = -jacob_spring_elastic;  // change sign jacobian due to negative sign in the measures
  jacob_spring_viscous = -jacob_spring_viscous;  // change sign jacobian due to negative sign in the measures
  jacob_spring = jacob_spring_viscous + process_params.H_sel.transpose() * process_params.H_sel * jacob_spring_elastic;
  // jacob_spring = process_params.H_sel.transpose() * process_params.H_sel * jacob_spring;

  // compute wrench
  Eigen::Matrix<double, 12, 1> ehe = Eigen::Matrix<double, 12, 1>::Zero();
  Eigen::Matrix<double, 12, 1> e_tau_g_e = Eigen::Matrix<double, 12, 1>::Zero();    // elastic
  Eigen::Matrix<double, 12, 1> e_twist_g_e = Eigen::Matrix<double, 12, 1>::Zero();  // viscous
  Eigen::Matrix<double, 12, 1> ehe_e = Eigen::Matrix<double, 12, 1>::Zero();        // elastic
  Eigen::Matrix<double, 12, 1> ehe_b = Eigen::Matrix<double, 12, 1>::Zero();        // viscous
  spring_model(s, process_params, e_tau_g_e, in, false);
  viscous_spring(s, process_params, e_twist_g_e, in, false);
  e_tau_g_e = -e_tau_g_e;      // change sign due to negative sign in the measures
  e_twist_g_e = -e_twist_g_e;  // change sign due to negative sign in the measures

  ehe_e.block<6, 1>(0, 0) = process_params.K1 * e_tau_g_e.block<6, 1>(0, 0);
  ehe_e.block<6, 1>(6, 0) = process_params.K2 * e_tau_g_e.block<6, 1>(6, 0);
  ehe_b.block<6, 1>(0, 0) = process_params.B1 * e_twist_g_e.block<6, 1>(0, 0);
  ehe_b.block<6, 1>(6, 0) = process_params.B2 * e_twist_g_e.block<6, 1>(6, 0);

  ehe = process_params.H_sel.transpose() * process_params.H_sel * ehe_e + ehe_b;

  // ehe = process_params.H_sel.transpose() * process_params.H_sel * ehe;
  Eigen::Matrix<double, 6, 1> e1he1_e1 = ehe.block<6, 1>(0, 0);
  Eigen::Matrix<double, 6, 1> e2he2_e2 = ehe.block<6, 1>(6, 0);

  // //! implent here the product for the adjoint jacobian - DON't forget premultiply inv(M)
  // Eigen::Matrix<double, 6, state_dof> jacob_spring_object_frame;  // resulting jacobian after adjont multiplication
  // for
  //                                                                 // h(x,u) springs
  // jacob_spring_object_frame.setZero();

  // Eigen::Matrix<double, 4, 4> g1Te1 =
  //     s.oTg1.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  // Eigen::Matrix<double, 4, 4> g2Te2 =
  //     s.oTg2.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;
  // Eigen::Matrix<double, 6, 6> Adj_t_inv_g1Te1 = transpose_adjoint_invT_SE3(g1Te1);
  // Eigen::Matrix<double, 6, 6> Adj_t_inv_g2Te2 = transpose_adjoint_invT_SE3(g2Te2);
  // Eigen::Matrix<double, 6, 6> Adj_t_inv_oTg1 = transpose_adjoint_invT_SE3(s.oTg1.mat);
  // Eigen::Matrix<double, 6, 6> Adj_t_inv_oTg2 = transpose_adjoint_invT_SE3(s.oTg2.mat);
  // Eigen::Matrix<double, 6, 6> jacob_Adj_t_inv_g1Te1 = Eigen::Matrix<double, 6, 6>::Zero();
  // Eigen::Matrix<double, 6, 6> jacob_Adj_t_inv_g2Te2 = Eigen::Matrix<double, 6, 6>::Zero();
  // jacobian_transpose_adjoint_invT_wrench_T(g1Te1, e1he1_e1, jacob_Adj_t_inv_g1Te1);
  // jacobian_transpose_adjoint_invT_wrench_T(g2Te2, e2he2_e2, jacob_Adj_t_inv_g2Te2);

  // // std::cout << "g1Te1 \n " << g1Te1 << "\n";
  // // std::cout << "g2Te2 \n " << g2Te2 << "\n";
  // // std::cout << "Adj_t_inv_g1Te1 \n " << Adj_t_inv_g1Te1 << "\n";
  // // std::cout << "Adj_t_inv_g2Te2 \n " << Adj_t_inv_g2Te2 << "\n";
  // // std::cout << "Adj_t_inv_oTg1 \n " << Adj_t_inv_oTg1 << "\n";
  // // std::cout << "Adj_t_inv_oTg2 \n " << Adj_t_inv_oTg2 << "\n";
  // // std::cout << "jacob_Adj_t_inv_g1Te1 \n " << jacob_Adj_t_inv_g1Te1 << "\n";
  // // std::cout << "jacob_Adj_t_inv_g2Te2 \n " << jacob_Adj_t_inv_g2Te2 << "\n";

  // //* adjoint products spring term - 1

  // // bTo - 1
  // jacobian_Ta_Tb_Ta(process_params.bTb1.mat * s.b1Te1.mat, temp_jacob3);
  // jacobian_invT_T(s.bTo.mat, temp_jacob5);
  // jacob_spring_object_frame.block<6, 6>(0, 0) +=
  //     M_inv * Adj_t_inv_oTg1 *
  //     (jacob_Adj_t_inv_g1Te1 * temp_jacob3 * temp_jacob5 + Adj_t_inv_g1Te1 * jacob_spring.block<6, 6>(0, 0));

  // // otwisto - 1
  // jacob_spring_object_frame.block<6, 6>(0, 6) +=
  //     M_inv * Adj_t_inv_oTg1 * Adj_t_inv_g1Te1 * jacob_spring.block<6, 6>(0, 6);

  // // b1Te1 - 1
  // jacob_spring_object_frame.block<6, 6>(0, 12) +=
  //     M_inv * Adj_t_inv_oTg1 * (jacob_Adj_t_inv_g1Te1 + Adj_t_inv_g1Te1 * jacob_spring.block<6, 6>(0, 12));

  // // b2Te2 - 1 - is zero

  // // b2Tb1 - 1 - is zero

  // // oTg1 - 1
  // Eigen::Vector<double, 6> tmp_vec_6 = Adj_t_inv_g1Te1 * e1he1_e1;
  // jacobian_transpose_adjoint_invT_wrench_T(s.oTg1.mat, tmp_vec_6, temp_jacob6);

  // jacobian_Ta_Tb_Ta(s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat, temp_jacob3);
  // jacobian_invT_T(s.oTg1.mat, temp_jacob5);
  // jacob_spring_object_frame.block<6, 6>(0, 30) +=
  //     M_inv * (temp_jacob6 + Adj_t_inv_oTg1 * (jacob_Adj_t_inv_g1Te1 * temp_jacob3 * temp_jacob5 +
  //                                              Adj_t_inv_g1Te1 * jacob_spring.block<6, 6>(0, 30)));

  // // oTg2 - 1 - is zero

  // //* adjoint products spring term - 2

  // // bTo - 2
  // jacobian_Ta_Tb_Ta(process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat, temp_jacob3);
  // jacobian_invT_T(s.bTo.mat, temp_jacob5);
  // jacob_spring_object_frame.block<6, 6>(0, 0) +=
  //     M_inv * Adj_t_inv_oTg2 *
  //     (jacob_Adj_t_inv_g2Te2 * temp_jacob3 * temp_jacob5 + Adj_t_inv_g2Te2 * jacob_spring.block<6, 6>(6, 0));

  // // otwisto - 2
  // jacob_spring_object_frame.block<6, 6>(0, 6) +=
  //     M_inv * Adj_t_inv_oTg2 * Adj_t_inv_g2Te2 * jacob_spring.block<6, 6>(6, 6);

  // // b1Te1 - 2 - is zero

  // // b2Te2 - 2
  // jacob_spring_object_frame.block<6, 6>(0, 18) +=
  //     M_inv * Adj_t_inv_oTg2 * (jacob_Adj_t_inv_g2Te2 + Adj_t_inv_g2Te2 * jacob_spring.block<6, 6>(6, 18));

  // // b2Tb1 - 2
  // jacobian_Ta_Tb_Ta(s.b2Te2.mat.inverse(), temp_jacob3);
  // jacobian_invT_T(s.b2Tb1.mat, temp_jacob5);
  // jacob_spring_object_frame.block<6, 6>(0, 24) +=
  //     M_inv * Adj_t_inv_oTg2 *
  //     (jacob_Adj_t_inv_g2Te2 * temp_jacob3 * temp_jacob5 + Adj_t_inv_g2Te2 * jacob_spring.block<6, 6>(6, 24));

  // // oTg1 - 2 - is zero

  // // oTg2 - 2
  // tmp_vec_6 = Adj_t_inv_g2Te2 * e2he2_e2;
  // jacobian_transpose_adjoint_invT_wrench_T(s.oTg2.mat, tmp_vec_6, temp_jacob6);

  // jacobian_Ta_Tb_Ta(s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat,
  // temp_jacob3); jacobian_invT_T(s.oTg2.mat, temp_jacob5); jacob_spring_object_frame.block<6, 6>(0, 36) +=
  //     M_inv * (temp_jacob6 + Adj_t_inv_oTg2 * (jacob_Adj_t_inv_g2Te2 * temp_jacob3 * temp_jacob5 +
  //                                              Adj_t_inv_g2Te2 * jacob_spring.block<6, 6>(6, 36)));

  // oTg2 - 1 - is zero

  // ####################

  //* i = 1
  Eigen::Matrix4d oTe1 = s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  Eigen::Matrix<double, 6, state_dof> jacob_oTe1_x;

  // jacob oTe1
  jacob_oTe1_x.setZero();

  // bTo
  jacobian_Ta_Tb_Ta(process_params.bTb1.mat * s.b1Te1.mat, temp_jacob6);
  jacobian_invT_T(s.bTo.mat, temp_jacob5);
  jacob_oTe1_x.block<6, 6>(0, 0) = temp_jacob6 * temp_jacob5;

  // b1Te1
  jacob_oTe1_x.block<6, 6>(0, 12).setIdentity();

  Eigen::Matrix<double, 6, state_dof> jacobian_spring1_object_frame;
  jacobian_transpose_adjoint_invTx_wrenchx<state_dof>(
      oTe1, e1he1_e1, jacob_oTe1_x, jacob_spring.block<6, state_dof>(0, 0), jacobian_spring1_object_frame);

  //* i = 2
  Eigen::Matrix4d oTe2 = s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;
  Eigen::Matrix<double, 6, state_dof> jacob_oTe2_x;

  // jacob oTe2
  jacob_oTe2_x.setZero();

  // bTo
  jacobian_Ta_Tb_Ta(process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat, temp_jacob6);
  jacobian_invT_T(s.bTo.mat, temp_jacob5);
  jacob_oTe2_x.block<6, 6>(0, 0) = temp_jacob6 * temp_jacob5;

  // b2Te2
  jacob_oTe2_x.block<6, 6>(0, 18).setIdentity();

  // b2Tb1
  jacobian_Ta_Tb_Ta(s.b2Te2.mat, temp_jacob6);
  jacobian_invT_T(s.b2Tb1.mat, temp_jacob5);
  jacob_oTe2_x.block<6, 6>(0, 24) = temp_jacob6 * temp_jacob5;

  Eigen::Matrix<double, 6, state_dof> jacobian_spring2_object_frame;
  jacobian_transpose_adjoint_invTx_wrenchx<state_dof>(
      oTe2, e2he2_e2, jacob_oTe2_x, jacob_spring.block<6, state_dof>(6, 0), jacobian_spring2_object_frame);

  // ##################

  //* fill the matrix
  cov.template block<6, state_dof>(6, 0) +=
      M_inv * (jacobian_spring1_object_frame + jacobian_spring2_object_frame);  // jacob_spring_object_frame;

  //* jacobian oTg frames
  Eigen::Matrix<double, 12, 1> e_twist_g_grasp_control = Eigen::Matrix<double, 12, 1>::Zero();  // elastic
  Eigen::Matrix<double, 12, state_dof> jacob_spring_grasp_control;
  Eigen::Matrix<double, 12, state_dof> jacob_spring_elastic_tau;
  Eigen::Matrix<double, 12, state_dof> jacob_spring_viscous_tau;
  Eigen::Matrix<double, 12, 12> K_grasp;
  K_grasp.setIdentity();
  K_grasp.block<6, 6>(0, 0) = process_params.K_grasp;
  K_grasp.block<6, 6>(6, 6) = process_params.K_grasp;

  e_twist_g_grasp_control = K_grasp * e_tau_g_e + e_twist_g_e;

  jacobian_spring_elastic(s, process_params, jacob_spring_elastic_tau, input_ikfom(), false);
  jacobian_spring_viscous(s, process_params, jacob_spring_viscous_tau, input_ikfom(), false);
  jacob_spring_elastic_tau = -jacob_spring_elastic_tau;  // change sign jacobian due to negative sign in the measures
  jacob_spring_viscous_tau = -jacob_spring_viscous_tau;  // change sign jacobian due to negative sign in the measures
  jacob_spring_grasp_control = K_grasp * jacob_spring_elastic_tau + jacob_spring_viscous_tau;

  Eigen::Matrix<double, 4, 4> g1Te1 =
      s.oTg1.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat;
  Eigen::Matrix<double, 4, 4> g2Te2 =
      s.oTg2.mat.inverse() * s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat;
  Eigen::Matrix<double, 6, 6> Adj_g1Te1 = adjoint_SE3(g1Te1);
  Eigen::Matrix<double, 6, 6> Adj_g2Te2 = adjoint_SE3(g2Te2);

  Eigen::Matrix<double, 6, 6> jacob_Adj_g1Te1 = Eigen::Matrix<double, 6, 6>::Zero();
  Eigen::Matrix<double, 6, 6> jacob_Adj_g2Te2 = Eigen::Matrix<double, 6, 6>::Zero();

  Eigen::Matrix<double, 12, 12> blkdiag_Adj_g1Te1_Adj_g2Te2 = Eigen::Matrix<double, 12, 12>::Zero();
  blkdiag_Adj_g1Te1_Adj_g2Te2.block<6, 6>(0, 0) = Adj_g1Te1;
  blkdiag_Adj_g1Te1_Adj_g2Te2.block<6, 6>(6, 6) = Adj_g2Te2;

  jacobian_adjoint_T_twist_T(g1Te1, e_twist_g_grasp_control.block<6, 1>(0, 0), jacob_Adj_g1Te1);
  jacobian_adjoint_T_twist_T(g2Te2, e_twist_g_grasp_control.block<6, 1>(6, 0), jacob_Adj_g2Te2);

  Eigen::Matrix<double, 12, 12> blkdiag_jacob_Adj_gTe = Eigen::Matrix<double, 12, 12>::Zero();
  blkdiag_jacob_Adj_gTe.block<6, 6>(0, 0) = jacob_Adj_g1Te1;
  blkdiag_jacob_Adj_gTe.block<6, 6>(6, 6) = jacob_Adj_g2Te2;

  //* evaluate jacobian giTei wrt x
  Eigen::Matrix<double, 12, state_dof> jacob_gTe_x;
  jacob_gTe_x.setZero();

  // i=1
  // bTo
  jacobian_Ta_Tb_Ta(process_params.bTb1.mat * s.b1Te1.mat, temp_jacob3);
  jacobian_invT_T(s.bTo.mat, temp_jacob5);
  jacob_gTe_x.block<6, 6>(0, 0) = temp_jacob3 * temp_jacob5;

  // b1Te1
  jacob_gTe_x.block<6, 6>(0, 12) = Eigen::Matrix<double, 6, 6>::Identity();

  // oTg1
  jacobian_Ta_Tb_Ta(s.bTo.mat.inverse() * process_params.bTb1.mat * s.b1Te1.mat, temp_jacob3);
  jacobian_invT_T(s.oTg1.mat, temp_jacob5);
  jacob_gTe_x.block<6, 6>(0, 30) = temp_jacob3 * temp_jacob5;

  // i=2

  // bTo
  jacobian_Ta_Tb_Ta(process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat, temp_jacob3);
  jacobian_invT_T(s.bTo.mat, temp_jacob5);
  jacob_gTe_x.block<6, 6>(6, 0) = temp_jacob3 * temp_jacob5;

  // b2Te2
  jacob_gTe_x.block<6, 6>(6, 18) = Eigen::Matrix<double, 6, 6>::Identity();

  // b2Tb1
  jacobian_Ta_Tb_Ta(s.b2Te2.mat, temp_jacob3);
  jacobian_invT_T(s.b2Tb1.mat, temp_jacob5);
  jacob_gTe_x.block<6, 6>(6, 24) = temp_jacob3 * temp_jacob5;

  // oTg2
  jacobian_Ta_Tb_Ta(s.bTo.mat.inverse() * process_params.bTb1.mat * s.b2Tb1.mat.inverse() * s.b2Te2.mat, temp_jacob3);
  jacobian_invT_T(s.oTg2.mat, temp_jacob5);
  jacob_gTe_x.block<6, 6>(6, 34) = temp_jacob3 * temp_jacob5;

  if (process_params.H_sel_neg.rows() != 0)
  {
    cov.template block<12, state_dof>(30, 0) =
        process_params.H_sel_neg.transpose() * process_params.H_sel_neg *
        (blkdiag_Adj_g1Te1_Adj_g2Te2 * jacob_spring_grasp_control + blkdiag_jacob_Adj_gTe * jacob_gTe_x);
  }
  // reaction wrench
  // cov.template block<6, 6>(6, 42) += M_inv;

  // std::cout << "JACOBIAN df_dx: " << std::endl;
  // std::cout << "cov: " << cov << std::endl;

  return cov;
}

Eigen::Matrix<double, state_dof, process_noise_dof> df_dw(state_ikfom& s, const input_ikfom& in,
                                                          ProcessParams& process_params)
{
  Eigen::Matrix<double, state_dof, process_noise_dof> cov =
      Eigen::Matrix<double, state_dof, process_noise_dof>::Identity();
  return cov;
}

template <int num_aruco>
MTK::MyManifold<double, num_aruco> get_h(state_ikfom& s, ProcessParams& process_params)
{
  MTK::MyManifold<double, num_aruco> h_(wrench_measure(), wrench_measure(), SEN(), SEN(),
                                        std::vector<SEN>(num_aruco, SEN()));

  Eigen::Matrix<double, 3, 1> b1po;
  Eigen::Matrix<double, 3, 1> b2po;

  SEN b1To(process_params.bTb1.mat.inverse() * s.bTo.mat);  // b1To = bTb1^-1 * bTo
  SEN b2To(s.b2Tb1.mat * process_params.bTb1.mat.inverse() * s.bTo.mat);

  for (int i = 0; i < process_params.num_pose_measure_robot1; i++)
  {
    h_.object_pose[i] = b1To;
  }
  for (int i = process_params.num_pose_measure_robot1;
       i < process_params.num_pose_measure_robot1 + process_params.num_pose_measure_robot2; i++)
  {
    h_.object_pose[i] = b2To;
  }

  // output of the wrenches exerted by the robots on the object
  Eigen::Matrix<double, 12, 1> ehe;  // wrench in the end effector frame applied by the object on the robots
  spring_model(s, process_params, ehe);
  // ehe = process_params.H_sel.transpose() * process_params.H_sel * ehe;
  h_.e1he1_e1.force = ehe.template block<3, 1>(0, 0);
  h_.e1he1_e1.torque = ehe.template block<3, 1>(3, 0);
  h_.e2he2_e2.force = ehe.template block<3, 1>(6, 0);
  h_.e2he2_e2.torque = ehe.template block<3, 1>(9, 0);

  // output of the fkine of the robots
  h_.b1_pose_e1 = s.b1Te1;  // position of robot1 end effector in the base frame
  h_.b2_pose_e2 = s.b2Te2;  // orientation of robot2 end effector in the base frame

  // std::cout << "h_.object_pose.size(): " << h_.object_pose.size() << std::endl;
  // for (const auto& pose : h_.object_pose)
  // {
  //   std::cout << "pose.position: " << pose.position.transpose() << std::endl;
  //   std::cout << "pose.orientation: " << pose.quaternion.coeffs().transpose() << std::endl;
  // }

  return h_;
}

template <int num_aruco>
Eigen::Matrix<double, measurement_dof(num_aruco), state_dof> dh_dx(state_ikfom& s, bool& valid,
                                                                   ProcessParams& process_params)
{
  //! measure order [h1,h2,b1Te1,b2Te2,object_pose1,object_pose2,...,object_poseN]
  Eigen::Matrix<double, measurement_dof(num_aruco), state_dof> cov =
      Eigen::Matrix<double, measurement_dof(num_aruco), state_dof>::Zero();

  // //! output spring model jacobians
  Eigen::Matrix<double, 12, state_dof> jacob_spring_elastic;
  jacobian_spring_elastic(s, process_params, jacob_spring_elastic);
  cov.template block<12, state_dof>(0, 0) =
      jacob_spring_elastic;  // process_params.H_sel.transpose() * process_params.H_sel

  //* output jacobians fkine robots
  cov.template block<12, 12>(12, 12) = Eigen::Matrix<double, 12, 12>::Identity();

  //* jacobian output 1
  if (process_params.num_pose_measure_robot1 > 0)
  {
    for (int i = 0; i < process_params.num_pose_measure_robot1; i++)
    {
      cov.template block<6, 6>(24 + 6 * i, 0) = Eigen::Matrix<double, 6, 6>::Identity();
    }
  }

  //* jacobian output 2
  if (process_params.num_pose_measure_robot2 > 0)
  {
    Eigen::Matrix<double, 6, 6> jacob_tmp3;
    jacobian_Ta_Tb_Ta(process_params.bTb1.mat.inverse() * s.bTo.mat, jacob_tmp3);

    for (int i = process_params.num_pose_measure_robot1;
         i < process_params.num_pose_measure_robot1 + process_params.num_pose_measure_robot2; i++)
    {
      cov.template block<6, 6>(24 + 6 * i, 0) = Eigen::Matrix<double, 6, 6>::Identity();
      cov.template block<6, 6>(24 + 6 * i, 24) = jacob_tmp3;
    }
  }

  return cov;
}

template <int num_aruco>
Eigen::Matrix<double, measurement_dof(num_aruco), measurement_dof(num_aruco)> dh_dv(state_ikfom& s, bool& valid,
                                                                                    ProcessParams& process_params)
{
  Eigen::Matrix<double, measurement_dof(num_aruco), measurement_dof(num_aruco)> cov =
      Eigen::Matrix<double, measurement_dof(num_aruco), measurement_dof(num_aruco)>::Identity();
  return cov;
}

template <int num_aruco>
Eigen::Matrix<double, measurement_dof(num_aruco), measurement_dof(num_aruco)>
compute_V_noise(ProcessParams& process_params)
{
  Eigen::Matrix<double, measurement_dof(num_aruco), measurement_dof(num_aruco)> cov =
      Eigen::Matrix<double, measurement_dof(num_aruco), measurement_dof(num_aruco)>::Identity();

  cov.template block<6, 6>(0, 0) = process_params.V_wrench_measure_rb1;
  cov.template block<6, 6>(6, 6) = process_params.V_wrench_measure_rb2;
  cov.template block<6, 6>(12, 12) = process_params.V_fkine_measure_rb1;
  cov.template block<6, 6>(18, 18) = process_params.V_fkine_measure_rb2;
  for (int i = 0; i < process_params.num_pose_measure_robot1; i++)
  {
    cov.template block<6, 6>(24 + 6 * i, 24 + 6 * i) = process_params.V_pose_measure_rb1;
  }
  for (int i = process_params.num_pose_measure_robot1;
       i < process_params.num_pose_measure_robot1 + process_params.num_pose_measure_robot2; i++)
  {
    cov.template block<6, 6>(24 + 6 * i, 24 + 6 * i) = process_params.V_pose_measure_rb2;
  }
  return cov;
}

template <int num_aruco>
MTK::MyManifold<double, num_aruco>
h_dyn_runtime_share(state_ikfom& s, esekfom::dyn_runtime_share_datastruct<double>& dyn_runtime_share_data,
                    ProcessParams& process_params)
{
  if (process_params.num_pose_measure_robot1 + process_params.num_pose_measure_robot2 != num_aruco)
  {
    throw std::runtime_error("Error: num_pose_measure_robot1 + num_pose_measure_robot2 != num_aruco");
  }

  if (dyn_runtime_share_data.converge)
  {
  }  // this value is true means iteration is converged

  dyn_runtime_share_data.h_x = dh_dx<num_aruco>(
      s, dyn_runtime_share_data.valid, process_params);  // H_x is the result matrix of the first differentiation
  dyn_runtime_share_data.h_v = dh_dv<num_aruco>(
      s, dyn_runtime_share_data.valid, process_params);  // H_v is the result matrix of the second differentiation
  dyn_runtime_share_data.R = compute_V_noise<num_aruco>(process_params);  // R is the
                                                                          // measurement noise
                                                                          // covariance

  return get_h<num_aruco>(s, process_params);
}

#endif