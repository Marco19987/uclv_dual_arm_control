#ifndef SO3_UTILS_HPP
#define SO3_UTILS_HPP

#include <Eigen/Dense>
#include "utils.hpp"


inline void jacobian_Rv_R(const Eigen::Ref<const Eigen::Matrix<double, 3, 3>>& R,const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>& v,
                          Eigen::Matrix<double, 3, 3>& jacob)
{
  Eigen::Matrix<double, 3, 3> skew_mat;
  skew_matrix(v, skew_mat);
  jacob = -R * skew_mat;
}

inline void jacobian_Rv_v(const Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 3>& jacob)
{
  jacob = R;
}

inline void jacobian_QR_R(Eigen::Matrix<double, 3, 3>& jacob)
{
  jacob = Eigen::Matrix<double, 3, 3>::Identity();
}
inline void jacobian_QR_Q(const Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 3>& jacob)
{
  jacob = R.transpose();
}
inline void jacobian_invR_R(const Eigen::Matrix<double, 3, 3>& R, Eigen::Matrix<double, 3, 3>& jacob)
{
  jacob = -R;
}

#endif  // SO3_UTILS_HPP
