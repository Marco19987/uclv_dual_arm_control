#ifndef UTILS_HPP
#define UTILS_HPP

#include <Eigen/Dense>

inline void print_transformation_matrix(const std::string &parameter_name,const Eigen::Matrix<double, 4, 4> &T)
{
  std::cout << "Transformation Matrix (" << parameter_name << "):" << std::endl;
  std::cout << T.format(Eigen::IOFormat(4, 0, ", ", "\n", "[", "]")) << std::endl;
}

inline void skew_matrix(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&v, Eigen::Matrix<double, 3, 3> &skew_mat)
{
  skew_mat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
}

inline Eigen::Matrix<double, 3, 3> skew_matrix(const Eigen::Ref<const Eigen::Matrix<double, 3, 1>>&v)
{
  Eigen::Matrix<double, 3, 3> skew_mat;
  skew_mat << 0, -v(2), v(1), v(2), 0, -v(0), -v(1), v(0), 0;
  return skew_mat;
}

inline void get_grasp_matrix(const Eigen::Matrix<double, 4, 4> &T1, const Eigen::Matrix<double, 4, 4> &T2, Eigen::Matrix<double, 6, 12> &W_)
{
  Eigen::Matrix3d skew_opg1, skew_opg2;

  skew_matrix(T1.block<3, 1>(0, 3), skew_opg1);
  skew_matrix(T2.block<3, 1>(0, 3), skew_opg2);

  W_.block<6, 6>(0, 0) << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), skew_opg1, Eigen::Matrix3d::Identity();
  W_.block<6, 6>(0, 6) << Eigen::Matrix3d::Identity(), Eigen::Matrix3d::Zero(), skew_opg2, Eigen::Matrix3d::Identity();
}

inline Eigen::Matrix<double, 6, 6> get_Rbar(const Eigen::Matrix<double, 3, 3> &R)
{
  Eigen::Matrix<double, 6, 6> Rbar = Eigen::Matrix<double, 6, 6>::Zero();
  Rbar.block<3, 3>(0, 0) = R;
  Rbar.block<3, 3>(3, 3) = R;
  return Rbar;
}
inline void flip_vector_components(Eigen::Matrix<double, 6, 1> &vec)
{
  Eigen::Matrix<double, 6, 1> vec_tmp;
  vec_tmp.block<3, 1>(0, 0) = vec.block<3, 1>(3, 0);
  vec_tmp.block<3, 1>(3, 0) = vec.block<3, 1>(0, 0);
  vec = vec_tmp;
}

Eigen::Matrix<double, 3, 1> angle_axis_error(Eigen::Matrix<double, 3, 3> R)
{
  // error = r*sin(theta); r = 0.5 * (1/sin(theta)) * (axis_versor)
  Eigen::AngleAxis<double> angle_axis(R);
  return angle_axis.axis() * angle_axis.angle();
}


inline void jacobian_skew_w_v_w(const Eigen::Matrix<double, 3, 1>& v, Eigen::Matrix<double, 3, 3>& jacob)
{
  // jacobian of skew(w) * v with respect to w
  Eigen::Matrix<double, 3, 3> skew_mat;
  skew_matrix(-v, skew_mat);
  jacob = skew_mat;
}

inline void jacobian_skew_w_Aw_w(const Eigen::Matrix<double, 3, 1>& w, const Eigen::Matrix<double, 3, 3>& A,
                               Eigen::Matrix<double, 3, 3>& jacob)
{
  // jacobian of skew(w) * A * w with respect to w. A is diagonal 3x3 matrix.
  double ax = A(2,2) - A(1,1);
  double ay = A(0,0) - A(2,2);
  double az = A(1,1) - A(0,0);

  jacob << 0, w(2)*ax, w(1)*ax, w(2)*ay, 0, w(0)*ay, w(1)*az, w(0)*az, 0;

}
#endif // UTILS_HPP
