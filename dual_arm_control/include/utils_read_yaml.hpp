#include <Eigen/Dense>
#include <vector>
#include <iostream>
#include <yaml-cpp/yaml.h>

void fill_selection_matrix(const std::vector<bool>& H_sel_diag,
                           Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& H)
{
  int H_sel_sum = 0;
  for (const auto& val : H_sel_diag)
  {
    H_sel_sum += val ? 1 : 0;
  }

  H.resize(H_sel_sum, H_sel_diag.size());
  H.setZero();

  int count = 0;
  for (int i = 0; i < H_sel_diag.size(); i++)
  {
    if (H_sel_diag.at(i))
    {
      H(count, i) = 1;
      count++;
    }
  }
}
void fill_selection_matrix_negate(const std::vector<bool>& H_sel_diag,
                                  Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic>& H)
{
  std::vector<bool> H_sel_diag_negate(H_sel_diag.size());
  for (size_t i = 0; i < H_sel_diag.size(); ++i)
  {
    H_sel_diag_negate[i] = !H_sel_diag[i];
  }
  fill_selection_matrix(H_sel_diag_negate, H);
}

void read_transform(const YAML::Node& node, Eigen::Matrix<double, 4, 4>& T,
                    const YAML::Node& config_node = YAML::Node())
{
  Eigen::Matrix<double, 4, 4> bTref;
  bTref.setIdentity();
  if (node["reference_frame"])
  {
    if (!config_node[node["reference_frame"].as<std::string>()])
      throw std::runtime_error("no reference frame");

    read_transform(config_node[node["reference_frame"].as<std::string>()], bTref, config_node);
  }

  T.setIdentity();
  if (node["translation"])
  {
    std::vector<double> translation = node["translation"].as<std::vector<double>>();
    T.block<3, 1>(0, 3) = Eigen::Vector3d(translation[0], translation[1], translation[2]);
  }

  if (node["quaternion"])
  {
    std::vector<double> quaternion = node["quaternion"].as<std::vector<double>>();
    // swap order of quaternion from x y z w to w x y z
    std::vector<double> quaternion_swap = { quaternion[3], quaternion[0], quaternion[1], quaternion[2] };

    Eigen::Quaterniond q(quaternion_swap[0], quaternion_swap[1], quaternion_swap[2], quaternion_swap[3]);
    q.normalize();
    T.block<3, 3>(0, 0) = q.toRotationMatrix();
  }

  if (!node["reference_frame"] && !node["translation"] && !node["quaternion"])
  {
    throw std::runtime_error("no reference frame, translation and quaternion");
  }

  if (node["premultiply"] && node["premultiply"].as<bool>())
  {
    T = T * bTref;
  }
  else
  {
    T = bTref * T;
  }
}

void read_matrix_6x6(const YAML::Node& object, Eigen::Matrix<double, 6, 6>& matrix,
                     const std::string& name = "viscous_friction")
{
  if (object[name])
  {
    std::vector<double> matrix_diag = object[name].as<std::vector<double>>();
    if (matrix_diag.size() == 6)
    {
      matrix.setZero();
      matrix.diagonal() << matrix_diag[0], matrix_diag[1], matrix_diag[2], matrix_diag[3], matrix_diag[4],
          matrix_diag[5];
      std::cout << name << ": \n" << matrix << std::endl;
    }
    else
    {
      std::cerr << name << " vector size is not 6" << std::endl;
    }
  }
  else
  {
    std::cerr << name << " not found in the YAML file" << std::endl;
  }
}
