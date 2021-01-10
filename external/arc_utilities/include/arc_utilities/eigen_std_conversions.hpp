#ifndef LGV_EIGEN_STD_CONVERSIONS
#define LGV_EIGEN_STD_CONVERSIONS

#include <Eigen/Dense>
#include <cstdlib>
#include <vector>

template <typename Output>
Output ConvertTo(Eigen::VectorXd const &vec);

template <>
std::vector<double> ConvertTo<std::vector<double>>(Eigen::VectorXd const &vec) {
  return std::vector<double>(vec.data(), vec.data() + static_cast<unsigned long>(vec.size()) * sizeof(double));
}

#endif  // LGV_EIGEN_STD_CONVERSIONS
