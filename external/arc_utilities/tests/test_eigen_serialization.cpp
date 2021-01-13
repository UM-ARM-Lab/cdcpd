#include <gtest/gtest.h>

#include "arc_utilities/serialization_eigen.hpp"

template <typename Scalar, int _Rows>
void testFloatVectors(const size_t num_tests, const ssize_t rows = -1) {
  typedef Eigen::Matrix<Scalar, _Rows, 1> EigenType;

  for (size_t idx = 0; idx < num_tests; ++idx) {
    EigenType vec;
    if (_Rows == Eigen::Dynamic) {
      ASSERT_GT(rows, 0) << "Dynamically sized vector must have positive number of rows";
      vec.resize(rows);
    }

    vec.setRandom();

    // First, serialize
    std::vector<uint8_t> buffer;
    const size_t bytes_used = arc_utilities::SerializeEigen(vec, buffer);

    // Then deserialze and compare
    const auto deserialized = arc_utilities::DeserializeEigen<EigenType>(buffer, 0);
    EXPECT_EQ(deserialized.second, bytes_used) << "Deserialized bytes does not match original bytes";
    EXPECT_EQ(deserialized.first, vec) << "Deserialized value does not match original";
  }
}

template <typename Scalar, int _Rows, int _Cols>
void testFloatMatrices(const size_t num_tests, const ssize_t rows = -1, const ssize_t cols = -1) {
  typedef Eigen::Matrix<Scalar, _Rows, _Cols> EigenType;

  for (size_t idx = 0; idx < num_tests; ++idx) {
    EigenType matrix;
    if (_Rows == Eigen::Dynamic || _Cols == Eigen::Dynamic) {
      ASSERT_GT(rows, 0) << "Dynamically sized matrix must have positive number of rows";
      matrix.resize(rows, cols);
    }

    matrix.setRandom();

    // First, serialize
    std::vector<uint8_t> buffer;
    const size_t bytes_used = arc_utilities::SerializeEigen(matrix, buffer);

    // Then deserialze and compare
    const auto deserialized = arc_utilities::DeserializeEigen<EigenType>(buffer, 0);
    EXPECT_EQ(deserialized.second, bytes_used) << "Deserialized bytes does not match original bytes";
    EXPECT_EQ(deserialized.first, matrix) << "Deserialized value does not match original";
  }
}

TEST(EigenSerialization, Vectors_have_same_value_after_serialize_and_deserialize) {
  testFloatVectors<double, 3>(10);
  testFloatVectors<double, 6>(10);
  testFloatVectors<double, 7>(10);
  testFloatVectors<double, Eigen::Dynamic>(10, 20);
}

TEST(EigenSerialization, Matrices_have_same_value_after_serialize_and_deserialize) {
  testFloatMatrices<double, Eigen::Dynamic, Eigen::Dynamic>(10, 40, 50);
  testFloatMatrices<double, 3, Eigen::Dynamic>(10, 3, 50);
  testFloatMatrices<float, 3, Eigen::Dynamic>(10, 3, 50);
}

GTEST_API_ int main(int argc, char **argv) {
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
