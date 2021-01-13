#ifndef SERIALIZATION_EIGEN_HPP
#define SERIALIZATION_EIGEN_HPP

#include "arc_utilities/eigen_typedefs.hpp"
#include "arc_utilities/serialization.hpp"

namespace arc_utilities {
////////////////////////////////////////////////////////////////////////////
// Templated serialization/deserialization functions - Base Vector/Matrices only
////////////////////////////////////////////////////////////////////////////

template <typename _Scalar, int _Rows, int _Cols>
inline uint64_t SerializedSizeEigen(const Eigen::Matrix<_Scalar, _Rows, _Cols>& value) {
  uint64_t size = 0;

  // Only dynamicly sized matrices have a row header
  if (_Rows == Eigen::Dynamic) {
    size += sizeof(uint64_t);
  }

  // Only dynamicly sized matrices have a col header
  if (_Cols == Eigen::Dynamic) {
    size += sizeof(uint64_t);
  }

  size += (uint64_t)value.size() * sizeof(_Scalar);
  return size;
}

// Takes a state to serialize and a buffer to serialize into
// Return number of bytes written to buffer
template <typename _Scalar, int _Rows, int _Cols>
inline uint64_t SerializeEigen(const Eigen::Matrix<_Scalar, _Rows, _Cols>& value, std::vector<uint8_t>& buffer) {
  const uint64_t serialized_size = SerializedSizeEigen(value);
  std::vector<uint8_t> temp_buffer(serialized_size, 0x00);
  uint64_t current = 0;
  // Make the header if needed
  if (_Rows == Eigen::Dynamic) {
    const uint64_t rows_header = (uint64_t)value.rows();
    memcpy(temp_buffer.data() + current, &rows_header, sizeof(rows_header));
    current += sizeof(rows_header);
  }
  if (_Cols == Eigen::Dynamic) {
    const uint64_t cols_header = (uint64_t)value.cols();
    memcpy(temp_buffer.data() + current, &cols_header, sizeof(cols_header));
    current += sizeof(cols_header);
  }
  // Copy the data
  memcpy(temp_buffer.data() + current, value.data(), serialized_size - current);
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return serialized_size;
}

// Takes a state to serialize and a buffer to serialize into
// Return number of bytes written to buffer
// Used to specialize specific Eigen types (such as Isometry3d)
template <typename EigenType>
inline uint64_t SerializeEigen(const EigenType& value, std::vector<uint8_t>& buffer);

// Takes a a buffer to read from, and the next value to read
// Return a object of the given type, and the number of bytes read
template <typename EigenType>
inline std::pair<EigenType, uint64_t> DeserializeEigen(const std::vector<uint8_t>& buffer, const uint64_t current);

////////////////////////////////////////////////////////////////////////////
// Concrete implementations for specific Eigen types
////////////////////////////////////////////////////////////////////////////

// Vector3d
template <>
inline std::pair<Eigen::Vector3d, uint64_t> DeserializeEigen<Eigen::Vector3d>(const std::vector<uint8_t>& buffer,
                                                                              const uint64_t current) {
  Eigen::Vector3d temp_value;
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert(current < buffer.size());
  assert(current + serialized_size <= buffer.size());
  memcpy(temp_value.data(), &buffer[current], serialized_size);
  return std::make_pair(temp_value, serialized_size);
}

// Vector6d
template <>
inline std::pair<Eigen::Matrix<double, 6, 1>, uint64_t> DeserializeEigen<Eigen::Matrix<double, 6, 1>>(
    const std::vector<uint8_t>& buffer, const uint64_t current) {
  Eigen::Matrix<double, 6, 1> temp_value;
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert(current < buffer.size());
  assert(current + serialized_size <= buffer.size());
  memcpy(temp_value.data(), &buffer[current], serialized_size);
  return std::make_pair(temp_value, serialized_size);
}

// Vector7d
template <>
inline std::pair<Eigen::Matrix<double, 7, 1>, uint64_t> DeserializeEigen<Eigen::Matrix<double, 7, 1>>(
    const std::vector<uint8_t>& buffer, const uint64_t current) {
  Eigen::Matrix<double, 7, 1> temp_value;
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert(current < buffer.size());
  assert(current + serialized_size <= buffer.size());
  memcpy(temp_value.data(), &buffer[current], serialized_size);
  return std::make_pair(temp_value, serialized_size);
}

// Generic VectorXd
template <>
inline std::pair<Eigen::VectorXd, uint64_t> DeserializeEigen<Eigen::VectorXd>(const std::vector<uint8_t>& buffer,
                                                                              const uint64_t current) {
  assert(current < buffer.size());
  assert((current + sizeof(uint64_t)) <= buffer.size());
  // Load the header
  uint64_t size_header = 0u;
  memcpy(&size_header, &buffer[current], sizeof(uint64_t));
  // Check buffer size
  Eigen::VectorXd temp_value = Eigen::VectorXd::Zero((ssize_t)size_header);
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert((current + serialized_size) <= buffer.size());
  // Load from the buffer
  memcpy(temp_value.data(), &buffer[current + sizeof(size_header)], (serialized_size - sizeof(size_header)));
  return std::make_pair(temp_value, serialized_size);
}

// Generic MatrixXd
template <>
inline std::pair<Eigen::MatrixXd, uint64_t> DeserializeEigen<Eigen::MatrixXd>(const std::vector<uint8_t>& buffer,
                                                                              const uint64_t current) {
  assert(current < buffer.size());
  assert((current + 2 * sizeof(uint64_t)) <= buffer.size());
  // Load the headers
  uint64_t rows_header = 0u;
  memcpy(&rows_header, &buffer[current], sizeof(uint64_t));
  uint64_t cols_header = 0u;
  memcpy(&cols_header, &buffer[current + sizeof(rows_header)], sizeof(uint64_t));
  // Check buffer size
  Eigen::MatrixXd temp_value = Eigen::MatrixXd::Zero((ssize_t)rows_header, (ssize_t)cols_header);
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert((current + serialized_size) <= buffer.size());
  // Load from the buffer
  memcpy(temp_value.data(), &buffer[current + 2 * sizeof(rows_header)], (serialized_size - 2 * sizeof(rows_header)));
  return std::make_pair(temp_value, serialized_size);
}

// Generic Matrix3Xd
template <>
inline std::pair<Eigen::Matrix3Xd, uint64_t> DeserializeEigen<Eigen::Matrix3Xd>(const std::vector<uint8_t>& buffer,
                                                                                const uint64_t current) {
  assert(current < buffer.size());
  assert((current + sizeof(uint64_t)) <= buffer.size());
  // Load the headers
  uint64_t cols_header = 0u;
  memcpy(&cols_header, &buffer[current], sizeof(uint64_t));
  // Check buffer size
  Eigen::Matrix3Xd temp_value = Eigen::Matrix3Xd::Zero(3, (ssize_t)cols_header);
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert((current + serialized_size) <= buffer.size());
  // Load from the buffer
  memcpy(temp_value.data(), &buffer[current + sizeof(cols_header)], (serialized_size - sizeof(cols_header)));
  return std::make_pair(temp_value, serialized_size);
}

// Generic Matrix3Xf
template <>
inline std::pair<Eigen::Matrix3Xf, uint64_t> DeserializeEigen<Eigen::Matrix3Xf>(const std::vector<uint8_t>& buffer,
                                                                                const uint64_t current) {
  assert(current < buffer.size());
  assert((current + sizeof(uint64_t)) <= buffer.size());
  // Load the headers
  uint64_t cols_header = 0u;
  memcpy(&cols_header, &buffer[current], sizeof(uint64_t));
  // Check buffer size
  Eigen::Matrix3Xf temp_value = Eigen::Matrix3Xf::Zero(3, (ssize_t)cols_header);
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert((current + serialized_size) <= buffer.size());
  // Load from the buffer
  memcpy(temp_value.data(), &buffer[current + sizeof(cols_header)], (serialized_size - sizeof(cols_header)));
  return std::make_pair(temp_value, serialized_size);
}

// Generic MatrixXf
template <>
inline std::pair<Eigen::MatrixXf, uint64_t> DeserializeEigen<Eigen::MatrixXf>(const std::vector<uint8_t>& buffer,
                                                                              const uint64_t current) {
  assert(current < buffer.size());
  assert((current + 2 * sizeof(uint64_t)) <= buffer.size());
  // Load the headers
  uint64_t rows_header = 0u;
  memcpy(&rows_header, &buffer[current], sizeof(uint64_t));
  uint64_t cols_header = 0u;
  memcpy(&cols_header, &buffer[current + sizeof(rows_header)], sizeof(uint64_t));
  // Check buffer size
  Eigen::MatrixXf temp_value = Eigen::MatrixXf::Zero((ssize_t)rows_header, (ssize_t)cols_header);
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert((current + serialized_size) <= buffer.size());
  // Load from the buffer
  memcpy(temp_value.data(), &buffer[current + 2 * sizeof(rows_header)], (serialized_size - 2 * sizeof(rows_header)));
  return std::make_pair(temp_value, serialized_size);
}

// Isometry3d
inline uint64_t SerializedSizeEigen(const Eigen::Isometry3d& value) {
  (void)(value);
  return (uint64_t)(16 * sizeof(double));
}

template <>
inline uint64_t SerializeEigen(const Eigen::Isometry3d& value, std::vector<uint8_t>& buffer) {
  const uint64_t serialized_size = SerializedSizeEigen(value);
  std::vector<uint8_t> temp_buffer(serialized_size, 0x00);
  memcpy(&temp_buffer.front(), value.matrix().data(), serialized_size);
  buffer.insert(buffer.end(), temp_buffer.begin(), temp_buffer.end());
  return serialized_size;
}

template <>
inline std::pair<Eigen::Isometry3d, uint64_t> DeserializeEigen<Eigen::Isometry3d>(const std::vector<uint8_t>& buffer,
                                                                                  const uint64_t current) {
  Eigen::Isometry3d temp_value;
  const uint64_t serialized_size = SerializedSizeEigen(temp_value);
  assert(current < buffer.size());
  assert(current + serialized_size <= buffer.size());
  memcpy(temp_value.matrix().data(), &buffer[current], serialized_size);
  return std::make_pair(temp_value, serialized_size);
}
}  // namespace arc_utilities

#endif  // SERIALIZATION_EIGEN_HPP
