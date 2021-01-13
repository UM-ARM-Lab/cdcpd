#ifndef ARC_UTILITIES_MATH_HELPERS_HPP
#define ARC_UTILITIES_MATH_HELPERS_HPP

#include <cassert>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <vector>

namespace EigenHelpers  // TODO: Change namespace to ArcMath, breaking change
{
////////////////////////////////////////////////////////////////////////////
// Misc
////////////////////////////////////////////////////////////////////////////

inline bool CloseEnough(const double p1, const double p2, const double threshold) {
  const double real_threshold = std::abs(threshold);
  const double abs_delta = std::abs(p2 - p1);
  return abs_delta <= real_threshold;
}

inline bool CloseEnough(const std::vector<double>& v1, const std::vector<double>& v2, const double threshold) {
  if (v1.size() != v2.size()) {
    return false;
  }
  for (size_t i = 0; i < v1.size(); i++) {
    if (!CloseEnough(v1[i], v2[i], threshold)) {
      return false;
    }
  }
  return true;
}

// Inspired by Eigen's "isApprox" function
inline bool IsApprox(const double p1, const double p2, const double precision) {
  const double smallest_element = std::min(std::abs(p1), std::abs(p2));
  const double threshold = precision * smallest_element;
  return CloseEnough(p1, p2, threshold);
}

// https://stackoverflow.com/questions/1903954/is-there-a-standard-sign-function-signum-sgn-in-c-c
// Returns -1, 0, or 1 depending on the sign of val
template <typename T>
inline int Sign(const T val) {
  return (T(0) < val) - (val < T(0));
}

////////////////////////////////////////////////////////////////////////////
// Interpolation functions
////////////////////////////////////////////////////////////////////////////

inline double EnforceContinuousRevoluteBounds(const double value) {
  if ((value <= -M_PI) || (value > M_PI)) {
    const double remainder = fmod(value, 2.0 * M_PI);
    if (remainder <= -M_PI) {
      return (remainder + (2.0 * M_PI));
    } else if (remainder > M_PI) {
      return (remainder - (2.0 * M_PI));
    } else {
      return remainder;
    }
  } else {
    return value;
  }
}

// TODO: this is almost the same as arc_helpers.hpp:ClampValueAndWarn(...)
//       we should resolve this redundency, pick one place for this function
template <typename FloatType>
inline FloatType SafetyCheckUnitInterval(const FloatType ratio) {
  static_assert(std::is_floating_point<FloatType>::value, "Type must be a float type");
  FloatType real_ratio = ratio;
  if (real_ratio < 0.0) {
    real_ratio = 0.0;
    std::cerr << "Interpolation ratio < 0.0, set to 0.0 " << std::endl;
  } else if (real_ratio > 1.0) {
    real_ratio = 1.0;
    std::cerr << "Interpolation ratio > 1.0, set to 1.0" << std::endl;
  }
  return real_ratio;
}

inline double Interpolate(const double p1, const double p2, const double ratio) {
  // Safety check ratio
  const double real_ratio = SafetyCheckUnitInterval(ratio);
  // Interpolate
  // This is the numerically stable version, rather than  (p1 + (p2 - p1) * real_ratio)
  return ((p1 * (1.0 - real_ratio)) + (p2 * real_ratio));
}

inline double InterpolateContinuousRevolute(const double p1, const double p2, const double ratio) {
  // Safety check ratio
  const double real_ratio = SafetyCheckUnitInterval(ratio);
  // Safety check args
  const double real_p1 = EnforceContinuousRevoluteBounds(p1);
  const double real_p2 = EnforceContinuousRevoluteBounds(p2);
  // Interpolate
  double interpolated = 0.0;
  double diff = real_p2 - real_p1;
  if (std::abs(diff) <= M_PI) {
    interpolated = real_p1 + diff * real_ratio;
  } else {
    if (diff > 0.0) {
      diff = 2.0 * M_PI - diff;
    } else {
      diff = -2.0 * M_PI - diff;
    }
    interpolated = real_p1 - diff * real_ratio;
    // Input states are within bounds, so the following check is sufficient
    if (interpolated > M_PI) {
      interpolated -= 2.0 * M_PI;
    } else {
      if (interpolated < -M_PI) {
        interpolated += 2.0 * M_PI;
      }
    }
  }
  return interpolated;
}

template <typename T1, typename T2>
inline std::pair<T1, T2> Interpolate(const std::pair<T1, T2>& p1, const std::pair<T1, T2>& p2, const double ratio) {
  // Safety check ratio
  const double real_ratio = SafetyCheckUnitInterval(ratio);
  // Interpolate
  // This is the numerically stable version, rather than  (p1 + (p2 - p1) * real_ratio)
  return std::make_pair<T1, T2>((p1.first * (1.0 - real_ratio)) + (p2.first * real_ratio),
                                (p1.second * (1.0 - real_ratio)) + (p2.second * real_ratio));
}

template <typename T>
inline std::vector<T> Interpolate(const std::vector<T>& v1, const std::vector<T>& v2, const double ratio) {
  // Safety check ratio
  const double real_ratio = SafetyCheckUnitInterval(ratio);
  // Safety check inputs
  const size_t len = v1.size();
  if (len != v2.size()) {
    std::cerr << "Vectors to interpolate are different sizes (" << v1.size() << " versus " << v2.size() << ")"
              << std::endl;
    return std::vector<T>();
  }
  // Interpolate
  // This is the numerically stable version, rather than  (p1 + (p2 - p1) * real_ratio)
  std::vector<T> interped(len, 0);
  for (size_t idx = 0; idx < len; idx++) {
    interped[idx] = ((v1[idx] * (1.0 - real_ratio)) + (v2[idx] * real_ratio));
  }
  return interped;
}

////////////////////////////////////////////////////////////////////////////
// Distance functions
////////////////////////////////////////////////////////////////////////////

inline double SquaredDistance(const std::vector<double>& p1, const std::vector<double>& p2) {
  if (p1.size() == p2.size()) {
    double distance = 0.0;
    for (size_t idx = 0; idx < p1.size(); idx++) {
      distance += (p2[idx] - p1[idx]) * (p2[idx] - p1[idx]);
    }
    return distance;
  } else {
    return INFINITY;
  }
}

inline double Distance(const std::vector<double>& p1, const std::vector<double>& p2) {
  if (p1.size() == p2.size()) {
    return sqrt(SquaredDistance(p1, p2));
  } else {
    return INFINITY;
  }
}

inline double ContinuousRevoluteSignedDistance(const double p1, const double p2) {
  // Safety check args
  const double real_p1 = EnforceContinuousRevoluteBounds(p1);
  const double real_p2 = EnforceContinuousRevoluteBounds(p2);
  const double raw_distance = real_p2 - real_p1;
  if ((raw_distance <= -M_PI) || (raw_distance > M_PI)) {
    if (raw_distance <= -M_PI) {
      return (-(2.0 * M_PI) - raw_distance);
    } else if (raw_distance > M_PI) {
      return ((2.0 * M_PI) - raw_distance);
    } else {
      return raw_distance;
    }
  } else {
    return raw_distance;
  }
}

inline double ContinuousRevoluteDistance(const double p1, const double p2) {
  return std::abs(ContinuousRevoluteSignedDistance(p1, p2));
}

inline double AddContinuousRevoluteValues(const double start, const double change) {
  return EnforceContinuousRevoluteBounds(start + change);
}

inline double GetContinuousRevoluteRange(const double start, const double end) {
  const double raw_range = ContinuousRevoluteSignedDistance(start, end);
  if (raw_range >= 0.0) {
    return raw_range;
  } else {
    return (2.0 * M_PI) + raw_range;
  }
}

inline bool CheckInContinuousRevoluteRange(const double start, const double range, const double val) {
  const double real_val = EnforceContinuousRevoluteBounds(val);
  const double real_start = EnforceContinuousRevoluteBounds(start);
  const double delta = ContinuousRevoluteSignedDistance(real_start, real_val);
  if (delta >= 0.0) {
    if (delta <= range) {
      return true;
    } else {
      return false;
    }
  } else {
    const double real_delta = (2.0 * M_PI) + delta;
    if (real_delta <= range) {
      return true;
    } else {
      return false;
    }
  }
}

inline bool CheckInContinuousRevoluteBounds(const double start, const double end, const double val) {
  const double range = GetContinuousRevoluteRange(start, end);
  return CheckInContinuousRevoluteRange(start, range, val);
}

// DistanceFn must match the following interface: std::function<double(const T&, const T&)>
template <typename T, class DistanceFn, typename Alloc = std::allocator<T>>
inline double CalculateTotalDistance(const std::vector<T, Alloc>& path, const DistanceFn& distance_fn) {
  double total_dist = 0;
  for (size_t path_idx = 0; path_idx + 1 < path.size(); path_idx++) {
    const double delta = distance_fn(path[path_idx], path[path_idx + 1]);
    total_dist += delta;
  }
  return total_dist;
}

////////////////////////////////////////////////////////////////////////////
// Averaging functions
// Numerically more stable averages taken from http://people.ds.cam.ac.uk/fanf2/hermes/doc/antiforgery/stats.pdf
////////////////////////////////////////////////////////////////////////////

inline double AverageStdVectorDouble(const std::vector<double>& values,
                                     const std::vector<double>& weights = std::vector<double>()) {
  // Get the weights
  assert(values.size() > 0);
  assert((weights.size() == values.size()) || (weights.size() == 0));
  const bool use_weights = (weights.size() != 0);
  // Find the first element with non-zero weight
  size_t starting_idx = 0;
  while (starting_idx < weights.size() && weights[starting_idx] == 0.0) {
    starting_idx++;
  }
  // If all weights are zero, result is undefined
  assert(starting_idx < values.size());
  // Start the recursive definition with the base case
  double average = values[starting_idx];
  const double starting_weight = use_weights ? std::abs(weights[starting_idx]) : 1.0;
  assert(starting_weight > 0.0);
  double weights_running_sum = starting_weight;
  // Do the weighted averaging on the rest of the vectors
  for (size_t idx = starting_idx + 1; idx < values.size(); idx++) {
    const double weight = use_weights ? std::abs(weights[idx]) : 1.0;
    weights_running_sum += weight;
    const double effective_weight = weight / weights_running_sum;
    const double prev_average = average;
    const double current = values[idx];
    average = prev_average + (effective_weight * (current - prev_average));
  }
  return average;
}

inline double ComputeStdDevStdVectorDouble(const std::vector<double>& values, const double mean) {
  assert(values.size() > 0);
  if (values.size() == 1) {
    return 0.0;
  } else {
    const double inv_n_minus_1 = 1.0 / (double)(values.size() - 1);
    double stddev_sum = 0.0;
    for (size_t idx = 0; idx < values.size(); idx++) {
      const double delta = values[idx] - mean;
      stddev_sum += (delta * delta);
    }
    return std::sqrt(stddev_sum * inv_n_minus_1);
  }
}

inline double ComputeStdDevStdVectorDouble(const std::vector<double>& values) {
  const double mean = AverageStdVectorDouble(values);
  return ComputeStdDevStdVectorDouble(values, mean);
}

/*
 * This function does not actually deal with the continuous revolute space correctly,
 * it just assumes a normal real Euclidean space
 */
inline double AverageContinuousRevolute(const std::vector<double>& angles,
                                        const std::vector<double>& weights = std::vector<double>()) {
  return AverageStdVectorDouble(angles, weights);
}
}  // namespace EigenHelpers

#endif  // ARC_UTILITIES_MATH_HELPERS_HPP
