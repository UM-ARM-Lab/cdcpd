#ifndef ARC_UTILITIES_VECTOR_MATH_HPP
#define ARC_UTILITIES_VECTOR_MATH_HPP

<<<<<<< HEAD
#include <stddef.h>

#include <cmath>
#include <vector>

namespace EigenHelpers  // TODO: Change namespace to ArcMath, breaking change
{
////////////////////////////////////////////////////////////////////////////
// Math functions on std::pair types
////////////////////////////////////////////////////////////////////////////

template <typename First, typename Second>
inline std::pair<First, Second> Abs(const std::pair<First, Second>& pair) {
  return {std::abs(pair.first), std::abs(pair.second)};
}

template <typename First, typename Second, typename Scalar>
inline std::pair<First, Second> Multiply(const std::pair<First, Second>& pair, const Scalar scalar) {
  return {pair.first * scalar, pair.second * scalar};
}

template <typename First, typename Second>
inline std::pair<First, Second> Multiply(const std::pair<First, Second>& pair1, const std::pair<First, Second>& pair2) {
  return {pair1.first * pair2.first, pair1.second * pair2.second};
}

template <typename First, typename Second, typename Scalar>
inline std::pair<First, Second> Divide(const std::pair<First, Second>& pair, const Scalar scalar) {
  return {pair.first / scalar, pair.second / scalar};
}

template <typename First, typename Second>
inline std::pair<First, Second> Divide(const std::pair<First, Second>& pair1, const std::pair<First, Second>& pair2) {
  return {pair1.first / pair2.first, pair1.second / pair2.second};
}

template <typename First, typename Second>
inline std::pair<First, Second> Add(const std::pair<First, Second>& pair1, const std::pair<First, Second>& pair2) {
  return {pair1.first + pair2.first, pair1.second + pair2.second};
}

template <typename First, typename Second>
inline std::pair<First, Second> Sub(const std::pair<First, Second>& pair1, const std::pair<First, Second>& pair2) {
  return {pair1.first - pair2.first, pair1.second - pair2.second};
}

////////////////////////////////////////////////////////////////////////////
// Math functions on std::vector types
////////////////////////////////////////////////////////////////////////////

template <typename T>
inline std::vector<T> Abs(const std::vector<T>& vec) {
  std::vector<T> absed(vec.size(), 0.0);
  for (size_t idx = 0; idx < absed.size(); idx++) {
    absed[idx] = std::abs(vec[idx]);
  }
  return absed;
}

template <typename T1, typename T2>
inline std::vector<T1> Multiply(const std::vector<T1>& vec, const T2 scalar) {
  std::vector<T1> multiplied(vec.size(), 0.0);
  for (size_t idx = 0; idx < multiplied.size(); idx++) {
    const double element = vec[idx];
    multiplied[idx] = element * scalar;
  }
  return multiplied;
}

template <typename T>
inline std::vector<T> Multiply(const std::vector<T>& vec1, const std::vector<T>& vec2) {
  if (vec1.size() == vec2.size()) {
    std::vector<T> multiplied(vec1.size(), 0.0);
    for (size_t idx = 0; idx < multiplied.size(); idx++) {
      const double element1 = vec1[idx];
      const double element2 = vec2[idx];
      multiplied[idx] = element1 * element2;
    }
    return multiplied;
  } else {
    return std::vector<T>();
  }
}

template <typename T1, typename T2>
inline std::vector<T1> Divide(const std::vector<T1>& vec, const T2 scalar) {
  const double inv_scalar = 1.0 / scalar;
  return Multiply(vec, inv_scalar);
}

template <typename T>
inline std::vector<T> Divide(const std::vector<T>& vec1, const std::vector<T>& vec2) {
  if (vec1.size() == vec2.size()) {
    std::vector<T> divided(vec1.size(), 0.0);
    for (size_t idx = 0; idx < divided.size(); idx++) {
      const T element1 = vec1[idx];
      const T element2 = vec2[idx];
      divided[idx] = element1 / element2;
    }
    return divided;
  } else {
    return std::vector<T>();
  }
}

template <typename T1, typename T2>
inline std::vector<T1> Add(const std::vector<T1>& vec, const T2 scalar) {
  std::vector<T1> added(vec.size(), 0.0);
  for (size_t idx = 0; idx < added.size(); idx++) {
    added[idx] = vec[idx] + scalar;
  }
  return added;
}

template <typename T>
inline std::vector<double> Add(const std::vector<T>& vec1, const std::vector<T>& vec2) {
  if (vec1.size() == vec2.size()) {
    std::vector<T> added(vec1.size(), 0.0);
    for (size_t idx = 0; idx < added.size(); idx++) {
      const T element1 = vec1[idx];
      const T element2 = vec2[idx];
      added[idx] = element1 + element2;
    }
    return added;
  } else {
    return std::vector<T>();
  }
}

template <typename T1, typename T2>
inline std::vector<T1> Sub(const std::vector<T1>& vec, const T2 scalar) {
  std::vector<T1> subed(vec.size(), 0.0);
  for (size_t idx = 0; idx < subed.size(); idx++) {
    subed[idx] = vec[idx] - scalar;
  }
  return subed;
}

template <typename T>
inline std::vector<T> Sub(const std::vector<T>& vec1, const std::vector<T>& vec2) {
  if (vec1.size() == vec2.size()) {
    std::vector<T> subed(vec1.size(), 0.0);
    for (size_t idx = 0; idx < subed.size(); idx++) {
      const T element1 = vec1[idx];
      const T element2 = vec2[idx];
      subed[idx] = element1 - element2;
    }
    return subed;
  } else {
    return std::vector<T>();
  }
}

template <typename T>
inline T Sum(const std::vector<T>& vec) {
  T sum = 0.0;
  for (size_t idx = 0; idx < vec.size(); idx++) {
    const T element = vec[idx];
    sum += element;
  }
  return sum;
}

}  // namespace EigenHelpers

#endif  // ARC_UTILITIES_VECTOR_MATH_HPP
=======
#include <vector>
#include <cmath>
#include <stddef.h>

namespace EigenHelpers //TODO: Change namespace to ArcMath, breaking change
{
    ////////////////////////////////////////////////////////////////////////////
    // Math functions on std::pair types
    ////////////////////////////////////////////////////////////////////////////

    template <typename First, typename Second>
    inline std::pair<First, Second> Abs(const std::pair<First, Second>& pair)
    {
        return {std::abs(pair.first), std::abs(pair.second)};
    }

    template <typename First, typename Second, typename Scalar>
    inline std::pair<First, Second> Multiply(const std::pair<First, Second>& pair, const Scalar scalar)
    {
        return {pair.first * scalar, pair.second * scalar};
    }

    template <typename First, typename Second>
    inline std::pair<First, Second> Multiply(const std::pair<First, Second>& pair1, const std::pair<First, Second>& pair2)
    {
        return {pair1.first * pair2.first, pair1.second * pair2.second};
    }

    template <typename First, typename Second, typename Scalar>
    inline std::pair<First, Second> Divide(const std::pair<First, Second>& pair, const Scalar scalar)
    {
        return {pair.first / scalar, pair.second / scalar};
    }

    template <typename First, typename Second>
    inline std::pair<First, Second> Divide(const std::pair<First, Second>& pair1, const std::pair<First, Second>& pair2)
    {
        return {pair1.first / pair2.first, pair1.second / pair2.second};
    }

    template <typename First, typename Second>
    inline std::pair<First, Second> Add(const std::pair<First, Second>& pair1, const std::pair<First, Second>& pair2)
    {
        return {pair1.first + pair2.first, pair1.second + pair2.second};
    }

    template <typename First, typename Second>
    inline std::pair<First, Second> Sub(const std::pair<First, Second>& pair1, const std::pair<First, Second>& pair2)
    {
        return {pair1.first - pair2.first, pair1.second - pair2.second};
    }

    ////////////////////////////////////////////////////////////////////////////
    // Math functions on std::vector types
    ////////////////////////////////////////////////////////////////////////////

    template <typename T>
    inline std::vector<T> Abs(const std::vector<T>& vec)
    {
        std::vector<T> absed(vec.size(), 0.0);
        for (size_t idx = 0; idx < absed.size(); idx++)
        {
            absed[idx] = std::abs(vec[idx]);
        }
        return absed;
    }

    template <typename T1, typename T2>
    inline std::vector<T1> Multiply(const std::vector<T1>& vec, const T2 scalar)
    {
        std::vector<T1> multiplied(vec.size(), 0.0);
        for (size_t idx = 0; idx < multiplied.size(); idx++)
        {
            const double element = vec[idx];
            multiplied[idx] = element * scalar;
        }
        return multiplied;
    }

    template <typename T>
    inline std::vector<T> Multiply(const std::vector<T>& vec1, const std::vector<T>& vec2)
    {
        if (vec1.size() == vec2.size())
        {
            std::vector<T> multiplied(vec1.size(), 0.0);
            for (size_t idx = 0; idx < multiplied.size(); idx++)
            {
                const double element1 = vec1[idx];
                const double element2 = vec2[idx];
                multiplied[idx] = element1 * element2;
            }
            return multiplied;
        }
        else
        {
            return std::vector<T>();
        }
    }

    template <typename T1, typename T2>
    inline std::vector<T1> Divide(const std::vector<T1>& vec, const T2 scalar)
    {
        const double inv_scalar = 1.0 / scalar;
        return Multiply(vec, inv_scalar);
    }

    template <typename T>
    inline std::vector<T> Divide(const std::vector<T>& vec1, const std::vector<T>& vec2)
    {
        if (vec1.size() == vec2.size())
        {
            std::vector<T> divided(vec1.size(), 0.0);
            for (size_t idx = 0; idx < divided.size(); idx++)
            {
                const T element1 = vec1[idx];
                const T element2 = vec2[idx];
                divided[idx] = element1 / element2;
            }
            return divided;
        }
        else
        {
            return std::vector<T>();
        }
    }

    template <typename T1, typename T2>
    inline std::vector<T1> Add(const std::vector<T1>& vec, const T2 scalar)
    {
        std::vector<T1> added(vec.size(), 0.0);
        for (size_t idx = 0; idx < added.size(); idx++)
        {
            added[idx] = vec[idx] + scalar;
        }
        return added;
    }

    template <typename T>
    inline std::vector<double> Add(const std::vector<T>& vec1, const std::vector<T>& vec2)
    {
        if (vec1.size() == vec2.size())
        {
            std::vector<T> added(vec1.size(), 0.0);
            for (size_t idx = 0; idx < added.size(); idx++)
            {
                const T element1 = vec1[idx];
                const T element2 = vec2[idx];
                added[idx] = element1 + element2;
            }
            return added;
        }
        else
        {
            return std::vector<T>();
        }
    }

    template <typename T1, typename T2>
    inline std::vector<T1> Sub(const std::vector<T1>& vec, const T2 scalar)
    {
        std::vector<T1> subed(vec.size(), 0.0);
        for (size_t idx = 0; idx < subed.size(); idx++)
        {
            subed[idx] = vec[idx] - scalar;
        }
        return subed;
    }

    template <typename T>
    inline std::vector<T> Sub(const std::vector<T>& vec1, const std::vector<T>& vec2)
    {
        if (vec1.size() == vec2.size())
        {
            std::vector<T> subed(vec1.size(), 0.0);
            for (size_t idx = 0; idx < subed.size(); idx++)
            {
                const T element1 = vec1[idx];
                const T element2 = vec2[idx];
                subed[idx] = element1 - element2;
            }
            return subed;
        }
        else
        {
            return std::vector<T>();
        }
    }

    template <typename T>
    inline T Sum(const std::vector<T>& vec)
    {
        T sum = 0.0;
        for (size_t idx = 0; idx < vec.size(); idx++)
        {
            const T element = vec[idx];
            sum += element;
        }
        return sum;
    }

}

#endif //ARC_UTILITIES_VECTOR_MATH_HPP
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
