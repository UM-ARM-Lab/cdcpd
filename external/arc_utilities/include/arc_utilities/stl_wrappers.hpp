#ifndef ARC_STL_WRAPPERS
#define ARC_STL_WRAPPERS

#include <algorithm>
<<<<<<< HEAD
#include <vector>

namespace arc_std {

/**
 *  Returns the index of element in vector v
 *   or returns -1 of element is not in v
 */
template <typename T>
inline int find(const std::vector<T>& v, const T& element) {
  const auto it = std::find(v.begin(), v.end(), element);
  if (it == v.end()) {
    return -1;
  }
  return std::distance(v.begin(), it);
}
}  // namespace arc_std

#endif  // ARC_STL_WRAPPERS
=======

namespace arc_std{

    /**
     *  Returns the index of element in vector v
     *   or returns -1 of element is not in v
     */
    template<typename T>
    inline int find(const std::vector<T>& v, const T& element)
    {
        const auto it = std::find(v.begin(), v.end(), element);
        if(it == v.end())
        {
            return -1;
        }
        return std::distance(v.begin(), it);
    }
}





#endif //ARC_STL_WRAPPERS
>>>>>>> 327be82... bring back local copy of arc_utilities, but IGNORE it by default
