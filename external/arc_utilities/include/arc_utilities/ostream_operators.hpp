#pragma once

#include <ostream>
#include <typeinfo>
#include <vector>

template <typename T>
static std::ostream &operator<<(std::ostream &out, std::vector<T> const &vec) {
  for (auto const &val : vec) {
    out << val;
    if (typeid(T) != typeid(std::string)) {
      out << " ";
    } else {
      out << "\n";
    }
  }
  return out;
}