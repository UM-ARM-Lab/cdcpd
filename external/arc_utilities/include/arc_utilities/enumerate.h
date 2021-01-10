#include <algorithm>
#include <iterator>
#include <utility>
#include <vector>

template <class Itr>
struct enumerate {
  explicit enumerate(Itr v) {
    auto first = std::cbegin(v);
    auto last = std::cend(v);
    auto i = 0;
    auto make_pair = [&i](auto const item) { return std::make_pair(i++, item); };
    internal_container.resize(std::distance(first, last));
    std::transform(first, last, internal_container.begin(), make_pair);
  }

  auto begin() const { return internal_container.cbegin(); }

  auto end() const { return internal_container.cend(); }

  std::vector<std::pair<size_t, typename Itr::value_type>> internal_container;
};
