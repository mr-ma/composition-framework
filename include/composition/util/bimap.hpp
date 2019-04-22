#ifndef COMPOSITION_UTIL_BIMAP
#define COMPOSITION_UTIL_BIMAP

#include <unordered_map>
#include <unordered_set>
namespace composition::util {
template<typename T, typename U> class bimap {
public:
  std::unordered_map<T, std::unordered_set<U>> left{};
  std::unordered_map<U, std::unordered_set<T>> right{};

  void insert(std::pair<T, U> v) {
    left[v.first].insert({v.second});
    right[v.second].insert({v.first});
  }

  size_t size() { return left.size() + right.size(); }

  void clear() {
    left.clear();
    right.clear();
  }
};
} // namespace composition::util
#endif // COMPOSITION_UTIL_BIMAP