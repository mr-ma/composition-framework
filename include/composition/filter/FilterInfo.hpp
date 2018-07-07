#ifndef COMPOSITION_FRAMEWORK_FILTER_FILTERINFO_HPP
#define COMPOSITION_FRAMEWORK_FILTER_FILTERINFO_HPP

#include <unordered_map>
#include <unordered_set>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>

namespace composition {
template<class T>
class FilterInfo {
private:
  const std::string EMPTY_NAME = "LEGACY";
public:
  void add(std::string name, T v) noexcept {
    llvm::dbgs() << "FilterInfo. Adding from:" << name << " with value: " << v->getName() << "\n";
    Values.insert({name, v});
  }

  void add(T v) noexcept {
    add(EMPTY_NAME, v);
  }

  bool has(const std::string &name, T v) const noexcept {
    if (v == nullptr) {
      return false;
    }
    if (Values.size() == 0) {
      return false;
    }
    auto byName = all(name);
    return byName.find(v) != byName.end();
  }

  bool has(T v) const noexcept {
    return has(EMPTY_NAME, v);
  }

  const std::unordered_set<T> &all() const noexcept {
    std::unordered_set<T> vals;
    vals.reserve(Values.size());

    for (auto kv : Values) {
      vals.insert(kv.second);
    }

    return std::move(vals);
  }

  const std::unordered_set<T> &all(const std::string &name) const noexcept {
    std::unordered_set<T> vals;
    vals.reserve(Values.size());

    auto result = Values.equal_range(name);
    for (auto it = result.first; it != result.second; it++) {
      vals.insert(it->second);
    }

    return std::move(vals);
  }

  size_t size() const noexcept {
    return Values.size();
  }

  size_t size(const std::string &name) const noexcept {
    return all(name).size();
  }

private:
  std::unordered_multimap<std::string, T> Values{};
};
}
#endif //COMPOSITION_FRAMEWORK_FILTER_FILTERINFO_HPP
