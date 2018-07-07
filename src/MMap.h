#ifndef COMPOSITION_FRAMEWORK_MMAP_H
#define COMPOSITION_FRAMEWORK_MMAP_H

#include <set>

template<class _A, class _B, class _Compare=std::less<_A>>
class MMap : public std::set<std::pair<_A, _B>, _Compare
> {
public:
  MMap() : std::set<std::pair<_A, _B>, _Compare
  >() {
  };

  ~
  MMap() = default;;
};

template<typename InPair>
struct MMapComp {
  bool operator()(InPair a, InPair b) {
    if (a.first == b.first)
      return a.second < b.second;
    else
      return a.first < b.first;
  }
};

#endif //COMPOSITION_FRAMEWORK_MMAP_H
