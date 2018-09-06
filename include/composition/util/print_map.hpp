#ifndef COMPOSITION_FRAMEWORK_UTIL_PRINTMAP_HPP
#define COMPOSITION_FRAMEWORK_UTIL_PRINTMAP_HPP

#include <iostream>

template<class MapType>
void print_map(const MapType &m) {
  for (auto it = m.begin(), it_end = m.end(); it != it_end; ++it) {
    std::cout << it->first << "-->" << it->second << "\n";
  }
}

#endif //COMPOSITION_FRAMEWORK_UTIL_PRINTMAP_HPP
