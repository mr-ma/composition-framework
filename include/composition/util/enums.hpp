#ifndef COMPOSITION_FRAMEWORK_UTIL_ENUMS_HPP
#define COMPOSITION_FRAMEWORK_UTIL_ENUMS_HPP

#include <cassert>
#include <type_traits>

namespace composition {
template<typename Enum>
struct EnableBitMaskOperators {
  static const bool enable = false;
};

template<typename Enum>
typename std::enable_if<EnableBitMaskOperators<Enum>::enable, Enum>::type operator|(Enum lhs, Enum rhs) {
  return static_cast<Enum> (static_cast<std::underlying_type_t<Enum>>(lhs)
      | static_cast<std::underlying_type_t<Enum>>(rhs));
}

#define ENABLE_BITMASK_OPERATORS(x)  \
template<>                           \
struct EnableBitMaskOperators<x>  {  \
    static const bool enable = true; \
};
}
#endif //COMPOSITION_FRAMEWORK_UTIL_ENUMS_HPP
