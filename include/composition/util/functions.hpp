#ifndef COMPOSITION_FRAMEWORK_UTIL_FUNCTIONS_HPP
#define COMPOSITION_FRAMEWORK_UTIL_FUNCTIONS_HPP

#include <string>

namespace composition {
std::string demangle(const std::string &mangled_name);

void extract_function_name(std::string &full_name);

std::string getPassName();
}

#endif //COMPOSITION_FRAMEWORK_UTIL_FUNCTIONS_HPP
