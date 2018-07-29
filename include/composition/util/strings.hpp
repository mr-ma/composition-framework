#ifndef COMPOSITION_FRAMEWORK_STRINGS_HPP
#define COMPOSITION_FRAMEWORK_STRINGS_HPP
#include <string>

namespace composition {
std::string &ltrim(std::string &str, const std::string &chars = "\t\n\v\f\r ");

std::string &rtrim(std::string &str, const std::string &chars = "\t\n\v\f\r ");

std::string &trim(std::string &str, const std::string &chars = "\t\n\v\f\r ");
}
#endif //COMPOSITION_FRAMEWORK_STRINGS_HPP
