#ifndef COMPOSITION_FRAMEWORK_UTIL_STRINGS_HPP
#define COMPOSITION_FRAMEWORK_UTIL_STRINGS_HPP

#include <string>

namespace composition::util {
/**
 * Removes leading whitespace from the string
 * @param str the string to trim
 * @param chars the characters to remove
 * @return the trimmed string
 */
std::string &ltrim(std::string &str, const std::string &chars = "\t\n\v\f\r ");

/**
 * Removes trailing whitespace from the string
 * @param str the string to trim
 * @param chars the characters to remove
 * @return the trimmed string
 */
std::string &rtrim(std::string &str, const std::string &chars = "\t\n\v\f\r ");

/**
 * Removes leading and trailing whitespace from the string
 * @param str the string
 * @param chars the characters to remove
 * @return the trimmed string
 */
std::string &trim(std::string &str, const std::string &chars = "\t\n\v\f\r ");
}
#endif //COMPOSITION_FRAMEWORK_UTIL_STRINGS_HPP
