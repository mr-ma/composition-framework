#ifndef COMPOSITION_FRAMEWORK_UTIL_FUNCTIONS_HPP
#define COMPOSITION_FRAMEWORK_UTIL_FUNCTIONS_HPP

#include <string>

namespace composition::util {
/**
 * Retrieves the pass name of the currently executing LLVM pass
 * @return the name of the pass
 */
std::string getPassName();
}

#endif //COMPOSITION_FRAMEWORK_UTIL_FUNCTIONS_HPP
