#ifndef COMPOSITION_FRAMEWORK_FUNCTIONS_HPP
#define COMPOSITION_FRAMEWORK_FUNCTIONS_HPP

#include <cxxabi.h>
#include <string>
#include <regex>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/raw_ostream.h>

std::string demangle(const std::string &mangled_name);

void extract_function_name(std::string &full_name);

std::string getPassName();

#endif //COMPOSITION_FRAMEWORK_FUNCTIONS_HPP
