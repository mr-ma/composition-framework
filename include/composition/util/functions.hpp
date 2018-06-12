#ifndef COMPOSITION_FRAMEWORK_FUNCTIONS_HPP
#define COMPOSITION_FRAMEWORK_FUNCTIONS_HPP

#include <cxxabi.h>
#include <string>

std::string demangle(const std::string &mangled_name) {
	int status = -1;
	char *demangled = abi::__cxa_demangle(mangled_name.c_str(), nullptr, nullptr, &status);
	if (status == 0) {
		return std::string(demangled);
	}
	return std::string();
}

void extract_function_name(std::string &full_name) {
	auto name_end = full_name.find_first_of('(');
	if (name_end != std::string::npos) {
		full_name = full_name.substr(0, name_end);
	}
}

#endif //COMPOSITION_FRAMEWORK_FUNCTIONS_HPP
