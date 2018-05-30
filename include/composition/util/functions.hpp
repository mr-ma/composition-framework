#ifndef COMPOSITION_FRAMEWORK_FUNCTIONS_HPP
#define COMPOSITION_FRAMEWORK_FUNCTIONS_HPP

#include <cxxabi.h>
#include <string>

std::string demangle(const std::string &mangled_name) {
	int status = -1;
	char *demangled = abi::__cxa_demangle(mangled_name.c_str(), NULL, NULL, &status);
	if (status == 0) {
		return std::string(demangled);
	}
	return std::string();
}

#endif //COMPOSITION_FRAMEWORK_FUNCTIONS_HPP
