#include <composition/util/functions.hpp>

// TODO: Look for a better way #1
std::string getPassName() {
	std::string passName;

	const void *val = llvm::SavePrettyStackState();
	if (val != nullptr) {
		auto *entry = (llvm::PrettyStackTraceEntry *) val;

		std::string out;
		auto stream = llvm::raw_string_ostream(out);
		entry->print(stream);

		std::regex passNameRegex("'(.*?)'");
		std::smatch sm;
		while (std::regex_search(stream.str(), sm, passNameRegex)) {
			passName = sm.str();
			break;
		}
	}
	return passName;
}

void extract_function_name(std::string &full_name) {
	auto name_end = full_name.find_first_of('(');
	if (name_end != std::string::npos) {
		full_name = full_name.substr(0, name_end);
	}
}

std::string demangle(const std::string &mangled_name) {
	int status = -1;
	char *demangled = abi::__cxa_demangle(mangled_name.c_str(), nullptr, nullptr, &status);
	if (status == 0) {
		return std::string(demangled);
	}
	return std::string();
}
