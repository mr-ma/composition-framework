#ifndef COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP

#include <string>
#include <map>
#include <vector>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/Pass.hpp>

typedef std::pair<char *, bool> PassRegistrationInfo;

class AnalysisRegistry {
public:
	// register a class name with a particular create method
	static bool Register(char *ID, bool *postPatchingRequired) {
		// add the pair to the map
		llvm::dbgs() << "Registering pass: " << std::to_string(reinterpret_cast<uintptr_t>(ID)) << "\n";
		RegisteredAnalysis()->push_back({ID, postPatchingRequired});
		return true;
	}

	static std::vector<PassRegistrationInfo> GetAll() {
		return *RegisteredAnalysis();
	};
protected:
	// a map to hold a ... mapping between strings and create functions
	static std::vector<PassRegistrationInfo> *RegisteredAnalysis() {
		static std::vector<PassRegistrationInfo> value = {};
		return &value;
	};
};

#endif //COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP
