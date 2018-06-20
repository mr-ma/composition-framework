#ifndef COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP

#include <string>
#include <map>
#include <vector>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/Pass.hpp>

class AnalysisRegistry {
public:
	// register a class name with a particular create method
	static bool Register(char *ID) {
		// add the pair to the map
		llvm::dbgs() << "Registering pass: " << std::to_string(reinterpret_cast<uintptr_t>(ID)) << "\n";
		RegisteredAnalysis()->push_back(ID);
		return true;
	}

	static std::vector<char *> *GetAll() {
		return RegisteredAnalysis();
	};
protected:
	// a map to hold a ... mapping between strings and create functions
	static std::vector<char *> *RegisteredAnalysis() {
		static std::vector<char *> value = {};
		return &value;
	};
};

#endif //COMPOSITION_FRAMEWORK_ANALYSISREGISTRY_HPP
