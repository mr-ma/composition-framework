#include <fstream>
#include <llvm/IR/LegacyPassManager.h>
#include <llvm/IR/Module.h>
#include <llvm/Support/Error.h>
#include <llvm/Transforms/IPO/PassManagerBuilder.h>
#include <composition/filter/FunctionFilter.hpp>
#include <composition/util/functions.hpp>

using namespace llvm;
using namespace composition;

static cl::opt<std::string> FilterFile("filter-file", cl::Hidden, cl::desc("Path to function filter file"));

//FunctionFilterPass
void FunctionFilterPass::getAnalysisUsage(llvm::AnalysisUsage &AU) const {
	AU.setPreservesAll();
}

char FunctionFilterPass::ID = 0;

void extract_function_name(std::string &full_name) {
	auto name_end = full_name.find_first_of("(");
	if (name_end != std::string::npos) {
		full_name = full_name.substr(0, name_end);
	}
}

void FunctionFilterPass::loadFile(llvm::Module &M, std::string file_name) {
	std::ifstream functions_strm(file_name);
	if (!functions_strm.is_open()) {
		llvm::dbgs() << " failed to open file\n";
		return;
	}
	dbgs() << "Filter file:" << file_name << "\n";
	std::string name;
	std::unordered_set<std::string> functionNames;
	while (!functions_strm.eof()) {
		functions_strm >> name;
		dbgs() << "here!:" << name << "\n";
		functionNames.insert(name);
	}
	dbgs() << "got filter function names\n";
	for (auto &F : M) {
		auto demangledName = demangle(F.getName());
		if (demangledName.empty()) {
			demangledName = F.getName();
		}
		//extract_function_name(demangledName);
		if (functionNames.find(demangledName) != functionNames.end() || functionNames.find(F.getName()) != functionNames.end()) {
			dbgs() << "Add filter function " << demangledName << "\n";
			this->FunctionsInfo.add(&F);
		}
	}
	if (this->FunctionsInfo.size() != functionNames.size()) {
		errs() << "ERR. filter functions count:" << functionNames.size()
		       << "!=" << "detected functions in binary is:" << this->FunctionsInfo.size() << "\n";
		dbgs() << "Detected functions:\n";
		for (auto &F: FunctionsInfo.all()) {
			dbgs() << F->getName() << "\t";
		}
		dbgs() << "\n";
		dbgs() << "Filter functions:\n";
		for (std::string S: functionNames) {
			dbgs() << S << "\t";
		}
		dbgs() << "\n";
		exit(1);
	}

}

bool FunctionFilterPass::runOnModule(llvm::Module &M) {
	dbgs() << "In  filter function pass " << "\n";
	if (!FilterFile.empty()) {
		loadFile(M, FilterFile);
	}
	return false;
}

FunctionInformation *FunctionFilterPass::getFunctionsInfo() {
	return &(this->FunctionsInfo);
}

static llvm::RegisterPass<FunctionFilterPass> X("filter-func", "Include functions in a given file in any transformation");
