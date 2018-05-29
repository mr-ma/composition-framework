#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <composition/trace/TraceableValue.hpp>
#include <llvm/PassSupport.h>
#include <regex>

using namespace llvm;

std::string getPassName() {
	std::string passName;

	const void *val = SavePrettyStackState();
	if (val != nullptr) {
		auto *entry = (PrettyStackTraceEntry *) val;

		std::string out;
		auto stream = raw_string_ostream(out);
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

template<typename ExtraDataT>
void TraceableValueState::Config::onRAUW(const ExtraDataT &, llvm::Value *oldValue, llvm::Value *newValue) {
	dbgs() << "RAUWed value but should be preserved\n";
	dbgs() << "Old: ";
	oldValue->print(dbgs(), true);
	dbgs() << " New: ";
	newValue->print(dbgs(), true);
	dbgs() << "\n";
	dbgs() << "Value was changed by pass: " << getPassName() << "\n";
}

template<typename ExtraDataT>
void TraceableValueState::Config::onDelete(const ExtraDataT &, llvm::Value *oldValue) {
	dbgs() << "Deleted value but should be preserved\n";
	dbgs() << "Old: ";
	oldValue->print(dbgs(), true);
	dbgs() << "\n";
	dbgs() << "Value was deleted by pass: " << getPassName() << "\n";
}


void TraceableValueState::clear() {
	GlobalNumbers.clear();
}

void TraceableValueState::erase(llvm::Value *Global) {
	GlobalNumbers.erase(Global);
}

uint64_t TraceableValueState::getNumber(llvm::Value *Global) {
	ValueNumberMap::iterator MapIter;
	bool Inserted;
	std::tie(MapIter, Inserted) = GlobalNumbers.insert({Global, NextNumber});
	if (Inserted)
		NextNumber++;
	return MapIter->second;
}
