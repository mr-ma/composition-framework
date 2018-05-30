#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/filter/FilterInfo.hpp>

using namespace llvm;
using namespace composition;

template<class T>
void FilterInfo<T>::add(std::string name, T v) noexcept {
	llvm::dbgs() << "FilterInfo. Adding from:" << name << " with value: " << v->getName() << "\n";
	Values.insert({name, v});
}

template<class T>
bool FilterInfo<T>::has(std::string name, T v) const noexcept {
	if (v == nullptr) {
		return false;
	}
	if (Values.size() == 0) {
		return false;
	}
	auto byName = allByName(name);
	return byName.find(v) != byName.end();
}

template<class T>
const std::unordered_set<T> &FilterInfo<T>::all() const noexcept {
	std::unordered_set<T> vals;
	vals.reserve(Values.size());

	for (auto kv : Values) {
		vals.insert(kv.second);
	}

	return std::move(vals);
}

template<class T>
const std::unordered_set<T> &FilterInfo<T>::allByName(std::string name) const noexcept {
	std::unordered_set<T> vals;
	vals.reserve(Values.size());

	auto result = Values.equal_range(name);
	for (auto it = result.first; it != result.second; it++) {
		vals.insert(it->second);
	}

	return std::move(vals);
}

template<class T>
size_t FilterInfo<T>::size() const noexcept {
	return Values.size();
}

template<class T>
size_t FilterInfo<T>::sizeByName(std::string name) const noexcept {

	return allByName(name).size();
}

