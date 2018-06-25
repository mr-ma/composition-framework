#include <composition/graph/vertex.hpp>

#include <sstream>
#include <composition/graph/graphviz.hpp>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>

std::ostream &operator<<(std::ostream &os, const vertex_type &obj) {
	os << static_cast<std::underlying_type<vertex_type>::type>(obj);
	return os;
}

vertex_t::vertex_t(
		vertex_idx_t index,
		const std::string &name,
		vertex_type type,
		const std::unordered_set<std::string> &attributes
) noexcept :
		index{index},
		name{name},
		type{type},
		attributes{attributes} {
}

std::ostream &operator<<(std::ostream &os, const vertex_t &v) noexcept {
	os << v.index
	   << ","
	   << graphviz_encode(v.name)
	   << ","
	   << v.type
	   << ",";
	for (const auto &a : v.attributes) {
		os << a << " ";
	}
	return os;
}

bool operator==(const vertex_t &lhs, const vertex_t &rhs) noexcept {
	return lhs.index == rhs.index;
}

bool operator!=(const vertex_t &lhs, const vertex_t &rhs) noexcept {
	return !(lhs == rhs);
}

void assertType(llvm::Value *value, vertex_type type) {
	if (llvm::isa<llvm::Instruction>(value)) {
		assert(type == vertex_type::INSTRUCTION);
	} else if (llvm::isa<llvm::BasicBlock>(value)) {
		assert(type == vertex_type::BASICBLOCK);
	} else if (llvm::isa<llvm::Function>(value)) {
		assert(type == vertex_type::FUNCTION);
	} else {
		assert(false);
	}
}
