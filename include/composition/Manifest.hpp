#ifndef COMPOSITION_FRAMEWORK_MANIFEST_HPP
#define COMPOSITION_FRAMEWORK_MANIFEST_HPP

#include <llvm/IR/Value.h>
#include <composition/graph/vertex.hpp>

namespace composition {


	class Manifest {
	public:
		uintptr_t idx{};
		std::string protection;
		llvm::Value *from;
		vertex_type fromType;
		llvm::Value *to;
		vertex_type toType;

		Manifest(const std::string &protection, llvm::Value *from, vertex_type fromType, llvm::Value *to, vertex_type toType)
				: protection(protection), from(from), fromType(fromType), to(to), toType(toType) {
			assertType(from, fromType);
			assertType(to, toType);
		}
		//TODO metadata if postpatching is needed - and its direction (from, to, both)

		bool operator==(const Manifest &other) const {
			return (protection == other.protection && reinterpret_cast<uintptr_t>(from) == reinterpret_cast<uintptr_t>(other.from) &&
			        reinterpret_cast<uintptr_t>(to) == reinterpret_cast<uintptr_t>(other.to));
		}

		bool operator<(const Manifest &other) const {
			return (protection < other.protection);
		}
	};

}

namespace std {
	template<>
	struct hash<composition::Manifest> {
		size_t operator()(const composition::Manifest &pt) const {
			size_t h1 = std::hash<std::string>()(pt.protection);
			auto h2 = reinterpret_cast<uintptr_t>(pt.from);
			auto h3 = reinterpret_cast<uintptr_t>(pt.to);
			return h1 ^ h2 ^ h3;
		}
	};
}

#endif //COMPOSITION_FRAMEWORK_MANIFEST_HPP
