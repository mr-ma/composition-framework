#ifndef COMPOSITION_FRAMEWORK_PROTECTION_HPP
#define COMPOSITION_FRAMEWORK_PROTECTION_HPP

#include <llvm/IR/Value.h>

namespace composition {
	class Protection {
	public:
		llvm::Value *from;
		llvm::Value *to;

	public:
		Protection() = default;

		Protection(llvm::Value *from, llvm::Value *to) {
			this->from = from;
			this->to = to;
		}

		Protection(const Protection &that) {
			from = that.from;
			to = that.to;
		}

	};
}

namespace std {
	template<>
	struct hash<composition::Protection> {
		size_t operator()(const composition::Protection &pt) const {
			auto h1 = reinterpret_cast<uintptr_t>(pt.from);
			auto h2 = reinterpret_cast<uintptr_t>(pt.to);
			return h1 ^ h2;
		}
	};
}
#endif //COMPOSITION_FRAMEWORK_PROTECTION_HPP
