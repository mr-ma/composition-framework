#ifndef COMPOSITION_FRAMEWORK_ANALYSIS_HPP
#define COMPOSITION_FRAMEWORK_ANALYSIS_HPP

#include <string>
#include <composition/Pass.hpp>
#include <composition/AnalysisRegistry.hpp>
#include <composition/Manifest.hpp>
#include <composition/ManifestRegistry.hpp>
#include <composition/graph/vertex_type.hpp>
#include <composition/PreservedValueRegistry.hpp>

namespace composition {
	template<typename T>
	class ComposableAnalysis : public Pass {
	public:
		template<typename S, typename U>
		void addProtection(std::string name, S protector, U protectee, PatchFunction p) {
			addProtection(Manifest{name, protector, TypeToNodeType<S>().value, protectee, TypeToNodeType<U>().value}, p);
		}

		void addProtection(Manifest m, PatchFunction p) {
			ManifestRegistry::Add(m, p);
		}

		void addPreserved(llvm::Value* value) {
			PreservedValueRegistry::Register(value);
		}

	protected:
		// to determine if the class definition is registered
		const static bool IsRegistered_;

		ComposableAnalysis() : Pass(IsRegistered_) {}
	};

	template<typename T>
// attempt to initialise the IsRegistered variable of derived classes
// whilst registering them to the factory
	const bool ComposableAnalysis<T>::IsRegistered_ = AnalysisRegistry::Register(&T::ID);
}
#endif //COMPOSITION_FRAMEWORK_ANALYSIS_HPP
