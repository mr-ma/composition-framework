#ifndef COMPOSITION_FRAMEWORK_PROTECTION_HPP
#define COMPOSITION_FRAMEWORK_PROTECTION_HPP

#include <composition/Pass.hpp>
#include <composition/ProtectionRegistry.hpp>

namespace composition {

	template<typename T>
	class ComposableProtection : public Pass {
	public:

	protected:
		// to determine if the class definition is registered
		const static bool IsRegistered_;

		ComposableProtection() : Pass(IsRegistered_) {}
	};

	template<typename T>
// attempt to initialise the IsRegistered variable of derived classes
// whilst registering them to the factory
	const bool ComposableProtection<T>::IsRegistered_ = ProtectionRegistry::Register(&T::ID);
}
#endif //COMPOSITION_FRAMEWORK_PROTECTION_HPP
