#ifndef COMPOSITION_FRAMEWORK_PASS_HPP
#define COMPOSITION_FRAMEWORK_PASS_HPP

#include <string>

namespace composition {

	class Pass {
	public:
		Pass() : IsRegistered_(false) {}

		Pass(bool isRegistered) : IsRegistered_(isRegistered) {}

		virtual ~Pass() {}

		// to determine if this instance class is an instance
		// of a derived class, registered to the factory.
		bool IsRegistered() const {
			return this->IsRegistered_;
		}

	private:
		const bool IsRegistered_;
	};
}
#endif //COMPOSITION_FRAMEWORK_PASS_HPP
