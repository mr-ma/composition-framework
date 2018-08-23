#ifndef COMPOSITION_FRAMEWORK_PASS_HPP
#define COMPOSITION_FRAMEWORK_PASS_HPP

#include <string>

namespace composition {

class Pass {
public:
  Pass();

  explicit Pass(bool isRegistered);

  virtual ~Pass() = default;

  // to determine if this instance class is an instance
  // of a derived class, registered to the factory.
  bool IsRegistered() const;

private:
  const bool IsRegistered_;
};
}
#endif //COMPOSITION_FRAMEWORK_PASS_HPP
