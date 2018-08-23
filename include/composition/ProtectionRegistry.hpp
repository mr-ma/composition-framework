#ifndef COMPOSITION_FRAMEWORK_PROTECTIONREGISTRY_HPP
#define COMPOSITION_FRAMEWORK_PROTECTIONREGISTRY_HPP

#include <vector>
#include <string>
#include <map>
#include <llvm/Support/Debug.h>
#include <llvm/Support/raw_ostream.h>
#include <composition/Pass.hpp>

namespace composition {

class ProtectionRegistry {
public:
  // register a class name with a particular create method
  static bool Register(char *ID);

  static std::vector<char *> GetAll();;
protected:
  // a map to hold a ... mapping between strings and create functions
  static std::vector<char *> *RegisteredProtections();;
};

}
#endif //COMPOSITION_FRAMEWORK_PROTECTIONREGISTRY_HPP
