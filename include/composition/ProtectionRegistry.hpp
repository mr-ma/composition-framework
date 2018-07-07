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
  static bool Register(char *ID) {
    // add the pair to the map
    llvm::dbgs() << "Registering post protection pass: " << std::to_string(reinterpret_cast<uintptr_t>(ID)) << "\n";
    RegisteredProtections()->push_back(ID);
    return true;
  }

  static std::vector<char *> GetAll() {
    return *RegisteredProtections();
  };
protected:
  // a map to hold a ... mapping between strings and create functions
  static std::vector<char *> *RegisteredProtections() {
    static std::vector<char *> value = {};
    return &value;
  };
};

}
#endif //COMPOSITION_FRAMEWORK_PROTECTIONREGISTRY_HPP
