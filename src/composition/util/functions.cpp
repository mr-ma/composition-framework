#include <composition/util/functions.hpp>
#include <cxxabi.h>
#include <llvm/Support/PrettyStackTrace.h>
#include <llvm/Support/raw_ostream.h>
#include <regex>

namespace composition::util {
// TODO: Look for a better way #1
std::string getPassName() {
  std::string passName;

  const void *val = llvm::SavePrettyStackState();
  if (val != nullptr) {
    auto *entry = static_cast<const llvm::PrettyStackTraceEntry *>(val);

    std::string out;
    auto stream = llvm::raw_string_ostream(out);
    entry->print(stream);

    std::regex passNameRegex("'(.*?)'");
    std::smatch sm;
    while (std::regex_search(stream.str(), sm, passNameRegex)) {
      passName = sm.str();
      break;
    }
  }
  return passName;
}
} // namespace composition::util