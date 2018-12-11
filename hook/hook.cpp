#include <cinttypes>
#include <unordered_set>

bool already_seen_placeholder(int64_t expected) {
  static std::unordered_set<int64_t> placeholders;
  return !placeholders.insert(expected).second;
}

extern "C" {
void guardMe(const unsigned int, const unsigned int, const unsigned int) {}

void response() {}

void do_assert(const int64_t* hash, int64_t expected) {
  if (*hash != expected) {
    fprintf(stderr, "$%" PRId64 "$%" PRId64 "$\n", *hash, expected);
    response();
  }
}

void assert(int64_t* hash, int64_t expected) {
  if (already_seen_placeholder(expected)) {
    return;
  }
  do_assert(hash, expected);
}

void oh_hash1(int64_t* hashVariable, int64_t value) { *hashVariable = *hashVariable + value; }

void oh_hash2(int64_t* hashVariable, int64_t value) { *hashVariable = *hashVariable ^ value; }

void registerFunction(char[]) {}

void deregisterFunction(const char[]) {}

void verifyStack() {}
}