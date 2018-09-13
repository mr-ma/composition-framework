#include <unordered_set>

bool already_seen_placeholder(long long expected) {
  static std::unordered_set<long long> placeholders;
  return !placeholders.insert(expected).second;
}

extern "C" {
void guardMe(const unsigned int address, const unsigned int length, const unsigned int expectedHash) {}

void response() {}

void do_assert(const long long *hash, long long expected) {
  if (*hash != expected) {
    printf("$%lli$%lli$\n", *hash, expected);
    response();
  }
}

void assert(long long *hash, long long expected) {
  if (already_seen_placeholder(expected)) {
    return;
  }
  do_assert(hash, expected);
}

void oh_hash1(long long *hashVariable, long long value) {
  *hashVariable = *hashVariable + value;
}

void oh_hash2(long long *hashVariable, long long value) {
  *hashVariable = *hashVariable ^ value;
}

void registerFunction(char functionName[]) {}

void deregisterFunction(const char functionName[]) {}

void verifyStack() {}
}