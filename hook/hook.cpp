extern "C" {
void guardMe(const unsigned int address, const unsigned int length, const unsigned int expectedHash) {} ;

void response() {} ;

void do_assert(long long *hash, long long expected) {} ;

void assert(long long *hash, long long expected) {} ;

void oh_hash1(long long *hashVariable, long long value) {} ;

void oh_hash2(long long *hashVariable, long long value) {} ;

void registerFunction(char functionName[]) {} ;

void deregisterFunction(const char functionName[]) {} ;

void verifyStack() {} ;
}