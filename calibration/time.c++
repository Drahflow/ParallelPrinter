#include "time.h"

#include <time.h>
#include <cstring>
#include <cerrno>
#include <iostream>

using namespace std;

uint64_t now() {
  struct timespec t;

  int ret = clock_gettime(CLOCK_MONOTONIC, &t);
  if(ret == -1) {
    cerr << "Could not read clock: " << strerror(errno) << endl;
    return 0;
  }

  return t.tv_sec * 1'000'000'000 + t.tv_nsec;
}
