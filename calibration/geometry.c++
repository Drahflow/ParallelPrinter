#include "geometry.h"

bool parseDouble(const std::string &input, double *v) {
  const char *buf = input.data();
  int pos = 0;
  const int endPos = input.size();

  if(input.empty()) return false;

  bool negative = false;
  *v = 0;
  while(pos < endPos && buf[pos] == ' ') ++pos;
  if(pos < endPos && buf[pos] == '-') {
    negative = true;
    ++pos;
  }
  if(!(pos < endPos && ((buf[pos] >= '0' && buf[pos] <= '9') || buf[pos] == '.'))) return false;

  while(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9') {
    *v = 10 * *v + (buf[pos] - '0');
    ++pos;
  }
  if(pos < endPos && buf[pos] == '.') {
    ++pos;
    double digit = .1;
    while(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9') {
      *v = *v + (buf[pos] - '0') * digit;
      digit /= 10;
      ++pos;
    }
  }

  if(negative) *v = -*v;

  return true;
}

