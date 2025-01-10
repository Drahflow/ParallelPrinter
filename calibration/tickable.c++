#include "tickable.h"

using namespace std;

void Tickable::startTicked(vector<Tickable *> &v) {
  v.push_back(this);
}

void Tickable::stopTicked(vector<Tickable *> &v) {
  auto i = ranges::find(v, this);
  if(i == v.end()) return;

  *i = v.back();
  v.pop_back();
}
