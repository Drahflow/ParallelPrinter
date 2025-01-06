#ifndef H_16123574_AA69_417C_9427_FB4ABFBD14DD
#define H_16123574_AA69_417C_9427_FB4ABFBD14DD

#include <string>
#include <ostream>

struct Displacement {
  double x, y, z;
};

struct Quaternion {
  double r, i, j, k;
};
typedef Quaternion Rotation; // unit quaternion

struct Position {
  Displacement disp;
  Rotation rot;
};

bool parseDouble(const std::string &input, double *v);

std::ostream &operator << (std::ostream &, const Position &);
std::ostream &operator << (std::ostream &, const Displacement &);
std::ostream &operator << (std::ostream &, const Quaternion &);

#endif
