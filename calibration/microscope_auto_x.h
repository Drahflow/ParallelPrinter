#ifndef H_07DE1AF9_7E35_42A3_9DA6_B84F5FDCCAAD
#define H_07DE1AF9_7E35_42A3_9DA6_B84F5FDCCAAD

#include "tickable.h"

#include <cstdint>
#include <memory>

class Connections;

class MicroscopeAutoX: public Tickable {
  private:
    Connections *connections;
    double xEnd, xStep;
    uint64_t nextStep;
    uint64_t resetXDistanceAt;

  public:
    static std::unique_ptr<MicroscopeAutoX> open(Connections *, double xEnd, double xStep);

    void tick() override;
};

#endif
