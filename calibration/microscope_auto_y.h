#ifndef H_B00DEB67_CAE3_47E5_9351_FB7BFD618456
#define H_B00DEB67_CAE3_47E5_9351_FB7BFD618456

#include "tickable.h"

#include <cstdint>
#include <memory>

class Connections;

class MicroscopeAutoY: public Tickable {
  private:
    Connections *connections;
    double yEnd, yStep;
    uint64_t nextStep;
    uint64_t resetYDistanceAt;

  public:
    static std::unique_ptr<MicroscopeAutoY> open(Connections *, double yEnd, double yStep);

    void tick() override;
};

#endif
