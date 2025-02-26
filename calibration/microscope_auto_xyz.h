#ifndef H_068C879B_6C08_43B0_9FD2_1B9695B46EDB
#define H_068C879B_6C08_43B0_9FD2_1B9695B46EDB

#include "tickable.h"

#include <cstdint>
#include <memory>

class Connections;

class MicroscopeAutoXYZ: public Tickable {
  private:
    Connections *connections;
    double scaleFactor;
    double precision;
    double focusSpread;
    double settleTime;

    uint64_t nextStep;
    int state;

    struct FocusMeasurement {
      double position;
      uint64_t data;
    };

    FocusMeasurement focusMeasurements[15];
    int focusTargetIndex;
    double focusStep;

  public:
    static std::unique_ptr<MicroscopeAutoXYZ> open(
        Connections *, double scaleFactor, double precision, double focusSpread, double settleTime);

    void tick() override;
};

#endif
