#ifndef H_FA667796_5463_4946_A52F_F2897C9E3784
#define H_FA667796_5463_4946_A52F_F2897C9E3784

#include "tickable.h"

#include <cstdint>
#include <memory>

class Connections;

class MicroscopeAutofocus: public Tickable {
  private:
    Connections *connections;
    double zEnd, zStep;
    uint64_t nextStep;
    uint64_t resetFocusAt;

  public:
    static std::unique_ptr<MicroscopeAutofocus> open(Connections *, double zEnd, double zStep);

    void tick() override;
};

#endif
