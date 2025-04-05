#ifndef H_935E4687_DE09_4FF5_BBC4_BF70F76A68ED
#define H_935E4687_DE09_4FF5_BBC4_BF70F76A68ED

#include "tickable.h"

#include <cstdint>
#include <memory>

class Connections;

class MeasureGrid: public Tickable {
  private:
    Connections *connections;

    int xStep, yStep;
    int xBegin, xEnd;
    int yBegin, yEnd;

    uint64_t nextStep;
    int nextAction;
    double scaleFactor;
    double coarsePrecision, coarseFocusSpread, coarseSettleTime;
    double finePrecision, fineFocusSpread, fineSettleTime;

    void startFineXYZ();
    void startCoarseXYZ();
    bool didFineXYZ;

  public:
    static std::unique_ptr<MeasureGrid> open(Connections *,
        int xStep, int yStep, int xEnd, int yEnd,
        double scaleFactor,
        double coarsePrecision, double coarseFocusSpread, double coarseSettleTime,
        double finePrecision, double fineFocusSpread, double fineSettleTime);

    void tick() override;
};

#endif
