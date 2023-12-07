#ifndef H_6A1536E1_948F_44D2_A74F_7C86F197424E
#define H_6A1536E1_948F_44D2_A74F_7C86F197424E

#ifndef TEST
#include "motor.h"
#endif

#include "tick.h"

// In numeric order
extern uint32_t homingThresholdInitialRevert;
extern uint32_t homingThresholdMinimumAxisEffect;
extern uint32_t homingThresholdFineScan;
extern uint32_t homingThresholdSingleAxisScan;
extern uint32_t homingThresholdInitialScan;
extern uint32_t homingThresholdRescan;

extern OutputSchedule homingStep;
extern OutputSchedule homingClearingStep;
extern OutputSchedule homingFineStep;

void runHoming();
void homingStop();
void homingUpwards();

extern bool homingDebug;

#endif
