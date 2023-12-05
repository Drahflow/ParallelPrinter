#ifndef H_6A1536E1_948F_44D2_A74F_7C86F197424E
#define H_6A1536E1_948F_44D2_A74F_7C86F197424E

#include "motor.h"
#include "tick.h"

typedef enum {
  NONE, SCANNING_GENERAL
} HomingState;

extern uint32_t homingThreshold;
extern OutputSchedule homingStep;

void runHoming();
void homingStop();
void homingUpwards();

#endif
