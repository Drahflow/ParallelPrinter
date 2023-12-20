#include "simulation.c"

int main(void) {
  initMotorDrivers();
  motorSchedule = NULL;
  homingThresholdInitialRevert = 3;
  homingThresholdMinimumAxisEffect = 4;
  homingThresholdFineScan = 4;
  homingThresholdSingleAxisScan = 4;
  homingThresholdInitialScan = 5;
  homingStep.count = 3;
  homingClearingStep.count = 5;
  homingFineStep.count = 1;

  homingUpwards();

  while(homingState != -1) {
    printf("HomingState: %d  Endstop: %d  ", homingState, endstopCharge);
    for(int i = 0; i < MOTOR_COUNT; ++i) printf("%d ", motors[i].pos);
    printf("\n");

    runHoming();

    simulate();
  }
}
