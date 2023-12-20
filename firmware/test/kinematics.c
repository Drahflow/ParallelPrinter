#include "simulation.c"

int main(void) {
  initMotorDrivers();

  for(int i = 0; i < MAIN_AXIS_COUNT; ++i) {
    motors[i].pos = 0;
    sliderZero[i].x = (i - 4) * -100.0;
    sliderZero[i].y = 0.0;
    sliderZero[i].z = 0.0;

    sliderUpStep[i].x = 0.0;
    sliderUpStep[i].y = 0.0;
    sliderUpStep[i].z = 1.75 / 256;

    strutLength[i] = 550.0;

    upperLimit[i] = 200 * 256 / 1.75;
    lowerLimit[i] = -700 * 256 / 1.75;
  }

  platformAttachment[0].x = 40.0 - 3 * 100.0;
  platformAttachment[1].x = 25.0 - 2 * 100.0;
  platformAttachment[2].x = 5.0 - 1 * 100.0;
  platformAttachment[3].x = 0.0 - 0 * 100.0;
  platformAttachment[4].x = -5.0 + 1 * 100.0;
  platformAttachment[5].x = -25.0 + 2 * 100.0;
  platformAttachment[6].x = -40.0 + 3 * 100.0;

  platformAttachment[0].y = 5.0;
  platformAttachment[1].y = 0.0;
  platformAttachment[2].y = 10.0;
  platformAttachment[3].y = 50.0;
  platformAttachment[4].y = 10.0;
  platformAttachment[5].y = 0.0;
  platformAttachment[6].y = 5.0;

  platformAttachment[0].z = 0.0;
  platformAttachment[1].z = 40.0;
  platformAttachment[2].z = -45.0;
  platformAttachment[3].z = 0.0;
  platformAttachment[4].z = -45.0;
  platformAttachment[5].z = 40.0;
  platformAttachment[6].z = 0.0;

  motorSchedule = NULL;

  Position toolAtCenter = {
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 0.0}
  };
  setToolAttachedAt(toolAtCenter);

  Position somewhereBelow = {
    {0.0, 0.0, -510.0},
    {1.0, 0.0, 0.0, 0.0}
  };

  Position hangingBelow = {
    {0.0, 0.0, -500.0},
    {1.0, 0.0, 0.0, 0.0}
  };

  setZero(somewhereBelow);
  moveTo(hangingBelow);

  while(true) {
    for(int i = 0; i < MOTOR_COUNT; ++i) printf("%d ", motors[i].pos);
    printf("\n");

    runKinematics();
    simulate();
  }
}
