#include "simulation.c"

int main(void) {
  Position delta = {
    {12.0, 34.0, 56.0},
    {0.9205048534524404, 0.3907311284892737, 0, 0}
  };
  Position base = {
    {11.0, 22.0, 33.0},
    {0.7431448254773942, 0, 0, -0.6691306063588582}
  };
  Rotation unit1 = quaternionMul(unitQuaternionInverse(base.rot), base.rot);
  Rotation unit2 = quaternionMul(base.rot, unitQuaternionInverse(base.rot));
  printf("unit1: @ %lf %lf %lf %lf\n", unit1.r, unit1.i, unit1.j, unit1.k);
  printf("unit2: @ %lf %lf %lf %lf\n", unit2.r, unit2.i, unit2.j, unit2.k);

  Position addSub = relativePositionAdd(relativePositionSub(base, delta), delta);
  Position subAdd = relativePositionSub(relativePositionAdd(base, delta), delta);

  printf("addSub: %lf %lf %lf @ %lf %lf %lf %lf\n",
      addSub.disp.x, addSub.disp.y, addSub.disp.z,
      addSub.rot.r, addSub.rot.i, addSub.rot.j, addSub.rot.k);
  printf("subAdd: %lf %lf %lf @ %lf %lf %lf %lf\n",
      subAdd.disp.x, subAdd.disp.y, subAdd.disp.z,
      subAdd.rot.r, subAdd.rot.i, subAdd.rot.j, subAdd.rot.k);

  initMotorDrivers();

  for(int i = 0; i < MAIN_AXIS_COUNT; ++i) {
    motors[i].pos = 0;
    sliderZero[i].x = (i - 3) * 100.0;
    sliderZero[i].y = 0.0;

    sliderUpStep[i].x = 0.0;
    sliderUpStep[i].y = 0.0;
    sliderUpStep[i].z = 1.75 / 256 / 200;

    strutLength[i] = 550.0;

    upperLimit[i] = 200 * 256 * 200 / 1.75;
    lowerLimit[i] = -700 * 256 * 200 / 1.75;
  }

  sliderZero[0].z = -42.0;
  sliderZero[1].z = 0.0;
  sliderZero[2].z = -85.0;
  sliderZero[3].z = -42.0;
  sliderZero[4].z = -85.0;
  sliderZero[5].z = 0.0;
  sliderZero[6].z = -42.0;

  platformAttachment[0].x = 40.0 - 3 * 100.0;
  platformAttachment[1].x = 25.0 - 2 * 100.0;
  platformAttachment[2].x = 5.0 - 1 * 100.0;
  platformAttachment[3].x = 0.0 - 0 * 100.0;
  platformAttachment[4].x = -5.0 + 1 * 100.0;
  platformAttachment[5].x = -25.0 + 2 * 100.0;
  platformAttachment[6].x = -40.0 + 3 * 100.0;

  platformAttachment[0].y = 5.0 - 75.0 - 21.0; // 21.0 == GimbalLowerBrace.BaseAxle.Sketch026.GimbalLowerBraceOffset
  platformAttachment[1].y = 0.0 - 75.0 - 21.0;
  platformAttachment[2].y = 10.0 - 75.0 - 21.0;
  platformAttachment[3].y = 50.0 - 75.0 - 21.0;
  platformAttachment[4].y = 10.0 - 75.0 - 21.0;
  platformAttachment[5].y = 0.0 - 75.0 - 21.0;
  platformAttachment[6].y = 5.0 - 75.0 - 21.0;

  platformAttachment[0].z = 0.0;
  platformAttachment[1].z = 40.0;
  platformAttachment[2].z = -45.0;
  platformAttachment[3].z = 0.0;
  platformAttachment[4].z = -45.0;
  platformAttachment[5].z = 40.0;
  platformAttachment[6].z = 0.0;

  kinematicsSubdivisionInterval = 0.05;
  maximumStepsPerSecond = 51200;

  motorSchedule = NULL;

  Position somewhereBelow = {
    {0.0, 65.0, -590.0},
    {1.0, 0.0, 0.0, 0.0}
  };

  setZero(somewhereBelow);

  // Position toolAtCenter = {
  //   {0.0, 20.0, -45.0},
  //   {1.0, 0.0, 0.0, 0.0}
  // };
  // setToolAttachedAt(toolAtCenter);

  // Position targetPosition = {
  //   {0.0, 20.0, -595.0},
  //   {1.0, 0.0, 0.0, 0.0}
  // };
  // moveTo(targetPosition);

  Position toolAtCenter = {
    {0.0, 0.0, 0.0},
    {1.0, 0.0, 0.0, 0.0}
  };
  setToolAttachedAt(toolAtCenter);

  Position targetPosition = {
    {0.0, 65.1, -592.0},
    {1.0, 0.0, 0.0, 0.0}
  };
  moveTo(targetPosition);

  uint64_t step = 0;
  while(
      tool.disp.x != targetPosition.disp.x ||
      tool.disp.y != targetPosition.disp.y ||
      tool.disp.z != targetPosition.disp.z ||
      tool.rot.r != targetPosition.rot.r ||
      tool.rot.i != targetPosition.rot.i ||
      tool.rot.j != targetPosition.rot.j ||
      tool.rot.k != targetPosition.rot.k ||
      motorsMoving()
  ) {
    if(++step % 20000 == 0) {
      for(int i = 0; i < MOTOR_COUNT; ++i) printf("%d ", motors[i].pos);
      printf(" Tool: %lf %lf %lf @ %lf %lf %lf %lf\n",
          tool.disp.x, tool.disp.y, tool.disp.z, tool.rot.r, tool.rot.i, tool.rot.j, tool.rot.k);
    }

    runKinematics();
    simulate();
  }
}
