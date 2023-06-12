#ifndef H_8FA03834_4167_4067_AC46_26B7722B6E99
#define H_8FA03834_4167_4067_AC46_26B7722B6E99

#include <stdint.h>

void initMotorDrivers();
void forward();
void backward();
void setupMotor(uint32_t stepResolution, uint32_t runPower);
void dumpMotorStatus();

#endif
