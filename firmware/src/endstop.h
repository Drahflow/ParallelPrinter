#ifndef H_052D191C_C080_43F7_B5D6_62A2651045CC
#define H_052D191C_C080_43F7_B5D6_62A2651045CC

#include <stdint.h>
#include <stdbool.h>

extern bool endstopScanning;

void runEndstop();

void endstopOn();
void endstopOff();
void endstopScan();

#endif
