#ifndef H_052D191C_C080_43F7_B5D6_62A2651045CC
#define H_052D191C_C080_43F7_B5D6_62A2651045CC

#include <stdint.h>
#include <stdbool.h>

void runEndstop();

void endstopOn();
void endstopOff();
void endstopFloat();
void endstopScan();

bool endstopInitializing();
bool endstopScanning();

extern bool endstopDebug;

#endif
