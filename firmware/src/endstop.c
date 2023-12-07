#include "endstop.h"

#include "gpio.h"
#include "console.h"
#include "tick.h"

#include <stdbool.h>

bool endstopDebug = false;

void endstopOn() {
  stopEndstopScan();

  gpio_out_setup(GPIO('F', 0), 1);
}

void endstopOff() {
  stopEndstopScan();

  gpio_out_setup(GPIO('F', 0), 0);
}

void endstopFloat() {
  stopEndstopScan();

  gpio_in_setup(GPIO('F', 0), 0);
}

bool endstopInitializing() {
  return endstopState == ENDSTOP_INIT || endstopState == ENDSTOP_WAIT;
}

bool endstopScanning() {
  return endstopState == ENDSTOP_INIT || endstopState == ENDSTOP_WAIT || endstopState == ENDSTOP_SCAN;
}

void endstopScan() {
  gpio_out_setup(GPIO('F', 0), 1);

  scheduleEndstopScan();
}

void runEndstop() {
  static EndstopState oldState;
  if(endstopState != oldState) {
    oldState = endstopState;
    switch(endstopState) {
      case ENDSTOP_INIT:
        if(endstopDebug) console_send_str("Endstop init.\r\n");
        break;
      case ENDSTOP_WAIT:
        if(endstopDebug) console_send_str("Endstop high.\r\n");
        break;
      case ENDSTOP_SCAN:
        if(endstopDebug) console_send_str("Endstop low.\r\n");
        break;
      case ENDSTOP_DONE:
        if(endstopDebug) {
          console_send_str("Endstop done and took: ");
          console_send_uint32(endstopDuration);
          console_send_str("\r\n");
        }
        break;
      default:
        console_send_str("Endstop undefined state.\r\n");
        break;
    }
  }
}
