#include "endstop.h"

#include "gpio.h"
#include "console.h"
#include "tick.h"

#include <stdbool.h>

void endstopOn() {
  stopEndstopScan();

  gpio_out_setup(GPIO('F', 0), 1);
}

void endstopOff() {
  stopEndstopScan();

  gpio_out_setup(GPIO('F', 0), 0);
}

void endstopScan(uint32_t level) {
  gpio_in_setup(GPIO('F', 0), 0);

  scheduleEndstopScan();
}

void runEndstop() {
  static EndstopState oldState;
  if(endstopState != oldState) {
    oldState = endstopState;
    switch(endstopState) {
      case ENDSTOP_WAIT:
        console_send_str("Endstop high.\r\n");
        break;
      case ENDSTOP_SCAN:
        console_send_str("Endstop low.\r\n");
        break;
      case ENDSTOP_DONE:
        console_send_str("Endstop done: ");
        console_send_uint32(endstopDuration);
        console_send_str("\r\n");
        break;
      default:
        console_send_str("Endstop undefined state.\r\n");
        break;
    }
  }
}
