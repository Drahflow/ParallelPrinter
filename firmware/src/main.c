#include "usb.h"

#include "beep.h"
#include "watchdog.h"

void main() {
  // watchdog_init();
  initUSB();

  beep_synchronous(100, 880);
  uint32_t t = 0;

  while(1) {
    if(t++ == 10000000) {
      beep_synchronous(5, 880);
      t = 0;
    }

    runUSB();
    watchdog_reset();
  }
}

void shutdown(char *error) { }
