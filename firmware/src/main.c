#include "usb.h"

#include "beep.h"

void main() {
  // initUSB();

  beep_synchronous(2000, 880);

  while(1) {
    // runUSB();
  }
}

void shutdown(char *error) { }
