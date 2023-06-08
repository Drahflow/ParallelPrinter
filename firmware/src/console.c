#include "console.h"
#include "usb.h"

int_fast8_t console_receive(uint8_t *buf, uint_fast8_t buf_len) {
  console_send("RCV: ", 4);
  console_send(buf, buf_len);
}

void console_send(uint8_t *buf, uint_fast8_t buf_len) {
  usb_console_send(buf, buf_len);
}
