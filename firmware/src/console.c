#include "console.h"

#include "usb.h"
#include "motor.h"

int_fast8_t console_receive(uint8_t *buf, uint_fast8_t buf_len) {
  console_send((uint8_t *)"RCV: ", 4);
  console_send(buf, buf_len);

  if(buf[0] == ' ') initMotorDrivers();
  if(buf[0] == ',') forward(16);
  if(buf[0] == 'o') backward(16);
  if(buf[0] == '<') forward(256);
  if(buf[0] == 'O') backward(256);

  return buf_len;
}

void console_send(uint8_t *buf, uint_fast8_t buf_len) {
  usb_console_send(buf, buf_len);
}
