#include "console.h"

#include "usb.h"
#include "motor.h"

int_fast8_t console_receive(uint8_t *buf, uint_fast8_t buf_len) {
  console_send((uint8_t *)"\r\nRCV:", 6);
  console_send(buf, buf_len);

  if(buf[0] == ' ') initMotorDrivers();
  if(buf[0] == ',') backward(1);
  if(buf[0] == 'o') forward(1);
  if(buf[0] == '<') backward(256);
  if(buf[0] == 'O') forward(256);

  if(buf[0] == '1') setResolution(0);
  if(buf[0] == '2') setResolution(1);
  if(buf[0] == '3') setResolution(2);
  if(buf[0] == '4') setResolution(3);
  if(buf[0] == '5') setResolution(4);
  if(buf[0] == '6') setResolution(5);
  if(buf[0] == '7') setResolution(6);
  if(buf[0] == '8') setResolution(7);
  if(buf[0] == '9') setResolution(8);

  if(buf[0] == 'S') dumpMotorStatus();

  return buf_len;
}

void console_send(uint8_t *buf, uint_fast8_t buf_len) {
  usb_console_send(buf, buf_len);
}

void console_send_str(char *str) {
  char *e = str;
  while(*e) ++e;

  console_send((uint8_t *)str, e - str);
}

void console_send_uint32(uint32_t val) {
  uint8_t buf[36];
  uint32_t p = 35;
  for(uint32_t i = 0; i < 32; ++i) {
    buf[p--] = (val & 1)? '1': '0';
    if(i == 7 || i == 15 || i == 23) buf[p--] = ' ';

    val >>= 1;
  }

  console_send(buf, sizeof(buf));
}

void console_send_uint8(uint32_t val) {
  uint8_t buf[8];
  uint32_t p = 7;
  for(uint32_t i = 0; i < 32; ++i) {
    buf[p--] = (val & 1)? '1': '0';
    val >>= 1;
  }

  console_send(buf, sizeof(buf));
}
