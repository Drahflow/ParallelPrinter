#include "console.h"

#include "usb.h"
#include "motor.h"
#include "tick.h"

#include <stddef.h>

OutputSchedule m1start;
OutputSchedule m1run;
OutputSchedule m1stop;

static uint32_t parseU32(uint32_t *v, uint8_t *buf, uint32_t pos, uint32_t endPos) {
  if(pos == ~0ul) return pos;

  *v = 0;
  while(buf[pos] == ' ' && pos < endPos) ++pos;
  if(!(buf[pos] >= '0' && buf[pos] <= '9')) return ~0ul;

  while(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9') {
    *v = 10 * *v + (buf[pos] - '0');
    ++pos;
  }

  return pos;
}

static void m1SingleStep() {
  m1start.count = 1;
  m1start.timer = ~0ul;
  m1start.dt = ~0ul;
  m1start.next = NULL;

  m1sched = &m1start;
}

uint32_t m1resolution = 0;
uint32_t m1power = 24;

static void updateM1() {
  setupMotor(m1resolution, m1power);

  if(m1resolution == 0) console_send_str("256 microsteps");
  if(m1resolution == 1) console_send_str("128 microsteps");
  if(m1resolution == 2) console_send_str("64 microsteps");
  if(m1resolution == 3) console_send_str("32 microsteps");
  if(m1resolution == 4) console_send_str("16 microsteps");
  if(m1resolution == 5) console_send_str("8 microsteps");
  if(m1resolution == 6) console_send_str("4 microsteps");
  if(m1resolution == 7) console_send_str("2 microsteps");
  if(m1resolution == 8) console_send_str("no microsteps");

  console_send_str(", power: ");
  console_send_uint8(m1power);
  console_send_str("\r\n");
}

int_fast8_t console_receive(uint8_t *buf, uint_fast8_t buf_len) {
  // console_send((uint8_t *)"\r\nRCV:", 6);
  // console_send(buf, buf_len);

  if(buf[0] == ',') {
    backward(1);
    m1SingleStep();
  }

  if(buf[0] == 'o') {
    forward(1);
    m1SingleStep();
  }

  if(buf[0] == '1') { m1resolution = 0; updateM1(); }
  if(buf[0] == '2') { m1resolution = 1; updateM1(); }
  if(buf[0] == '3') { m1resolution = 2; updateM1(); }
  if(buf[0] == '4') { m1resolution = 3; updateM1(); }
  if(buf[0] == '5') { m1resolution = 4; updateM1(); }
  if(buf[0] == '6') { m1resolution = 5; updateM1(); }
  if(buf[0] == '7') { m1resolution = 6; updateM1(); }
  if(buf[0] == '8') { m1resolution = 7; updateM1(); }
  if(buf[0] == '9') { m1resolution = 8; updateM1(); }
  if(buf[0] == '/') { m1power = 16; updateM1(); }
  if(buf[0] == '-') { m1power = 24; updateM1(); }
  if(buf[0] == '+') { m1power = 28; updateM1(); }
  if(buf[0] == '*') { m1power = 31; updateM1(); }

  if(buf[0] == 's') dumpMotorStatus();
  if(buf[0] == 't') {
    enableSystick();
    console_send_str("SysTick enabled\r\n");
  }
  if(buf[0] == 'm') {
    initMotorDrivers();
    console_send_str("Motors enabled\r\n");
  }

  if(buf[0] == 'M') {
    if(buf[buf_len - 1] != '\n' && buf[buf_len - 1] != '\r') return 0; // wait for more

    int pos = 1;
    pos = parseU32(&m1start.count, buf, pos, buf_len);
    pos = parseU32(&m1start.dt, buf, pos, buf_len);
    pos = parseU32(&m1start.ddt, buf, pos, buf_len);
    pos = parseU32(&m1start.dddt, buf, pos, buf_len);

    pos = parseU32(&m1run.count, buf, pos, buf_len);
    pos = parseU32(&m1run.dt, buf, pos, buf_len);
    pos = parseU32(&m1run.ddt, buf, pos, buf_len);
    pos = parseU32(&m1run.dddt, buf, pos, buf_len);

    pos = parseU32(&m1stop.count, buf, pos, buf_len);
    pos = parseU32(&m1stop.dt, buf, pos, buf_len);
    pos = parseU32(&m1stop.ddt, buf, pos, buf_len);
    pos = parseU32(&m1stop.dddt, buf, pos, buf_len);

    if(pos == ~0ull) {
      console_send_str("Failed to parse\r\n");
      return buf_len;
    }

    console_send_str("Start: ");
    console_send_uint32(m1start.count); console_send_str(" ");
    console_send_uint32(m1start.dt); console_send_str(" ");
    console_send_uint32(m1start.ddt); console_send_str(" ");
    console_send_uint32(m1start.dddt); console_send_str(" \r\n");
    console_send_str("Run: ");
    console_send_uint32(m1run.count); console_send_str(" ");
    console_send_uint32(m1run.dt); console_send_str(" ");
    console_send_uint32(m1run.ddt); console_send_str(" ");
    console_send_uint32(m1run.dddt); console_send_str(" \r\n");
    console_send_str("Stop: ");
    console_send_uint32(m1stop.count); console_send_str(" ");
    console_send_uint32(m1stop.dt); console_send_str(" ");
    console_send_uint32(m1stop.ddt); console_send_str(" ");
    console_send_uint32(m1stop.dddt); console_send_str(" \r\n");

    m1start.next = &m1run;
    m1run.next = &m1stop;
    m1stop.next = NULL;

    m1sched = &m1start;
  }

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
