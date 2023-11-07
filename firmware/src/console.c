#include "console.h"

#include "usb.h"
#include "motor.h"
#include "tick.h"

#include <stddef.h>
#include <string.h>

OutputSchedule startTemplate;
OutputSchedule runTemplate;
OutputSchedule stopTemplate;

OutputSchedule start[MOTOR_COUNT];
OutputSchedule run[MOTOR_COUNT];
OutputSchedule stop[MOTOR_COUNT];

OutputSchedule singleSteps[MOTOR_COUNT];

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

static void singleStep(Motor *m) {
  OutputSchedule *schedule = singleSteps + m->index;
  schedule->count = 1;
  schedule->timer = ~0ul;
  schedule->dt = ~0ul;
  schedule->next = NULL;

  scheduleMotor(m->index, schedule);
}

static void multiStep(Motor *m) {
  memcpy(&start[m->index], &startTemplate, sizeof(OutputSchedule));
  memcpy(&run[m->index], &runTemplate, sizeof(OutputSchedule));
  memcpy(&stop[m->index], &stopTemplate, sizeof(OutputSchedule));

  start[m->index].next = &run[m->index];
  run[m->index].next = &stop[m->index];
  stop[m->index].next = NULL;

  scheduleMotor(m->index, &start[m->index]);
}

uint32_t motorResolution = 0;
uint32_t motorPower = 8;

static void updateMotors() {
  setupMotor(motors + 0, motorResolution, motorPower);
  setupMotor(motors + 1, motorResolution, motorPower);
  setupMotor(motors + 2, motorResolution, motorPower);
  setupMotor(motors + 3, motorResolution, motorPower);
  setupMotor(motors + 4, motorResolution, motorPower);
  setupMotor(motors + 5, motorResolution, motorPower);
  setupMotor(motors + 6, motorResolution, motorPower);

  if(motorResolution == 0) console_send_str("256 microsteps");
  if(motorResolution == 1) console_send_str("128 microsteps");
  if(motorResolution == 2) console_send_str("64 microsteps");
  if(motorResolution == 3) console_send_str("32 microsteps");
  if(motorResolution == 4) console_send_str("16 microsteps");
  if(motorResolution == 5) console_send_str("8 microsteps");
  if(motorResolution == 6) console_send_str("4 microsteps");
  if(motorResolution == 7) console_send_str("2 microsteps");
  if(motorResolution == 8) console_send_str("no microsteps");

  console_send_str(", power: ");
  console_send_uint8(motorPower);
  console_send_str("\r\n");
}

void interactiveMotor(uint8_t *buf, Motor *m, char up, char UP, char down, char DOWN) {
  if(buf[0] == up) { backward(m); singleStep(m); }
  if(buf[0] == UP) { backward(m); multiStep(m); }

  if(buf[0] == down) { forward(m); singleStep(m); }
  if(buf[0] == DOWN) { forward(m); multiStep(m); }
}

int_fast8_t console_receive(uint8_t *buf, uint_fast8_t buf_len) {
  // console_send((uint8_t *)"\r\nRCV:", 6);
  // console_send(buf, buf_len);

  interactiveMotor(buf, motors + 0, ';', ':', 'a', 'A');
  interactiveMotor(buf, motors + 1, ',', '<', 'o', 'O');
  interactiveMotor(buf, motors + 2, '.', '>', 'e', 'E');
  interactiveMotor(buf, motors + 3, 'p', 'P', 'u', 'U');
  interactiveMotor(buf, motors + 4, 'y', 'Y', 'i', 'I');
  interactiveMotor(buf, motors + 5, 'f', 'F', 'd', 'D');
  interactiveMotor(buf, motors + 6, 'g', 'G', 'h', 'H');

  if(buf[0] == '1') { motorResolution = 0; updateMotors(); }
  if(buf[0] == '2') { motorResolution = 1; updateMotors(); }
  if(buf[0] == '3') { motorResolution = 2; updateMotors(); }
  if(buf[0] == '4') { motorResolution = 3; updateMotors(); }
  if(buf[0] == '5') { motorResolution = 4; updateMotors(); }
  if(buf[0] == '6') { motorResolution = 5; updateMotors(); }
  if(buf[0] == '7') { motorResolution = 6; updateMotors(); }
  if(buf[0] == '8') { motorResolution = 7; updateMotors(); }
  if(buf[0] == '9') { motorResolution = 8; updateMotors(); }
  if(buf[0] == '/') { motorPower = 8; updateMotors(); }
  if(buf[0] == '-') { motorPower = 16; updateMotors(); }
  if(buf[0] == '+') { motorPower = 20; updateMotors(); }
  if(buf[0] == '*') { motorPower = 24; updateMotors(); }

  if(buf[0] == 's') {
    dumpMotorStatus(motors + 0);
    dumpMotorStatus(motors + 1);
    dumpMotorStatus(motors + 2);
    dumpMotorStatus(motors + 3);
    dumpMotorStatus(motors + 4);
    dumpMotorStatus(motors + 5);
    dumpMotorStatus(motors + 6);
  }

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
    pos = parseU32(&startTemplate.count, buf, pos, buf_len);
    pos = parseU32(&startTemplate.dt, buf, pos, buf_len);
    pos = parseU32(&startTemplate.ddt, buf, pos, buf_len);
    pos = parseU32(&startTemplate.dddt, buf, pos, buf_len);

    pos = parseU32(&runTemplate.count, buf, pos, buf_len);
    pos = parseU32(&runTemplate.dt, buf, pos, buf_len);
    pos = parseU32(&runTemplate.ddt, buf, pos, buf_len);
    pos = parseU32(&runTemplate.dddt, buf, pos, buf_len);

    pos = parseU32(&stopTemplate.count, buf, pos, buf_len);
    pos = parseU32(&stopTemplate.dt, buf, pos, buf_len);
    pos = parseU32(&stopTemplate.ddt, buf, pos, buf_len);
    pos = parseU32(&stopTemplate.dddt, buf, pos, buf_len);

    if(pos == ~0ull) {
      console_send_str("Failed to parse\r\n");
      return buf_len;
    }

    console_send_str("Start: ");
    console_send_uint32(startTemplate.count); console_send_str(" ");
    console_send_uint32(startTemplate.dt); console_send_str(" ");
    console_send_uint32(startTemplate.ddt); console_send_str(" ");
    console_send_uint32(startTemplate.dddt); console_send_str(" \r\n");
    console_send_str("Run: ");
    console_send_uint32(runTemplate.count); console_send_str(" ");
    console_send_uint32(runTemplate.dt); console_send_str(" ");
    console_send_uint32(runTemplate.ddt); console_send_str(" ");
    console_send_uint32(runTemplate.dddt); console_send_str(" \r\n");
    console_send_str("Stop: ");
    console_send_uint32(stopTemplate.count); console_send_str(" ");
    console_send_uint32(stopTemplate.dt); console_send_str(" ");
    console_send_uint32(stopTemplate.ddt); console_send_str(" ");
    console_send_uint32(stopTemplate.dddt); console_send_str(" \r\n");
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

void console_send_uint8(uint32_t n) {
  uint8_t buf[14];
  uint32_t p = 7;
  uint32_t val = n;
  for(uint32_t i = 0; i < 8; ++i) {
    buf[p--] = (val & 1)? '1': '0';
    val >>= 1;
  }

  buf[8] = ' ';
  buf[9] = '(';
  p = 12;
  val = n;
  for(uint32_t i = 0; i < 3; ++i) {
    buf[p--] = '0' + (val % 10);
    val /= 10;
  }

  buf[13] = ')';

  console_send(buf, sizeof(buf));
}
