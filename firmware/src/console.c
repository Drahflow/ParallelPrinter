#include "console.h"

#ifndef TEST
#include "usb.h"
#include "motor.h"
#endif
#include "tick.h"
#include "endstop.h"
#include "homing.h"
#include "kinematics.h"

#include <stddef.h>
#include <string.h>
#include <stdbool.h>

MotorSchedule startTemplate;
MotorSchedule runTemplate;
MotorSchedule stopTemplate;

OutputSchedule start;
OutputSchedule run;
OutputSchedule stop;

OutputSchedule oneStep;

static uint32_t parseU32(uint32_t *v, const uint8_t *buf, uint32_t pos, uint32_t endPos) {
  if(pos == ~0u) return pos;

  *v = 0;
  while(pos < endPos && buf[pos] == ' ') ++pos;
  if(!(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9')) return ~0u;

  while(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9') {
    *v = 10 * *v + (buf[pos] - '0');
    ++pos;
  }

  return pos;
}

static uint32_t parseI32(int32_t *v, const uint8_t *buf, uint32_t pos, uint32_t endPos) {
  if(pos == ~0u) return pos;

  bool negative = false;
  *v = 0;
  while(pos < endPos && buf[pos] == ' ') ++pos;
  if(pos < endPos && buf[pos] == '-') {
    negative = true;
    ++pos;
  }
  if(!(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9')) return ~0u;

  while(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9') {
    *v = 10 * *v + (buf[pos] - '0');
    ++pos;
  }

  if(negative) *v = -*v;

  return pos;
}

static uint32_t parseDouble(double *v, const uint8_t *buf, uint32_t pos, uint32_t endPos) {
  if(pos == ~0u) return pos;

  bool negative = false;
  *v = 0;
  while(pos < endPos && buf[pos] == ' ') ++pos;
  if(pos < endPos && buf[pos] == '-') {
    negative = true;
    ++pos;
  }
  if(!(pos < endPos && ((buf[pos] >= '0' && buf[pos] <= '9') || buf[pos] == '.'))) return ~0u;

  while(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9') {
    *v = 10 * *v + (buf[pos] - '0');
    ++pos;
  }
  if(pos < endPos && buf[pos] == '.') {
    ++pos;
    double digit = .1;
    while(pos < endPos && buf[pos] >= '0' && buf[pos] <= '9') {
      *v = *v + (buf[pos] - '0') * digit;
      digit /= 10;
      ++pos;
    }
  }

  if(negative) *v = -*v;

  return pos;
}

static void singleStep(Motor *m, uint8_t direction) {
  oneStep = noSteps;

  MotorSchedule *motor = oneStep.motors + m->index;
  motor->count = 1;
  motor->timer = ~0u;
  motor->dt = ~0u;
  motor->dir = direction;
  oneStep.next = NULL;

  scheduleMotors(&oneStep);
}

static void multiStep(Motor *m, uint8_t direction) {
  start = noSteps;
  run = noSteps;
  stop = noSteps;

  start.motors[m->index] = startTemplate;
  run.motors[m->index] = runTemplate;
  stop.motors[m->index] = stopTemplate;

  start.motors[m->index].dir = direction;
  run.motors[m->index].dir = direction;
  stop.motors[m->index].dir = direction;

  start.next = &run;
  run.next = &stop;

  scheduleMotors(&start);
}

void interactiveMotor(const uint8_t *buf, Motor *m, char up, char UP, char down, char DOWN) {
  if(buf[0] == up) { singleStep(m, DIR_MAIN_AXIS_UP); }
  if(buf[0] == UP) { multiStep(m, DIR_MAIN_AXIS_UP); }

  if(buf[0] == down) { singleStep(m, DIR_MAIN_AXIS_DOWN); }
  if(buf[0] == DOWN) { multiStep(m, DIR_MAIN_AXIS_DOWN); }
}

uint32_t parseLinearMotionConfig(
  char *name,
  const uint8_t *buf, uint32_t pos, uint32_t buf_len,
  MotorSchedule *start, MotorSchedule *run, MotorSchedule *stop
) {
  pos = parseU32(&start->count, buf, pos, buf_len);
  pos = parseU32(&start->timer, buf, pos, buf_len);
  pos = parseU32(&start->dt, buf, pos, buf_len);
  pos = parseU32(&start->ddt, buf, pos, buf_len);
  pos = parseU32(&start->dddt, buf, pos, buf_len);

  pos = parseU32(&run->count, buf, pos, buf_len);
  pos = parseU32(&run->timer, buf, pos, buf_len);
  pos = parseU32(&run->dt, buf, pos, buf_len);
  pos = parseU32(&run->ddt, buf, pos, buf_len);
  pos = parseU32(&run->dddt, buf, pos, buf_len);

  pos = parseU32(&stop->count, buf, pos, buf_len);
  pos = parseU32(&stop->timer, buf, pos, buf_len);
  pos = parseU32(&stop->dt, buf, pos, buf_len);
  pos = parseU32(&stop->ddt, buf, pos, buf_len);
  pos = parseU32(&stop->dddt, buf, pos, buf_len);

  if(pos == ~0u) {
    console_send_str("Failed to parse\r\n");
    return ~0u;
  }

  console_send_str("New ");
  console_send_str(name);
  console_send_str(" config: Start: ");
  console_send_uint32(start->count); console_send_str(" ");
  console_send_uint32(start->timer); console_send_str(" ");
  console_send_uint32(start->dt); console_send_str(" ");
  console_send_uint32(start->ddt); console_send_str(" ");
  console_send_uint32(start->dddt); console_send_str("\r\n");
  console_send_str("Run: ");
  console_send_uint32(run->count); console_send_str(" ");
  console_send_uint32(run->timer); console_send_str(" ");
  console_send_uint32(run->dt); console_send_str(" ");
  console_send_uint32(run->ddt); console_send_str(" ");
  console_send_uint32(run->dddt); console_send_str("\r\n");
  console_send_str("Stop: ");
  console_send_uint32(stop->count); console_send_str(" ");
  console_send_uint32(stop->timer); console_send_str(" ");
  console_send_uint32(stop->dt); console_send_str(" ");
  console_send_uint32(stop->ddt); console_send_str(" ");
  console_send_uint32(stop->dddt); console_send_str("\r\n");

  return pos;
}

uint32_t parseMotorScheduleConfig(
  char *name,
  const uint8_t *buf, uint32_t pos, uint32_t buf_len,
  MotorSchedule *schedule
) {
  pos = parseU32(&schedule->count, buf, pos, buf_len);
  pos = parseU32(&schedule->timer, buf, pos, buf_len);
  pos = parseU32(&schedule->dt, buf, pos, buf_len);
  pos = parseU32(&schedule->ddt, buf, pos, buf_len);
  pos = parseU32(&schedule->dddt, buf, pos, buf_len);

  if(pos == ~0u) {
    console_send_str("Failed to parse\r\n");
    return ~0u;
  }

  console_send_str("New ");
  console_send_str(name);
  console_send_str(" config: ");
  console_send_uint32(schedule->count); console_send_str(" ");
  console_send_uint32(schedule->timer); console_send_str(" ");
  console_send_uint32(schedule->dt); console_send_str(" ");
  console_send_uint32(schedule->ddt); console_send_str(" ");
  console_send_uint32(schedule->dddt); console_send_str("\r\n");

  return pos;
}

uint32_t parseDisplacement(
  char *name,
  const uint8_t *buf, uint32_t pos, uint32_t buf_len,
  Displacement *disp
) {
  pos = parseDouble(&disp->x, buf, pos, buf_len);
  pos = parseDouble(&disp->y, buf, pos, buf_len);
  pos = parseDouble(&disp->z, buf, pos, buf_len);

  if(pos == ~0u) {
    console_send_str("Failed to parse\r\n");
    return ~0u;
  }

  console_send_str("New ");
  console_send_str(name);
  console_send_str(": ");
  console_send_double(disp->x); console_send_str(" ");
  console_send_double(disp->y); console_send_str(" ");
  console_send_double(disp->z); console_send_str("\r\n");

  return pos;
}

uint32_t parsePosition(
  char *name,
  const uint8_t *buf, uint32_t pos, uint32_t buf_len,
  Position *position
) {
  pos = parseDisplacement(name, buf, pos, buf_len, &position->disp);

  pos = parseDouble(&position->rot.r, buf, pos, buf_len);
  pos = parseDouble(&position->rot.i, buf, pos, buf_len);
  pos = parseDouble(&position->rot.j, buf, pos, buf_len);
  pos = parseDouble(&position->rot.k, buf, pos, buf_len);

  if(pos == ~0u) {
    console_send_str("Failed to parse\r\n");
    return ~0u;
  }

  console_send_str("New ");
  console_send_str(name);
  console_send_str(" rotation: ");
  console_send_double(position->rot.r); console_send_str(" ");
  console_send_double(position->rot.i); console_send_str(" ");
  console_send_double(position->rot.j); console_send_str(" ");
  console_send_double(position->rot.k); console_send_str("\r\n");

  return pos;
}

uint32_t rawMoveAxis(const uint8_t *buf, uint32_t pos, uint32_t buf_len, uint8_t dir) {
  uint32_t axis = -1;
  pos = parseU32(&axis, buf, pos, buf_len);

  if(axis >= MOTOR_COUNT) {
    console_send_str("Axis index too large.\r\n");
    return ~0u;
  }

  start = noSteps;
  run = noSteps;
  stop = noSteps;

  pos = parseLinearMotionConfig("Step", buf, pos, buf_len, &start.motors[axis], &run.motors[axis], &stop.motors[axis]);

  if(pos == ~0u) {
    console_send_str("Not moving after parse problem.\r\n");
    return ~0u;
  }

  start.motors[axis].dir = dir;
  run.motors[axis].dir = dir;
  stop.motors[axis].dir = dir;

  start.next = &run;
  run.next = &stop;
  stop.next = NULL;

  scheduleMotors(&start);

  return pos;
}

static uint_fast8_t echoed = 0;
static bool interactive = false;
int_fast8_t console_receive(uint8_t *buf, uint_fast8_t buf_len) {
  // console_send((uint8_t *)"\r\nRCV:", 6);
  // console_send(buf, buf_len);

  if(interactive) {
    interactiveMotor(buf, motors + 0, ';', ':', 'a', 'A');
    interactiveMotor(buf, motors + 1, ',', '<', 'o', 'O');
    interactiveMotor(buf, motors + 2, '.', '>', 'e', 'E');
    interactiveMotor(buf, motors + 3, 'p', 'P', 'u', 'U');
    interactiveMotor(buf, motors + 4, 'y', 'Y', 'i', 'I');
    interactiveMotor(buf, motors + 5, 'f', 'F', 'd', 'D');
    interactiveMotor(buf, motors + 6, 'g', 'G', 'h', 'H');

    if(buf[0] == ' ') {
      console_send_str("Interactive mode off.\r\n");
      interactive = false;
    }

    return buf_len;
  }

  console_send(buf + echoed, buf_len - echoed);
  echoed = buf_len;

  if(buf[buf_len - 1] != '\n' && buf[buf_len - 1] != '\r') {
    return 0; // wait for more
  }

  echoed = 0;

  char *cmd = (char *)buf;
  int cmdEnd;
  for(cmdEnd = 0; cmdEnd < buf_len && buf[cmdEnd] > ' '; ++cmdEnd);
  buf[cmdEnd] = '\0';

  if(strncmp(cmd, "status:motors", buf_len) == 0) {
    dumpMotorStatus(motors + 0);
    dumpMotorStatus(motors + 1);
    dumpMotorStatus(motors + 2);
    dumpMotorStatus(motors + 3);
    dumpMotorStatus(motors + 4);
    dumpMotorStatus(motors + 5);
    dumpMotorStatus(motors + 6);
  }

  if(strncmp(cmd, "status:schedule", buf_len) == 0) {
    dumpScheduleStatus();
  }

  if(strncmp(cmd, "interactive", buf_len) == 0) {
    interactive = true;
    console_send_str("Interactive mode\r\n");
  }

  if(strncmp(cmd, "tick:on", buf_len) == 0) {
    enableSystick();
    console_send_str("SysTick enabled\r\n");
  }
  if(strncmp(cmd, "tick:off", buf_len) == 0) {
    disableSystick();
    console_send_str("SysTick disabled\r\n");
  }

  if(strncmp(cmd, "motors:on", buf_len) == 0) {
    initMotorDrivers();
    console_send_str("Motors enabled\r\n");
  }

  if(strncmp(cmd, "endstop:on", buf_len) == 0) {
    endstopOn();
    console_send_str("Endstop enabled\r\n");
  }
  if(strncmp(cmd, "endstop:off", buf_len) == 0) {
    endstopOff();
    console_send_str("Endstop disabled\r\n");
  }
  if(strncmp(cmd, "endstop:scan", buf_len) == 0) {
    console_send_str("Endstop scanning\r\n");
    endstopScan();
  }

  if(strncmp(cmd, "stop", buf_len) == 0) {
    homingStop();
    kinematicsStop();
  }

  if(strncmp(cmd, "homing:up", buf_len) == 0) {
    homingUpwards();
  }

  if(strncmp(cmd, "move:axis:up", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    rawMoveAxis(buf, pos, buf_len, DIR_MAIN_AXIS_UP);
  }

  if(strncmp(cmd, "move:axis:down", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    rawMoveAxis(buf, pos, buf_len, DIR_MAIN_AXIS_DOWN);
  }

  if(strncmp(cmd, "config:interactive:big_step", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    parseLinearMotionConfig("Big Step", buf, pos, buf_len, &startTemplate, &runTemplate, &stopTemplate);
  }

  if(strncmp(cmd, "config:homing:threshold", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    pos = parseU32(&homingThresholdInitialRevert, buf, pos, buf_len);
    pos = parseU32(&homingThresholdMinimumAxisEffect, buf, pos, buf_len);
    pos = parseU32(&homingThresholdFineScan, buf, pos, buf_len);
    pos = parseU32(&homingThresholdSingleAxisScan, buf, pos, buf_len);
    pos = parseU32(&homingThresholdInitialScan, buf, pos, buf_len);
    pos = parseU32(&homingThresholdRescan, buf, pos, buf_len);

    console_send_str("New homing thresholds:\r\n");
    console_send_uint32(homingThresholdInitialRevert); console_send_str("\r\n");
    console_send_uint32(homingThresholdMinimumAxisEffect); console_send_str("\r\n");
    console_send_uint32(homingThresholdFineScan); console_send_str("\r\n");
    console_send_uint32(homingThresholdSingleAxisScan); console_send_str("\r\n");
    console_send_uint32(homingThresholdInitialScan); console_send_str("\r\n");
    console_send_uint32(homingThresholdRescan); console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:endstop:init_duration", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    pos = parseU32(&endstopInitDuration, buf, pos, buf_len);

    console_send_str("New endstop init duration: ");
    console_send_uint32(endstopInitDuration);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:homing:scan", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    parseMotorScheduleConfig("Homing Step", buf, pos, buf_len, &homingStep);
  }

  if(strncmp(cmd, "config:homing:clear", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    parseMotorScheduleConfig("Clearing Step", buf, pos, buf_len, &homingClearingStep);
  }

  if(strncmp(cmd, "config:homing:fine", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    parseMotorScheduleConfig("Homing Fine Step", buf, pos, buf_len, &homingFineStep);
  }

  if(strncmp(cmd, "config:motor", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t motor;
    uint32_t steps;
    uint32_t power;

    pos = parseU32(&motor, buf, pos, buf_len);
    pos = parseU32(&steps, buf, pos, buf_len);
    pos = parseU32(&power, buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Failed to parse\r\n");
      return buf_len;
    }

    if(motor > 7) {
      console_send_str("Invalid motor index\r\n");
      return buf_len;
    }

    switch(steps) {
      case 256: steps = 0; break;
      case 128: steps = 1; break;
      case 64: steps = 2; break;
      case 32: steps = 3; break;
      case 16: steps = 4; break;
      case 8: steps = 5; break;
      case 4: steps = 6; break;
      case 2: steps = 7; break;
      case 1: steps = 8; break;
      default:
        console_send_str("Invalid microstep resolution\r\n");
        return buf_len;
    }

    if(power > 31) {
      console_send_str("Invalid motor power\r\n");
      return buf_len;
    }

    setupMotor(motors + motor, steps, power);

    console_send_str("Motor ");
    console_send_uint8(motor);

    if(steps == 0) console_send_str(": 256 microsteps");
    if(steps == 1) console_send_str(": 128 microsteps");
    if(steps == 2) console_send_str(": 64 microsteps");
    if(steps == 3) console_send_str(": 32 microsteps");
    if(steps == 4) console_send_str(": 16 microsteps");
    if(steps == 5) console_send_str(": 8 microsteps");
    if(steps == 6) console_send_str(": 4 microsteps");
    if(steps == 7) console_send_str(": 2 microsteps");
    if(steps == 8) console_send_str(": no microsteps");

    console_send_str(", power: ");
    console_send_uint8(power);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:slider:zero", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseDisplacement("zero", buf, pos, buf_len, &sliderZero[axis]);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Slider zero configured on axis ");
    console_send_uint8(axis);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:slider:up", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseDisplacement("up", buf, pos, buf_len, &sliderUpStep[axis]);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Slider up vector configured on axis ");
    console_send_uint8(axis);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:strut", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseDouble(&strutLength[axis], buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Strut length configured on axis ");
    console_send_uint8(axis);
    console_send_str(" to be ");
    console_send_double(strutLength[axis]);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:strut:backslash", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseDouble(&sliderBackslash[axis], buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Strut backslash configured on axis ");
    console_send_uint8(axis);
    console_send_str(" to be ");
    console_send_double(sliderBackslash[axis]);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:strut:elasticity", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseDouble(&sliderElasticity[axis], buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Strut elasticity configured on axis ");
    console_send_uint8(axis);
    console_send_str(" to be ");
    console_send_double(sliderElasticity[axis]);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:limit:upper", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseI32(&upperLimit[axis], buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Upper step limit on axis ");
    console_send_uint8(axis);
    console_send_str(" now ");
    console_send_int32(upperLimit[axis]);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:limit:lower", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseI32(&lowerLimit[axis], buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Lower step limit on axis ");
    console_send_uint8(axis);
    console_send_str(" now ");
    console_send_int32(lowerLimit[axis]);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:limit:steps_per_second", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    pos = parseDouble(&maximumStepsPerSecond, buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Maximum steps per second now ");
    console_send_uint32(maximumStepsPerSecond);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:limit:force", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseDouble(&forceLimit[axis], buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Force limit on axis ");
    console_send_uint8(axis);
    console_send_str(" now ");
    console_send_double(forceLimit[axis]);
    console_send_str("\r\n");

    checkForceLimiting();
  }

  if(strncmp(cmd, "config:kinematics:interval", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    pos = parseDouble(&kinematicsSubdivisionInterval, buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Kinematics subdivision interval now ");
    console_send_uint32(kinematicsSubdivisionInterval);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "config:kinematics:attachment", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    uint32_t axis;
    pos = parseU32(&axis, buf, pos, buf_len);

    if(axis >= MAIN_AXIS_COUNT) {
      console_send_str("Axis index too large.\r\n");
      return buf_len;
    }

    pos = parseDisplacement("platform attachment", buf, pos, buf_len, &platformAttachment[axis]);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    console_send_str("Platform attachment configured on axis ");
    console_send_uint8(axis);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "kinematics:zero", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    Position initialTool;
    pos = parsePosition("tool zero", buf, pos, buf_len, &initialTool);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    setZero(initialTool);
    console_send_str("Kinematics zeroed and enabled.\r\n");
  }

  if(strncmp(cmd, "kinematics:again", buf_len) == 0) {
    moveAgain();
    console_send_str("Moving to current position again.\r\n");
  }

  if(strncmp(cmd, "tool:attached", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    Position toolAttachment;
    pos = parsePosition("tool attachment", buf, pos, buf_len, &toolAttachment);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    setToolAttachedAt(toolAttachment);
    console_send_str("Tool attachment set.\r\n");
  }

  if(strncmp(cmd, "tool:move", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    Position target;
    pos = parsePosition("target", buf, pos, buf_len, &target);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    moveTo(target);
    console_send_str("Moving.\r\n");
  }

  if(strncmp(cmd, "tool:speed:movement", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    double speed;
    pos = parseDouble(&speed, buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    setMovementSpeed(speed);
    console_send_str("Movement speed now ");
    console_send_double(speed);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "tool:speed:rotation", buf_len) == 0) {
    uint32_t pos = cmdEnd + 1;

    double speed;
    pos = parseDouble(&speed, buf, pos, buf_len);

    if(pos == ~0u) {
      console_send_str("Parse problem.\r\n");
      return buf_len;
    }

    setRotationSpeed(speed);
    console_send_str("Rotation speed now ");
    console_send_double(speed);
    console_send_str("\r\n");
  }

  if(strncmp(cmd, "debug:endstop:on", buf_len) == 0) {
    endstopDebug = true;
    console_send_str("Endstop debug enabled\r\n");
  }

  if(strncmp(cmd, "debug:endstop:off", buf_len) == 0) {
    endstopDebug = false;
    console_send_str("Endstop debug disabled\r\n");
  }

  if(strncmp(cmd, "debug:homing:on", buf_len) == 0) {
    homingDebug = true;
    console_send_str("Homing debug enabled\r\n");
  }

  if(strncmp(cmd, "debug:homing:off", buf_len) == 0) {
    homingDebug = false;
    console_send_str("Homing debug disabled\r\n");
  }

  return buf_len;
}

void console_send(const uint8_t *buf, uint_fast8_t buf_len) {
  usb_console_send(buf, buf_len);
}

void console_send_str(char *str) {
  char *e = str;
  while(*e) ++e;

  console_send((uint8_t *)str, e - str);
}

void console_send_uint32(uint32_t n) {
  uint8_t buf[48];
  uint32_t p = 34;
  uint32_t val = n;
  for(uint32_t i = 0; i < 32; ++i) {
    buf[p--] = (val & 1)? '1': '0';
    if(i == 7 || i == 15 || i == 23) buf[p--] = ' ';

    val >>= 1;
  }

  buf[35] = ' ';
  buf[36] = '(';
  p = 46;
  val = n;
  for(uint32_t i = 0; i < 10; ++i) {
    buf[p--] = '0' + (val % 10);
    val /= 10;
  }

  buf[47] = ')';

  console_send(buf, sizeof(buf));
}

void console_send_int32(int32_t n) {
  if(n < 0) {
    console_send_str("-");
    console_send_uint32(-n);
  } else {
    console_send_uint32(n);
  }
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

void console_send_double(double d) {
  if(d == 0) {
    console_send_str("0");
    return;
  }

  if(d < 0) {
    console_send_str("-");
    d = -d;
  }

  const int maxDigits = 16;
  double v = 0.999999999999999;

  int i;
  for(i = 0; v < d && i < maxDigits; ++i, v *= 10);
  if(i == maxDigits) {
    console_send_str("inf");
    return;
  }
  v /= 10;

  uint8_t buf[20];
  uint8_t *t = buf;
  while(v > .5) {
    *t++ = '0' + (d / v);
    d -= (int32_t)(d / v) * v;
    if(d < 0) d = 0;

    v /= 10;
  }

  if(d > 0) {
    *t++ = '.';

    while(d > 0 && t - buf < maxDigits + 1) {
      *t++ = '0' + (d / v);
      d -= (int32_t)(d / v) * v;
      v /= 10;
    }
  }
  
  console_send(buf, t - buf);
}
