#define TEST

#include <stdint.h>
#include <stdbool.h>
#include <stdio.h>

#include "../src/config.h"
#include "../src/tick.h"

typedef struct {
  uint32_t index;

  int pos;
} Motor;
Motor motors[MOTOR_COUNT];

void dumpMotorStatus(Motor *) { }
void dumpScheduleStatus() { }
void initMotorDrivers() {
  for(int i = 0; i < MOTOR_COUNT; ++i) {
    motors[i].index = i;
    motors[i].pos = 100 - i * 10;
  }
}
void setupMotor(Motor *m, int steps, int power) {
  printf("Motor %d: %d steps, %d power\n", m->index, steps, power);
}

void usb_console_send(const uint8_t *buf, uint32_t len) {
  printf("%.*s", len, buf);
}
void enableSystick() { }
void disableSystick() { }

OutputSchedule *motorSchedule;
void scheduleMotors(OutputSchedule *schedule) {
  motorSchedule = schedule;
}

bool endstopDebug;
int endstopCharge;
uint32_t endstopDuration;
uint32_t endstopWaitDuration;
uint32_t endstopInitDuration;

void endstopOn() { }
void endstopOff() { }
void endstopFloat() { }

void endstopScan() {
  endstopDuration = 0;
  endstopCharge = 100;
}

bool endstopInitializing() {
  return endstopCharge > 95;
}

bool endstopScanning() {
  return endstopCharge > 0;
}

bool motorsMoving() {
  return motorSchedule;
}

static uint64_t globalTime = 0;
void simulate() {
  globalTime += 1;
  if(globalTime % CONFIG_TICK_FREQ == 0) {
    printf("%ld seconds elapsed\n", globalTime / CONFIG_TICK_FREQ);
  }

  if(motorSchedule) {
    bool moreSteps = false;
    for(int i = 0; i < MOTOR_COUNT; ++i) {
      MotorSchedule *schedule = motorSchedule->motors + i;
      if(!schedule->count) continue;

      uint32_t oldTimer = schedule->timer;
      schedule->timer += schedule->dt;
      schedule->dt += schedule->ddt;
      schedule->ddt += schedule->dddt;

      if(schedule->timer < oldTimer) {
        if(schedule->dir == DIR_MAIN_AXIS_UP) motors[i].pos++;
        if(schedule->dir == DIR_MAIN_AXIS_DOWN) motors[i].pos--;

        --schedule->count;
      }

      if(!schedule->count) continue;
      moreSteps = true;
    }

    if(!moreSteps) {
      motorSchedule->completed = 1;
      motorSchedule = motorSchedule->next;
      printf("Moving to motorSchedule %p\n", motorSchedule);
    }
  }

  bool blocked = false;
  for(int i = 0; i < MOTOR_COUNT; ++i) {
    if(motors[i].pos > 100) blocked = true;
  }

  if(endstopCharge >= 0) endstopCharge -= blocked? 1: 50;
  ++endstopDuration;
}

#include "../src/console.c"
#include "../src/homing.c"
#include "../src/no_step.c"
#include "../src/kinematics.c"
