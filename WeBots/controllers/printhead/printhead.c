/*
 * File:          printhead.c
 * Date:
 * Description:
 * Author:
 * Modifications:
 */

/*
 * You may need to add include files like <webots/distance_sensor.h> or
 * <webots/motor.h>, etc.
 */
#include <webots/robot.h>
#include <webots/motor.h>
#include <webots/keyboard.h>
#include <stdio.h>

/*
 * You may want to add macros here.
 */
#define TIME_STEP 8
#define NUM_DRIVES 8
#define MAX_VELOCITY 0.05

typedef struct {
  float x, y, z;
} vec3;

typedef struct {
  vec3 s, e, a;
  float mp;
  WbDeviceTag m;
  vec3 r;
  vec3 strut;
  float sLen;
  float rLen;
  bool fromBelow;
} drive;

typedef struct {
  vec3 p;
  float alpha, beta, gamma;
} platform;

float dot(vec3 *a, vec3 *b) {
  return a->x * b->x + a->y * b->y + a->z * b->z;
}

/*
 * This is the main program.
 * The arguments of the main function can be specified by the
 * "controllerArgs" field of the Robot node
 */
int main(int argc, char **argv) {
  /* necessary to initialize webots stuff */
  wb_robot_init();
  wb_keyboard_enable(100);

  /*
   * You should declare here WbDeviceTag variables for storing
   * robot devices like this:
   *  WbDeviceTag my_sensor = wb_robot_get_device("my_sensor");
   *  WbDeviceTag my_actuator = wb_robot_get_device("my_actuator");
   */
  drive drives[NUM_DRIVES] = {
    { // m1
      .s = { .x = 0, .y = 0, .z = 0.05, },
      .e = { .x = 0, .y = -0.9, .z = 0.05, },
      .a = { .x = 0.1, .y = -0.9915151, .z = 0.15, },
      .mp = 0.075,
      .m = wb_robot_get_device("m1"),
    },
    { // m2
      .s = { .x = 0.1, .y = 0, .z = 0.05, },
      .e = { .x = 0.1, .y = -0.9, .z = 0.05, },
      .a = { .x = 0.2, .y = -0.95, .z = 0.25, },
      .mp = 0.05,
      .m = wb_robot_get_device("m2"),
    },
    { // m3
      .s = { .x = 0.2, .y = 0, .z = 0.05, },
      .e = { .x = 0.2, .y = -0.9, .z = 0.05, },
      .a = { .x = 0.3, .y = -1.0165151, .z = 0.15, },
      .mp = 0.1,
      .m = wb_robot_get_device("m3"),
    },
    { // m4
      .s = { .x = 0.3, .y = 0, .z = 0.05, },
      .e = { .x = 0.3, .y = -0.9, .z = 0.05, },
      .a = { .x = 0.3, .y = -0.905538, .z = 0.25, },
      .mp = 0,
      .m = wb_robot_get_device("m4"),
    },
    { // m5
      .s = { .x = 0.4, .y = 0, .z = 0.05, },
      .e = { .x = 0.4, .y = -0.9, .z = 0.05, },
      .a = { .x = 0.4, .y = -0.905538, .z = 0.25, },
      .mp = 0,
      .m = wb_robot_get_device("m5"),
    },
    { // m6
      .s = { .x = 0.5, .y = 0, .z = 0.05, },
      .e = { .x = 0.5, .y = -0.9, .z = 0.05, },
      .a = { .x = 0.4, .y = -1.0165151, .z = 0.15, },
      .mp = 0.1,
      .m = wb_robot_get_device("m6"),
    },
    { // m7
      .s = { .x = 0.6, .y = 0, .z = 0.05, },
      .e = { .x = 0.6, .y = -0.9, .z = 0.05, },
      .a = { .x = 0.5, .y = -0.95, .z = 0.25, },
      .mp = 0.05,
      .m = wb_robot_get_device("m7"),
    },
    { // m8
      .s = { .x = 0.7, .y = 0, .z = 0.05, },
      .e = { .x = 0.7, .y = -0.9, .z = 0.05, },
      .a = { .x = 0.6, .y = -0.9915151, .z = 0.15, },
      .mp = 0.075,
      .m = wb_robot_get_device("m8"),
    },
  };

  for(int i = 0; i < NUM_DRIVES; ++i) {
    drives[i].r.x = drives[i].e.x - drives[i].s.x;
    drives[i].r.y = drives[i].e.y - drives[i].s.y;
    drives[i].r.z = drives[i].e.z - drives[i].s.z;

    drives[i].rLen = sqrtf(dot(&drives[i].r, &drives[i].r));

    drives[i].r.x /= drives[i].rLen;
    drives[i].r.y /= drives[i].rLen;
    drives[i].r.z /= drives[i].rLen;
    
    drives[i].strut.x = drives[i].a.x - drives[i].s.x - drives[i].mp * drives[i].r.x;
    drives[i].strut.y = drives[i].a.y - drives[i].s.y - drives[i].mp * drives[i].r.y;
    drives[i].strut.z = drives[i].a.z - drives[i].s.z - drives[i].mp * drives[i].r.z;
    
    drives[i].sLen = sqrtf(dot(&drives[i].strut, &drives[i].strut));
    drives[i].fromBelow = false;
  }

  platform initial = {
    .p = { .x = 0.35, .y = -0.95, .z = 0.15 },
    .alpha = 0,
    .beta = 0,
    .gamma = 0,
  };
  platform target = {
    .p = { .x = 0.35, .y = -0.95, .z = 0.15 },
    .alpha = 0,
    .beta = 0,
    .gamma = 0,
  };

  /* main loop
   * Perform simulation steps of TIME_STEP milliseconds
   * and leave the loop when the simulation is over
   */
  while (wb_robot_step(TIME_STEP) != -1) {
    /*
     * Read the sensors :
     * Enter here functions to read sensor data, like:
     *  double val = wb_distance_sensor_get_value(my_sensor);
     */

    /* Process sensor data here */

    /*
     * Enter here functions to send actuator commands, like:
     * wb_motor_set_position(my_actuator, 10.0);
     */
    
    printf("Target: %f,%f,%f %f %f %f\n", target.p.x, target.p.y, target.p.z, target.alpha, target.beta, target.gamma);
    
    bool valid = true;
    float positions[NUM_DRIVES];

    for(int i = 0; i < NUM_DRIVES; ++i) {
      vec3 d0 = {
        .x = drives[i].a.x - initial.p.x,
        .y = drives[i].a.y - initial.p.y,
        .z = drives[i].a.z - initial.p.z,
      };

      vec3 d1 = {
        .x = d0.x,
        .y = cos(target.alpha) * d0.y - sin(target.alpha) * d0.z,
        .z = sin(target.alpha) * d0.y + cos(target.alpha) * d0.z,
      };

      vec3 d2 = {
        .x = cos(target.beta) * d1.x - sin(target.beta) * d1.y,
        .y = sin(target.beta) * d1.x + cos(target.beta) * d1.y,
        .z = d1.z,
      };

      vec3 d3 = {
        .x = cos(target.gamma) * d2.x - sin(target.gamma) * d2.z,
        .y = d2.y,
        .z = sin(target.gamma) * d2.x + cos(target.gamma) * d2.z,
      };

      vec3 d = {
        .x = target.p.x + d3.x,
        .y = target.p.y + d3.y,
        .z = target.p.z + d3.z,
      };

      d.x -= drives[i].s.x;
      d.y -= drives[i].s.y;
      d.z -= drives[i].s.z;

      float dotDD = dot(&d, &d);
      float dotRR = dot(&drives[i].r, &drives[i].r);
      float dotDR = dot(&d, &drives[i].r);

      float l1 = (dotDR - sqrtf(dotDR * dotDR - dotRR * (dotDD - drives[i].sLen * drives[i].sLen))) / dotRR;
      float l2 = (dotDR + sqrtf(dotDR * dotDR - dotRR * (dotDD - drives[i].sLen * drives[i].sLen))) / dotRR;

      printf("drive[%d]: %c%f / %c%f\n", i, drives[i].fromBelow? ' ': '*', l1, drives[i].fromBelow? '*': ' ', l2);
      
      if(0 <= l1 && l1 <= drives[i].rLen && 0 <= l2 && l2 <= drives[i].rLen) {
        printf("^^^^ Two solutions\n");
      }

      if(!drives[i].fromBelow) {
        if(0 <= l1 && l1 <= drives[i].rLen) {
          positions[i] = l1;
        } else {
          printf("^^^^ No solution.\n");
          valid = false;
        }
      } else {
        if(0 <= l2 && l2 <= drives[i].rLen) {
          positions[i] = l2;
        } else {
          printf("^^^^ No solution.\n");
          valid = false;
        }
      }
    }
    
    if(valid) {
      for(int i = 0; i < NUM_DRIVES; ++i) {
        wb_motor_set_position(drives[i].m, positions[i]);
      }
    }
    
    int k;
    while((k = wb_keyboard_get_key()) != -1) {
      if(k == 'A') target.p.x -= 0.001;
      if(k == 'E') target.p.x += 0.001;
      if(k == 'O') target.p.y -= 0.001;
      if(k == ',') target.p.y += 0.001;
      if(k == ';') target.p.z -= 0.001;
      if(k == '.') target.p.z += 0.001;
      if(k == 'D') target.beta -= 0.01;
      if(k == 'U') target.beta += 0.01;
      if(k == 'Y') target.gamma -= 0.01;
      if(k == 'I') target.gamma += 0.01;
      if(k == 'P') target.alpha -= 0.01;
      if(k == 'F') target.alpha += 0.01;

      if((k & 0xff) == '0') drives[0].fromBelow = !drives[0].fromBelow;
      if((k & 0xff) == '1') drives[1].fromBelow = !drives[1].fromBelow;
      if((k & 0xff) == '2') drives[2].fromBelow = !drives[2].fromBelow;
      if((k & 0xff) == '3') drives[3].fromBelow = !drives[3].fromBelow;
      if((k & 0xff) == '4') drives[4].fromBelow = !drives[4].fromBelow;
      if((k & 0xff) == '5') drives[5].fromBelow = !drives[5].fromBelow;
      if((k & 0xff) == '6') drives[6].fromBelow = !drives[6].fromBelow;
      if((k & 0xff) == '7') drives[7].fromBelow = !drives[7].fromBelow;
    }
  };

  /* Enter your cleanup code here */

  /* This is necessary to cleanup webots resources */
  wb_keyboard_disable();
  wb_robot_cleanup();

  return 0;
}
