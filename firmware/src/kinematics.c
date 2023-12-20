#include "kinematics.h"

#include "console.h"
#include "tick.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

bool kinematicsAvailable = false;
double maximumStepsPerSecond = 60 * 256 * 60; // 60 RPM
double minimumMotionDuration = 0.000001;
double kinematicsSubdivisionInterval = 0.01; // seconds, i.e 10ms
double rotationLinearThreshold = 0.9995;

Displacement platformAttachment[MAIN_AXIS_COUNT];
Position platformTool;
Displacement sliderZero[MAIN_AXIS_COUNT];
Displacement sliderUpStep[MAIN_AXIS_COUNT];
double strutLength[MAIN_AXIS_COUNT];
int32_t upperLimit[MAIN_AXIS_COUNT];
int32_t lowerLimit[MAIN_AXIS_COUNT];
bool abovePlatform[MAIN_AXIS_COUNT];

#define SCHEDULE_BUFFER_COUNT 256
static OutputSchedule scheduleBuffer[SCHEDULE_BUFFER_COUNT];
static int nextFreeSchedule;

typedef struct ScheduledTarget {
  Position target;
  uint8_t completed;
} ScheduledTarget;

#define TARGET_BUFFER_COUNT 16
static ScheduledTarget targetBuffer[TARGET_BUFFER_COUNT];
static int currentTarget; // -1 if nothing is to be done
static int nextFreeTarget;

int32_t stepsUp[MAIN_AXIS_COUNT];
Position tool;

void setZero(Position initialToolPosition) {
  for(int i = 0; i < MAIN_AXIS_COUNT; ++i) {
    stepsUp[i] = 0;
    abovePlatform[i] = true;
  }

  const Position zero = {{0, 0, 0}, {1, 0, 0, 0}};
  platformTool = zero;
  tool = initialToolPosition;

  kinematicsAvailable = true;
  for(int i = 0; i < SCHEDULE_BUFFER_COUNT; ++i) {
    scheduleBuffer[i].completed = 1;
  }
  nextFreeSchedule = 0;

  for(int i = 0; i < TARGET_BUFFER_COUNT; ++i) {
    targetBuffer[i].completed = 1;
  }
  nextFreeTarget = 0;
  currentTarget = -1;
}

Quaternion unitQuaternionInverse(Quaternion q) {
  Quaternion result = {q.r, -q.i, -q.j, -q.k};
  return result;
}

Quaternion quaternionFromDisplacement(Displacement d) {
  Quaternion result = {0, d.x, d.y, d.z};
  return result;
}

Displacement displacementFromQuaternion(Quaternion q) {
  Displacement result = {q.i, q.j, q.k};
  return result;
}

Quaternion quaternionMul(Quaternion a, Quaternion b) {
  Quaternion result = {
    a.r * b.r - a.i * b.i - a.j * b.j - a.k - b.k,
    a.r * b.i + a.i * b.r + a.j * b.k - a.k * b.j,
    a.r * b.j - a.i * b.k + a.j * b.r + a.k * b.i,
    a.r * b.k + a.i * b.j - a.j * b.i + a.k * b.r
  };
  return result;
}

double quaternionDot(Quaternion a, Quaternion b) {
  return a.r * b.r + a.i * b.i + a.j * b.j + a.k * b.k;
}

Quaternion quaternionAdd(Quaternion a, Quaternion b) {
  Quaternion result = {
    a.r + b.r,
    a.i + b.i,
    a.j + b.j,
    a.k + b.k
  };
  return result;
}

Quaternion quaternionSub(Quaternion a, Quaternion b) {
  Quaternion result = {
    a.r - b.r,
    a.i - b.i,
    a.j - b.j,
    a.k - b.k
  };
  return result;
}

Quaternion quaternionScale(Quaternion q, double s) {
  Quaternion result = {
    q.r * s,
    q.i * s,
    q.j * s,
    q.k * s
  };
  return result;
}

double quaternionLength(Quaternion q) {
  return sqrt(
      q.r * q.r +
      q.i * q.i +
      q.j * q.j +
      q.k * q.k
  );
}

Quaternion quaternionNormalize(Quaternion q) {
  return quaternionScale(q, 1 / quaternionLength(q));
}

Displacement displacementAdd(Displacement a, Displacement b) {
  Displacement result = {a.x + b.x, a.y + b.y, a.z + b.z};
  return result;
}

Displacement displacementSub(Displacement a, Displacement b) {
  Displacement result = {a.x - b.x, a.y - b.y, a.z - b.z};
  return result;
}

double displacementDot(Displacement a, Displacement b) {
  double result = a.x * b.x + a.y * b.y + a.z * b.z;
  return result;
}

Position relativePositionAdd(Position base, Position rel) {
  // TODO: Could use https://www.johndcook.com/blog/2021/06/16/faster-quaternion-rotations/ fast rotation instead
  // disp: base.disp + base.rot * rel.disp * base.rot^-1
  // rot: rel.rot * base.rot
  Position result = {
    displacementAdd(
      base.disp,
      displacementFromQuaternion(quaternionMul(
        base.rot,
        quaternionMul(
          quaternionFromDisplacement(rel.disp),
          unitQuaternionInverse(base.rot))))),
      quaternionMul(rel.rot, base.rot)
  };
  return result;
}

Position relativePositionSub(Position base, Position rel) {
  // disp: base.disp - base.rot * rel.disp * base.rot^-1
  // rot: rel.rot^-1 * base.rot
  Position result = {
    displacementSub(
      base.disp,
      displacementFromQuaternion(quaternionMul(
        base.rot,
        quaternionMul(
          quaternionFromDisplacement(rel.disp),
          unitQuaternionInverse(base.rot))))),
      quaternionMul(unitQuaternionInverse(rel.rot), base.rot)
  };
  return result;
}

void setToolAttachedAt(Position pos) {
  Position platform = relativePositionSub(tool, platformTool);
  tool = relativePositionAdd(platform, pos);

  platformTool = pos;
}

static double movementSpeed = 1;
void setMovementSpeed(double mmPerSecond) {
  movementSpeed = mmPerSecond;
}

static double rotationSpeed = 5;
void setRotationSpeed(double degreesPerSecond) {
  rotationSpeed = degreesPerSecond / 180 * M_PI;
}

void completeCurrentTarget() {
    targetBuffer[currentTarget].completed = 1;
    currentTarget = (currentTarget + 1) % TARGET_BUFFER_COUNT;

    if(currentTarget == nextFreeTarget) currentTarget = -1;
}

void runKinematics() {
  if(!kinematicsAvailable) return;
  if(currentTarget == -1) return;
  int lookAhead = (nextFreeTarget + SCHEDULE_BUFFER_COUNT / 4) % SCHEDULE_BUFFER_COUNT;
  if(!scheduleBuffer[lookAhead].completed) return;

  Position *target = &targetBuffer[currentTarget].target;
  printf("Current target: %lf %lf %lf @ %lf %lf %lf %lf\n",
      target->disp.x, target->disp.y, target->disp.z,
      target->rot.r, target->rot.i, target->rot.j, target->rot.k);

// cf. http://number-none.com/product/Understanding%20Slerp,%20Then%20Not%20Using%20It/
  double dx = target->disp.x - tool.disp.x;
  double dy = target->disp.y - tool.disp.y;
  double dz = target->disp.z - tool.disp.z;
  double distanceMoved = sqrt(dx * dx + dy * dy + dz * dz);
  double movementDuration = distanceMoved / movementSpeed;

  double rotationDuration;
  double dot = quaternionDot(tool.rot, target->rot);
  double theta_0 = 0;
  Quaternion ortho;

  if(dot > rotationLinearThreshold) {
    // will use linear interpolation
    Quaternion delta = quaternionSub(tool.rot, target->rot);
    rotationDuration = sqrt(quaternionDot(delta, delta)) / rotationSpeed;
  } else {
    if(dot < -1) dot = -1;
    theta_0 = acos(dot);
    rotationDuration = theta_0 / rotationSpeed;

    ortho = quaternionNormalize(quaternionSub(target->rot, quaternionScale(tool.rot, dot)));
  }

  double duration = movementDuration > rotationDuration? movementDuration: rotationDuration;
  if(duration < minimumMotionDuration) {
    completeCurrentTarget();
    return;
  }

  int intervals = (duration / kinematicsSubdivisionInterval) + 1;
  printf("Expected duration: %lf, intervals: %d\n", duration, intervals);

  bool targetComplete = true;
  bool startSchedule = false;
  Position toolBefore = tool;
  int newSchedules = nextFreeSchedule;
  double divisions = 1.0 / intervals;
  for(int i = 0; i < intervals; ++i) {
    Position endPos;
    if(i == intervals - 1) {
      endPos = *target;
    } else {
      double b = i * divisions;
      double a = (1 - b);

      // printf("a,b: %lf %lf\n", a, b);

      endPos.disp.x = toolBefore.disp.x * a + target->disp.x * b;
      endPos.disp.y = toolBefore.disp.y * a + target->disp.y * b;
      endPos.disp.z = toolBefore.disp.z * a + target->disp.z * b;

      if(dot > rotationLinearThreshold) {
        endPos.rot = quaternionNormalize(
            quaternionAdd(
              quaternionScale(toolBefore.rot, a),
              quaternionScale(target->rot, b)));
      } else {
        double theta = theta_0 * a;

        endPos.rot = quaternionAdd(
            quaternionScale(toolBefore.rot, cos(theta)),
            quaternionScale(ortho, sin(theta)));
      }
    }

    printf("endPos: %lf %lf %lf @ %lf %lf %lf %lf\n",
        endPos.disp.x, endPos.disp.y, endPos.disp.z,
        endPos.rot.r, endPos.rot.i, endPos.rot.j, endPos.rot.k);

    int32_t stepDelta[MAIN_AXIS_COUNT];
    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      Position relativeAttachment = {
        platformAttachment[axis],
        {1, 0, 0, 0}
      };
      Position attachmentTarget =
        relativePositionAdd(
            relativePositionSub(endPos, platformTool),
            relativeAttachment);

      Displacement d = displacementSub(attachmentTarget.disp, sliderZero[axis]);
      Displacement u = sliderUpStep[axis];
      double s = strutLength[axis];
      // k == stepsUp
      // |d - k*u| = s
      // (d - k*u)^2 = s^2
      // d^2 - 2*d*u*k + u^2k^2 - s^2 = 0
      // k^2 - 2*d*u*k / u^2 + (d^2 - s^2) / u^2 = 0
      // k = d*u/u^2 +- sqrt(((d*u)^2 / u^4) - (d^2-s^2)/u^2)
      // k = (d*u +- sqrt((d*u)^2 - u^2*(d^2-s^2)) / u^2

      double dotDU = displacementDot(d, u);
      double dotUU = displacementDot(u, u);
      double dotDD = displacementDot(d, d);

      double k1 = (dotDU + sqrt(dotDU * dotDU - dotUU * (dotDD - s*s))) / dotUU;
      double k2 = (dotDU - sqrt(dotDU * dotDU - dotUU * (dotDD - s*s))) / dotUU;

      double k = abovePlatform[axis]? k1: k2;

      if(k < lowerLimit[axis] || k > upperLimit[axis]) {
        console_send_str("Target requires slider outside limits. Axis ");
        console_send_uint8(axis);
        console_send_str(" target ");
        console_send_uint32(k);

        completeCurrentTarget(); // TODO global error handling
        return;
      }

      stepDelta[axis] = k - stepsUp[axis];
    }

    double intervalDuration = kinematicsSubdivisionInterval;
    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      // FIXME: Smooth startup
      double dur = abs(stepDelta[axis]) / maximumStepsPerSecond;
      if(dur > intervalDuration) intervalDuration = dur;
    }

    OutputSchedule *sched = scheduleBuffer + nextFreeSchedule;
    if(!sched->completed) {
      targetComplete = false;
      break; // All bufferable schedules used
    }

    sched->next = NULL;

    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      sched->motors[axis].dir = stepDelta[axis] >= 0? DIR_MAIN_AXIS_UP: DIR_MAIN_AXIS_DOWN;
      sched->motors[axis].count = abs(stepDelta[axis]);
      // FIXME: Make this smooth across segments
      sched->motors[axis].timer = 0;
      sched->motors[axis].dt = 4294967296.0 * abs(stepDelta[axis]) / (intervalDuration * CONFIG_TICK_FREQ);
      sched->motors[axis].ddt = 0;
      sched->motors[axis].dddt = 0;
    }

    int lastSchedule = (nextFreeSchedule + SCHEDULE_BUFFER_COUNT - 1) % SCHEDULE_BUFFER_COUNT;
    int last2Schedule = (nextFreeSchedule + SCHEDULE_BUFFER_COUNT - 2) % SCHEDULE_BUFFER_COUNT;
    if(scheduleBuffer[last2Schedule].completed) {
      // Resynchronize with systick
      while(motorsMoving());

      startSchedule = true;
    } else {
      scheduleBuffer[lastSchedule].next = sched;
    }

    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      stepsUp[axis] += stepDelta[axis];
    }
    tool = endPos;

    nextFreeSchedule = (nextFreeSchedule + 1) % SCHEDULE_BUFFER_COUNT;
  }
  if(startSchedule) scheduleMotors(scheduleBuffer + newSchedules);
  if(targetComplete) completeCurrentTarget();
}

void moveTo(Position target) {
  if(!targetBuffer[nextFreeTarget].completed) {
    console_send_str("No free target buffers. Send slower.\r\n");
    return;
  }

  targetBuffer[nextFreeTarget].target = target;
  targetBuffer[nextFreeTarget].completed = 0;
  if(currentTarget == -1) currentTarget = nextFreeTarget;

  nextFreeTarget = (nextFreeTarget + 1) % TARGET_BUFFER_COUNT;
}
