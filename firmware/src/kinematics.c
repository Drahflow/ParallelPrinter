#include "kinematics.h"

#include "console.h"
#include "tick.h"

#include <math.h>
#include <string.h>
#include <stdlib.h>

#ifndef TEST
#define printf(...)
#endif

bool kinematicsAvailable = false;
double maximumStepsPerSecond = 60 * 256 * 200 / 60; // 60 RPM
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
double sliderBackslash[MAIN_AXIS_COUNT]; // mm
double sliderElasticity[MAIN_AXIS_COUNT]; // mm per platform weight force
bool forceLimiting = false;
double forceLimit[MAIN_AXIS_COUNT];

#define SCHEDULE_BUFFER_COUNT 128
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

static void initializeScheduleBuffers() {
  for(int i = 0; i < SCHEDULE_BUFFER_COUNT; ++i) {
    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      scheduleBuffer[i].motors[axis].dddt = 0;
      scheduleBuffer[i].motors[axis].ddt = 0;
      scheduleBuffer[i].motors[axis].dt = 0;
      scheduleBuffer[i].motors[axis].timer = 0;
    }
  }
  for(int i = 0; i < SCHEDULE_BUFFER_COUNT; ++i) {
    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      scheduleBuffer[i].motors[axis].count = 0;
    }

    scheduleBuffer[i].next = NULL;
    scheduleBuffer[i].completed = 1;
  }

  nextFreeSchedule = 0;
}

static void initializeTargetBuffers() {
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
    a.r * b.r - a.i * b.i - a.j * b.j - a.k * b.k,
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

Displacement displacementCross(Displacement a, Displacement b) {
  Displacement result = {
    a.y * b.z - a.z * b.y,
    a.z * b.x - a.x * b.z,
    a.x * b.y - a.y * b.x,
  };
  return result;
}

Displacement displacementScale(Displacement d, double s) {
  Displacement result = {
    d.x * s,
    d.y * s,
    d.z * s,
  };
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

// Return x such that relativePositionAdd(x, rel) == base
Position relativePositionSub(Position base, Position rel) {
  Rotation relRotInverse = unitQuaternionInverse(rel.rot);
  Rotation resultRot = quaternionMul(relRotInverse, base.rot);
  // disp: base.disp - rel.rot^-1 * base.rot * rel.disp * base.rot^-1 * rel.rot
  // rot: rel.rot^-1 * base.rot
  Position result = {
    displacementSub(
      base.disp,
      displacementFromQuaternion(quaternionMul(
        resultRot,
        quaternionMul(
          quaternionFromDisplacement(rel.disp),
          unitQuaternionInverse(resultRot))))),
      resultRot
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

static double rotationSpeed = 5.0 / 180 * M_PI;
void setRotationSpeed(double degreesPerSecond) {
  rotationSpeed = degreesPerSecond / 180 * M_PI;
}

void completeCurrentTarget() {
    targetBuffer[currentTarget].completed = 1;
    currentTarget = (currentTarget + 1) % TARGET_BUFFER_COUNT;

    if(currentTarget == nextFreeTarget) {
      currentTarget = -1;

      console_send_str("Movement scheduling completed.\r\n");
    }
}

int calculateStrutForces(
    Displacement platform,
    Displacement *attachmentTarget,
    Displacement *strutForceDirection,
    double expectedCenterProportion,
    double *forces // positive number: platform pulls on the strut
) {
  Displacement centerMoment[MAIN_AXIS_COUNT];

  Displacement down = {0, 0, -1};
  double centerForceAlignment =
    displacementDot(strutForceDirection[3], down);
  if(fabs(centerForceAlignment) < 0.05) {
    // Center point force near perdendicular, let's clamp and hope
    centerForceAlignment = centerForceAlignment < 0? -0.05: 0.05;
  }
  double expectedCenterForce = -expectedCenterProportion / centerForceAlignment;

  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    centerMoment[axis] = displacementCross(
      displacementSub(attachmentTarget[axis], platform),
      strutForceDirection[axis]);
  }

#define MATRIX_SIZE 6

  // Now we make a system of linear equations to estimate the force per strut.
  double x[MATRIX_SIZE];
  double A[MATRIX_SIZE * MATRIX_SIZE]; // row-major indexing
  int rowOrder[MATRIX_SIZE];
  for(int axis = 0; axis < MATRIX_SIZE; ++axis) {
    rowOrder[axis] = axis;
  }

  // TODO: Consider changing this for accelleration optimization
  x[0] = 0; // no x component of result force
  x[1] = 0; // no y component of result force
  x[2] = 1; // one unit of force upward (1 == magnitude of gravitational force of platform)
  x[3] = 0; // no moment-x at platform center
  x[4] = 0; // no moment-y at platform center
  x[5] = 0; // no moment-z at platform center

  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    if(axis == 3) {
      x[0] -= strutForceDirection[axis].x * expectedCenterForce;
      x[1] -= strutForceDirection[axis].y * expectedCenterForce;
      x[2] -= strutForceDirection[axis].z * expectedCenterForce;
      // no center moments for center strut

      continue;
    }

    const int i = axis < 3? axis: axis - 1;
    A[0 * MATRIX_SIZE + i] = strutForceDirection[axis].x;
    A[1 * MATRIX_SIZE + i] = strutForceDirection[axis].y;
    A[2 * MATRIX_SIZE + i] = strutForceDirection[axis].z;
    A[3 * MATRIX_SIZE + i] = centerMoment[axis].x;
    A[4 * MATRIX_SIZE + i] = centerMoment[axis].y;
    A[5 * MATRIX_SIZE + i] = centerMoment[axis].z;
  }

  // cf. https://en.wikipedia.org/wiki/LU_decomposition#Code_examples
  for(int i = 0; i < MATRIX_SIZE; ++i) {
    int largestK = -1;
    double largestCoeff = 1e-15;
    for(int k = i; k < MATRIX_SIZE; ++k) {
      const double abs = fabs(A[k * MATRIX_SIZE + i]);
      if(abs > largestCoeff) {
        largestK = k;
        largestCoeff = abs;
      }
    }

    if(largestK < 0) {
      console_send_str("Platform forces could not be calculated, entry too small in row: ");
      console_send_uint8(i);
      console_send_str("\r\n");
      return 0;
    }

    if(largestK != i) {
      int tmp = rowOrder[i];
      rowOrder[i] = rowOrder[largestK];
      rowOrder[largestK] = tmp;

      double tmp2[MATRIX_SIZE];
      memmove(tmp2, A + i * MATRIX_SIZE, sizeof(double) * MATRIX_SIZE);
      memmove(A + i * MATRIX_SIZE, A + largestK * MATRIX_SIZE, sizeof(double) * MATRIX_SIZE);
      memmove(A + largestK * MATRIX_SIZE, tmp2, sizeof(double) * MATRIX_SIZE);
    }

    for(int j = i + 1; j < MATRIX_SIZE; ++j) {
      A[j * MATRIX_SIZE + i] /= A[i * MATRIX_SIZE + i];

      for(int k = i + 1; k < MATRIX_SIZE; ++k) {
        A[j * MATRIX_SIZE + k] -= A[j * MATRIX_SIZE + i] * A[i * MATRIX_SIZE + k];
      }
    }
  }

  for(int i = 0; i < MATRIX_SIZE; ++i) {
    forces[i] = x[rowOrder[i]];

    for(int j = 0; j < i; ++j) {
      forces[i] -= A[i * MATRIX_SIZE + j] * forces[j];
    }
  }

  for(int i = MATRIX_SIZE - 1; i >= 0; --i) {
    for(int j = i + 1; j < MATRIX_SIZE; j++) {
      forces[i] -= A[i * MATRIX_SIZE + j] * forces[j];
    }

    forces[i] /= A[i * MATRIX_SIZE + i];
  }

  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    double force;
    if(axis == 3) {
      force = expectedCenterForce;
    } else {
      force = forces[axis < 3? axis: axis - 1];
    }

    printf("Force on axis %d: %lf\n", axis, force);

    if(fabs(force) > forceLimit[axis]) {
      console_send_str("Force on axis ");
      console_send_uint8(axis);
      console_send_str("exceeds expected limits.");
      return 0;
    }
  }

  forces[6] = forces[5];
  forces[5] = forces[4];
  forces[4] = forces[3];
  forces[3] = expectedCenterForce;

  return 1;
}

int calculateSliderPositions(
    Position platform,
    Displacement attachmentTarget[MAIN_AXIS_COUNT],
    double effectiveStrutLength[MAIN_AXIS_COUNT],
    Displacement strutForceDirection[MAIN_AXIS_COUNT],
    double sliderLinearPos[MAIN_AXIS_COUNT]) {
  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    Position relativeAttachment = {
      platformAttachment[axis],
      {1, 0, 0, 0}
    };
    attachmentTarget[axis] = relativePositionAdd(platform, relativeAttachment).disp;

    Displacement d = displacementSub(attachmentTarget[axis], sliderZero[axis]);
    Displacement u = sliderUpStep[axis];
    double s = effectiveStrutLength[axis];
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

    printf("attachmentTarget[%d]: %lf %lf %lf; displacement: %lf %lf %lf\n",
        axis,
        attachmentTarget[axis].x, attachmentTarget[axis].y, attachmentTarget[axis].z,
        d.x, d.y, d.z);

    double sqrtInner = dotDU * dotDU - dotUU * (dotDD - s*s);
    if(sqrtInner < 0) {
      console_send_str("Target outside maximum strut reach. Axis ");
      console_send_uint8(axis);
      console_send_str("\r\n");

      return 0;
    }

    double k1 = (dotDU + sqrt(sqrtInner)) / dotUU;
    double k2 = (dotDU - sqrt(sqrtInner)) / dotUU;

    const double k = abovePlatform[axis]? k1: k2;
    sliderLinearPos[axis] = k;

    printf("attachmentTarget[%d]: %lf %lf %lf; step: %lf\n",
        axis,
        attachmentTarget[axis].x, attachmentTarget[axis].y, attachmentTarget[axis].z,
        k);

    if(k < lowerLimit[axis] || k > upperLimit[axis]) {
      console_send_str("Target requires slider outside limits. Axis ");
      console_send_uint8(axis);
      console_send_str(" target ");
      console_send_uint32(k);
      console_send_str("\r\n");

      return 0;
    }

    strutForceDirection[axis] = displacementScale(
        displacementSub(
          displacementAdd(sliderZero[axis], displacementScale(sliderUpStep[axis], k)),
          attachmentTarget[axis]),
        1 / effectiveStrutLength[axis]);
  }

  return 1;
}

static int calculateStepDeltas(Position endPos, int32_t stepDelta[MAIN_AXIS_COUNT]) {
  Displacement attachmentTarget[MAIN_AXIS_COUNT];
  Displacement strutForceDirection[MAIN_AXIS_COUNT];
  double sliderLinearPos[MAIN_AXIS_COUNT];

  Position platform = relativePositionSub(endPos, platformTool);

  double effectiveStrutLength[MAIN_AXIS_COUNT];
  memcpy(effectiveStrutLength, strutLength, sizeof(strutLength));

  if(!calculateSliderPositions(platform, attachmentTarget, effectiveStrutLength, strutForceDirection, sliderLinearPos)) {
    return 0;
  }

  if(forceLimiting) {
    const double expectedCenterProportion = 0.8;
    double forces[MAIN_AXIS_COUNT];
    if(!calculateStrutForces(platform.disp, attachmentTarget, strutForceDirection, expectedCenterProportion, forces)) {
      return 0;
    }

    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      effectiveStrutLength[axis] += forces[axis] > 0? sliderBackslash[axis]: -sliderBackslash[axis];
      effectiveStrutLength[axis] += forces[axis] * sliderElasticity[axis];

      printf("strut len delta[%d]: %lf\n", axis, effectiveStrutLength[axis] - strutLength[axis]);
    }

    if(!calculateSliderPositions(platform, attachmentTarget, effectiveStrutLength, strutForceDirection, sliderLinearPos)) {
      return 0;
    }
  }

  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    stepDelta[axis] = sliderLinearPos[axis] - stepsUp[axis];

    printf("attachmentTarget[%d]: %lf %lf %lf; step delta: %d\n",
        axis,
        attachmentTarget[axis].x, attachmentTarget[axis].y, attachmentTarget[axis].z,
        stepDelta[axis]);
  }

  return 1;
}

void setZero(Position initialToolPosition) {
  for(int i = 0; i < MAIN_AXIS_COUNT; ++i) {
    stepsUp[i] = 0;
    abovePlatform[i] = true;
  }

  const Position zero = {{0, 0, 0}, {1, 0, 0, 0}};
  platformTool = zero;
  tool = initialToolPosition;

  initializeScheduleBuffers();
  initializeTargetBuffers();

  int32_t stepDelta[MAIN_AXIS_COUNT];
  if(!calculateStepDeltas(initialToolPosition, stepDelta)) {
    kinematicsAvailable = false;
    return;
  }

  console_send_str("Required step deltas to reach initial position:\r\n");
  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    console_send_str("Axis ");
    console_send_uint8(axis);
    console_send_str(": ");
    console_send_int32(stepDelta[axis]);
    console_send_str("\r\n");
  }

  kinematicsAvailable = true;
}

static OutputSchedule *getFreeSchedule() {
    printf("Scheduling into buffer %d\n", nextFreeSchedule);
    OutputSchedule *sched = scheduleBuffer + nextFreeSchedule;
    if(!sched->completed) {
      return NULL;
    }

    return sched;
}

static void scheduleStepDelta(OutputSchedule *sched, double intervalDuration, int32_t stepDelta[MAIN_AXIS_COUNT],
    bool *startSchedule) {
  sched->next = NULL;
  sched->completed = 0;

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

    *startSchedule = true;
  }
  scheduleBuffer[lastSchedule].next = sched;

  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    stepsUp[axis] += stepDelta[axis];
  }

  nextFreeSchedule = (nextFreeSchedule + 1) % SCHEDULE_BUFFER_COUNT;
}

void runKinematics() {
  if(!kinematicsAvailable) return;
  if(currentTarget == -1) return;
  int lookAhead = (nextFreeSchedule + SCHEDULE_BUFFER_COUNT / 4) % SCHEDULE_BUFFER_COUNT;
  if(!scheduleBuffer[lookAhead].completed) {
    //printf("Waiting for schedule buffer %d\n", lookAhead);
    return;
  }

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
      double b = (i + 1) * divisions;
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
        double theta = theta_0 * b;

        endPos.rot = quaternionAdd(
            quaternionScale(toolBefore.rot, cos(theta)),
            quaternionScale(ortho, sin(theta)));
      }
    }

    printf("endPos: %lf %lf %lf @ %lf %lf %lf %lf\n",
        endPos.disp.x, endPos.disp.y, endPos.disp.z,
        endPos.rot.r, endPos.rot.i, endPos.rot.j, endPos.rot.k);

    int32_t stepDelta[MAIN_AXIS_COUNT];
    if(!calculateStepDeltas(endPos, stepDelta)) {
      console_send_str("step delta calculation failed, skipping current goal\r\n");
      completeCurrentTarget(); // TODO global error handling
      return;
    }

    double intervalDuration = kinematicsSubdivisionInterval;
    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      // FIXME: Smooth startup
      double dur = abs(stepDelta[axis]) / maximumStepsPerSecond;
      if(dur > intervalDuration) intervalDuration = dur;
    }

    OutputSchedule *sched = getFreeSchedule();
    if(!sched) {
      targetComplete = false;
      break; // All bufferable schedules used
    }

    scheduleStepDelta(sched, intervalDuration, stepDelta, &startSchedule);
    tool = endPos;
  }

  console_send_str("Remaining target duration: ");
  console_send_double(duration);
  console_send_str("\r\n");

  console_send_str("Tool position after move: ");
  console_send_double(tool.disp.x); console_send_str(" ");
  console_send_double(tool.disp.y); console_send_str(" ");
  console_send_double(tool.disp.z); console_send_str(" @ ");
  console_send_double(tool.rot.r); console_send_str(" ");
  console_send_double(tool.rot.i); console_send_str(" ");
  console_send_double(tool.rot.j); console_send_str(" ");
  console_send_double(tool.rot.k); console_send_str("\r\n");

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

void moveAgain() {
  if(!kinematicsAvailable) {
    console_send_str("Kinematics not available.\r\n");
    return;
  }

  if(currentTarget != -1) {
    console_send_str("Current move in progres.\r\n");
    return;
  }

  int32_t stepDelta[MAIN_AXIS_COUNT];
  if(!calculateStepDeltas(tool, stepDelta)) {
    console_send_str("step delta calculation failed\r\n");
    return;
  }

  double intervalDuration = 1.0;
  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    double dur = abs(stepDelta[axis]) / maximumStepsPerSecond;
    if(dur > intervalDuration) intervalDuration = dur;
  }

  OutputSchedule *sched = getFreeSchedule();
  if(!sched) {
    console_send_str("no free schedule buffer\r\n");
    return;
  }

  int newSchedules = nextFreeSchedule;
  bool startSchedule;
  scheduleStepDelta(sched, intervalDuration, stepDelta, &startSchedule);
  if(startSchedule) scheduleMotors(scheduleBuffer + newSchedules);
}

void kinematicsStop() {
  currentTarget = -1;
  for(int i = 0; i < 1000; ++i) {
    scheduleMotors(NULL);
  }

  for(int i = 0; i < SCHEDULE_BUFFER_COUNT; ++i) {
    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      if(scheduleBuffer[i].motors[axis].dir == DIR_MAIN_AXIS_UP) {
        stepsUp[axis] -= scheduleBuffer[i].motors[axis].count;
      } else if(scheduleBuffer[i].motors[axis].dir == DIR_MAIN_AXIS_DOWN) {
        stepsUp[axis] += scheduleBuffer[i].motors[axis].count;
      }
    }
  }

  initializeScheduleBuffers();
  initializeTargetBuffers();
}

void checkForceLimiting() {
  forceLimiting = false;

  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    if(forceLimit[axis] != 0) {
      forceLimiting = true;
    }
  }
}
