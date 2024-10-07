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
Displacement platformCenterOfMass;
double platformGravitationalForce = 0.0;
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

void setCenterOfMassAt(Displacement disp, double force) {
  platformCenterOfMass = disp;
  platformGravitationalForce = force;
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

bool debugSolver = true;

void dumpLinearEquations(double *A, double *x, int matrixRows, int matrixCols) {
  for(int i = 0; i < matrixRows; ++i) {
    for(int j = 0; j < matrixCols; ++j) {
      printf("%6.2f ", A[i * matrixCols + j]);
    }
    printf("= %6.2f\n", x[i]);
  }
}

int solveLinearEquationsByLU(double *A, double *x, double *result, int matrixRows, int matrixCols) {
  if(debugSolver) dumpLinearEquations(A, x, matrixRows, matrixCols);

  int rowOrder[matrixRows];
  for(int r = 0; r < matrixRows; ++r) {
    rowOrder[r] = r;
  }

  // cf. https://en.wikipedia.org/wiki/LU_decomposition#Code_examples
  for(int i = 0; i < matrixCols; ++i) {
    int largestK = -1;
    double largestCoeff = 1e-15;
    for(int k = i; k < matrixRows; ++k) {
      const double abs = fabs(A[matrixCols * k + i]);
      if(abs > largestCoeff) {
        largestK = k;
        largestCoeff = abs;
      }
    }

    if(largestK < 0) {
      printf("Solution could not be calculated, entry too small in column: %d\n", i);
      if(debugSolver) dumpLinearEquations(A, x, matrixRows, matrixCols);
      return 0;
    }

    if(largestK != i) {
      int tmp = rowOrder[i];
      rowOrder[i] = rowOrder[largestK];
      rowOrder[largestK] = tmp;

      double tmp2[matrixCols];
      memmove(tmp2, A + i * matrixCols, sizeof(double) * matrixCols);
      memmove(A + i * matrixCols, A + largestK * matrixCols, sizeof(double) * matrixCols);
      memmove(A + largestK * matrixCols, tmp2, sizeof(double) * matrixCols);
    }

    for(int j = i + 1; j < matrixRows; ++j) {
      A[j * matrixCols + i] /= A[i * matrixCols + i];

      for(int k = i + 1; k < matrixCols; ++k) {
        A[j * matrixCols + k] -= A[j * matrixCols + i] * A[i * matrixCols + k];
      }
    }
  }

  if(debugSolver) {
    printf("After LU decomposition ==============\n");
    dumpLinearEquations(A, x, matrixRows, matrixCols);
  }

  for(int i = 0; i < matrixCols; ++i) {
    result[i] = x[rowOrder[i]];

    for(int j = 0; j < i; ++j) {
      result[i] -= A[i * matrixCols + j] * result[j];
    }
  }

  if(debugSolver) {
    printf("Result A =============\n");
    for(int i = 0; i < matrixCols; ++i) {
      printf("%6.2f ", result[i]);
    }
    printf("\n");
  }

  for(int i = matrixCols - 1; i >= 0; --i) {
    for(int j = i + 1; j < matrixCols; j++) {
      result[i] -= A[i * matrixCols + j] * result[j];
    }

    if(debugSolver) {
      if(isnan(A[i * matrixCols + i]) || fabs(A[i * matrixCols + i]) < 0.00001) {
        printf("Broken division at i = %d, %6.2f\n", i, A[i * matrixCols + i]);
        dumpLinearEquations(A, x, matrixRows, matrixCols);
      }
    }

    result[i] /= A[i * matrixCols + i];
  }

  if(debugSolver) {
    printf("Result ==============\n");
    for(int i = 0; i < matrixCols; ++i) {
      printf("%6.2f ", result[i]);
    }
    printf("\n");
  }

  return 1;
}

int calculateStrutForces(
    Displacement centerOfMass,
    Displacement *attachmentTarget,
    Displacement *sliderPositions,
    double *strutLength,
    double *forces // positive number: strut is compressed
) {
  struct Strut {
    Displacement start, end;
    Displacement dir;
    double length;
    double elasticity;
    int startVertexIndex, endVertexIndex;
  };
  const int strutCount = MAIN_AXIS_COUNT * 4 - 3;
  const int vertexCount = MAIN_AXIS_COUNT + 1;

  struct Strut struts[strutCount];
  int s = 0;
  for(int i = 0; i < MAIN_AXIS_COUNT; ++i) {
    struts[s].start = attachmentTarget[i];
    struts[s].startVertexIndex = i;
    struts[s].end = sliderPositions[i];
    struts[s].endVertexIndex = -1;
    struts[s].dir = displacementSub(struts[s].end, struts[s].start);
    struts[s].length = strutLength[i];
    struts[s].elasticity = 0.1;
    ++s;
  }
  for(int i = 0; i < MAIN_AXIS_COUNT - 1; ++i) {
    struts[s].start = attachmentTarget[i];
    struts[s].startVertexIndex = i;
    struts[s].end = attachmentTarget[i + 1];
    struts[s].endVertexIndex = i + 1;
    struts[s].dir = displacementSub(struts[s].end, struts[s].start);
    // TODO: Static after attachment target config
    struts[s].length = sqrt(displacementDot(struts[s].dir, struts[s].dir));
    struts[s].elasticity = 0.001;
    ++s;
  }
  for(int i = 0; i < MAIN_AXIS_COUNT - 2; ++i) {
    struts[s].start = attachmentTarget[i];
    struts[s].startVertexIndex = i;
    struts[s].end = attachmentTarget[i + 2];
    struts[s].endVertexIndex = i + 2;
    struts[s].dir = displacementSub(struts[s].end, struts[s].start);
    // TODO: Static after attachment target config
    struts[s].length = sqrt(displacementDot(struts[s].dir, struts[s].dir));
    struts[s].elasticity = 0.001;
    ++s;
  }
  for(int i = 0; i < MAIN_AXIS_COUNT; ++i) {
    struts[s].start = attachmentTarget[i];
    struts[s].startVertexIndex = i;
    struts[s].end = centerOfMass;
    struts[s].endVertexIndex = MAIN_AXIS_COUNT;
    struts[s].dir = displacementSub(struts[s].end, struts[s].start);
    // TODO: Static after attachment target config
    struts[s].length = sqrt(displacementDot(struts[s].dir, struts[s].dir));
    struts[s].elasticity = 0.001;
    ++s;
  }

  for(int i = 0; i < strutCount; ++i) {
    struts[s].dir = displacementScale(struts[s].dir, 1 / struts[s].length);
  }

  int matrixCols =
    strutCount + // one force per strut
    vertexCount * 3; // dx/dy/dz per vertex
  int matrixRows =
    strutCount + // force in terms of displacements
    vertexCount * 3 + // vertex forces in equilibrium along all three axes
    3; // anchor robot in global space

  double x[matrixRows];
  double A[matrixRows * matrixCols]; // row-major
  
  for(int r = 0; r < matrixRows; ++r) {
    x[r] = 0;

    for(int c = 0; c < matrixCols; ++c) {
      A[matrixCols * r + c] = 0;
    }
  }

  // rows 0 to vertexCount * 3 => forces on vertices
  for(int i = 0; i < strutCount; ++i) {
    Displacement dir = struts[i].dir;

    const int forceVarIndex = i;
    // Strut forces at each vertex are in equilibrium along each axis
    A[matrixCols * (struts[i].startVertexIndex * 3 + 0) + forceVarIndex] = -dir.x;
    A[matrixCols * (struts[i].startVertexIndex * 3 + 1) + forceVarIndex] = -dir.y;
    A[matrixCols * (struts[i].startVertexIndex * 3 + 2) + forceVarIndex] = -dir.z;

    if(struts[i].endVertexIndex >= 0) {
      A[matrixCols * (struts[i].endVertexIndex * 3 + 0) + forceVarIndex] = dir.x;
      A[matrixCols * (struts[i].endVertexIndex * 3 + 1) + forceVarIndex] = dir.y;
      A[matrixCols * (struts[i].endVertexIndex * 3 + 2) + forceVarIndex] = dir.z;
    }
  }

  // Gravity load on center of mass vertex, in z direction
  x[(MAIN_AXIS_COUNT * 3 + 0)] = 0;
  x[(MAIN_AXIS_COUNT * 3 + 1)] = 0;
  x[(MAIN_AXIS_COUNT * 3 + 2)] = platformGravitationalForce; // 15N

  // rows vertexCount * 3 to vertexCount * 3 + strutCount => forces in struts
  for(int i = 0; i < strutCount; ++i) {
    Displacement dir = struts[i].dir;

    const int forceVarIndex = i;
    const int startDxdydzVarIndex = strutCount + struts[i].startVertexIndex * 3;
    const int endDxdydzVarIndex = strutCount + struts[i].endVertexIndex * 3;

    // Non-equal strut endpoint displacement implies internal force
    A[matrixCols * (vertexCount * 3 + i) + startDxdydzVarIndex + 0] = dir.x;
    A[matrixCols * (vertexCount * 3 + i) + startDxdydzVarIndex + 1] = dir.y;
    A[matrixCols * (vertexCount * 3 + i) + startDxdydzVarIndex + 2] = dir.z;
    if(struts[i].endVertexIndex >= 0) {
      A[matrixCols * (vertexCount * 3 + i) + endDxdydzVarIndex + 0] = -dir.x;
      A[matrixCols * (vertexCount * 3 + i) + endDxdydzVarIndex + 1] = -dir.y;
      A[matrixCols * (vertexCount * 3 + i) + endDxdydzVarIndex + 2] = -dir.z;
    }
    A[matrixCols * (vertexCount * 3 + i) + forceVarIndex] = struts[i].elasticity;
  }

  {
    const int anchorRows = vertexCount * 3 + strutCount;
    const int vertexIndex = 0;
    const int dxdydzVarIndex = strutCount + vertexIndex * 3;

    A[matrixCols * (anchorRows + 0) + dxdydzVarIndex + 0] = 1;
    A[matrixCols * (anchorRows + 1) + dxdydzVarIndex + 1] = 1;
    A[matrixCols * (anchorRows + 2) + dxdydzVarIndex + 2] = 1;
    x[(anchorRows + 0)] = 0;
    x[(anchorRows + 1)] = 0;
    x[(anchorRows + 2)] = 0;
  }

  double result[matrixCols];
  int success = solveLinearEquationsByLU(A, x, result, matrixRows, matrixCols);
  if(!success) return 0;

  for(int i = 0; i < MAIN_AXIS_COUNT; ++i) {
    const int forceVarIndex = i;

    forces[i] = result[forceVarIndex];
  }

  return 1;
}

int calculateSliderPositions(
    Position platform,
    Displacement attachmentTarget[MAIN_AXIS_COUNT],
    double effectiveStrutLength[MAIN_AXIS_COUNT],
    Displacement sliderPositions[MAIN_AXIS_COUNT],
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

    sliderPositions[axis] = displacementAdd(sliderZero[axis], displacementScale(sliderUpStep[axis], k));
  }

  return 1;
}

static bool printAdjustedLengths = false;
static int calculateStepDeltas(Position endPos, int32_t stepDelta[MAIN_AXIS_COUNT]) {
  Displacement attachmentTarget[MAIN_AXIS_COUNT];
  Displacement sliderPositions[MAIN_AXIS_COUNT];
  double sliderLinearPos[MAIN_AXIS_COUNT];

  Position platform = relativePositionSub(endPos, platformTool);
  Position centerOfMassPosition = {
    platformCenterOfMass,
    { 1, 0, 0, 0 },
  };
  Position centerOfMass = relativePositionAdd(platform, centerOfMassPosition);

  double effectiveStrutLength[MAIN_AXIS_COUNT];
  memcpy(effectiveStrutLength, strutLength, sizeof(strutLength));

  if(!calculateSliderPositions(platform, attachmentTarget, effectiveStrutLength, sliderPositions, sliderLinearPos)) {
    return 0;
  }

  if(forceLimiting) {
    double forces[MAIN_AXIS_COUNT];
    if(!calculateStrutForces(centerOfMass.disp, attachmentTarget, sliderPositions, strutLength, forces)) {
      return 0;
    }

    for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
      effectiveStrutLength[axis] -= forces[axis] > 0? sliderBackslash[axis]: -sliderBackslash[axis];
      effectiveStrutLength[axis] -= forces[axis] * sliderElasticity[axis];

      printf("strut len delta[%d]: %lf\n", axis, effectiveStrutLength[axis] - strutLength[axis]);
    }

    if(printAdjustedLengths) {
      console_send_str("Adjusted strut lengths (mm): ");
      for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
        console_send_double(effectiveStrutLength[axis] - strutLength[axis]);
        console_send_str(" ");
      }
      console_send_str("\r\n");
    }

    if(!calculateSliderPositions(platform, attachmentTarget, effectiveStrutLength, sliderPositions, sliderLinearPos)) {
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
  platformCenterOfMass = zero.disp;
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
  printAdjustedLengths = true;
  if(!calculateStepDeltas(tool, stepDelta)) {
    console_send_str("step delta calculation failed\r\n");
    return;
  }
  printAdjustedLengths = false;

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

void dumpStepPositions() {
  console_send_str("Step positions ");
  for(int axis = 0; axis < MAIN_AXIS_COUNT; ++axis) {
    console_send_int32(stepsUp[axis]);
    console_send_str(" ");
  }
  console_send_str("\r\n");
}
