#ifndef H_5573E317_457B_412D_9174_591FD89948D5
#define H_5573E317_457B_412D_9174_591FD89948D5

#include "config.h"

#include <stdbool.h>
#include <stdint.h>

// Axis orientation, as seen from printer front:
// x goes to the right
// y goes to the back
// z goes up

extern bool kinematicsAvailable;
typedef struct {
  double x, y, z;
} Displacement;
typedef struct {
  double r, i, j, k;
} Quaternion;
typedef Quaternion Rotation; // unit quaternion

typedef struct {
  Displacement disp;
  Rotation rot;
} Position;

extern Displacement platformAttachment[MAIN_AXIS_COUNT]; // relative to platform center
extern Position platformTool; // relative to platform center
extern Displacement sliderZero[MAIN_AXIS_COUNT];
extern Displacement sliderUpStep[MAIN_AXIS_COUNT]; // vector of movement for 1 up microstep
extern double strutLength[MAIN_AXIS_COUNT];
extern double maximumStepsPerSecond;
extern double minimumMotionDuration;
extern double kinematicsSubdivisionInterval;
extern double rotationLinearThreshold;
extern int32_t upperLimit[MAIN_AXIS_COUNT];
extern int32_t lowerLimit[MAIN_AXIS_COUNT];
extern bool abovePlatform[MAIN_AXIS_COUNT];
extern double sliderBackslash[MAIN_AXIS_COUNT];
extern double sliderElasticity[MAIN_AXIS_COUNT];
extern double forceLimit[MAIN_AXIS_COUNT];

// Positions at end of current kinematics motor schedule
extern int32_t stepsUp[MAIN_AXIS_COUNT];
extern Position tool; // used only to interpolate continuous tool motion

void runKinematics();
void setZero(Position initialToolPosition); // initialToolPosition only used for first motion plan
void setToolAttachedAt(Position);
void setMovementSpeed(double mmPerSecond);
void setRotationSpeed(double degreesPerSecond);
void moveTo(Position);
void moveAgain();
void kinematicsStop();
void checkForceLimiting();

#endif
