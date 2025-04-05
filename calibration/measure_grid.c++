#include "measure_grid.h"

#include "connections.h"
#include "terminal.h"
#include "microscope_auto_xyz.h"
#include "current_position.h"
#include "tablet.h"
#include "calibration_log.h"
#include "time.h"

#include <iostream>

using namespace std;

constexpr int NO_ACTION = 0;
constexpr int START_COARSE = 1;
constexpr int START_FINE = 2;

unique_ptr<MeasureGrid> MeasureGrid::open(Connections *connections,
    int xStep, int yStep, int xEnd, int yEnd,
    double scaleFactor,
    double coarsePrecision, double coarseFocusSpread, double coarseSettleTime,
    double finePrecision, double fineFocusSpread, double fineSettleTime) {
  auto result = make_unique<MeasureGrid>();

  if(!connections->currentPosition) {
    cerr << "No current position available during grid measurement." << endl;
    return {};
  }

  auto target = connections->currentPosition->readTarget();
  if(!target) {
    cerr << "No current target known during grid measurement." << endl;
    return {};
  }

  result->connections = connections;
  result->xBegin = target->x;
  result->yBegin = target->y;
  result->xEnd = xEnd;
  result->yEnd = yEnd;
  result->xStep = xStep;
  result->yStep = yStep;
  result->scaleFactor = scaleFactor;
  result->coarsePrecision = coarsePrecision;
  result->coarseFocusSpread = coarseFocusSpread;
  result->coarseSettleTime = coarseSettleTime;
  result->finePrecision = finePrecision;
  result->fineFocusSpread = fineFocusSpread;
  result->fineSettleTime = fineSettleTime;
  result->nextStep = now();
  result->nextAction = NO_ACTION;

  result->startCoarseXYZ();

  return result;
}

void MeasureGrid::tick() {
  if(now() < nextStep) return;
  if(nextAction == START_COARSE) {
    nextAction = NO_ACTION;
    startCoarseXYZ();
    return;
  }
  if(nextAction == START_FINE) {
    nextAction = NO_ACTION;
    startFineXYZ();
    return;
  }

  if(connections->microscopeAutoXYZ && !connections->microscopeAutoXYZ->done()) return;

  if(!connections->calibrationLog) {
    cerr << "No calibration log available." << endl;
    stopTicked(connections->tickers);
    return;
  }

  if(!connections->terminal) {
    cerr << "No terminal available." << endl;
    stopTicked(connections->tickers);
    return;
  }

  if(!connections->tablet) {
    cerr << "No tablet available." << endl;
    stopTicked(connections->tickers);
    return;
  }

  if(!connections->currentPosition) {
    cerr << "No current position available." << endl;
    stopTicked(connections->tickers);
    return;
  }

  auto target = connections->currentPosition->readTarget();
  if(!target) {
    cerr << "No current target known during grid measurement." << endl;
    stopTicked(connections->tickers);
    return;
  }

  bool shouldFineXYZ = ((target->x - xBegin) % xStep == 0) && ((target->y - yBegin) % yStep == 0);
  if(didFineXYZ) {
    connections->calibrationLog->writeCurrentPosition();
  } else if(shouldFineXYZ) {
    connections->tablet->setTarget(target->x, target->y);
    nextStep = now() + 2'000'000'000;
    nextAction = START_FINE;
    return;
  }

  bool movingToLargerX = ((target->y - yBegin) / yStep) % 2 == 0;
  if(movingToLargerX) {
    if(target->x < xEnd) {
      connections->tablet->setTarget(target->x + 2, target->y);
    } else if(target->y < yEnd) {
      connections->tablet->setTarget(target->x, target->y + 2);
    } else {
      connections->terminal->write("Grid measurement complete.\n");
      stopTicked(connections->tickers);
      return;
    }
  } else {
    if(target->x > xBegin) {
      connections->tablet->setTarget(target->x - 2, target->y);
    } else if(target->y < yEnd) {
      connections->tablet->setTarget(target->x, target->y + 2);
    } else {
      connections->terminal->write("Grid measurement complete.\n");
      stopTicked(connections->tickers);
      return;
    }
  }

  nextStep = now() + 2'000'000'000;
  nextAction = START_COARSE;
}

void MeasureGrid::startFineXYZ() {
  auto autoXYZ = MicroscopeAutoXYZ::open(connections, scaleFactor, finePrecision, fineFocusSpread, fineSettleTime);
  if(!autoXYZ) {
    cerr << "Could not start auto-XYZ procedure." << endl;
    return;
  }

  autoXYZ->startTicked(connections->tickers);
  connections->microscopeAutoXYZ = std::move(autoXYZ);
  didFineXYZ = true;
}

void MeasureGrid::startCoarseXYZ() {
  auto autoXYZ = MicroscopeAutoXYZ::open(connections, scaleFactor, coarsePrecision, coarseFocusSpread, coarseSettleTime);
  if(!autoXYZ) {
    cerr << "Could not start auto-XYZ procedure." << endl;
    return;
  }

  autoXYZ->startTicked(connections->tickers);
  connections->microscopeAutoXYZ = std::move(autoXYZ);
  didFineXYZ = false;
}
