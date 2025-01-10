#include "microscope_auto_y.h"

#include "time.h"
#include "connections.h"
#include "terminal.h"
#include "current_position.h"
#include "printer.h"
#include "microscope_y_distance.h"

#include <memory>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace std;

unique_ptr<MicroscopeAutoY> MicroscopeAutoY::open(Connections *connections, double yEnd, double yStep) {
  auto result = make_unique<MicroscopeAutoY>();
  result->connections = connections;
  result->yEnd = yEnd;

  if(!connections->currentPosition) {
    cerr << "No current position available during auto y-target." << endl;
    return {};
  }

  if(!connections->microscopeYDistance) {
    cerr << "No distance measurements available during auto y-target." << endl;
    return {};
  }

  auto pos = connections->currentPosition->readPrinter();
  if(!pos) {
    cerr << "No current position known during auto y-target." << endl;
    return {};
  }

  result->yStep = pos->disp.y < yEnd? yStep: -yStep;

  result->nextStep = 0;
  result->resetYDistanceAt = now() + 12'000'000'000;

  return result;
}

void MicroscopeAutoY::tick() {
  if(resetYDistanceAt < now()) {
    connections->microscopeYDistance->reset();
    auto pos = connections->currentPosition->readPrinter();
    if(!pos) {
      cerr << "No current position known during auto y-target." << endl;
      stopTicked(connections->tickers);
      return;
    }

    double lower = min(pos->disp.y, yEnd);
    double upper = max(pos->disp.y, yEnd);
    connections->microscopeYDistance->enableRangePlot(lower, upper);
    resetYDistanceAt = ~0ull;
  }
  if(now() < nextStep) return;
  if(!connections->currentPosition) return;

  auto pos = connections->currentPosition->readPrinter();
  if(!pos) {
    cerr << "No current position known during auto y-target." << endl;
    stopTicked(connections->tickers);
    return;
  }

  auto newPos = *pos;
  newPos.disp.y += yStep;

  if((yStep < 0 && newPos.disp.y < yEnd) ||
      (yStep > 0 && newPos.disp.y > yEnd)) {
    connections->terminal->write("Auto-Y completed.\n");
    stopTicked(connections->tickers);

    newPos = connections->microscopeYDistance->readBestPosition();
  }

  ostringstream command;
  command << setprecision(10)
    << "tool:move " << newPos.disp.x
    << " " << newPos.disp.y
    << " " << newPos.disp.z
    << " " << newPos.rot.r
    << " " << newPos.rot.i
    << " " << newPos.rot.j
    << " " << newPos.rot.k
    << "\r\n";
  auto cmdStr = command.str();

  connections->terminal->write(cmdStr.data(), cmdStr.size());
  if(connections->printer) {
    connections->printer->write(cmdStr.data(), cmdStr.size());
  }

  nextStep = now() + 10'000'000'000;
}
