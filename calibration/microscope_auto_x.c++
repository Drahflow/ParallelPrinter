#include "microscope_auto_x.h"

#include "time.h"
#include "connections.h"
#include "terminal.h"
#include "current_position.h"
#include "printer.h"
#include "microscope_x_distance.h"

#include <memory>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace std;

unique_ptr<MicroscopeAutoX> MicroscopeAutoX::open(Connections *connections, double xEnd, double xStep) {
  auto result = make_unique<MicroscopeAutoX>();
  result->connections = connections;
  result->xEnd = xEnd;

  if(!connections->currentPosition) {
    cerr << "No current position available during auto x-target." << endl;
    return {};
  }

  if(!connections->microscopeXDistance) {
    cerr << "No distance measurements available during auto x-target." << endl;
    return {};
  }

  auto pos = connections->currentPosition->readPrinter();
  if(!pos) {
    cerr << "No current position known during auto x-target." << endl;
    return {};
  }

  result->xStep = pos->disp.x < xEnd? xStep: -xStep;

  result->nextStep = 0;
  result->resetXDistanceAt = now() + 12'000'000'000;

  return result;
}

void MicroscopeAutoX::tick() {
  if(resetXDistanceAt < now()) {
    connections->microscopeXDistance->reset();
    auto pos = connections->currentPosition->readPrinter();
    if(!pos) {
      cerr << "No current position known during auto x-target." << endl;
      stopTicked(connections->tickers);
      return;
    }

    double lower = min(pos->disp.x, xEnd);
    double upper = max(pos->disp.x, xEnd);
    connections->microscopeXDistance->enableRangePlot(lower, upper);
    resetXDistanceAt = ~0ull;
  }
  if(now() < nextStep) return;
  if(!connections->currentPosition) return;

  auto pos = connections->currentPosition->readPrinter();
  if(!pos) {
    cerr << "No current position known during auto x-target." << endl;
    stopTicked(connections->tickers);
    return;
  }

  auto newPos = *pos;
  newPos.disp.x += xStep;

  if((xStep < 0 && newPos.disp.x < xEnd) ||
      (xStep > 0 && newPos.disp.x > xEnd)) {
    connections->terminal->write("Auto-X completed.\n");
    stopTicked(connections->tickers);

    newPos = connections->microscopeXDistance->readBestPosition();
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
