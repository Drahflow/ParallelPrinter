#include "microscope_autofocus.h"

#include "time.h"
#include "connections.h"
#include "terminal.h"
#include "current_position.h"
#include "printer.h"
#include "microscope_focus.h"

#include <memory>
#include <iostream>
#include <iomanip>
#include <sstream>

using namespace std;

unique_ptr<MicroscopeAutofocus> MicroscopeAutofocus::open(Connections *connections, double zEnd, double zStep) {
  auto result = make_unique<MicroscopeAutofocus>();
  result->connections = connections;
  result->zEnd = zEnd;

  if(!connections->currentPosition) {
    cerr << "No current position available during autofocus." << endl;
    return {};
  }

  if(!connections->microscopeFocus) {
    cerr << "No focus measurements available during autofocus." << endl;
    return {};
  }

  auto pos = connections->currentPosition->readPrinter();
  if(!pos) {
    cerr << "No current position known during autofocus." << endl;
    return {};
  }

  result->zStep = pos->disp.z < zEnd? zStep: -zStep;

  result->nextStep = 0;
  result->resetFocusAt = now() + 12'000'000'000;

  return result;
}

void MicroscopeAutofocus::tick() {
  if(resetFocusAt < now()) {
    connections->microscopeFocus->reset();
    auto pos = connections->currentPosition->readPrinter();
    if(!pos) {
      cerr << "No current position known during autofocus." << endl;
      stopTicked(connections->tickers);
      return;
    }

    double lower = min(pos->disp.z, zEnd);
    double upper = max(pos->disp.z, zEnd);
    connections->microscopeFocus->enableRangePlot(lower, upper);
    resetFocusAt = ~0ull;
  }
  if(now() < nextStep) return;
  if(!connections->currentPosition) return;

  auto pos = connections->currentPosition->readPrinter();
  if(!pos) {
    cerr << "No current position known during autofocus." << endl;
    stopTicked(connections->tickers);
    return;
  }

  auto newPos = *pos;
  newPos.disp.z += zStep;

  if((zStep < 0 && newPos.disp.z < zEnd) ||
      (zStep > 0 && newPos.disp.z > zEnd)) {
    connections->terminal->write("Autofocus completed.\n");
    stopTicked(connections->tickers);

    newPos = connections->microscopeFocus->readBestPosition();
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
