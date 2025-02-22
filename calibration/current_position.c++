#include "current_position.h"

#include <iostream>

using namespace std;

unique_ptr<CurrentPosition> CurrentPosition::open(Connections *connections) {
  auto result = make_unique<CurrentPosition>();
  result->connections = connections;

  return result;
}

void CurrentPosition::parsePrinterCommand(const vector<string> &args) {
  if(args[0] == "tool:move") {
    Position pos;

    if(!(
      parseDouble(args[1], &pos.disp.x) &&
      parseDouble(args[2], &pos.disp.y) &&
      parseDouble(args[3], &pos.disp.z) &&
      parseDouble(args[4], &pos.rot.r) &&
      parseDouble(args[5], &pos.rot.i) &&
      parseDouble(args[6], &pos.rot.j) &&
      parseDouble(args[7], &pos.rot.k)
    )) {
      cerr << "Could not parse 'tool:move' command for current position." << endl;
      return;
    }

    printer = pos;
  }
}

void CurrentPosition::parsePrinterReply(const vector<string> &args) {
  if(args.size() == 6 &&
    args[0] == "Motor:" &&
    args[2] == "Kinematics:" &&
    args[4] == "Interrupt:") {

    if(args[3] != args[5]) {
      cerr << "Kinematic and interrupt based steps counts disagree." << endl;
    }

    double axisDbl;
    double stepsDbl;

    if(!(
      parseDouble(args[1], &axisDbl) &&
      parseDouble(args[5], &stepsDbl)
    )) {
      cerr << "Could not parse step count reply for current position." << endl;
      return;
    }

    auto axis = static_cast<int>(axisDbl);
    if(axis < 0 || axis > 6) {
      cerr << "Step position report for axis outside bounds." << endl;
      return;
    }

    if(!steps) steps = StepCounts{};
    steps->counts[axis] = static_cast<int>(stepsDbl);
  }
}

void CurrentPosition::parseTabletCommand(const vector<string> &args) {
  if(args[0] == "target" || args[0] == "target:xy" || args[0] == "target:xy:focus") {
    double x, y;

    if(!(
      parseDouble(args[1], &x) &&
      parseDouble(args[2], &y)
    )) {
      cerr << "Could not parse 'target' command for current position." << endl;
      return;
    }

    target = TargetPosition{.x = static_cast<int>(x), .y = static_cast<int>(y)};
  }
}
