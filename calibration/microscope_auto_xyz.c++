#include "microscope_auto_xyz.h"

#include "time.h"
#include "connections.h"
#include "terminal.h"
#include "current_position.h"
#include "printer.h"
#include "tablet.h"
#include "microscope_x_distance.h"
#include "microscope_y_distance.h"
#include "microscope_focus.h"

#include <memory>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <cassert>
#include <cmath>

using namespace std;

constexpr int FIRST_XY = 1;
constexpr int FIRST_Z = 2;
constexpr int SECOND_XY = 3;
constexpr int DONE = 4;

unique_ptr<MicroscopeAutoXYZ> MicroscopeAutoXYZ::open(Connections *connections,
    double scaleFactor, double precision, double focusSpread, double settleTime) {
  auto result = make_unique<MicroscopeAutoXYZ>();
  result->connections = connections;
  result->scaleFactor = scaleFactor;
  result->precision = precision;
  result->focusSpread = focusSpread;
  result->settleTime = settleTime;
  result->state = FIRST_XY;

  if(!connections->currentPosition) {
    cerr << "No current position available during auto xyz-target." << endl;
    return {};
  }

  if(!connections->microscopeXDistance) {
    cerr << "No x-distance measurements available during auto xyz-target." << endl;
    return {};
  }

  if(!connections->microscopeYDistance) {
    cerr << "No y-distance measurements available during auto xyz-target." << endl;
    return {};
  }

  if(!connections->microscopeFocus) {
    cerr << "No focus-distance measurements available during auto xyz-target." << endl;
    return {};
  }

  auto pos = connections->currentPosition->readPrinter();
  if(!pos) {
    cerr << "No current position known during auto xyz-target." << endl;
    return {};
  }

  auto target = connections->currentPosition->readTarget();
  if(!target) {
    cerr << "No current target known during auto xyz-target." << endl;
    return {};
  }

  result->nextStep = 0;

  return result;
}

void MicroscopeAutoXYZ::tick() {
  if(now() < nextStep) return;
  if(!connections->currentPosition || !connections->printer) {
    connections->terminal->write("No current position reader or no target printer, aborting auto xyz.\n");
    stopTicked(connections->tickers);
    return;
  }

  auto pos = connections->currentPosition->readPrinter();
  if(!pos) {
    connections->terminal->write("Couldn't read position from printer, aborting auto xyz.\n");
    stopTicked(connections->tickers);
    return;
  }

  auto target = connections->currentPosition->readTarget();
  if(!target) {
    cerr << "No current target known during auto xyz-target." << endl;
    return;
  }

  if(state == FIRST_XY || state == SECOND_XY) {
    double dx = connections->microscopeXDistance->readDistance() * scaleFactor;
    double dy = connections->microscopeYDistance->readDistance() * scaleFactor;
    if(connections->terminal) {
      ostringstream out;
      out << "X/Y deltas: dx: " << dx << ", dy: " << dy << endl;
      connections->terminal->write(out.str());
    }

    if(dx * dx + dy * dy > 25) {
      cerr << "Next move >5mm, aborting for safety reasons. Choose a smaller scale factor." << endl;
      stopTicked(connections->tickers);
      return;
    }

    if(dx * dx + dy * dy < precision * precision) {
      connections->terminal->write("X/Y complete.\n");

      if(state == SECOND_XY) {
        string statusCmd("status:steps\r\n");
        connections->printer->write(statusCmd.data(), statusCmd.size());
        connections->terminal->write("XYZ complete.\n");
        stopTicked(connections->tickers);
        state = DONE;
        return;
      }

      state = FIRST_Z;
      focusStep = focusSpread * 2 / 14;
      for(int i = 0; i < 15; ++i) {
        focusMeasurements[i] = FocusMeasurement{
          .position = (pos->disp.z - focusSpread) + focusStep * i,
          .data = ~0ull,
        };
      }
      focusTargetIndex = -1;

      connections->tablet->setFocusTarget(target->x, target->y);
      nextStep = now() + 100'000'000;
      return;
    }

    auto newPos = *pos;
    newPos.disp.x -= dx;
    newPos.disp.y -= dy;

    connections->printer->moveTo(newPos);
    connections->tablet->setTarget(target->x, target->y);

    nextStep = now() + settleTime;
    return;
  }

  if(state == FIRST_Z) {
    if(focusTargetIndex != -1) {
      focusMeasurements[focusTargetIndex].data = connections->microscopeFocus->readFocus();
    }

    for(int i = 0; i < 15; ++i) {
      if(focusMeasurements[i].data != ~0ull) continue;

      auto z = focusMeasurements[i].position;
      if(fabs(pos->disp.z - z) > 5) {
        cerr << "Next move >5mm, aborting for safety reasons. Choose a smaller focus spread." << endl;
        stopTicked(connections->tickers);
        return;
      }

      auto newPos = *pos;
      newPos.disp.z = z;
      focusTargetIndex = i;

      connections->printer->moveTo(newPos);
      nextStep = now() + settleTime;
      return;
    }

    // All focus data is now available.
    if(connections->terminal) {
      ostringstream out;
      out << "Focus data: " << endl;
      for(int i = 0; i < 15; ++i) {
        out << setw(10) << focusMeasurements[i].position << ": " << focusMeasurements[i].data << endl;
      }
      connections->terminal->write(out.str());
    }

    sort(focusMeasurements, focusMeasurements + 15, [](const FocusMeasurement &a, const FocusMeasurement &b) {
      return b.data < a.data;
    });

    focusStep /= 2;
    if(focusStep < precision) {
      if(focusMeasurements[0].data > 30'000'000) {
        connections->terminal->write("Z complete.\n");

        auto newPos = *pos;
        newPos.disp.z = focusMeasurements[0].position;
        focusTargetIndex = -1;

        connections->printer->moveTo(newPos);
        nextStep = now() + settleTime;
        state = SECOND_XY;
      } else if(focusMeasurements[0].data < 100'000) {
        cerr << "Focus lost. Aborting." << endl;
        stopTicked(connections->tickers);
        return;
      } else {
        connections->terminal->write("Z complete, but focus seems bad.\n");

        double startZ = focusMeasurements[0].position;
        double largerFocusSpread = focusSpread * 2;

        focusStep = largerFocusSpread * 2 / 14;
        for(int i = 0; i < 15; ++i) {
          focusMeasurements[i] = FocusMeasurement{
            .position = (startZ - largerFocusSpread) + focusStep * i,
            .data = ~0ull,
          };
        }
        focusTargetIndex = -1;

        connections->tablet->setFocusTarget(target->x, target->y);
        nextStep = now() + 100'000'000;
      }
      return;
    }

    int source = 0;
    int target = 14;
    while(source < target) {
      focusMeasurements[target] = FocusMeasurement{
        .position = focusMeasurements[source].position - focusStep,
        .data = ~0ull,
      };

      {
        bool exists = false;
        for(int i = 0; i < 15; ++i) {
          if(i == target) continue;
          if(fabs(focusMeasurements[i].position - focusMeasurements[target].position) < 1e-12) exists = true;
        }
        if(!exists) --target;
      }

      if(!(source < target)) break;

      focusMeasurements[target] = FocusMeasurement{
        .position = focusMeasurements[source].position + focusStep,
        .data = ~0ull,
      };

      {
        bool exists = false;
        for(int i = 0; i < 15; ++i) {
          if(i == target) continue;
          if(fabs(focusMeasurements[i].position - focusMeasurements[target].position) < 1e-12) exists = true;
        }
        if(!exists) --target;
      }

      ++source;
    }

    sort(focusMeasurements, focusMeasurements + 15, [](const FocusMeasurement &a, const FocusMeasurement &b) {
      return a.position < b.position;
    });

    nextStep = now() + 100'000'000;
    return;
  }

  assert(false);
}

bool MicroscopeAutoXYZ::done() const {
  return state == DONE;
}
