#include "terminal.h"

#include "connections.h"
#include "microscope.h"
#include "printer.h"
#include "tablet.h"
#include "video_feed.h"
#include "calibration_log.h"
#include "current_position.h"
#include "microscope_focus.h"
#include "microscope_x_distance.h"
#include "microscope_y_distance.h"
#include "globals.h"
#include "main.h"
#include "time.h"
#include "microscope_autofocus.h"
#include "microscope_auto_x.h"
#include "microscope_auto_y.h"
#include "microscope_auto_xyz.h"
#include "measure_grid.h"

#include <iostream>
#include <cstring>
#include <vector>
#include <sstream>
#include <unistd.h>
#include <ranges>

using namespace std;
using namespace std::views;
using namespace std::ranges;

unique_ptr<Terminal> Terminal::open(Connections *connections) {
  auto result = make_unique<Terminal>();
  result->connections = connections;

  result->write("Terminal open.\n", strlen("Terminal open.\n"));

  return result;
}

int Terminal::getEpollFd() {
  return 0;
}

Terminal::~Terminal() { }

void Terminal::available() {
  if(readPos >= sizeof(buffer)) {
    cerr << "Overly long input line: " << buffer << endl;
    readPos = 0;
  }

  int ret = read(0, buffer + readPos, sizeof(buffer) - readPos);
  if(ret == -1) {
    cerr << "Error while reading input: " << strerror(errno) << endl;
    return;
  }
  if(ret == 0) {
    cerr << "EOF on terminal." << endl;
    runMainLoopUntil = now() + 100'000'000;
    removeFromEpoll(connections->epollFd);
    return;
  }
  readPos += ret;

  for(unsigned int i = 0; i < readPos; ++i) {
    if(buffer[i] == '\n') {
      buffer[i] = '\0';
      parse(buffer);

      ++i;
      memmove(buffer, buffer + i, readPos - i);
      readPos -= i;
      i = 0;
    }
  }
}

void Terminal::parse(const char *input) {
  vector<string> args;
  for(const char *i = input, *start = input; ; ++i) {
    if(!*i || *i == ' ') {
      args.push_back(string(start, i - start));
      start = i + 1;
    }

    if(!*i) break;
  }

#ifdef DEBUG_INTERCEPTIONS
  cerr << "Got:";
  for(auto &i: args) cerr << " " << i;
  cerr << endl;
#endif

  if(args.empty()) return;

  if(args[0] == "connect:printer" && args.size() > 1) {
    cout << "Connecting to printer on " << args[1] << endl;
    if((connections->printer = Printer::open(args[1].c_str(), connections))) {
      connections->printer->addToEpoll(connections->epollFd);
    }
  } else if(args[0] == "connect:microscope" && args.size() > 1) {
    cout << "Connecting to microscope on " << args[1] << endl;
    if((connections->microscope = Microscope::open(args[1].c_str(), connections))) {
      connections->microscope->addToEpoll(connections->epollFd);
    }
  } else if(args[0] == "connect:tablet") {
    cout << "Connecting to tablet" << endl;
    if((connections->tablet = Tablet::open(connections))) {
      connections->tablet->addToEpoll(connections->epollFd);
    }
  } else if(args[0] == "connect:video") {
    cout << "Connecting to video output to " << args[1] << endl;
    if((connections->videoFeed = VideoFeed::open(args[1], connections))) {
      connections->videoFeed->addToEpoll(connections->epollFd);
    }
  } else if(args[0] == "connect:log") {
    cout << "Appending to calibration log " << args[1] << endl;
    connections->calibrationLog = CalibrationLog::open(args[1], connections);
  } else if(args[0] == "tablet") {
    if(connections->tablet) {
      auto s = (args | drop(1) | join_with(' ') | to<string>()) + "\n";
      connections->tablet->write(s.data(), s.size());
    } else {
      cerr << "Tablet not yet connected." << endl;
    }
  } else if(args[0] == "log:placement") {
    if(!connections->calibrationLog) {
      cerr << "Calibration log not connected." << endl;
      return;
    }
    if(args.size() != 2) {
      cerr << "Expected <placement index>." << endl;
      return;
    }

    double index;
    if(!parseDouble(args[1], &index)) {
      cerr << "Could not parse index." << endl;
      return;
    }

    connections->calibrationLog->setTabletPlacement(static_cast<int>(index));
  } else if(args[0] == "log:comment") {
    if(!connections->calibrationLog) {
      cerr << "Calibration log not connected." << endl;
      return;
    }

    connections->calibrationLog->writeComment(args | drop(1) | join_with(' ') | to<string>());
  } else if(args[0] == "log:position") {
    if(!connections->calibrationLog) {
      cerr << "Calibration log not connected." << endl;
      return;
    }

    connections->calibrationLog->writeCurrentPosition();
  } else if(args[0] == "focus:reset") {
    if(!connections->microscopeFocus) {
      cerr << "Microscope focus not setup." << endl;
      return;
    }

    connections->microscopeFocus->reset();
  } else if(args[0] == "focus:auto") {
    if(!connections->microscopeFocus) {
      cerr << "Microscope focus not setup." << endl;
      return;
    }

    double zEnd;
    if(!parseDouble(args[1], &zEnd)) {
      cerr << "Could not parse zEnd." << endl;
      return;
    }

    double zStep;
    if(!parseDouble(args[2], &zStep)) {
      cerr << "Could not parse zStep." << endl;
      return;
    }

    auto autofocus = MicroscopeAutofocus::open(connections, zEnd, zStep);
    if(!autofocus) {
      cerr << "Could not start autofocus procedure." << endl;
      return;
    }

    autofocus->startTicked(connections->tickers);
    connections->microscopeAutofocus = std::move(autofocus);
  } else if(args[0] == "x-dist:reset") {
    if(!connections->microscopeXDistance) {
      cerr << "Microscope x distance measurement not setup." << endl;
      return;
    }

    connections->microscopeXDistance->reset();
  } else if(args[0] == "x-dist:auto") {
    if(!connections->microscopeXDistance) {
      cerr << "Microscope x distance not setup." << endl;
      return;
    }

    double xEnd;
    if(!parseDouble(args[1], &xEnd)) {
      cerr << "Could not parse xEnd." << endl;
      return;
    }

    double xStep;
    if(!parseDouble(args[2], &xStep)) {
      cerr << "Could not parse xStep." << endl;
      return;
    }

    auto autoX = MicroscopeAutoX::open(connections, xEnd, xStep);
    if(!autoX) {
      cerr << "Could not start auto-X procedure." << endl;
      return;
    }

    autoX->startTicked(connections->tickers);
    connections->microscopeAutoX = std::move(autoX);
  } else if(args[0] == "y-dist:reset") {
    if(!connections->microscopeYDistance) {
      cerr << "Microscope y distance measurement not setup." << endl;
      return;
    }

    connections->microscopeYDistance->reset();
  } else if(args[0] == "y-dist:auto") {
    if(!connections->microscopeYDistance) {
      cerr << "Microscope y distance not setup." << endl;
      return;
    }

    double yEnd;
    if(!parseDouble(args[1], &yEnd)) {
      cerr << "Could not parse yEnd." << endl;
      return;
    }

    double yStep;
    if(!parseDouble(args[2], &yStep)) {
      cerr << "Could not parse yStep." << endl;
      return;
    }

    auto autoY = MicroscopeAutoY::open(connections, yEnd, yStep);
    if(!autoY) {
      cerr << "Could not start auto-Y procedure." << endl;
      return;
    }

    autoY->startTicked(connections->tickers);
    connections->microscopeAutoY = std::move(autoY);
  } else if(args[0] == "xyz:auto") {
    if(!connections->microscopeXDistance) {
      cerr << "Microscope x distance not setup." << endl;
      return;
    }

    if(!connections->microscopeYDistance) {
      cerr << "Microscope y distance not setup." << endl;
      return;
    }

    if(!connections->microscopeFocus) {
      cerr << "Microscope focus not setup." << endl;
      return;
    }

    double scaleFactor;
    if(!parseDouble(args[1], &scaleFactor)) {
      cerr << "Could not parse scaleFactor." << endl;
      return;
    }

    double precision;
    if(!parseDouble(args[2], &precision)) {
      cerr << "Could not parse precision." << endl;
      return;
    }

    double focusSpread;
    if(!parseDouble(args[3], &focusSpread)) {
      cerr << "Could not parse focus spread." << endl;
      return;
    }

    double settleTime; // input as ms, used as ns
    if(!parseDouble(args[4], &settleTime)) {
      cerr << "Could not parse settle time." << endl;
      return;
    }

    auto autoXYZ = MicroscopeAutoXYZ::open(connections, scaleFactor, precision, focusSpread, settleTime * 1'000'000);
    if(!autoXYZ) {
      cerr << "Could not start auto-XYZ procedure." << endl;
      return;
    }

    autoXYZ->startTicked(connections->tickers);
    connections->microscopeAutoXYZ = std::move(autoXYZ);
  } else if(args[0] == "measure:grid") {
    if(!connections->microscopeXDistance) {
      cerr << "Microscope x distance not setup." << endl;
      return;
    }

    if(!connections->microscopeYDistance) {
      cerr << "Microscope y distance not setup." << endl;
      return;
    }

    if(!connections->microscopeFocus) {
      cerr << "Microscope focus not setup." << endl;
      return;
    }

    double xStep, yStep, xEnd, yEnd;
    double scaleFactor;
    double coarsePrecision, coarseFocusSpread, coarseSettleTime;
    double finePrecision, fineFocusSpread, fineSettleTime;

    if(!parseDouble(args[1], &xStep)) {
      cerr << "Could not parse xStep." << endl;
      return;
    }

    if(!parseDouble(args[2], &yStep)) {
      cerr << "Could not parse yStep." << endl;
      return;
    }

    if(!parseDouble(args[3], &xEnd)) {
      cerr << "Could not parse xEnd." << endl;
      return;
    }

    if(!parseDouble(args[4], &yEnd)) {
      cerr << "Could not parse yEnd." << endl;
      return;
    }

    if(!parseDouble(args[5], &scaleFactor)) {
      cerr << "Could not parse scaleFactor." << endl;
      return;
    }

    if(!parseDouble(args[6], &coarsePrecision)) {
      cerr << "Could not parse coarsePrecision." << endl;
      return;
    }

    if(!parseDouble(args[7], &coarseFocusSpread)) {
      cerr << "Could not parse coarseFocusSpread." << endl;
      return;
    }

    if(!parseDouble(args[8], &coarseSettleTime)) {
      cerr << "Could not parse coarseSettleTime." << endl;
      return;
    }

    if(!parseDouble(args[9], &finePrecision)) {
      cerr << "Could not parse finePrecision." << endl;
      return;
    }

    if(!parseDouble(args[10], &fineFocusSpread)) {
      cerr << "Could not parse fineFocusSpread." << endl;
      return;
    }

    if(!parseDouble(args[11], &fineSettleTime)) {
      cerr << "Could not parse fineSettleTime." << endl;
      return;
    }

    auto measureGrid = MeasureGrid::open(connections,
        xStep, yStep, xEnd, yEnd,
        scaleFactor,
        coarsePrecision, coarseFocusSpread, coarseSettleTime * 1'000'000,
        finePrecision, fineFocusSpread, fineSettleTime * 1'000'000);
    if(!measureGrid) {
      cerr << "Could not start grid measurement." << endl;
      return;
    }

    measureGrid->startTicked(connections->tickers);
    connections->measureGrid = std::move(measureGrid);
  } else if(args[0] == "measure:grid:pause") {
    if(!connections->measureGrid) {
      cerr << "No grid measurement ongoing." << endl;
      return;
    }

    connections->measureGrid->stopTicked(connections->tickers);

    if(connections->microscopeAutoXYZ) {
      connections->microscopeAutoXYZ->stopTicked(connections->tickers);
    }
  } else if(args[0] == "measure:grid:unpause") {
    if(!connections->measureGrid) {
      cerr << "No grid measurement ongoing." << endl;
      return;
    }

    connections->measureGrid->startTicked(connections->tickers);
  } else if(args[0] == "stop") {
    if(connections->printer) {
      connections->printer->write(input, strlen(input));
      connections->printer->write("\r\n", 2);
    }

    if(connections->microscopeAutofocus) {
      connections->microscopeAutofocus->stopTicked(connections->tickers);
      connections->microscopeAutofocus.reset();
    }

    if(connections->microscopeAutoX) {
      connections->microscopeAutoX->stopTicked(connections->tickers);
      connections->microscopeAutoX.reset();
    }

    if(connections->microscopeAutoY) {
      connections->microscopeAutoY->stopTicked(connections->tickers);
      connections->microscopeAutoY.reset();
    }

    if(connections->microscopeAutoXYZ) {
      connections->microscopeAutoXYZ->stopTicked(connections->tickers);
      connections->microscopeAutoXYZ.reset();
    }

    if(connections->measureGrid) {
      connections->measureGrid->stopTicked(connections->tickers);
      connections->measureGrid.reset();
    }
  } else if(connections->printer) {
    connections->printer->write(input, strlen(input));
    connections->printer->write("\r\n", 2);
  } else {
    cerr << "Printer not yet connected, dropping unknown command." << endl;
  }
}

void Terminal::write(const char *buf) {
  write(buf, strlen(buf));
}

void Terminal::write(const string &str) {
  write(str.data(), str.length());
}

void Terminal::write(const char *buf, int len) {
  while(len) {
    int ret = ::write(1, buf, len);
    if(ret == -1) {
      cerr << "Terminal write failed: " << strerror(errno) << endl;
      return;
    }

    buf += ret;
    len -= ret;
  }
}
