#include "calibration_log.h"

#include "connections.h"
#include "terminal.h"
#include "current_position.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <sstream>
#include <unistd.h>

using namespace std;

unique_ptr<CalibrationLog> CalibrationLog::open(const string &filename, Connections *connections) {
  auto result = make_unique<CalibrationLog>();
  result->connections = connections;
  result->tabletPlacement = 0;

  result->fd = ::open(filename.c_str(), O_WRONLY | O_APPEND | O_CLOEXEC | O_CREAT, 0666);
  if(result->fd == -1) {
    cerr << "Could not append to " << filename << ": " << strerror(errno) << endl;
    return {};
  }

  connections->terminal->write("Log file appending.\n", strlen("Log file appending.\n"));

  return result;
}

CalibrationLog::~CalibrationLog() {
  if(fd != -1) {
    int ret = close(fd);
    if(ret == -1) {
      cerr << "Could not close calibration log: " << strerror(errno) << endl;
    }
  }
}

void CalibrationLog::write(const char *buf, int len) {
  while(len) {
    int ret = ::write(fd, buf, len);
    if(ret == -1) {
      cerr << "Calibration log write failed: " << strerror(errno) << endl;
      return;
    }

    buf += ret;
    len -= ret;
  }
}

void CalibrationLog::setTabletPlacement(int n) {
  tabletPlacement = n;
}

void CalibrationLog::writeCurrentPosition() {
  if(!connections->currentPosition) {
    cerr << "No current position tracking." << endl;
    return;
  }

  auto &cur = connections->currentPosition;
  auto printer = cur->readPrinter();
  if(!printer) {
    cerr << "No current printer position available." << endl;
    return;
  }

  auto steps = cur->readSteps();
  if(!steps) {
    cerr << "No current steps position available." << endl;
    return;
  }

  auto target = cur->readTarget();
  if(!target) {
    cerr << "No current target position available." << endl;
    return;
  }

  ostringstream lineBuf;
  lineBuf << "Placement " << tabletPlacement << " ";
  lineBuf << "Target " << target->x << " " << target->y << " ";
  lineBuf << "Steps";
  for(int i = 0; i < mainAxisCount; ++i) {
    lineBuf << " " << steps->counts[i];
  }
  lineBuf << "\n";

  auto line = lineBuf.str();
  write(line.data(), line.size());
}

void CalibrationLog::writeComment(const string &s) {
  auto line = string("# ") + s + "\n";

  write(line.data(), line.size());
}
