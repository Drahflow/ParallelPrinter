#include "calibration_log.h"

#include "connections.h"
#include "terminal.h"

#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <cstring>
#include <iostream>
#include <unistd.h>

using namespace std;

unique_ptr<CalibrationLog> CalibrationLog::open(const string &filename, Connections *connections) {
  auto result = make_unique<CalibrationLog>();
  result->connections = connections;

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
      cerr << "Terminal write failed: " << strerror(errno) << endl;
      return;
    }

    buf += ret;
    len -= ret;
  }
}
