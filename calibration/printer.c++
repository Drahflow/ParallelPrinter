#include "printer.h"

#include "connections.h"
#include "terminal.h"
#include "current_position.h"
#include "globals.h"
#include "geometry.h"

#include <iostream>
#include <sstream>
#include <iomanip>
#include <cstring>
#include <vector>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

unique_ptr<Printer> Printer::open(const string &device, Connections *connections) {
  auto result = make_unique<Printer>();
  result->connections = connections;
  result->lineBufferEnd = result->lineBuffer;

  result->fd = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_CLOEXEC);
  if(result->fd == -1) {
    cerr << "Could not open " << device << ": " << strerror(errno) << endl;
    return {};
  }

  struct termios setup;
  {
    int ret = tcgetattr(result->fd, &setup);
    if(ret == -1) {
      cerr << "Could not get terminal attributes: " << strerror(errno) << endl;
      return {};
    }
  }

  cfmakeraw(&setup);

  {
    int ret = cfsetispeed(&setup, B500000);
    if(ret == -1) {
      cerr << "Could not request input baud rate: " << strerror(errno) << endl;
      return {};
    }
  }

  {
    int ret = cfsetospeed(&setup, B500000);
    if(ret == -1) {
      cerr << "Could not request output baud rate: " << strerror(errno) << endl;
      return {};
    }
  }

  {
    int ret = tcsetattr(result->fd, TCSANOW, &setup);
    if(ret == -1) {
      cerr << "Could not set terminal attributes: " << strerror(errno) << endl;
      return {};
    }
  }

  return result;
}

Printer::~Printer() {
  if(fd != -1) {
    int ret = close(fd);
    if(ret == -1) {
      cerr << "Could not close tty: " << strerror(errno) << endl;
    }
  }
}

int Printer::getEpollFd() {
  return fd;
}

void Printer::parsePrinterReply(const char *buffer, int len) {
  auto copyLen = min(static_cast<ptrdiff_t>(len), lineBuffer + sizeof(lineBuffer) - lineBufferEnd);
  memcpy(lineBufferEnd, buffer, copyLen);
  lineBufferEnd += copyLen;

  if(lineBufferEnd == lineBuffer + sizeof(lineBuffer)) {
    cerr << "Printer reply parsing buffer exceeded. Resetting." << endl;
    lineBufferEnd = lineBuffer;
  }

  for(char *c = lineBuffer; c < lineBufferEnd; ++c) {
    if(*c == '\n') {
#ifdef DEBUG_INTERCEPTIONS
      cout << "Line from printer: " << string(lineBuffer, c - lineBuffer) << flush;
#endif
      char *lineEnd = c;
      if(lineEnd > lineBuffer && *(lineEnd - 1) == '\r') --lineEnd;

      vector<string> args;
      const char *i, *start;
      for(i = lineBuffer, start = lineBuffer; i < lineEnd; ++i) {
        if(*i == ' ') {
          args.push_back(string(start, i - start));
          start = i + 1;
        }
      }
      args.push_back(string(start, i - start));

#ifdef DEBUG_INTERCEPTIONS
      cerr << "Intercepted from printer:";
      for(auto &i: args) cerr << " " << i;
      cerr << endl;
#endif

      if(connections->currentPosition) connections->currentPosition->parsePrinterReply(args);

      memmove(lineBuffer, c + 1, lineBufferEnd - (c + 1));
      lineBufferEnd -= (c + 1) - lineBuffer;
      c = lineBuffer;
    }
  }
}

void Printer::available() {
  char buffer[4096];
  int ret = read(fd, buffer, sizeof(buffer));
  if(ret == -1) {
    cerr << "Failed to read tty: " << strerror(errno) << endl;
    return;
  }

  if(ret == 0) {
    cerr << "EOF on tty: " << strerror(errno) << endl;
    removeFromEpoll(connections->epollFd);
    return;
  }

  parsePrinterReply(buffer, ret);

  connections->terminal->write(buffer, ret);
}

void Printer::write(const char *buf, int len) {
  vector<string> args;
  const char *i, *start;
  for(i = buf, start = buf; i < buf + len; ++i) {
    if(*i == ' ') {
      args.push_back(string(start, i - start));
      start = i + 1;
    }
  }
  args.push_back(string(start, i - start));

#ifdef DEBUG_INTERCEPTIONS
  cerr << "Intercepted to printer:";
  for(auto &i: args) cerr << " " << i;
  cerr << endl;
#endif

  if(connections->currentPosition) connections->currentPosition->parsePrinterCommand(args);

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

void Printer::moveTo(const Position &pos) {
  ostringstream command;
  command << setprecision(10)
    << "tool:move " << pos.disp.x
    << " " << pos.disp.y
    << " " << pos.disp.z
    << " " << pos.rot.r
    << " " << pos.rot.i
    << " " << pos.rot.j
    << " " << pos.rot.k
    << "\r\n";
  auto cmdStr = command.str();

  if(connections->terminal) connections->terminal->write(cmdStr.data(), cmdStr.size());
  write(cmdStr.data(), cmdStr.size());
}
