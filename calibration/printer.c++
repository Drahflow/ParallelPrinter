#include "printer.h"

#include "connections.h"
#include "terminal.h"

#include <iostream>
#include <cstring>

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

using namespace std;

unique_ptr<Printer> Printer::open(const string &device, Connections *connections) {
  auto result = make_unique<Printer>();
  result->connections = connections;

  result->fd = ::open(device.c_str(), O_RDWR | O_NOCTTY);
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

void Printer::available() {
  char buffer[4096];
  int ret = read(fd, buffer, sizeof(buffer));
  if(ret == -1) {
    cerr << "Failed to read tty: " << strerror(errno) << endl;
    return;
  }

  if(ret == 0) {
    cerr << "EOF on tty: " << strerror(errno) << endl;
    return;
  }

  connections->terminal->write(buffer, ret);
}

void Printer::write(const char *buf, int len) {
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
