#include "terminal.h"

#include "connections.h"
#include "microscope.h"
#include "printer.h"
#include "tablet.h"
#include "video_feed.h"

#include <iostream>
#include <cstring>
#include <vector>
#include <unistd.h>

using namespace std;

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

void Terminal::parse(char *input) {
  vector<string> args;
  for(char *i = input, *start = input; ; ++i) {
    if(!*i || *i == ' ') {
      args.push_back(string(start, i - start));
      start = i + 1;
    }

    if(!*i) break;
  }

  cerr << "Got:";
  for(auto &i: args) cerr << " " << i;
  cerr << endl;

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
  } else if(connections->printer) {
    connections->printer->write(input, strlen(input));
    connections->printer->write("\r\n", 2);
  } else {
    cerr << "Printer not yet connected, dropping unknown command." << endl;
  }
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
