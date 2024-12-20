#include "tablet.h"

#include "connections.h"
#include "terminal.h"

#include <cerrno>
#include <cstring>
#include <iostream>
#include <unistd.h>
#include <sys/types.h>
#include <sys/wait.h>
#include <signal.h>
#include <fcntl.h>

using namespace std;

unique_ptr<Tablet> Tablet::open(Connections *connections) {
  auto result = make_unique<Tablet>();
  result->connections = connections;
  result->readFd = -1;
  result->writeFd = -1;;
  result->childPid = -1;;

  // Named as seen from the main process side
  int readPipe[2];
  int writePipe[2];

  {
    int ret = pipe2(readPipe, O_CLOEXEC);
    if(ret == -1) {
      cerr << "Could not create read pipe: " << strerror(errno) << endl;
      return {};
    }
  }

  {
    int ret = pipe2(writePipe, O_CLOEXEC);
    if(ret == -1) {
      cerr << "Could not create write pipe: " << strerror(errno) << endl;
      return {};
    }
  }

  result->readFd = readPipe[0];
  result->writeFd = writePipe[1];

  result->childPid = fork();
  if(result->childPid == -1) {
    cerr << "Could not fork adb child: " << strerror(errno) << endl;
    return {};
  }

  if(result->childPid == 0) {
    {
      int ret = dup2(writePipe[0], 0);
      if(ret == -1) {
        cerr << "Could not switch stdin in adb child: " << strerror(errno) << endl;
        exit(1);
      }
    }

    {
      int ret = dup2(readPipe[1], 1);
      if(ret == -1) {
        cerr << "Could not switch stdout in adb child: " << strerror(errno) << endl;
        exit(1);
      }
    }

    {
      int ret = dup2(readPipe[1], 2);
      if(ret == -1) {
        cerr << "Could not switch stderr in adb child: " << strerror(errno) << endl;
        exit(1);
      }
    }

    char *args[] = {
      strdup("adb"), strdup("shell"), strdup("-e"), strdup("none"), nullptr
    };
    execvp("adb", args);
    cerr << "Could not execute adb child: " << strerror(errno) << endl;
    exit(1);
  }

  return result;
}

Tablet::~Tablet() {
  if(childPid != -1) {
    int ret = kill(childPid, SIGTERM);
    if(ret == -1) {
      cerr << "Could not kill adb child: " << strerror(errno) << endl;
    }

    {
      int wstatus;
      int ret = waitpid(childPid, &wstatus, 0);
      if(ret == -1) {
        cerr << "Could not wait for adb child: " << strerror(errno) << endl;
      } else if(WIFEXITED(wstatus)) {
        cout << "Adb child exit code: " << WEXITSTATUS(wstatus) << endl;
      } else if(WTERMSIG(wstatus)) {
        cout << "Adb child exit signal: " << WTERMSIG(wstatus) << endl;
      } else {
        cout << "Adb child exited due to unknown causes." << endl;
      }
    }
  }

  if(writeFd != -1) {
    int ret = close(writeFd);
    if(ret == -1) {
      cerr << "Could not close write pipe: " << strerror(errno) << endl;
    }
  }

  if(readFd != -1) {
    int ret = close(readFd);
    if(ret == -1) {
      cerr << "Could not close read pipe: " << strerror(errno) << endl;
    }
  }
}

int Tablet::getEpollFd() {
  return readFd;
}

void Tablet::available() {
  char buffer[4096];
  int ret = read(readFd, buffer, sizeof(buffer));
  if(ret == -1) {
    cerr << "Failed to read from tablet: " << strerror(errno) << endl;
    return;
  }

  if(ret == 0) {
    cerr << "EOF on tablet: " << strerror(errno) << endl;
    return;
  }

  connections->terminal->write("Tablet: ", strlen("TABLET: "));
  connections->terminal->write(buffer, ret);
}

void Tablet::write(const char *buf, int len) {
  while(len) {
    int ret = ::write(writeFd, buf, len);
    if(ret == -1) {
      cerr << "Tablet write failed: " << strerror(errno) << endl;
      return;
    }

    buf += ret;
    len -= ret;
  }
}
