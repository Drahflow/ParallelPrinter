#include "globals.h"
#include "connections.h"
#include "terminal.h"
#include "current_position.h"
#include "microscope_focus.h"

#include <iostream>
#include <cerrno>
#include <cstring>
#include <cstdint>
#include <memory>

#include <sys/epoll.h>
#include <unistd.h>

/**
 * The overall plan
 *
 * Configure stdin as a raw terminal -> term
 * Configure /dev/ttyACM0 as raw terminal -> printer
 * Configure /dev/video0 as streaming capture -> microscope
 * Configure TCP/IP socket for video output -> stream
 * Configure piped adb to tabletTarget -> tablet
 * Configure calibration.dat -> calibration
 * Configure timer interval -> time
 *
 * Event on term ->
 *   Maintain local line buffer
 *   Parse local commands
 *   Forward everything to -> printer
 * Event on printer ->
 *   Maintain local line buffer
 *   Parse expected replies
 *   Forward everything to (filtered) -> term
 *   Update movement finished info
 *   Update steps info
 * Event on microscope ->
 *   If enabled and movement finished, run computer vision
 *     On hit log to -> calibration.dat
 *     On miss issue new commands to -> tablet
 *   Send CV data with raw image to -> stream
 * No events on stream
 * Event on tablet ->
 *   Prefix and forward to -> term
 * No events on calibration.dat
 * Tick on time ->
 *   Run calibration sequence step
 *   Request current status from -> printer
 */

using namespace std;

int main(void) {
  Connections connections;
  if(!(connections.currentPosition = CurrentPosition::open(&connections))) return 1;
  if(!(connections.terminal = Terminal::open(&connections))) return 1;
  if(!(connections.microscopeFocus = MicroscopeFocus::open(&connections))) return 1;

  int epoll = epoll_create(8);
  if(epoll == -1) {
    cerr << "Cannot create epoll: " << strerror(errno) << endl;
    return 1;
  }

  connections.epollFd = epoll;
  if(!connections.terminal->addToEpoll(epoll)) return 1;

  while(true) {
    constexpr int eventCount = 8;
    struct epoll_event events[eventCount];
    int ret = epoll_wait(epoll, events, eventCount, 1);
    if(ret == -1) {
      cerr << "Failure to epoll_wait: " << strerror(errno) << endl;
      break;
    }

    for(int i = 0; i < ret; ++i) {
      reinterpret_cast<Epollable *>(events[i].data.ptr)->available();
    }
  }

  {
    int ret = close(epoll);
    if(ret == -1) {
      cerr << "Cannot close epoll: " << strerror(errno) << endl;
    }
  }

  return 0;
}
