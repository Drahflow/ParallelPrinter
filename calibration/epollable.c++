#include "epollable.h"

#include <iostream>
#include <sys/epoll.h>
#include <cstring>

using namespace std;

bool Epollable::addToEpoll(int epollFd) {
  struct epoll_event setup{
    .events = EPOLLIN | EPOLLERR,
    .data = {.ptr = this},
  };
  int ret = epoll_ctl(epollFd, EPOLL_CTL_ADD, getEpollFd(), &setup);
  if(ret == -1) {
    cerr << "Cannot poll input: " << strerror(errno) << endl;
    return false;
  }

  return true;
}
