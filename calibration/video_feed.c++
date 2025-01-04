#include "video_feed.h"

#include "connections.h"
#include "terminal.h"

#include <iostream>
#include <cstring>
#include <unistd.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>

using namespace std;

unique_ptr<VideoFeed> VideoFeed::open(const string &peer, Connections *connections) {
  string host, port;
  for(unsigned int i = 0; i < peer.length(); ++i) {
    if(peer[i] == ':') {
      host = peer.substr(0, i);
      port = peer.substr(i + 1);
    }
  }

  if(host == "" || port == "") {
    cerr << "Could not extract host:port from " << peer << endl;
    return {};
  }

  struct addrinfo hints;
  struct addrinfo *peerInfos;

  memset(&hints, 0, sizeof(hints));
  hints.ai_family = AF_UNSPEC;
  hints.ai_socktype = SOCK_STREAM;
  hints.ai_flags = 0;
  hints.ai_protocol = 0;

  {
    int ret = getaddrinfo(host.c_str(), port.c_str(), &hints, &peerInfos);
    if(ret != 0) {
      cerr << "Could not resolve peer address: " << gai_strerror(ret) << endl;
      return {};
    }
  }

  auto result = make_unique<VideoFeed>();
  result->connections = connections;

  for(auto peerInfo = peerInfos; peerInfo; peerInfo = peerInfo->ai_next) {
    result->fd = socket(peerInfo->ai_family, peerInfo->ai_socktype | SOCK_CLOEXEC, peerInfo->ai_protocol);
    if(result->fd == -1) {
      cerr << "Could not create socket: " << strerror(errno) << endl;
      freeaddrinfo(peerInfos);
      return {};
    }

    {
      int ret = connect(result->fd, peerInfo->ai_addr, peerInfo->ai_addrlen);
      if(ret == -1) {
        cerr << "Failed to connect to " << host << ": " << strerror(errno) << endl;
        close(result->fd);
        result->fd = -1;
      }
    }
  }

  if(result->fd == -1) {
    cerr << "Could not connect to " << host << ": " << strerror(errno) << endl;
    return {};
  }

  cerr << "Video feed forwarded to " << host << endl;
  freeaddrinfo(peerInfos);

  result->write("========\n", 9);
  return result;
}

VideoFeed::~VideoFeed() {
  if(fd != -1) {
    int ret = close(fd);
    if(ret == -1) {
      cerr << "Could not close socket: " << strerror(errno) << endl;
    }
  }
}

int VideoFeed::getEpollFd() {
  return fd;
}

void VideoFeed::available() {
  char buffer[4096];
  int ret = read(fd, buffer, sizeof(buffer));
  if(ret == -1) {
    cerr << "Failed to read video feed socket: " << strerror(errno) << endl;
    return;
  }

  if(ret == 0) {
    cerr << "EOF on video feed socket: " << strerror(errno) << endl;
    removeFromEpoll(connections->epollFd);
    return;
  }

  connections->terminal->write("Video: ", strlen("Video: "));
  connections->terminal->write(buffer, ret);
}

void VideoFeed::write(const char *buf, int len) {
  while(len) {
    int ret = ::write(fd, buf, len);
    if(ret == -1) {
      cerr << "Video feed write failed: " << strerror(errno) << endl;
      return;
    }

    buf += ret;
    len -= ret;
  }
}
