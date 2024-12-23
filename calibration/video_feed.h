#ifndef H_10EE1C4F_D56D_48BB_9A2F_D0E591467C30
#define H_10EE1C4F_D56D_48BB_9A2F_D0E591467C30

#include "epollable.h"

#include <memory>
#include <string>

class Connections;

class VideoFeed: public Epollable {
  private:
    Connections *connections;
    int fd;

  public:
    static std::unique_ptr<VideoFeed> open(const std::string &peer, Connections *);
    ~VideoFeed();

    int getEpollFd() override;
    void available() override;

    void write(const char *buf, int len);
};

#endif
