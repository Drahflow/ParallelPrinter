#ifndef H_75F1F0E7_1D75_4FBB_B226_7AA634499B36
#define H_75F1F0E7_1D75_4FBB_B226_7AA634499B36

#include "epollable.h"

#include <memory>

class Connections;

class Printer: public Epollable {
  private:
    Connections *connections;
    int fd;

  public:
    static std::unique_ptr<Printer> open(const char *device, Connections *);
    ~Printer();

    int getEpollFd() override;
    void available() override;

    void write(const char *buf, int len);
};

#endif
