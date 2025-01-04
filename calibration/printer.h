#ifndef H_75F1F0E7_1D75_4FBB_B226_7AA634499B36
#define H_75F1F0E7_1D75_4FBB_B226_7AA634499B36

#include "epollable.h"

#include <memory>
#include <string>

class Connections;

class Printer: public Epollable {
  private:
    Connections *connections;
    int fd;

    char lineBuffer[4096];
    char *lineBufferEnd;

    void parsePrinterReply(const char *buffer, int len);

  public:
    static std::unique_ptr<Printer> open(const std::string &device, Connections *);
    ~Printer();

    int getEpollFd() override;
    void available() override;

    void write(const char *buf, int len);
};

#endif
