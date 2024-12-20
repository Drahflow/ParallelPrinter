#ifndef H_773ED289_6C4F_469F_902D_33927F3D4DDA
#define H_773ED289_6C4F_469F_902D_33927F3D4DDA

#include "epollable.h"

#include <memory>

class Connections;

class Microscope: public Epollable {
  private:
    static constexpr int bufferCount = 8;
    Connections *connections;

    int v4l;
    uint8_t *buffers[bufferCount];
    uint64_t bufferLengths[bufferCount];
    std::unique_ptr<struct v4l2_format> yuv422;

  public:
    static std::unique_ptr<Microscope> open(const char *dev, Connections *);
    ~Microscope();

    int getEpollFd() override;
    void available() override;
};

#endif
