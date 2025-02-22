#ifndef H_DB00D9EE_C502_4EF2_BCCB_4C5BC402DE04
#define H_DB00D9EE_C502_4EF2_BCCB_4C5BC402DE04

#include "epollable.h"

#include <memory>

struct Connections;

class Tablet: public Epollable {
  private:
    Connections *connections;
    int writeFd, readFd;
    int childPid;

  public:
    static std::unique_ptr<Tablet> open(Connections *);
    ~Tablet();

    int getEpollFd() override;
    void available() override;

    void write(const char *buf, int len);
    void setTarget(int x, int y);
    void setFocusTarget(int x, int y);
};

#endif
