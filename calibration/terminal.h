#ifndef H_77B266FA_799C_4AAE_B742_C729533FFF3F
#define H_77B266FA_799C_4AAE_B742_C729533FFF3F

#include "epollable.h"

#include <memory>

struct Connections;

class Terminal: public Epollable {
  private:
    Connections *connections;

    char buffer[4096];
    unsigned int readPos;

    void parse(char *);

  public:
    static std::unique_ptr<Terminal> open(Connections *);
    ~Terminal();

    int getEpollFd() override;
    void available() override;

    void write(const char *buf, int len);
};

#endif
