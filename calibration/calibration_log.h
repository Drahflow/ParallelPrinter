#ifndef H_29E64D3B_C346_4029_9918_2BBDA9E2E631
#define H_29E64D3B_C346_4029_9918_2BBDA9E2E631

#include <memory>

struct Connections;

class CalibrationLog {
  private:
    Connections *connections;
    int fd;

  public:
    static std::unique_ptr<CalibrationLog> open(const std::string &filename, Connections *);
    ~CalibrationLog();

    void write(const char *buf, int len);
};

#endif
