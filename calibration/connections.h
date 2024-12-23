#ifndef H_08625D32_6735_4E35_8205_11224A3BF181
#define H_08625D32_6735_4E35_8205_11224A3BF181

#include <memory>

class Terminal;
class Microscope;
class Printer;
class Tablet;
class VideoFeed;

struct Connections {
  int epollFd;

  Connections();
  ~Connections();

  std::unique_ptr<Terminal> terminal;
  std::unique_ptr<Microscope> microscope;
  std::unique_ptr<Printer> printer;
  std::unique_ptr<Tablet> tablet;
  std::unique_ptr<VideoFeed> videoFeed;
  // int calibrationLog = -1;
};

#endif
