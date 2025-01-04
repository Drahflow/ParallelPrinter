#ifndef H_E164FEC1_EC06_41AB_9D3E_A079FA84D2FB
#define H_E164FEC1_EC06_41AB_9D3E_A079FA84D2FB

#include <memory>

class Connections;
class VideoFrame;

class MicroscopeFocus {
  private:
    Connections *connections;

  public:
    std::unique_ptr<MicroscopeFocus> open(Connections *);

    void evaluate(VideoFrame *);
};

#endif
