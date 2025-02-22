#ifndef H_E164FEC1_EC06_41AB_9D3E_A079FA84D2FB
#define H_E164FEC1_EC06_41AB_9D3E_A079FA84D2FB

#include "geometry.h"

#include <memory>
#include <cstdint>
#include <array>

class Connections;
class VideoFrame;

class MicroscopeFocus {
  private:
    Connections *connections;
    uint64_t focus;
    int nextMeasurement;
    std::array<uint64_t, 11> lastMeasurements;

    double zRangeBegin, zRangeEnd;
    std::array<uint64_t, 256> zRangePlot;

    Position bestPosition;
    uint64_t bestFocus;

  public:
    static std::unique_ptr<MicroscopeFocus> open(Connections *);

    void evaluate(VideoFrame *);
    void render(VideoFrame *);
    void reset();
    void enableRangePlot(double begin, double end);

    auto readFocus() const -> auto { return focus; }
    auto readBestPosition() const -> auto { return bestPosition; }
};

#endif
