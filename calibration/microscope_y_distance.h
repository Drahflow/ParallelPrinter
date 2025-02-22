#ifndef H_F4076738_4B1A_4375_9BC9_85281608BDB3
#define H_F4076738_4B1A_4375_9BC9_85281608BDB3

#include "geometry.h"

#include <memory>
#include <cstdint>
#include <array>

class Connections;
class VideoFrame;

class MicroscopeYDistance {
  private:
    Connections *connections;
    int64_t distance;
    int nextMeasurement;
    std::array<uint64_t, 31> lastMeasurements;

    double yRangeBegin, yRangeEnd;
    std::array<uint64_t, 256> yRangePlot;

    Position bestPosition;
    uint64_t bestDistance;

  public:
    static std::unique_ptr<MicroscopeYDistance> open(Connections *);

    void evaluate(VideoFrame *);
    void render(VideoFrame *);
    void reset();
    void enableRangePlot(double begin, double end);

    auto readDistance() const -> auto { return distance; }
    auto readBestPosition() const -> auto { return bestPosition; }
};

#endif
