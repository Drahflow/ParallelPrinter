#ifndef H_7022715A_93DC_4480_8594_58D147D2DAD7
#define H_7022715A_93DC_4480_8594_58D147D2DAD7

#include "geometry.h"

#include <memory>
#include <cstdint>
#include <array>

class Connections;
class VideoFrame;

class MicroscopeXDistance {
  private:
    Connections *connections;
    int64_t distance;
    int nextMeasurement;
    std::array<uint64_t, 31> lastMeasurements;

    double xRangeBegin, xRangeEnd;
    std::array<uint64_t, 256> xRangePlot;

    Position bestPosition;
    uint64_t bestDistance;

  public:
    static std::unique_ptr<MicroscopeXDistance> open(Connections *);

    void evaluate(VideoFrame *);
    void render(VideoFrame *);
    void reset();
    void enableRangePlot(double begin, double end);

    auto readDistance() const -> auto { return distance; }
    auto readBestPosition() const -> auto { return bestPosition; }
};

#endif
