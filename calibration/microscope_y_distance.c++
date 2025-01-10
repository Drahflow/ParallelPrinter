#include "microscope_y_distance.h"

#include "video_frame.h"
#include "connections.h"
#include "globals.h"
#include "current_position.h"

#include <sstream>
#include <algorithm>
#include <iomanip>

using namespace std;

constexpr uint64_t LARGE = 1'000'000'000'000;

unique_ptr<MicroscopeYDistance> MicroscopeYDistance::open(Connections *connections) {
  auto result = make_unique<MicroscopeYDistance>();
  result->connections = connections;
  result->reset();

  return result;
}

void MicroscopeYDistance::evaluate(VideoFrame *frame) {
  const uint8_t *data = frame->data;

  int64_t top = 0;
  int64_t bottom = 0;

  for(int y = 0; y < videoHeight / 2; ++y) {
    for(int x = 0; x < videoWidth; ++x) {
      top += data[y * 2 * videoWidth + x * 2];
      bottom += data[(y + videoHeight / 2) * 2 * videoWidth + x * 2];
    }
  }

  distance = top - bottom;

  lastMeasurements[nextMeasurement] = distance;
  nextMeasurement = (nextMeasurement + 1) % lastMeasurements.size();

  auto tmp = lastMeasurements;
  ranges::sort(tmp);
  uint64_t absDistance = llabs(tmp[lastMeasurements.size() / 2 + 1]);

  if(connections->currentPosition) {
    auto pos = connections->currentPosition->readPrinter();
    if(pos) {
      if(absDistance < bestDistance && connections->currentPosition) {
        bestPosition = *pos;
        bestDistance = absDistance;
      }

      if(yRangeBegin < yRangeEnd) {
        int index = (pos->disp.x - yRangeBegin) / (yRangeEnd - yRangeBegin) * yRangePlot.size();
        if(index >= 0 && index < (int)yRangePlot.size()) {
          if(absDistance < yRangePlot[index]) yRangePlot[index] = absDistance;
        }
      }
    }
  }
}

void MicroscopeYDistance::render(VideoFrame *frame) {
  ostringstream text;
  text << "delta Y: " << distance;
  frame->renderText(2, 30, text.str());

  for(int x = videoWidth / 2 - 25; x < videoWidth / 2; ++x) {
    frame->setPixel(x, videoHeight / 2);
  }

  if(bestDistance < LARGE) {
    ostringstream bestText;
    bestText << setprecision(10) << "Best: " << bestDistance << " at " << bestPosition;
    frame->renderText(200, 30, bestText.str());
  }

  if(yRangeBegin < yRangeEnd) {
    for(unsigned int i = 0; i < yRangePlot.size(); ++i) {
      if(yRangePlot[i] < LARGE) {
        int y = 50 + yRangePlot[i] / 1024 ;
        if(y > 50 && y < videoHeight) {
          frame->setPixel(200 + i, y);
        }
      }
    }
  }
}

void MicroscopeYDistance::reset() {
  for(auto &i: lastMeasurements) i = LARGE;
  bestDistance = LARGE;
  nextMeasurement = 0;
  yRangeBegin = 1;
  yRangeEnd = -1;
}

void MicroscopeYDistance::enableRangePlot(double begin, double end) {
  for(auto &i: yRangePlot) i = LARGE;

  yRangeBegin = begin;
  yRangeEnd = end;
}
