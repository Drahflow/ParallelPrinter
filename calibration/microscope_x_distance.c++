#include "microscope_x_distance.h"

#include "video_frame.h"
#include "connections.h"
#include "globals.h"
#include "current_position.h"

#include <sstream>
#include <algorithm>
#include <iomanip>

using namespace std;

constexpr uint64_t LARGE = 1'000'000'000'000;

unique_ptr<MicroscopeXDistance> MicroscopeXDistance::open(Connections *connections) {
  auto result = make_unique<MicroscopeXDistance>();
  result->connections = connections;
  result->reset();

  return result;
}

void MicroscopeXDistance::evaluate(VideoFrame *frame) {
  const uint8_t *data = frame->data;

  int64_t left = 0;
  int64_t right = 0;

  for(int y = 0; y < videoHeight; ++y) {
    for(int x = 0; x < videoWidth / 2; ++x) {
      left += data[y * 2 * videoWidth + x * 2];
      right += data[y * 2 * videoWidth + (x + videoWidth / 2) * 2];
    }
  }

  distance = left - right;

  lastMeasurements[nextMeasurement] = distance;
  nextMeasurement = (nextMeasurement + 1) % lastMeasurements.size();

  auto tmp = lastMeasurements;
  ranges::sort(tmp);

  distance = tmp[lastMeasurements.size() / 2 + 1];
  uint64_t absDistance = llabs(distance);

  if(connections->currentPosition) {
    auto pos = connections->currentPosition->readPrinter();
    if(pos) {
      if(absDistance < bestDistance && connections->currentPosition) {
        bestPosition = *pos;
        bestDistance = absDistance;
      }

      if(xRangeBegin < xRangeEnd) {
        int index = (pos->disp.x - xRangeBegin) / (xRangeEnd - xRangeBegin) * xRangePlot.size();
        if(index >= 0 && index < (int)xRangePlot.size()) {
          if(absDistance < xRangePlot[index]) xRangePlot[index] = absDistance;
        }
      }
    }
  }
}

void MicroscopeXDistance::render(VideoFrame *frame) {
  ostringstream text;
  text << "delta X: " << distance;
  frame->renderText(2, 16, text.str());

  for(int y = videoHeight / 2 - 25; y < videoHeight / 2; ++y) {
    frame->setPixel(videoWidth / 2, y);
  }

  if(bestDistance < LARGE) {
    ostringstream bestText;
    bestText << setprecision(10) << "Best: " << bestDistance << " at " << bestPosition;
    frame->renderText(200, 16, bestText.str());
  }

  if(xRangeBegin < xRangeEnd) {
    for(unsigned int i = 0; i < xRangePlot.size(); ++i) {
      if(xRangePlot[i] < LARGE) {
        int y = 50 + xRangePlot[i] / 128;
        if(y > 50 && y < videoHeight) {
          frame->setPixel(200 + i, y);
        }
      }
    }
  }
}

void MicroscopeXDistance::reset() {
  for(auto &i: lastMeasurements) i = LARGE;
  bestDistance = LARGE;
  nextMeasurement = 0;
  xRangeBegin = 1;
  xRangeEnd = -1;
}

void MicroscopeXDistance::enableRangePlot(double begin, double end) {
  for(auto &i: xRangePlot) i = LARGE;

  xRangeBegin = begin;
  xRangeEnd = end;
}
