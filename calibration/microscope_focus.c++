#include "microscope_focus.h"
#include "video_frame.h"
#include "connections.h"
#include "globals.h"
#include "current_position.h"

#include <sstream>
#include <algorithm>

using namespace std;

unique_ptr<MicroscopeFocus> MicroscopeFocus::open(Connections *connections) {
  auto result = make_unique<MicroscopeFocus>();
  result->connections = connections;
  result->reset();

  return result;
}

static uint16_t scaledFrame[videoWidth / 2 * videoHeight / 2];

void MicroscopeFocus::evaluate(VideoFrame *frame) {
  const uint8_t *data = frame->data;

  for(int y = 0; y < videoHeight / 2; ++y) {
    for(int x = 0; x < videoWidth / 2; ++x) {
      scaledFrame[y * videoWidth / 2 + x] =
        data[(y * 2 + 0) * 2 * videoWidth + (x * 2 + 0) * 2] +
        data[(y * 2 + 0) * 2 * videoWidth + (x * 2 + 1) * 2] +
        data[(y * 2 + 1) * 2 * videoWidth + (x * 2 + 0) * 2] +
        data[(y * 2 + 1) * 2 * videoWidth + (x * 2 + 1) * 2];
    }
  }

  constexpr int maskSize = 2;
  constexpr int pixels = (maskSize * 2 + 1) * (maskSize * 2 + 1);
  focus = 0;

  for(int cy = maskSize; cy < videoHeight / 2 - maskSize; ++cy) {
    for(int cx = maskSize; cx < videoWidth / 2 - maskSize; ++cx) {
      uint32_t blockBrightness = 0;
      for(int y = cy - maskSize; y < cy + maskSize + 1; ++y) {
        for(int x = cx - maskSize; x < cx + maskSize + 1; ++x) {
          blockBrightness += scaledFrame[y * videoWidth / 2 + x];
        }
      }

      uint32_t centerBrightness = scaledFrame[cy * videoWidth / 2 + cx] * pixels;
      uint32_t distance = centerBrightness < blockBrightness?
          blockBrightness - centerBrightness: centerBrightness - blockBrightness;

      focus += distance;
    }
  }

  lastMeasurements[nextMeasurement] = focus;
  nextMeasurement = (nextMeasurement + 1) % lastMeasurements.size();

  auto tmp = lastMeasurements;
  ranges::sort(tmp);
  focus = tmp[2];

  if(focus > bestFocus && connections->currentPosition) {
    auto pos = connections->currentPosition->readPrinter();
    if(pos) {
      bestPosition = *pos;
      bestFocus = focus;
    }
  }
}

void MicroscopeFocus::render(VideoFrame *frame) {
  ostringstream focusText;
  focusText << "Focus: " << focus;
  frame->renderText(2, 2, focusText.str());

  if(bestFocus) {
    ostringstream bestText;
    bestText << "Best: " << bestFocus << " at " << bestPosition << endl;
    frame->renderText(80, 2, bestText.str());
  }
}

void MicroscopeFocus::reset() {
  for(auto &i: lastMeasurements) i = 0;
  bestFocus = 0;
  nextMeasurement = 0;
}
