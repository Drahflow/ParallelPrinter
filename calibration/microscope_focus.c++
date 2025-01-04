#include "microscope_focus.h"
#include "video_frame.h"
#include "connections.h"
#include "globals.h"

#include <sstream>

using namespace std;

unique_ptr<MicroscopeFocus> MicroscopeFocus::open(Connections *connections) {
  auto result = make_unique<MicroscopeFocus>();
  result->connections = connections;

  return result;
}

void MicroscopeFocus::evaluate(VideoFrame *frame) {
  const uint8_t *data = frame->data;
  constexpr int maskSize = 2;
  uint32_t focus = 0;

  for(int cy = maskSize; cy < videoHeight - maskSize; ++cy) {
    for(int cx = maskSize; cx < videoWidth - maskSize; ++cx) {
      uint32_t blockBrightness = 0;
      for(int y = cy - maskSize; y < cy + maskSize + 1; ++y) {
        for(int x = cx - maskSize; x < cx + maskSize + 1; ++x) {
          blockBrightness += data[y * videoWidth * 2 + x * 2];
        }
      }

      uint32_t centerBrightness = data[cy * videoWidth * 2 + cx * 2];
      uint32_t distance = centerBrightness < blockBrightness?
          blockBrightness - centerBrightness: centerBrightness - blockBrightness;

      focus += distance;
    }
  }

  ostringstream focusText;
  focusText << "Focus: " << focus;

  frame->renderText(0, 8, focusText.str());
}
