#ifndef H_AA8BEAA4_4A24_43A2_8209_02610527FB61
#define H_AA8BEAA4_4A24_43A2_8209_02610527FB61

#include "globals.h"

#include <string>
#include <cstdint>

struct VideoFrame {
  uint8_t data[videoHeight * videoWidth * 2]; // YUV422 encoded image, row major

  void renderText(int x, int y, const std::string &);
  void setPixel(int x, int y);
};

#endif
