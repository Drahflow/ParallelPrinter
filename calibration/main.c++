#include <iostream>
#include <cerrno>
#include <cstring>
#include <cstdint>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <fcntl.h>
#include <unistd.h>
#include <linux/videodev2.h>

/**
 * The overall plan
 *
 * Configure stdin as a raw terminal -> term
 * Configure /dev/ttyACM0 as raw terminal -> printer
 * Configure /dev/video0 as streaming capture -> camera
 * Configure TCP/IP socket for video output -> stream
 * Configure piped adb to tabletTarget -> tablet
 * Configure calibration.dat -> calibration
 * Configure timer interval -> time
 *
 * Event on term ->
 *   Maintain local line buffer
 *   Parse local commands
 *   Forward everything to -> printer
 * Event on printer ->
 *   Maintain local line buffer
 *   Parse expected replies
 *   Forward everything to (filtered) -> term
 *   Update movement finished info
 *   Update steps info
 * Event on camera ->
 *   If enabled and movement finished, run computer vision
 *     On hit log to -> calibration.dat
 *     On miss issue new commands to -> tablet
 *   Send CV data with raw image to -> stream
 * No events on stream
 * Event on tablet ->
 *   Prefix and forward to -> term
 * No events on calibration.dat
 * Tick on time ->
 *   Run calibration sequence step
 *   Request current status from -> printer
 */

using namespace std;

constexpr int videoWidth = 640;
constexpr int videoHeight = 480;

int main(int argc, const char **argv) {
  if(argc != 2) {
    cerr << "Usage: " << argv[0] << " /dev/videoX" << endl;
    return 1;
  }

  int v4l = open(argv[1], O_RDWR);
  if(v4l == -1) {
    cerr << "Failed to open: " << argv[1] << ": " << strerror(errno) << endl;
    return 1;
  }

  struct v4l2_capability capabilities;

  int ret = ioctl(v4l, VIDIOC_QUERYCAP, &capabilities);
  if(ret == -1) {
    cerr << "Failed to get video capabilities: " << strerror(errno) << endl;
    return 1;
  }

  cout << "Device: " << string(reinterpret_cast<const char *>(&capabilities.card)) << endl;
  if(capabilities.device_caps & V4L2_CAP_READWRITE) {
    cout << "Supports read()/write()" << endl;
  }
  if(capabilities.device_caps & V4L2_CAP_STREAMING) {
    cout << "Supports streaming i/o" << endl;
  }

  struct v4l2_format yuv422;
  yuv422.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  ret = ioctl(v4l, VIDIOC_G_FMT, &yuv422);
  if(ret == -1) {
    cerr << "Failed to get video format: " << strerror(errno) << endl;
    return 1;
  }

  yuv422.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

  ret = ioctl(v4l, VIDIOC_S_FMT, &yuv422);
  if(ret == -1) {
    cerr << "Failed to set YUV422: " << strerror(errno) << endl;
    return 1;
  }

  ret = ioctl(v4l, VIDIOC_G_FMT, &yuv422);
  if(ret == -1) {
    cerr << "Failed to get video format: " << strerror(errno) << endl;
    return 1;
  }

  cout << "Image: " << yuv422.fmt.pix.width << "x" << yuv422.fmt.pix.height << endl;
  cout << "Pixel format: " << string(reinterpret_cast<const char *>(&yuv422.fmt.pix.pixelformat), 4) << endl;

  constexpr int bufferCount = 8;

  struct v4l2_requestbuffers bufReq;
  bufReq.count = bufferCount;
  bufReq.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  bufReq.memory = V4L2_MEMORY_MMAP;
  bufReq.capabilities = 0;
  bufReq.flags = 0;
  bzero(bufReq.reserved, sizeof(bufReq.reserved));

  ret = ioctl(v4l, VIDIOC_REQBUFS, &bufReq);
  if(ret == -1) {
    cerr << "Failed to request i/o bufReq: " << strerror(errno) << endl;
    return 1;
  }

  uint8_t *buffers[bufferCount];
  for(int i = 0; i < bufferCount; ++i) {
    struct v4l2_buffer buf;
    buf.type = bufReq.type;
    buf.memory = bufReq.memory;
    buf.index = i;
    buf.reserved = buf.reserved2 = 0;

    int ret = ioctl(v4l, VIDIOC_QUERYBUF, &buf);
    if(ret == -1) {
      cerr << "Failed to get buffer parameters: " << strerror(errno) << endl;
      return 1;
    }

    cout << "Memory offset: " << buf.m.offset << " length: " << buf.length << endl;
    buffers[i] = reinterpret_cast<uint8_t *>(mmap(NULL, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, v4l, buf.m.offset));
    if(buffers[i] == MAP_FAILED) {
      cerr << "Failed to map buffer: " << strerror(errno) << endl;
      return 1;
    }

    ret = ioctl(v4l, VIDIOC_QBUF, &buf);
    if(ret == -1) {
      cerr << "Failed to enqueue buffer: " << strerror(errno) << endl;
      return 1;
    }
  }

  int type = bufReq.type;
  ret = ioctl(v4l, VIDIOC_STREAMON, &type);
  if(ret == -1) {
    cerr << "Cannot start input stream: " << strerror(errno) << endl;
    return 1;
  }

  cout << "========" << endl;

  while(true) {
    struct v4l2_buffer buf;
    buf.type = bufReq.type;
    buf.memory = bufReq.memory;
    buf.reserved = buf.reserved2 = 0;

    int ret = ioctl(v4l, VIDIOC_DQBUF, &buf);
    if(ret == -1) {
      cerr << "Cannot dequeue buffer: " << strerror(errno) << endl;
      return 1;
    }

    for(unsigned int y = 0; y < videoHeight; ++y) {
      uint8_t lineOut[videoWidth * 2];

      for(unsigned int x = 0; x < videoWidth; ++x) {
        uint8_t lum = buffers[buf.index][y * yuv422.fmt.pix.bytesperline + x * 2];
        uint8_t Cb = buffers[buf.index][y * yuv422.fmt.pix.bytesperline + ((x * 2) & ~3) + 1];
        uint8_t Cr = buffers[buf.index][y * yuv422.fmt.pix.bytesperline + ((x * 2) & ~3) + 3];

        lineOut[x * 2] = lum;
        lineOut[x * 2 + 1] = x & 1? Cb: Cr;
        // if(lum < 64) {
        //   cout << " ";
        // } else if(lum < 128) {
        //   cout << ".";
        // } else if(lum < 192) {
        //   cout << ":";
        // } else {
        //   cout << "#";
        // }
      }

      cout << string(reinterpret_cast<const char *>(lineOut), sizeof(lineOut));
    }
    flush(cout);

    ret = ioctl(v4l, VIDIOC_QBUF, &buf);
    if(ret == -1) {
      cerr << "Cannot re-queue buffer: " << strerror(errno) << endl;
      return 1;
    }
  }

  ret = close(v4l);
  if(ret == -1) {
    cerr << "Failed to close: " << argv[1] << ": " << strerror(errno) << endl;
    return 1;
  }

  return 0;
}
