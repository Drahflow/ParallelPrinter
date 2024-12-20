#include "microscope.h"
#include "globals.h"

#include <iostream>
#include <cerrno>
#include <cstring>

#include <sys/types.h>
#include <sys/stat.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <fcntl.h>
#include <unistd.h>

using namespace std;

unique_ptr<Microscope> Microscope::open(const char *dev, Connections *connections) {
  int v4l = ::open(dev, O_RDWR);
  if(v4l == -1) {
    cerr << "Failed to open: " << dev << ": " << strerror(errno) << endl;
    return {};
  }

  auto result = make_unique<Microscope>();
  result->connections = connections;
  result->v4l = v4l;
  for(int i = 0; i < bufferCount; ++i) result->buffers[i] = nullptr;

  struct v4l2_capability capabilities;

  int ret = ioctl(v4l, VIDIOC_QUERYCAP, &capabilities);
  if(ret == -1) {
    cerr << "Failed to get video capabilities: " << strerror(errno) << endl;
    return {};
  }

  cout << "Device: " << string(reinterpret_cast<const char *>(&capabilities.card)) << endl;
  if(capabilities.device_caps & V4L2_CAP_READWRITE) {
    cout << "Supports read()/write()" << endl;
  }
  if(capabilities.device_caps & V4L2_CAP_STREAMING) {
    cout << "Supports streaming i/o" << endl;
  }

  result->yuv422 = make_unique<struct v4l2_format>();
  result->yuv422->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  ret = ioctl(v4l, VIDIOC_G_FMT, result->yuv422.get());
  if(ret == -1) {
    cerr << "Failed to get video format: " << strerror(errno) << endl;
    return {};
  }

  result->yuv422->fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;

  ret = ioctl(v4l, VIDIOC_S_FMT, result->yuv422.get());
  if(ret == -1) {
    cerr << "Failed to set YUV422: " << strerror(errno) << endl;
    return {};
  }

  ret = ioctl(v4l, VIDIOC_G_FMT, result->yuv422.get());
  if(ret == -1) {
    cerr << "Failed to get video format: " << strerror(errno) << endl;
    return {};
  }

  cout << "Image: " << result->yuv422->fmt.pix.width << "x" << result->yuv422->fmt.pix.height << endl;
  cout << "Pixel format: " << string(reinterpret_cast<const char *>(&result->yuv422->fmt.pix.pixelformat), 4) << endl;

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
    return {};
  }

  for(int i = 0; i < bufferCount; ++i) {
    struct v4l2_buffer buf;
    buf.type = bufReq.type;
    buf.memory = bufReq.memory;
    buf.index = i;
    buf.reserved = buf.reserved2 = 0;

    int ret = ioctl(v4l, VIDIOC_QUERYBUF, &buf);
    if(ret == -1) {
      cerr << "Failed to get buffer parameters: " << strerror(errno) << endl;
      return {};
    }

    cout << "Memory offset: " << buf.m.offset << " length: " << buf.length << endl;
    result->bufferLengths[i] = buf.length;
    result->buffers[i] =
      reinterpret_cast<uint8_t *>(mmap(NULL, result->bufferLengths[i], PROT_READ | PROT_WRITE, MAP_SHARED,
            v4l, buf.m.offset));
    if(result->buffers[i] == MAP_FAILED) {
      cerr << "Failed to map buffer: " << strerror(errno) << endl;
      return {};
    }

    ret = ioctl(v4l, VIDIOC_QBUF, &buf);
    if(ret == -1) {
      cerr << "Failed to enqueue buffer: " << strerror(errno) << endl;
      return {};
    }
  }

  int type = bufReq.type;
  ret = ioctl(v4l, VIDIOC_STREAMON, &type);
  if(ret == -1) {
    cerr << "Cannot start input stream: " << strerror(errno) << endl;
    return {};
  }

  return result;
}

Microscope::~Microscope() {
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  int ret = ioctl(v4l, VIDIOC_STREAMOFF, &type);
  if(ret == -1) {
    cerr << "Cannot stop input stream: " << strerror(errno) << endl;
  }

  for(int i = 0; i < bufferCount; ++i) {
    if(!buffers[i]) continue;

    int ret = munmap(buffers[i], bufferLengths[i]);
    if(ret == -1) {
      cerr << "Failed to unmap video buffer: " << strerror(errno) << endl;
    }
  }

  ret = close(v4l);
  if(ret == -1) {
    cerr << "Failed to close video device: " << strerror(errno) << endl;
  }
}

int Microscope::getEpollFd() {
  return v4l;
}

void Microscope::available() {
  struct v4l2_buffer buf;
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  buf.reserved = buf.reserved2 = 0;

  int ret = ioctl(v4l, VIDIOC_DQBUF, &buf);
  if(ret == -1) {
    cerr << "Cannot dequeue buffer: " << strerror(errno) << endl;
    return;
  }

  for(unsigned int y = 0; y < videoHeight; ++y) {
    uint8_t lineOut[videoWidth * 2];

    for(unsigned int x = 0; x < videoWidth; ++x) {
      uint8_t lum = buffers[buf.index][y * yuv422->fmt.pix.bytesperline + x * 2];
      uint8_t Cb = buffers[buf.index][y * yuv422->fmt.pix.bytesperline + ((x * 2) & ~3) + 1];
      uint8_t Cr = buffers[buf.index][y * yuv422->fmt.pix.bytesperline + ((x * 2) & ~3) + 3];

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
    return;
  }
}
