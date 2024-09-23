#define _GNU_SOURCE

#include <linux/input.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <sys/wait.h>
#include <sys/mman.h>
#include <sys/epoll.h>
#include <fcntl.h>

#include <stdio.h>
#include <stdlib.h>
#include <errno.h>
#include <stdint.h>
#include <string.h>
#include <math.h>

#define EVENT_STREAM_STDIN 0

#define FRAMEBUFFER_DEVICE "/dev/graphics/fb0"
#define FRAMEBUFFER_WIDTH 2560
#define FRAMEBUFFER_HEIGHT 1600
#define BACKLIGHT_DEVICE "/sys/class/backlight/pwm-backlight/brightness"

typedef struct __attribute__((packed)) rgba {
  uint8_t r, g, b, a;
} rgba;

rgba *framebuffer;

uint8_t initFramebuffer() {
  int fb = open(FRAMEBUFFER_DEVICE, O_RDWR);
  if(fb < 0) {
    fprintf(stderr, "Opening /dev/fb0 failed: %s", strerror(errno));
    return 0;
  }

  framebuffer = mmap(NULL, FRAMEBUFFER_WIDTH * FRAMEBUFFER_HEIGHT * sizeof(rgba),
      PROT_READ | PROT_WRITE, MAP_SHARED, fb, 0);
  if(!framebuffer) {
    fprintf(stderr, "Mapping framebuffer failed: %s", strerror(errno));
    return 0;
  }

  return 1;
}

// 65535 == full brightness
uint8_t setBacklight(uint16_t brightness) {
  int fd = open(BACKLIGHT_DEVICE, O_WRONLY);
  if(fd == -1) {
    fprintf(stderr, "Could not open backlight control file: %s", strerror(errno));
    return 0;
  }

  char buf[16];
  int len = snprintf(buf, 16, "%d\n", (brightness >> 8));
  int ret = write(fd, buf, len);
  if(ret < len) {
    fprintf(stderr, "Could not write to backlight control file: %s", strerror(errno));
    return 0;
  }

  if(close(fd) == -1) {
    fprintf(stderr, "Could not close backlight control file: %s", strerror(errno));
    return 0;
  }

  return 1;
}

uint8_t mainRunning = 1;

void handleInput(char *buffer) {
  if(!strcmp(buffer, "exit")) {
    mainRunning = 0;
    return;
  }

  int tx, ty;
  if(sscanf(buffer, "target %d %d", &tx, &ty) == 2) {
    for(int y = 0; y < FRAMEBUFFER_HEIGHT; ++y) {
      for(int x = 0; x < FRAMEBUFFER_WIDTH; ++x) {
        int g = 0;
        if(x == tx || x == tx + 1) {
          g = 255;
        } else if(y == ty || y == ty + 1) {
          g = 255;
        } else {
          int dx = abs(tx - x);
          int dy = abs(ty - y);

          g += 32 - dx % 32;
          g += 32 - dy % 32;
        }

        rgba *p = framebuffer + (FRAMEBUFFER_WIDTH * y + x);
        p->r = 0;
        p->g = g;
        p->b = 0;
        p->a = 0;
      }
    }
  }

  printf("Unknown command.\n");
}

int main(int, const char **) {
  if(!initFramebuffer()) return 1;
  if(!setBacklight(128 << 8)) return 1;

  int epollFd = epoll_create(8);
  if(epollFd < 0) {
    fprintf(stderr, "Could not init epoll: %s", strerror(errno));
    return 1;
  }

  {
    struct epoll_event stdinEvent;
    stdinEvent.events = EPOLLIN;
    stdinEvent.data.u64 = EVENT_STREAM_STDIN;
    if(epoll_ctl(epollFd, EPOLL_CTL_ADD, 0, &stdinEvent) < 0) {
      fprintf(stderr, "Could not start polling for stdin events: %s", strerror(errno));
      return 1;
    }
  }

  char stdinBuffer[256];
  unsigned int stdinFill = 0;

  struct epoll_event eventBuffer[8];
  while(mainRunning) {
    int ret = epoll_wait(epollFd, eventBuffer, sizeof(eventBuffer) / sizeof(struct epoll_event), 100);
    if(ret == -1) {
      fprintf(stderr, "Failed to poll for events: %s", strerror(errno));
      mainRunning = 0;
    }

    for(int i = 0; i < ret; ++i) {
      switch(eventBuffer[i].data.u64) {
        case EVENT_STREAM_STDIN: {
          int ret = read(0, stdinBuffer + stdinFill, 1);
          if(ret == 0) {
            mainRunning = 0;
            break;
          }

          if(stdinBuffer[stdinFill] == '\n') {
            stdinBuffer[stdinFill] = '\0';
            handleInput(stdinBuffer);

            stdinFill = 0;
            break;
          }

          if(stdinFill + 1 < sizeof(stdinBuffer)) ++stdinFill;
          break;
        }
      }
    }
  }

  return 0;
}
