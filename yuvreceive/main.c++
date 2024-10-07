#include <iostream>
#include <SDL2/SDL.h>
#include <unistd.h>

using namespace std;

constexpr int width = 640;
constexpr int height = 480;

void exitWithDetails() {
  cerr << "Failed SDL call: " << SDL_GetError() << endl;
  exit(1);
}

void waitForStreamBegin() {
  int equals = 0;

  while(true) {
    char c;
    int len = read(0, &c, 1);
    if(len == 0) {
      cout << "Input ended." << endl;
      exit(0);
    }

    if(c == '=') {
      ++equals;
    } else if(c == '\n') {
      if(equals == 8) break;

      equals = 0;
    } else {
      equals = 0;
    }
  }
}

int main(void) {
  int failed = SDL_Init(SDL_INIT_VIDEO | SDL_INIT_EVENTS);
  if(failed) exitWithDetails();

  cout << "Waiting for stream begin." << endl;
  waitForStreamBegin();

  SDL_Window *window = SDL_CreateWindow("Video Receiver",
      SDL_WINDOWPOS_CENTERED, SDL_WINDOWPOS_CENTERED,
      width, height, 0);
  if(!window) exitWithDetails();

  SDL_Surface *surface = SDL_GetWindowSurface(window);
  if(!surface) exitWithDetails();

  SDL_Event event;

  while(true) {
    while(SDL_PollEvent(&event)) {
      if(event.type == SDL_QUIT) {
        return 0;
      } else if(event.type == SDL_WINDOWEVENT) {
        if(event.window.event == SDL_WINDOWEVENT_CLOSE) {
          return 0;
        }
      }
    }

    int ret = SDL_LockSurface(surface);
    if(ret) exitWithDetails();

    uint8_t input[width * height * 2];
    unsigned int completed = 0;
    while(completed < sizeof(input)) {
      int len = read(0, input + completed, sizeof(input) - completed);
      if(len == 0) {
        cout << "Input ended." << endl;
        return 0;
      }

      if(len < 0) {
        cerr << "Error reading input: " << strerror(errno) << endl;
        return 1;
      }

      completed += len;
    }

    for(int y = 0; y < height; ++y) {
      for(int x = 0; x < width; ++x) {
        int Y = input[(y * width + x) * 2];
        int V = input[(((y * width + x) * 2) & ~3) + 1];
        int U = input[(((y * width + x) * 2) & ~3) + 3];

        Y -=  16;
        U -= 128;
        V -= 128;
        int r = (1164 * Y            + 1596 * V) / 1000;
        int g = (1164 * Y -  392 * U -  813 * V) / 1000;
        int b = (1164 * Y + 2017 * U) / 1000;
        if(r < 0) r = 0;
        if(r > 255) r = 255;
        if(g < 0) g = 0;
        if(g > 255) g = 255;
        if(b < 0) b = 0;
        if(b > 255) b = 255;

        reinterpret_cast<uint32_t *>(surface->pixels)[y * surface->pitch / 4 + x] =
          0x10000 * r + 0x100 * g + 0x1 * b;
      }
    }

    SDL_UnlockSurface(surface);
    if(SDL_UpdateWindowSurface(window)) exitWithDetails();
  }

  SDL_Quit();
  return 0;
}
