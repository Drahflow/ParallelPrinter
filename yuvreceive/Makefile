CXX := g++-14
CXXFLAGS=-W -Wall -Wextra -Werror -O3 -flto

SOURCES := $(wildcard *.c++)
OBJECTS := $(SOURCES:%.c++=%.o)

yuvreceive: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $< -lSDL2

%.o: %.c++
	$(CXX) $(CXXFLAGS) -c -o $@ $<
