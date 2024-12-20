CXX := g++
CXXFLAGS=-W -Wall -Wextra -Werror -O3 -flto

SOURCES := $(wildcard *.c++)
OBJECTS := $(SOURCES:%.c++=%.o)

calibration: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -o $@ $^

%.o: %.c++
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -fv calibration *.o
