CXX := arm-linux-gnueabihf-g++
CXXFLAGS=-W -Wall -Wextra -Werror -O3 -flto -ggdb --std=c++23

SOURCES := $(wildcard *.c++)
OBJECTS := $(SOURCES:%.c++=%.o)

calibration: $(OBJECTS)
	$(CXX) $(CXXFLAGS) -static -o $@ $^

%.o: %.c++ *.h
	$(CXX) $(CXXFLAGS) -c -o $@ $<

clean:
	rm -fv calibration *.o

install: calibration
	scp calibration root@192.168.3.3:calibration/
