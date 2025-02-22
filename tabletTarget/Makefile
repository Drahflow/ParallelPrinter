tabletTarget: *.c
	arm-linux-gnueabihf-gcc -static -W -Wall -Wextra -Werror -std=c11 -ggdb -O4 $^ -o $@

install: tabletTarget
	scp tabletTarget root@192.168.3.3:calibration/
	ssh root@192.168.3.3 adb push calibration/tabletTarget /data/local/tmp/tabletTarget
