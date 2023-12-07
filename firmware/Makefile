CROSS_PREFIX=arm-none-eabi

CC=$(CROSS_PREFIX)-gcc
OBJCOPY=$(CROSS_PREFIX)-objcopy
CFLAGS=-mcpu=cortex-m7 -mthumb -Ilib -Ilib/cmsis-core \
       -ffunction-sections -fdata-sections \
       -Wall
SOURCES=$(wildcard src/*.c)
LIBS=$(wildcard lib/*.c)

OBJECTS=\
	$(SOURCES:src/%.c=build/%.o) \
	$(LIBS:lib/%.c=build/%.o)

build/firmware.bin: build/firmware.elf
	$(OBJCOPY) -O binary $< $@

build/firmware.elf: src/stm32h723ze.ld $(OBJECTS)
	$(CC) $(CFLAGS) \
	  -Wl,--gc-sections \
	  --specs=nano.specs --specs=nosys.specs -T src/stm32h723ze.ld \
	  -flto -fwhole-program -fno-use-linker-plugin \
	  -o $@ $(OBJECTS)

build/%.o: src/%.c src/*.h
	$(CC) $(CFLAGS) -c -o $@ $<

build/%.o: lib/%.c
	$(CC) $(CFLAGS) -c -o $@ $<

flash: build/firmware.bin
	scp build/firmware.bin root@192.168.3.3:3dprinter-fw.bin
	ssh root@192.168.3.3 ./flash_usb.py -d 0483:df11 -t stm32h7 -s 0x8020000 3dprinter-fw.bin

test/%: test/%.c src/*.c src/*.h
	gcc -Wall -Wextra -ggdb -o $@ $<
	valgrind $@
