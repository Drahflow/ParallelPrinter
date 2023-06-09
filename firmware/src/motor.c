#include "motor.h"

#include "gpio.h"
#include "console.h"

static struct gpio_out m1enable;
static struct gpio_out m1step;
static struct gpio_out m1dir;
static struct gpio_out m1uart_out;
static struct gpio_in m1uart_in;

uint8_t m1stepPulse = 0;

void wait(uint32_t n) {
  for(volatile uint32_t i = 0; i < n; ++i);
}

void initMotorDrivers() {
  m1enable = gpio_out_setup(GPIO('E', 6), 0);
  m1step = gpio_out_setup(GPIO('C', 13), 0);
  m1dir = gpio_out_setup(GPIO('C', 14), 0);
  m1uart_in = gpio_in_setup(GPIO('G', 14), 0);
  m1uart_out = gpio_out_setup(GPIO('G', 14), 1);
}

#define WAIT_STEP 7500
#define WAIT_UART 10000

static void drive(uint32_t n) {
    for(uint32_t i = 0; i < n; ++i) {
      gpio_out_write(m1step, m1stepPulse);
      m1stepPulse = !m1stepPulse;
      wait(WAIT_STEP);
    }
}

void forward(uint32_t n) {
    gpio_out_write(m1dir, 1);
    wait(WAIT_STEP);

    drive(n);
}

void backward(uint32_t n) {
    gpio_out_write(m1dir, 0);
    wait(WAIT_STEP);

    drive(n);
}

static void updateCRC(uint8_t byte, uint8_t *crc) {
  for(uint8_t j = 0; j < 8; ++j) {
    if((*crc >> 7) ^ (byte & 1)) {
      *crc = (*crc << 1) ^ 0x07;
    } else {
      *crc = (*crc << 1);
    }

    byte = byte >> 1;
  }
}

static void uartSendByte(uint8_t byte) {
  gpio_out_write(m1uart_out, 0); wait(WAIT_UART);

  for(uint8_t i = 0; i < 8; ++i) {
    gpio_out_write(m1uart_out, byte & 1); wait(WAIT_UART);
    byte >>= 1;
  }

  gpio_out_write(m1uart_out, 1); wait(WAIT_UART);
}

// Automatically adds initial sync byte and final crc byte
static void uartSend(uint8_t *data, uint32_t len) {
  uint8_t crc = 0;
  updateCRC(0x05, &crc);
  for(uint32_t i = 0; i < len; ++i) updateCRC(data[i], &crc);

  uartSendByte(0x05);
  for(uint32_t i = 0; i < len; ++i) uartSendByte(data[i]);
  uartSendByte(crc);
}

static void uartWriteRegister(uint8_t reg, uint32_t data) {
  gpio_out_reset(m1uart_out, 1);

  uint8_t msg[] = {
    0x00, // chip address,
    reg | 0x80,
    data >> 24,
    data >> 16,
    data >> 8,
    data,
  };

  uartSend(msg, sizeof(msg));
}

void setResolution(uint32_t powerOfTwo) {
  uartWriteRegister(0x00,
      (1u << 0) | // use VREF
      // (1u << 1) | // use internal resistors
      (1u << 2) | // use spread cycle
      (1u << 6) | // uart pin doesn't control power-down
      (1u << 7) | // microstep resolution in MSTEP
      // (1u << 8) | // filter steps
      0);
  uartWriteRegister(0x10,
      (16u << 0) | // smallish IHOLD,
      (24u << 8) | // medium IRUN,
      (4u << 16) | // small-ish IHOLDDELAY
      0);
  uartWriteRegister(0x6C,
      (1u << 29) | // step on both edges
      // (1u << 28) | // interpolate to full 256 microsteps
      (powerOfTwo << 24) |
      (5u << 7) | // default hysteresis
      (3u << 0) | // default hysteresis
      0);
}

static uint8_t uartRecv(uint8_t *buf, uint32_t len) {
  for(int i = 0; i < len; ++i) {
    buf[i] = 0;

    uint8_t zero = gpio_in_read(m1uart_in); wait(WAIT_UART);
    if(zero != 0) return 0;

    for(int j = 0; j < 8; ++j) {
      buf[i] |= gpio_in_read(m1uart_in) << j; wait(WAIT_UART);
    }

    uint8_t one = gpio_in_read(m1uart_in); wait(WAIT_UART);
    if(one != 1) return 0;
  }

  uint8_t crc = 0;
  for(uint32_t i = 0; i < len - 1; ++i) updateCRC(buf[i], &crc);

  return buf[len - 1] == crc;
}

static uint32_t uartReadRegister(uint8_t reg) {
  gpio_out_reset(m1uart_out, 1);

  uint8_t msg[] = {
    0x00, // chip address,
    reg,
  };

  uartSend(msg, sizeof(msg));

  gpio_in_reset(m1uart_in, 0);
  wait(WAIT_UART * 8);

  uint8_t buf[8];
  uint8_t success = uartRecv(buf, sizeof(buf));
  if(!success) {
    console_send_str("UART FAIL!\r\n");
  }

  wait(WAIT_UART * 8);

  gpio_out_reset(m1uart_out, 1);

  return
    (uint32_t)(buf[3]) << 24 |
    (uint32_t)(buf[4]) << 16 |
    (uint32_t)(buf[5]) << 8 |
    (uint32_t)(buf[6]);
}

void dumpMotorStatus() {
  uint32_t gconf = uartReadRegister(0x00);
  uint32_t chopconf = uartReadRegister(0x6C);

  // Reset Value 00000000 00000000 00000001 00000001
  console_send_str("GConf: ");
  console_send_uint32(gconf);

  // Reset Value 00010101 00000001 00000000 01010011
  console_send_str("\r\nChop: ");
  console_send_uint32(chopconf);
  console_send_str("\r\n");
}
