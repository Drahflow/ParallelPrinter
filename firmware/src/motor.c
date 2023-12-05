#include "motor.h"

#include "console.h"

Motor motors[MOTOR_COUNT];

void wait(uint32_t n) {
  for(volatile uint32_t i = 0; i < n; ++i);
}

void initMotorDrivers() {
  Motor *m;

#if MOTOR_COUNT > 0
  m = motors + 0;
  m->index = 0;
  m->enable = gpio_out_setup(GPIO('E', 6), 1);
  m->step = gpio_out_setup(GPIO('C', 13), 0);
  m->dir = gpio_out_setup(GPIO('C', 14), 0);
  m->uart_in = gpio_in_setup(GPIO('G', 14), 0);
  m->uart_out = gpio_out_setup(GPIO('G', 14), 1);
#endif

#if MOTOR_COUNT > 1
  m = motors + 1;
  m->index = 1;
  m->enable = gpio_out_setup(GPIO('E', 3), 1);
  m->step = gpio_out_setup(GPIO('E', 4), 0);
  m->dir = gpio_out_setup(GPIO('E', 5), 0);
  m->uart_in = gpio_in_setup(GPIO('G', 13), 0);
  m->uart_out = gpio_out_setup(GPIO('G', 13), 1);
#endif

#if MOTOR_COUNT > 2
  m = motors + 2;
  m->index = 2;
  m->enable = gpio_out_setup(GPIO('E', 2), 1);
  m->step = gpio_out_setup(GPIO('E', 1), 0);
  m->dir = gpio_out_setup(GPIO('E', 0), 0);
  m->uart_in = gpio_in_setup(GPIO('G', 12), 0);
  m->uart_out = gpio_out_setup(GPIO('G', 12), 1);
#endif

#if MOTOR_COUNT > 3
  m = motors + 3;
  m->index = 3;
  m->enable = gpio_out_setup(GPIO('B', 7), 1);
  m->step = gpio_out_setup(GPIO('B', 8), 0);
  m->dir = gpio_out_setup(GPIO('B', 9), 0);
  m->uart_in = gpio_in_setup(GPIO('G', 11), 0);
  m->uart_out = gpio_out_setup(GPIO('G', 11), 1);
#endif

#if MOTOR_COUNT > 4
  m = motors + 4;
  m->index = 4;
  m->enable = gpio_out_setup(GPIO('B', 6), 1);
  m->step = gpio_out_setup(GPIO('B', 5), 0);
  m->dir = gpio_out_setup(GPIO('B', 4), 0);
  m->uart_in = gpio_in_setup(GPIO('G', 10), 0);
  m->uart_out = gpio_out_setup(GPIO('G', 10), 1);
#endif

#if MOTOR_COUNT > 5
  m = motors + 5;
  m->index = 5;
  m->enable = gpio_out_setup(GPIO('D', 5), 1);
  m->step = gpio_out_setup(GPIO('G', 15), 0);
  m->dir = gpio_out_setup(GPIO('B', 3), 0);
  m->uart_in = gpio_in_setup(GPIO('G', 9), 0);
  m->uart_out = gpio_out_setup(GPIO('G', 9), 1);
#endif

#if MOTOR_COUNT > 6
  m = motors + 6;
  m->index = 6;
  m->enable = gpio_out_setup(GPIO('D', 4), 1);
  m->step = gpio_out_setup(GPIO('D', 3), 0);
  m->dir = gpio_out_setup(GPIO('D', 2), 0);
  m->uart_in = gpio_in_setup(GPIO('D', 7), 0);
  m->uart_out = gpio_out_setup(GPIO('D', 7), 1);
#endif

#if MOTOR_COUNT > 7
  m = motors + 7;
  m->index = 7;
  m->enable = gpio_out_setup(GPIO('A', 15), 1);
  m->step = gpio_out_setup(GPIO('A', 10), 0);
  m->dir = gpio_out_setup(GPIO('A', 9), 0);
  m->uart_in = gpio_in_setup(GPIO('D', 6), 0);
  m->uart_out = gpio_out_setup(GPIO('D', 6), 1);
#endif

#if MOTOR_COUNT > 8
  m = motors + 8;
  m->index = 8;
  m->enable = gpio_out_setup(GPIO('C', 9), 1);
  m->step = gpio_out_setup(GPIO('A', 8), 0);
  m->dir = gpio_out_setup(GPIO('C', 7), 0);
  m->uart_in = gpio_in_setup(GPIO('G', 8), 0);
  m->uart_out = gpio_out_setup(GPIO('G', 8), 1);
#endif

#if MOTOR_COUNT > 9
  m = motors + 9;
  m->index = 9;
  m->enable = gpio_out_setup(GPIO('C', 8), 1);
  m->step = gpio_out_setup(GPIO('G', 6), 0);
  m->dir = gpio_out_setup(GPIO('C', 6), 0);
  m->uart_in = gpio_in_setup(GPIO('G', 7), 0);
  m->uart_out = gpio_out_setup(GPIO('G', 7), 1);
#endif
}

#define WAIT_UART 10000

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

static void uartSendByte(Motor *m, uint8_t byte) {
  gpio_out_write(m->uart_out, 0); wait(WAIT_UART);

  for(uint8_t i = 0; i < 8; ++i) {
    gpio_out_write(m->uart_out, byte & 1); wait(WAIT_UART);
    byte >>= 1;
  }

  gpio_out_write(m->uart_out, 1); wait(WAIT_UART);
}

// Automatically adds initial sync byte and final crc byte
static void uartSend(Motor *m, uint8_t *data, uint32_t len) {
  uint8_t crc = 0;
  updateCRC(0x05, &crc);
  for(uint32_t i = 0; i < len; ++i) updateCRC(data[i], &crc);

  uartSendByte(m, 0x05);
  for(uint32_t i = 0; i < len; ++i) uartSendByte(m, data[i]);
  uartSendByte(m, crc);
}

static void uartWriteRegister(Motor *m, uint8_t reg, uint32_t data) {
  gpio_out_reset(m->uart_out, 1);

  uint8_t msg[] = {
    0x00, // chip address,
    reg | 0x80,
    data >> 24,
    data >> 16,
    data >> 8,
    data,
  };

  uartSend(m, msg, sizeof(msg));
}

// stepResolution == 0 => 256 microsteps
// stepResolution == 8 => 1 (micro)steps
// runPower == 16 => minimal recommended
// runPower == 31 => maximum
void setupMotor(Motor *m, uint32_t stepResolution, uint32_t runPower) {
  uartWriteRegister(m, 0x00,
      (1u << 0) | // use VREF
      // (1u << 1) | // use internal resistors (tried, but was worse)
      (1u << 2) | // use spread cycle
      (1u << 6) | // uart pin doesn't control power-down
      (1u << 7) | // microstep resolution in MSTEP
      // (1u << 8) | // filter steps
      0);
  uartWriteRegister(m, 0x10,
      ((runPower * 2 / 3) << 0) | // smallish IHOLD,
      (runPower << 8) | // medium IRUN,
      (4u << 16) | // small-ish IHOLDDELAY
      0);
  uartWriteRegister(m, 0x6C,
      (1u << 29) | // step on both edges
      // (1u << 28) | // interpolate to full 256 microsteps, klipper doc advises against this
      (stepResolution << 24) |
      (5u << 7) | // default hysteresis
      (3u << 0) | // default hysteresis
      0);

  gpio_out_write(m->enable, 0);
}

static uint8_t uartRecv(Motor *m, uint8_t *buf, uint32_t len) {
  for(int i = 0; i < len; ++i) {
    buf[i] = 0;

    uint8_t zero = gpio_in_read(m->uart_in); wait(WAIT_UART);
    if(zero != 0) return 0;

    for(int j = 0; j < 8; ++j) {
      buf[i] |= gpio_in_read(m->uart_in) << j; wait(WAIT_UART);
    }

    uint8_t one = gpio_in_read(m->uart_in); wait(WAIT_UART);
    if(one != 1) return 0;
  }

  uint8_t crc = 0;
  for(uint32_t i = 0; i < len - 1; ++i) updateCRC(buf[i], &crc);

  return buf[len - 1] == crc;
}

static uint32_t uartReadRegister(Motor *m, uint8_t reg) {
  gpio_out_reset(m->uart_out, 1);

  uint8_t msg[] = {
    0x00, // chip address,
    reg,
  };

  uartSend(m, msg, sizeof(msg));

  gpio_in_reset(m->uart_in, 0);
  wait(WAIT_UART * 8);

  uint8_t buf[8];
  uint8_t success = uartRecv(m, buf, sizeof(buf));
  if(!success) {
    console_send_str("UART FAIL!\r\n");
  }

  wait(WAIT_UART * 8);

  gpio_out_reset(m->uart_out, 1);

  return
    (uint32_t)(buf[3]) << 24 |
    (uint32_t)(buf[4]) << 16 |
    (uint32_t)(buf[5]) << 8 |
    (uint32_t)(buf[6]);
}

void dumpMotorStatus(Motor *m) {
  uint32_t gconf = uartReadRegister(m, 0x00);
  uint32_t chopconf = uartReadRegister(m, 0x6C);

  // Reset Value 00000000 00000000 00000001 00000001
  console_send_str("GConf: ");
  console_send_uint32(gconf);

  // Reset Value 00010101 00000001 00000000 01010011
  console_send_str("\r\nChop: ");
  console_send_uint32(chopconf);
  console_send_str("\r\n");
}
