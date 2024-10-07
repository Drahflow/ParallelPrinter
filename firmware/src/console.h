#ifndef H_4C0309B0_844E_434B_B993_AC3FCDECEC5A
#define H_4C0309B0_844E_434B_B993_AC3FCDECEC5A

#include <stdint.h>

// Returns number of bytes processed
int_fast16_t console_receive(uint8_t *buf, uint_fast16_t buf_len);

void console_send(const uint8_t *buf, uint_fast16_t buf_len);
void console_send_str(char *str);
void console_send_uint8(uint32_t val);
void console_send_uint32(uint32_t val);
void console_send_int32(int32_t val);
void console_send_double(double val);

#endif
