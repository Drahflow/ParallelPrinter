#ifndef H_7C43E5D6_FE70_4955_BA9D_7220716565EC
#define H_7C43E5D6_FE70_4955_BA9D_7220716565EC

#include <stdint.h>

void initUSB(void);
void runUSB(void);

void OTG_FS_IRQHandler(void);

void usb_console_send(const uint8_t *buf, uint_fast8_t buf_len);

#endif
