#ifndef H_549D313F_F477_418B_8CEC_C3160FC41105
#define H_549D313F_F477_418B_8CEC_C3160FC41105

#include <stdint.h>

typedef uint32_t irqstatus_t;

void irq_disable(void);
void irq_enable(void);
irqstatus_t irq_save(void);
void irq_restore(irqstatus_t flag);
void irq_wait(void);

#endif
