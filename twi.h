#ifndef TWI_H_
#define TWI_H_

#include <avr/io.h>

#define I2C_WRITE 0
#define I2C_READ 1

extern void TWIInit(void);
extern void TWIStart(void);
extern void TWIStop(void);
extern void TWIWrite(uint8_t u8data);
extern uint8_t TWIReadACK(void);
extern uint8_t TWIReadNACK(void);
extern uint8_t TWIGetStatus(void);

#endif