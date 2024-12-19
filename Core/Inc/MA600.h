#ifndef __MA600_H__
#define __MA600_H__

#include <stdbool.h>


#define SPI1_CS_Pin GPIO_PIN_0
#define SPI1_CS_GPIO_Port GPIOB

void MA600_Init(void);


uint16_t readMagAlphaAngleWithParityBitCheck(bool* error);
uint16_t readMagAlphaAngle(void);
uint8_t readMagAlphaRegister(uint8_t address);
uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value);




#endif
