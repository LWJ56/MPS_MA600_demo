#include "main.h"   
#include "MA600.h"
#include "spi.h"

uint16_t readMagAlphaAngleWithParityBitCheck(bool* error);
uint16_t readMagAlphaAngle(void);
uint8_t readMagAlphaRegister(uint8_t address);
uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value);


uint8_t readbackRegValue;

void MA600_Init(void)
{
	
  uint8_t regAddress;
  uint8_t regValue;
  bool error;
	
	//Example of MagAlpha Register Settings (Set Reg 0 to 0x80)
  regAddress = 0;
  regValue = 0x80;
  //Read the initial register value
  readbackRegValue=readMagAlphaRegister(regAddress);
  //write the register with the desired value
  readbackRegValue=writeMagAlphaRegister(regAddress, regValue);

  //remove warning during compilation
  (void)readbackRegValue;
  (void)error;
}

uint16_t readMagAlphaAngle(void)
{
  uint32_t timeout=10;
  uint8_t txData[2];
  uint8_t rxData[2];
  txData[1]=0;
  txData[0]=0;
  uint16_t angleSensor;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 2, timeout);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  angleSensor=rxData[0]<<8 | rxData[1];
  return angleSensor;
}

uint8_t readMagAlphaRegister(uint8_t address)
{
  uint32_t timeout=10;
  uint32_t delay=1;//ms
  uint8_t txData1[2];
  uint8_t rxData1[2];
  uint8_t txData2[2];
  uint8_t rxData2[2];
  txData1[0]=(0x2<<5)|(0x1F&address);
  txData1[1]=0x00;
  txData2[0]=0x00;
  txData2[1]=0x00;
  uint8_t registerReadbackValue;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, txData1, rxData1, 2, timeout);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, txData2, rxData2, 2, timeout);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  registerReadbackValue=rxData2[0];
  return registerReadbackValue;
}

uint8_t writeMagAlphaRegister(uint8_t address, uint8_t value)
{
  uint32_t timeout=10;
  uint32_t delay=20;//ms
  uint8_t txData1[2];
  uint8_t rxData1[2];
  uint8_t txData2[2];
  uint8_t rxData2[2];
  txData1[0]=(0x4<<5)|(0x1F&address);
  txData1[1]=value;
  txData2[0]=0x00;
  txData2[1]=0x00;
  uint8_t registerReadbackValue;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, txData1, rxData1, 2, timeout);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  HAL_Delay(delay);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, txData2, rxData2, 2, timeout);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  registerReadbackValue=rxData2[0];
  return registerReadbackValue;
}

uint16_t readMagAlphaAngleWithParityBitCheck(bool* error)
{
  uint32_t timeout=10;
  uint8_t highStateCount = 0;
  uint8_t parity;
  uint8_t txData[3];
  uint8_t rxData[3];
  txData[2]=0;
  txData[1]=0;
  txData[0]=0;
  uint16_t angleSensor;
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_RESET);
  HAL_SPI_TransmitReceive(&hspi1, txData, rxData, 3, timeout);
  HAL_GPIO_WritePin(SPI1_CS_GPIO_Port, SPI1_CS_Pin, GPIO_PIN_SET);
  angleSensor=rxData[0]<<8 | rxData[1];
  parity = ((rxData[2] & 0x80) >> 7);
  //Count the number of 1 in the angle binary value
  for (int i=0;i<16;++i)
  {
    if ((angleSensor & (1 << i)) != 0)
    {
        highStateCount++;
    }
  }
  //check if parity bit is correct
  if ((highStateCount % 2) == 0) //number of bits set to 1 in the angle is even
  {
    if (parity == 0)
    {
      *error = false;
    }
    else
    {
      *error = true;
    }
  }
  else //number of bits set to 1 in the angle is odd
  {
    if (parity == 1)
    {
      *error = false;
    }
    else
    {
      *error = true;
    }
  }
  return angleSensor;
}
