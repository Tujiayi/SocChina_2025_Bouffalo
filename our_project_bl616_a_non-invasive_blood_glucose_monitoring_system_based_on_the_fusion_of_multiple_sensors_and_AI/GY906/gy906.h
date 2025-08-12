#ifndef __GY906_H
#define __GY906_H
#include "bflb_gpio.h"
 
/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/



#define ACK	 0 //应答
#define	NACK 1 //无应答
#define SA				0x00 //Slave address 单个MLX90614时地址为0x00,多个时地址默认为0x5a
#define RAM_ACCESS		0x00 //RAM access command RAM存取命令
#define EEPROM_ACCESS	0x20 //EEPROM access command EEPROM存取命令
#define RAM_TOBJ1		0x07 //To1 address in the eeprom 目标1温度,检测到的红外温度 -70.01 ~ 382.19度
 

#define SMBUS_SCL		GPIO_PIN_26 //PIN14：SCL
#define SMBUS_SDA		GPIO_PIN_28 //PIN15：SDA
 
#define SMBUS_SCL_H()	    bflb_gpio_set ( gpio, GPIO_PIN_26 )	 //置高电平
#define SMBUS_SCL_L()	    bflb_gpio_reset ( gpio, GPIO_PIN_26 )  //置低电平
#define SMBUS_SDA_H()	    bflb_gpio_set ( gpio, GPIO_PIN_28 )	
#define SMBUS_SDA_L()	    bflb_gpio_reset ( gpio, GPIO_PIN_28 )
 
#define SMBUS_SDA_PIN()	    bflb_gpio_read ( gpio, GPIO_PIN_28 ) //读取引脚电平

void SMBus_StartBit(void);
void SMBus_StopBit(void);
uint8_t SMBus_SendByte(uint8_t Tx_buffer);
void SMBus_SendBit(uint8_t bit_out);
uint8_t SMBus_ReceiveByte(uint8_t ack_nack);
uint8_t SMBus_ReceiveBit(void);
void SMBus_SDA_OUT(void);
void SMBus_SDA_IN(void);
void SMBus_Init();
uint16_t SMBus_ReadMemory(uint8_t slaveAddress, uint8_t command);
uint8_t PEC_Calculation(uint8_t pec[]);
float SMBus_ReadTemp(void);


#endif

