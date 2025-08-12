#ifndef	_SI7021_H
#define	_SI7021_H

#include "bflb_gpio.h"


//SI7021_SCL
//SI7021_SDA
typedef struct {
	uint8_t sda_pin;
	uint8_t scl_pin;
	} SI7021_Handle;

//数据缓存区结构体
typedef	struct
{
	float temp;
	float humi;
	uint8_t crc;
}_si7021_value;

//平均值滤波器结构体
typedef struct
{
	uint8_t curI;
	uint8_t thAmount;
	float tBufs[10];
	float hBufs[10];
}_si7021_filter;

//外部声明，滤波后的最终结果，可使用串口打印
extern float TEMP_buf,Humi_buf;

#define     SI7021_I2C_SCL_1(handle)                     bflb_gpio_set ( gpio, handle->scl_pin )			
#define     SI7021_I2C_SCL_0(handle)                     bflb_gpio_reset ( gpio, handle->scl_pin  )
#define     SI7021_I2C_SDA_1(handle)                     bflb_gpio_set ( gpio, handle->sda_pin  )			
#define     SI7021_I2C_SDA_0(handle)                     bflb_gpio_reset ( gpio, handle->sda_pin  )

#define     SI7021_I2C_SDA_READ(handle)                  bflb_gpio_read ( gpio, handle->sda_pin )	/* 读SDA口线状态 */

/************多少个数据参与平均值滤波************/
#define MEAN_NUM  10

/*******************传感器相关*******************/
#define	SI7021_W_ADDR				0x80		//写地址
#define	SI7021_R_ADDR				0x81		//读地址
#define SLAVE_ADDR		0x40		//设备地址

#define	HUMI_HOLD_MASTER	0xE5
#define	TEMP_HOLD_MASTER	0xE3	
#define	HUMI_NOHOLD_MASTER	0xF5
#define	TEMP_NOHOLD_MASTER	0xF3


#define Si7021_RST			0xFE
#define Write_UserReg		0xE6
#define Read_UserReg		0xE7

 
#define	READ_TEMP_from_Previous_RH		0xE0				//读取先前RH测量的温度值
#define RESET_SI7021					0xFE				//重置
#define WRITE_RH_T_USER_Register1		0xE6				//写RH/T用户寄存器1
#define READ_RH_T_USER_Register1		0xE7				//读RH/T用户寄存器1
#define	READ_Electronic_ID_1st_Byte		0xFA 0x0F		//读取电子ID第1字节
#define	READ_Electronic_ID_2nd_Byte		0xFC 0xC9		//读取电子ID第2字节	需要checksum byte
#define READ_Firmware_Revision			0x84 0xB8		//读取固件版本

//初始化IIC
void SI7021_IIC_Init(SI7021_Handle *handle);
void IIC_Start(SI7021_Handle *handle);
void IIC_Stop(SI7021_Handle *handle);
uint8_t IIC_Wait_Ack(SI7021_Handle *handle);
void IIC_Ack(SI7021_Handle *handle);	    
void IIC_NAck(SI7021_Handle *handle);		  
void IIC_Send_Byte(SI7021_Handle *handle, uint8_t txd);
uint8_t IIC_Read_Byte(SI7021_Handle *handle, unsigned char ack);


float Si7021_Measure(SI7021_Handle *handle, uint8_t Cmd);
float Si7021_TEMP_Measure(SI7021_Handle *handle);
uint8_t Read_Si7021_Firmware_Revision(SI7021_Handle *handle);


void single_write_Si7021(SI7021_Handle *handle, uint8_t REG_address);
void Multiple_read_Si7021(SI7021_Handle *handle, uint8_t REG_address, uint16_t *value);
void measure_Si7021(SI7021_Handle *handle);



#endif
