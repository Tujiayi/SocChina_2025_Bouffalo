#include "bflb_mtimer.h"
#include "si7021.h"

static struct bflb_device_s *gpio;	

//SI7021引脚输出模式控制
void SDA_OUT(SI7021_Handle *handle)//SDA输出方向配置
{
	gpio = bflb_device_get_by_name("gpio");		
	bflb_gpio_init(gpio, handle->sda_pin, GPIO_OUTPUT| GPIO_FLOAT| GPIO_SMT_EN | GPIO_DRV_1);				

}

void SDA_IN(SI7021_Handle *handle)//SDA输入方向配置
{
	gpio = bflb_device_get_by_name("gpio");		
	bflb_gpio_init(gpio, handle->sda_pin, GPIO_INPUT| GPIO_PULLUP| GPIO_SMT_EN | GPIO_DRV_1);				
}

//初始化IIC
void SI7021_IIC_Init(SI7021_Handle *handle)
{					     
	gpio = bflb_device_get_by_name("gpio");
    printf("gpio output si7021\r\n");
    /* I2C0_SDA */
    bflb_gpio_init(gpio, handle->sda_pin, GPIO_OUTPUT| GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2C0_SCL */
    bflb_gpio_init(gpio, handle->scl_pin, GPIO_OUTPUT| GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_1);	
 
	SI7021_I2C_SCL_1(handle);
	SI7021_I2C_SDA_1(handle);

}
//产生IIC起始信号
void IIC_Start(SI7021_Handle *handle)
{
	SDA_OUT(handle);     //sda线输出
	SI7021_I2C_SDA_1(handle);	  
	bflb_mtimer_delay_us(10);
	SI7021_I2C_SCL_1(handle);
	bflb_mtimer_delay_us(10);
    SI7021_I2C_SDA_0(handle);//START:when CLK is high,DATA change form high to low 
    bflb_mtimer_delay_us(10);
    SI7021_I2C_SCL_0(handle);//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void IIC_Stop(SI7021_Handle *handle)
{
	SDA_OUT(handle);//sda线输出
	SI7021_I2C_SCL_0(handle);//STOP:when CLK is high DATA change form low to high
	SI7021_I2C_SDA_0(handle); 
	bflb_mtimer_delay_us(10);
	SI7021_I2C_SCL_1(handle);
	SI7021_I2C_SDA_1(handle);//发送I2C总线结束信号
	bflb_mtimer_delay_us(10);						   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t IIC_Wait_Ack(SI7021_Handle *handle)
{
	uint8_t ucErrTime=0;
	SDA_IN(handle);      //SDA设置为输入  
	SI7021_I2C_SDA_1(handle);bflb_mtimer_delay_us(10);	   
	SI7021_I2C_SCL_1(handle);bflb_mtimer_delay_us(10);	 
	while(SI7021_I2C_SDA_READ(handle))
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop(handle);
			return 1;
		}
	}
	SI7021_I2C_SCL_0(handle);//时钟输出0  
	return 0;  
} 
//产生ACK应答
void IIC_Ack(SI7021_Handle *handle)
{
	SI7021_I2C_SCL_0(handle);
	SDA_OUT(handle);
	SI7021_I2C_SDA_0(handle);
	bflb_mtimer_delay_us(10);
	SI7021_I2C_SCL_1(handle);
	bflb_mtimer_delay_us(10);
	SI7021_I2C_SCL_0(handle);
}
//不产生ACK应答		    
void IIC_NAck(SI7021_Handle *handle)
{
	SI7021_I2C_SCL_0(handle);
	SDA_OUT(handle);
	SI7021_I2C_SDA_1(handle);
	bflb_mtimer_delay_us(10);
	SI7021_I2C_SCL_1(handle);
	bflb_mtimer_delay_us(10);
	SI7021_I2C_SCL_0(handle);
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void IIC_Send_Byte(SI7021_Handle *handle, uint8_t txd)
{                        
    uint8_t i;
	SDA_OUT(handle); 
	SI7021_I2C_SCL_0(handle);
	/* 先发送字节的高位bit7 */
	for (i=0; i<8;i++)
	{
		if (txd & 0x80)
		{
			SI7021_I2C_SDA_1(handle);
		}
		else
		{
			SI7021_I2C_SDA_0(handle);
		}
		bflb_mtimer_delay_us(10);
		SI7021_I2C_SCL_1(handle);
		bflb_mtimer_delay_us(10);
		SI7021_I2C_SCL_0(handle);
		if (i == 7)
		{
			SI7021_I2C_SDA_1(handle); // 释放总线
		}
		txd <<= 1;	/* 左移一个bit */
		bflb_mtimer_delay_us(10);
	}
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t IIC_Read_Byte(SI7021_Handle *handle, unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN(handle);//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        SI7021_I2C_SCL_0(handle); 
        bflb_mtimer_delay_us(10);
		SI7021_I2C_SCL_1(handle);
        receive<<=1;
        if(SI7021_I2C_SDA_READ(handle))receive++;   
		bflb_mtimer_delay_us(10); 
    }					 
    if (!ack)
        IIC_NAck(handle);//发送nACK
    else
        IIC_Ack(handle); //发送ACK   
    return receive;
}



//结构体定义
_si7021_value si7021;//数据缓存区结构体
_si7021_filter si7021_filter;//平均值滤波器结构体

//变量定义，滤波后的最终结果，可使用串口打印
float TEMP_buf,Humi_buf;

//函数名称：single_write_Si7021
//函数功能：单字节写入传感器
//参数描述：
//返 回 值：
void single_write_Si7021(SI7021_Handle *handle, uint8_t REG_address)
{
	IIC_Start(handle);
	
	IIC_Send_Byte(handle,(SLAVE_ADDR<<1)|0);
	IIC_Wait_Ack(handle);
	
	IIC_Send_Byte(handle,REG_address);
	IIC_Wait_Ack(handle);
	
	IIC_Stop(handle);
}

//函数名称：Multiple_read_Si7021
//函数功能：多字节读取传感器
//参数描述：
//返 回 值：
void Multiple_read_Si7021(SI7021_Handle *handle, uint8_t REG_address, uint16_t *value)
{
	uint8_t Si7021_BUF[2]={0};
	
	IIC_Start(handle);
	
	IIC_Send_Byte(handle, (SLAVE_ADDR<<1)|0);
	IIC_Wait_Ack(handle);
//	if(IIC_Wait_Ack()) 
//	printf("\r\n fail to receive ack \r\n");
	
	IIC_Send_Byte(handle, REG_address);
	IIC_Wait_Ack(handle);
	
	bflb_mtimer_delay_ms(20);
	
	IIC_Start(handle);
	IIC_Send_Byte(handle, (SLAVE_ADDR<<1)|1);
	IIC_Wait_Ack(handle);	
	Si7021_BUF[0] = IIC_Read_Byte(handle, 1);
	Si7021_BUF[1] = IIC_Read_Byte(handle, 0);
	
	IIC_Stop(handle);
	
	*value=((Si7021_BUF[0]<<8)+Si7021_BUF[1]);
}

//函数名称：measure_si7021
//函数功能：NO HOLD MASTER模式下读取温湿度 
//参数描述：无
//返 回 值：无
void measure_Si7021(SI7021_Handle *handle)
{
	//缓存变量定义
	uint16_t TEMP,HUMI;
	uint8_t curI;
	
	//读取温度
	Multiple_read_Si7021(handle, TEMP_NOHOLD_MASTER,&TEMP);
//	printf("\r\nTempPrime:%.2f\r\n",TEMP);
	si7021.temp=(((((float)TEMP)*175.72f)/65536.0f) - 46.85f);//将原始温度数据计算为实际温度数据并传递给缓存区，单位 ℃
//	printf("\r\nTempProcess:%.2f\r\n",si7021.temp);
//	TEMP_buf=(((((float)TEMP)*175.72f)/65536.0f) - 46.85f);
	
	Multiple_read_Si7021(handle, HUMI_NOHOLD_MASTER,&HUMI);

	si7021.humi=(((((float)HUMI)*125.0f)/65535.0f) - 6.0f);//将原始湿度数据计算为实际湿度数据并传递给缓存区，单位 %RH
//	Humi_buf=(((((float)HUMI)*125.0f)/65535.0f) - 6.0f);
	
	//以下为平均值滤波代码，循环储存10次的数据，调用一次measure_Si7021()就存一次
	if(MEAN_NUM > si7021_filter.curI)//当MEAN_NUM==10时，完成10次读取
	{
		si7021_filter.tBufs[si7021_filter.curI] = si7021.temp;
		si7021_filter.hBufs[si7021_filter.curI] = si7021.humi;

		si7021_filter.curI++;
	}
	else
	{
		si7021_filter.curI = 0;

		si7021_filter.tBufs[si7021_filter.curI] = si7021.temp;
		si7021_filter.hBufs[si7021_filter.curI] = si7021.humi;

		si7021_filter.curI++;
	}
	
	if(MEAN_NUM <= si7021_filter.curI) 
    {
        si7021_filter.thAmount = MEAN_NUM;
    }

	//判断是否初次循环
    if(0 == si7021_filter.thAmount) 
    {
        //计算采集第10次数据之前的平均值
        for(curI = 0; curI < si7021_filter.curI; curI++)
        {
            si7021.temp += si7021_filter.tBufs[curI];
            si7021.humi += si7021_filter.hBufs[curI];
        }

        si7021.temp = si7021.temp / si7021_filter.curI;
        si7021.humi = si7021.humi / si7021_filter.curI; 
        
        TEMP_buf = si7021.temp;
        Humi_buf = si7021.humi;
    }
    else if(MEAN_NUM == si7021_filter.thAmount) 
    {
        //计算采集第10次数据之后的平均值
        for(curI = 0; curI < si7021_filter.thAmount; curI++) 
        {
            si7021.temp += si7021_filter.tBufs[curI];
            si7021.humi += si7021_filter.hBufs[curI];
        }

        si7021.temp = si7021.temp / si7021_filter.thAmount; 
        si7021.humi = si7021.humi / si7021_filter.thAmount; 
        
        TEMP_buf = si7021.temp; 
        Humi_buf = si7021.humi; 
    }
}




///////////////////
/// @param Cmd 
/// @return 
float Si7021_Measure(SI7021_Handle *handle, uint8_t Cmd)
{
	uint16_t data ;
	float value;
	IIC_Start(handle);
	IIC_Send_Byte(handle, SI7021_W_ADDR);
	if(IIC_Wait_Ack(handle)) return 9;
	IIC_Send_Byte(handle, Cmd);
	if(IIC_Wait_Ack(handle)) return 2;
	bflb_mtimer_delay_ms(20);
	IIC_Start(handle);
	IIC_Send_Byte(handle, SI7021_R_ADDR);
	if(IIC_Wait_Ack(handle)) return 3;
	data = IIC_Read_Byte(handle, 1);
	data = data<<8;
	data = data + IIC_Read_Byte(handle, 0);
	IIC_Stop(handle);
	if(Cmd == TEMP_NOHOLD_MASTER)
	{
		value = 175.72 * data / 65536.0 - 46.85;
	}
	if(Cmd == HUMI_NOHOLD_MASTER)
	{
		value = 125.00f * data / 65536 - 6;
	}
	return value;
}

float Si7021_TEMP_Measure(SI7021_Handle *handle)
{
	uint16_t data ;
	float value;
	IIC_Start(handle);
	IIC_Send_Byte(handle, SI7021_W_ADDR);
	if(IIC_Wait_Ack(handle)) return 1;
	IIC_Send_Byte(handle, READ_TEMP_from_Previous_RH);
	if(IIC_Wait_Ack(handle)) return 2;
	//SysTick_Delay_ms_INT(20);
	IIC_Start(handle);
	IIC_Send_Byte(handle, SI7021_R_ADDR);
	if(IIC_Wait_Ack(handle)) return 3;
	data = IIC_Read_Byte(handle, 1);
	data = data<<8;
	data = data + IIC_Read_Byte(handle, 0);
	IIC_Stop(handle);
	value = 175.72 * data / 65536.0 - 46.85;
	return value;
}

uint8_t Read_Si7021_Firmware_Revision(SI7021_Handle *handle)
{
	uint8_t FWREV=0;
	IIC_Start(handle);
	IIC_Send_Byte(handle,SI7021_W_ADDR);
	if(IIC_Wait_Ack(handle)) return 1;
	IIC_Send_Byte(handle, 0x84);
	if(IIC_Wait_Ack(handle)) return 1;
	IIC_Send_Byte(handle, 0xB8);
	if(IIC_Wait_Ack(handle)) return 1;
	bflb_mtimer_delay_ms(20);
	IIC_Start(handle);
	IIC_Send_Byte(handle, SI7021_R_ADDR);
	if(IIC_Wait_Ack(handle)) return 1;
	FWREV = IIC_Read_Byte(handle,1);
	if(IIC_Wait_Ack(handle)) return 1;
	//if(!I2C_WaitAck()) return 1;
	IIC_Stop(handle);
	return FWREV;
}
	
