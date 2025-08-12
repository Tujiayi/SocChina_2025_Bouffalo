#include "max30102.h"
#include "bflb_mtimer.h"

static struct bflb_device_s *gpio;	

uint8_t max30102_Bus_Write(uint8_t Register_Address, uint8_t Word_Data)
{

	/* 采用串行EEPROM随即读取指令序列，连续读取若干字节 */

	/* 第1步：发起I2C总线启动信号 */
	MAX30102_IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址 */
	MAX30102_IIC_Send_Byte(Register_Address);
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	
	/* 第5步：开始写入数据 */
	MAX30102_IIC_Send_Byte(Word_Data);

	/* 第6步：发送ACK */
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 发送I2C总线停止信号 */
	MAX30102_IIC_Stop();
	return 1;	/* 执行成功 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MAX30102_IIC_Stop();
	return 0;
}



uint8_t max30102_Bus_Read(uint8_t Register_Address)
{
	uint8_t  data;


	/* 第1步：发起I2C总线启动信号 */
	MAX30102_IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	MAX30102_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	MAX30102_IIC_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	{
		data = MAX30102_IIC_Read_Byte(0);	/* 读1个字节 */

		MAX30102_IIC_NAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
	}
	/* 发送I2C总线停止信号 */
	MAX30102_IIC_Stop();
	return data;	/* 执行成功 返回data值 */

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MAX30102_IIC_Stop();
	return 0;
}


void max30102_FIFO_ReadWords(uint8_t Register_Address,uint16_t Word_Data[][2],uint8_t count)
{
	uint8_t i=0;
	uint8_t no = count;
	uint8_t data1, data2;
	/* 第1步：发起I2C总线启动信号 */
	MAX30102_IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	MAX30102_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	MAX30102_IIC_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	while (no)
	{
		data1 = MAX30102_IIC_Read_Byte(0);	
		MAX30102_IIC_Ack();
		data2 = MAX30102_IIC_Read_Byte(0);
		MAX30102_IIC_Ack();
		Word_Data[i][0] = (((uint16_t)data1 << 8) | data2);  //

		
		data1 = MAX30102_IIC_Read_Byte(0);	
		MAX30102_IIC_Ack();
		data2 = MAX30102_IIC_Read_Byte(0);
		if(1==no)
			MAX30102_IIC_NAck();	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
		else
			MAX30102_IIC_Ack();
		Word_Data[i][1] = (((uint16_t)data1 << 8) | data2); 

		no--;	
		i++;
	}
	/* 发送I2C总线停止信号 */
	MAX30102_IIC_Stop();

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MAX30102_IIC_Stop();
}

void max30102_FIFO_ReadBytes(uint8_t Register_Address,uint8_t* Data)
{	
	max30102_Bus_Read(REG_INTR_STATUS_1);
	max30102_Bus_Read(REG_INTR_STATUS_2);
	
	/* 第1步：发起I2C总线启动信号 */
	MAX30102_IIC_Start();

	/* 第2步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_WR);	/* 此处是写指令 */

	/* 第3步：发送ACK */
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第4步：发送字节地址， */
	MAX30102_IIC_Send_Byte((uint8_t)Register_Address);
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}
	

	/* 第6步：重新启动I2C总线。下面开始读取数据 */
	MAX30102_IIC_Start();

	/* 第7步：发起控制字节，高7bit是地址，bit0是读写控制位，0表示写，1表示读 */
	MAX30102_IIC_Send_Byte(max30102_WR_address | I2C_RD);	/* 此处是读指令 */

	/* 第8步：发送ACK */
	if (MAX30102_IIC_Wait_Ack() != 0)
	{
		goto cmd_fail;	/* EEPROM器件无应答 */
	}

	/* 第9步：读取数据 */
	Data[0] = MAX30102_IIC_Read_Byte(1);	
	Data[1] = MAX30102_IIC_Read_Byte(1);	
	Data[2] = MAX30102_IIC_Read_Byte(1);	
	Data[3] = MAX30102_IIC_Read_Byte(1);
	Data[4] = MAX30102_IIC_Read_Byte(1);	
	Data[5] = MAX30102_IIC_Read_Byte(0);
	/* 最后1个字节读完后，CPU产生NACK信号(驱动SDA = 1) */
	/* 发送I2C总线停止信号 */
	MAX30102_IIC_Stop();

cmd_fail: /* 命令执行失败后，切记发送停止信号，避免影响I2C总线上其他设备 */
	/* 发送I2C总线停止信号 */
	MAX30102_IIC_Stop();

//	u8 i;
//	u8 fifo_wr_ptr;
//	u8 firo_rd_ptr;
//	u8 number_tp_read;
//	//Get the FIFO_WR_PTR
//	fifo_wr_ptr = max30102_Bus_Read(REG_FIFO_WR_PTR);
//	//Get the FIFO_RD_PTR
//	firo_rd_ptr = max30102_Bus_Read(REG_FIFO_RD_PTR);
//	
//	number_tp_read = fifo_wr_ptr - firo_rd_ptr;
//	
//	//for(i=0;i<number_tp_read;i++){
//	if(number_tp_read>0){
//		MAX30102_IIC_ReadBytes(max30102_WR_address,REG_FIFO_DATA,Data,6);
//	}
	
	//max30102_Bus_Write(REG_FIFO_RD_PTR,fifo_wr_ptr);
}


void MAX30102_Init(void)
{
    gpio = bflb_device_get_by_name("gpio");

	bflb_gpio_init(gpio, MAX30102_INT_PIN, GPIO_INPUT | GPIO_PULLUP | GPIO_SMT_EN | GPIO_DRV_1);   //上拉输入模式

	MAX30102_IIC_Init();
	
	MAX30102_Reset();
	
//	max30102_Bus_Write(REG_MODE_CONFIG, 0x0b);  //mode configuration : temp_en[3]      MODE[2:0]=010 HR only enabled    011 SP02 enabled
//	max30102_Bus_Write(REG_INTR_STATUS_2, 0xF0); //open all of interrupt
//	max30102_Bus_Write(REG_INTR_STATUS_1, 0x00); //all interrupt clear
//	max30102_Bus_Write(REG_INTR_ENABLE_2, 0x02); //DIE_TEMP_RDY_EN
//	max30102_Bus_Write(REG_TEMP_CONFIG, 0x01); //SET   TEMP_EN

//	max30102_Bus_Write(REG_SPO2_CONFIG, 0x47); //SPO2_SR[4:2]=001  100 per second    LED_PW[1:0]=11  16BITS

//	max30102_Bus_Write(REG_LED1_PA, 0x47); 
//	max30102_Bus_Write(REG_LED2_PA, 0x47); 
	
	
	
	max30102_Bus_Write(REG_INTR_ENABLE_1,0xc0);	// INTR setting
	max30102_Bus_Write(REG_INTR_ENABLE_2,0x00);
	max30102_Bus_Write(REG_FIFO_WR_PTR,0x00);  	//FIFO_WR_PTR[4:0]
	max30102_Bus_Write(REG_OVF_COUNTER,0x00);  	//OVF_COUNTER[4:0]
	max30102_Bus_Write(REG_FIFO_RD_PTR,0x00);  	//FIFO_RD_PTR[4:0]
	max30102_Bus_Write(REG_FIFO_CONFIG,0x0f);  	//sample avg = 1, fifo rollover=false, fifo almost full = 17
	max30102_Bus_Write(REG_MODE_CONFIG,0x03);  	//0x02 for Red only, 0x03 for SpO2 mode 0x07 multimode LED
	max30102_Bus_Write(REG_SPO2_CONFIG,0x27);  	// SPO2_ADC range = 4096nA, SPO2 sample rate (100 Hz), LED pulseWidth (400uS)  
	max30102_Bus_Write(REG_LED1_PA,0x24);   	//Choose value for ~ 7mA for LED1
	max30102_Bus_Write(REG_LED2_PA,0x24);   	// Choose value for ~ 7mA for LED2
	max30102_Bus_Write(REG_PILOT_PA,0x7f);   	// Choose value for ~ 25mA for Pilot LED


}

void MAX30102_Reset(void)
{
	max30102_Bus_Write(REG_MODE_CONFIG,0x40);
	max30102_Bus_Write(REG_MODE_CONFIG,0x40);
}

void maxim_max30102_write_reg(uint8_t uch_addr, uint8_t uch_data)
{
//  char ach_i2c_data[2];
//  ach_i2c_data[0]=uch_addr;
//  ach_i2c_data[1]=uch_data;
//	
//  MAX30102_IIC_WriteBytes(I2C_WRITE_ADDR, ach_i2c_data, 2);
	MAX30102_IIC_Write_One_Byte(I2C_WRITE_ADDR,uch_addr,uch_data);
}

void maxim_max30102_read_reg(uint8_t uch_addr, uint8_t *puch_data)
{
//  char ch_i2c_data;
//  ch_i2c_data=uch_addr;
//  MAX30102_IIC_WriteBytes(I2C_WRITE_ADDR, &ch_i2c_data, 1);
//	
//  i2c.read(I2C_READ_ADDR, &ch_i2c_data, 1);
//  
//   *puch_data=(uint8_t) ch_i2c_data;
	MAX30102_IIC_Read_One_Byte(I2C_WRITE_ADDR,uch_addr,puch_data);
}

void maxim_max30102_read_fifo(uint32_t *pun_red_led, uint32_t *pun_ir_led)
{
	uint32_t un_temp;
	unsigned char uch_temp;
	char ach_i2c_data[6];
	*pun_red_led=0;
	*pun_ir_led=0;

  
  //read and clear status register
  maxim_max30102_read_reg(REG_INTR_STATUS_1, &uch_temp);
  maxim_max30102_read_reg(REG_INTR_STATUS_2, &uch_temp);
  
  MAX30102_IIC_ReadBytes(I2C_WRITE_ADDR,REG_FIFO_DATA,(uint8_t *)ach_i2c_data,6);
  
  un_temp=(unsigned char) ach_i2c_data[0];
  un_temp<<=16;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[1];
  un_temp<<=8;
  *pun_red_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[2];
  *pun_red_led+=un_temp;
  
  un_temp=(unsigned char) ach_i2c_data[3];
  un_temp<<=16;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[4];
  un_temp<<=8;
  *pun_ir_led+=un_temp;
  un_temp=(unsigned char) ach_i2c_data[5];
  *pun_ir_led+=un_temp;
  *pun_red_led&=0x03FFFF;  //Mask MSB [23:18]
  *pun_ir_led&=0x03FFFF;  //Mask MSB [23:18]
}


/*********************************************************************************************/
/*********************************************************************************************/
//MAX30102引脚输出模式控制
void MAX30102_IIC_SDA_OUT(void)//SDA输出方向配置
{
	gpio = bflb_device_get_by_name("gpio");		
	bflb_gpio_init(gpio, MAX30102_IIC_SDA_PIN, GPIO_OUTPUT| GPIO_FLOAT| GPIO_SMT_EN | GPIO_DRV_1);				

}

void MAX30102_IIC_SDA_IN(void)//SDA输入方向配置
{
	gpio = bflb_device_get_by_name("gpio");		
	bflb_gpio_init(gpio, MAX30102_IIC_SDA_PIN, GPIO_INPUT| GPIO_PULLUP| GPIO_SMT_EN | GPIO_DRV_1);				
}

//初始化IIC
void MAX30102_IIC_Init(void)
{			
	gpio = bflb_device_get_by_name("gpio");
    printf("gpio output max30102\r\n");
    /* I2C0_SDA */
    bflb_gpio_init(gpio, MAX30102_IIC_SDA_PIN, GPIO_OUTPUT| GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2C0_SCL */
    bflb_gpio_init(gpio, MAX30102_IIC_SCL_PIN, GPIO_OUTPUT| GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_1);	     
 
	MAX30102_I2C_SCL_1();
	MAX30102_I2C_SDA_1();

}
//产生IIC起始信号
 void MAX30102_IIC_Start(void)
{
	MAX30102_IIC_SDA_OUT();     //sda线输出
	MAX30102_I2C_SDA_1();	  	  
	MAX30102_I2C_SCL_1();
	bflb_mtimer_delay_us(4);
	MAX30102_I2C_SDA_0();//START:when CLK is high,DATA change form high to low 
	bflb_mtimer_delay_us(4);
	MAX30102_I2C_SCL_0();//钳住I2C总线，准备发送或接收数据 
}	  
//产生IIC停止信号
void MAX30102_IIC_Stop(void)
{
	MAX30102_IIC_SDA_OUT();//sda线输出
	MAX30102_I2C_SCL_0();
	MAX30102_I2C_SDA_0();//STOP:when CLK is high DATA change form low to high
	bflb_mtimer_delay_us(4);
	MAX30102_I2C_SCL_1(); 
	MAX30102_I2C_SDA_1();//发送I2C总线结束信号
	bflb_mtimer_delay_us(4);							   	
}
//等待应答信号到来
//返回值：1，接收应答失败
//        0，接收应答成功
uint8_t MAX30102_IIC_Wait_Ack(void)
{
	uint8_t ucErrTime=0;
	MAX30102_IIC_SDA_IN();      //SDA设置为输入
	MAX30102_I2C_SDA_1();bflb_mtimer_delay_us(1);	   
	MAX30102_I2C_SCL_1();bflb_mtimer_delay_us(1);	 
	while(MAX30102_I2C_SDA_READ())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			MAX30102_IIC_Stop();
			return 1;
		}
	}
	MAX30102_I2C_SCL_0();//时钟输出0 	   
	return 0;  
} 
//产生ACK应答
void MAX30102_IIC_Ack(void)
{
	MAX30102_I2C_SCL_0();
	MAX30102_IIC_SDA_OUT();
	MAX30102_I2C_SDA_0();
	bflb_mtimer_delay_us(2);
	MAX30102_I2C_SCL_1();
	bflb_mtimer_delay_us(2);
	MAX30102_I2C_SCL_0();
}
//不产生ACK应答		    
void MAX30102_IIC_NAck(void)
{
	MAX30102_I2C_SCL_0();
	MAX30102_IIC_SDA_OUT();
	MAX30102_I2C_SDA_1();
	bflb_mtimer_delay_us(2);
	MAX30102_I2C_SCL_1();
	bflb_mtimer_delay_us(2);
	MAX30102_I2C_SCL_0();
}					 				     
//IIC发送一个字节
//返回从机有无应答
//1，有应答
//0，无应答			  
void MAX30102_IIC_Send_Byte(uint8_t _ucByte)
{                        
    uint8_t i;
	MAX30102_IIC_SDA_OUT(); 
	MAX30102_I2C_SCL_0();

	/* 先发送字节的高位bit7 */
	for (i=0; i<8;i++)
	{
		if (_ucByte & 0x80)
		{
			MAX30102_I2C_SDA_1();
		}
		else
		{
			MAX30102_I2C_SDA_0();
		}
		bflb_mtimer_delay_us(2);
		MAX30102_I2C_SCL_1();
		bflb_mtimer_delay_us(2);
		MAX30102_I2C_SCL_0();
		if (i == 7)
		{
			MAX30102_I2C_SDA_1(); // 释放总线
		}
		_ucByte <<= 1;	/* 左移一个bit */
		bflb_mtimer_delay_us(2);
	}
} 	    
//读1个字节，ack=1时，发送ACK，ack=0，发送nACK   
uint8_t MAX30102_IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	MAX30102_IIC_SDA_IN();//SDA设置为输入
    for(i=0;i<8;i++ )
	{
        MAX30102_I2C_SCL_0(); 
        bflb_mtimer_delay_us(2);
		MAX30102_I2C_SCL_1();
        receive<<=1;
        if(MAX30102_I2C_SDA_READ())receive++;   
		bflb_mtimer_delay_us(1); 
    }					 
    if (!ack)
        MAX30102_IIC_NAck();//发送nACK
    else
        MAX30102_IIC_Ack(); //发送ACK   
    return receive;
}


void MAX30102_IIC_WriteBytes(uint8_t WriteAddr,uint8_t* data,uint8_t dataLength)
{		
	uint8_t i;	
    MAX30102_IIC_Start();  

	MAX30102_IIC_Send_Byte(WriteAddr);	    //发送写命令
	MAX30102_IIC_Wait_Ack();
	
	for(i=0;i<dataLength;i++)
	{
		MAX30102_IIC_Send_Byte(data[i]);
		MAX30102_IIC_Wait_Ack();
	}				    	   
    MAX30102_IIC_Stop();//产生一个停止条件 
	bflb_mtimer_delay_ms(10);	 
}

void MAX30102_IIC_ReadBytes(uint8_t deviceAddr, uint8_t writeAddr,uint8_t* data,uint8_t dataLength)
{		
	uint8_t i;	
    MAX30102_IIC_Start();  

	MAX30102_IIC_Send_Byte(deviceAddr);	    //发送写命令
	MAX30102_IIC_Wait_Ack();
	MAX30102_IIC_Send_Byte(writeAddr);
	MAX30102_IIC_Wait_Ack();
	MAX30102_IIC_Send_Byte(deviceAddr|0X01);//进入接收模式			   
	MAX30102_IIC_Wait_Ack();
	
	for(i=0;i<dataLength-1;i++)
	{
		data[i] = MAX30102_IIC_Read_Byte(1);
	}		
	data[dataLength-1] = MAX30102_IIC_Read_Byte(0);	
    MAX30102_IIC_Stop();//产生一个停止条件 
	bflb_mtimer_delay_ms(10);	 
}

void MAX30102_IIC_Read_One_Byte(uint8_t daddr,uint8_t addr,uint8_t* data)
{				  	  	    																 
    MAX30102_IIC_Start();  
	
	MAX30102_IIC_Send_Byte(daddr);	   //发送写命令
	MAX30102_IIC_Wait_Ack();
	MAX30102_IIC_Send_Byte(addr);//发送地址
	MAX30102_IIC_Wait_Ack();		 
	MAX30102_IIC_Start();  	 	   
	MAX30102_IIC_Send_Byte(daddr|0X01);//进入接收模式			   
	MAX30102_IIC_Wait_Ack();	 
    *data = MAX30102_IIC_Read_Byte(0);		   
    MAX30102_IIC_Stop();//产生一个停止条件	    
}

void MAX30102_IIC_Write_One_Byte(uint8_t daddr,uint8_t addr,uint8_t data)
{				   	  	    																 
    MAX30102_IIC_Start();  
	
	MAX30102_IIC_Send_Byte(daddr);	    //发送写命令
	MAX30102_IIC_Wait_Ack();
	MAX30102_IIC_Send_Byte(addr);//发送地址
	MAX30102_IIC_Wait_Ack();	   	 										  		   
	MAX30102_IIC_Send_Byte(data);     //发送字节							   
	MAX30102_IIC_Wait_Ack();  		    	   
    MAX30102_IIC_Stop();//产生一个停止条件 
	bflb_mtimer_delay_ms(10);	 
}







const uint16_t auw_hamm[31]={ 41,    276,    512,    276,     41 }; //Hamm=  long16(512* hamming(5)');
//uch_spo2_table is computed as  -45.060*ratioAverage* ratioAverage + 30.354 *ratioAverage + 94.845 ;
const uint8_t uch_spo2_table[184]={ 95, 95, 95, 96, 96, 96, 97, 97, 97, 97, 97, 98, 98, 98, 98, 98, 99, 99, 99, 99, 
                            99, 99, 99, 99, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 100, 
                            100, 100, 100, 100, 99, 99, 99, 99, 99, 99, 99, 99, 98, 98, 98, 98, 98, 98, 97, 97, 
                            97, 97, 96, 96, 96, 96, 95, 95, 95, 94, 94, 94, 93, 93, 93, 92, 92, 92, 91, 91, 
                            90, 90, 89, 89, 89, 88, 88, 87, 87, 86, 86, 85, 85, 84, 84, 83, 82, 82, 81, 81, 
                            80, 80, 79, 78, 78, 77, 76, 76, 75, 74, 74, 73, 72, 72, 71, 70, 69, 69, 68, 67, 
                            66, 66, 65, 64, 63, 62, 62, 61, 60, 59, 58, 57, 56, 56, 55, 54, 53, 52, 51, 50, 
                            49, 48, 47, 46, 45, 44, 43, 42, 41, 40, 39, 38, 37, 36, 35, 34, 33, 31, 30, 29, 
                            28, 27, 26, 25, 23, 22, 21, 20, 19, 17, 16, 15, 14, 12, 11, 10, 9, 7, 6, 5, 
                            3, 2, 1 } ;
static  int32_t an_dx[ BUFFER_SIZE-MA4_SIZE]; // delta
static  int32_t bn_dx[ BUFFER_SIZE-MA4_SIZE]; // delta2
static  int32_t an_x[ BUFFER_SIZE]; //ir
static  int32_t bn_x[ BUFFER_SIZE]; //ir
static  int32_t an_y[ BUFFER_SIZE]; //red

void maxim_heart_rate_and_oxygen_saturation(uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length, uint32_t *pun_red_buffer, int32_t *pn_spo2, int8_t *pch_spo2_valid, 
                              int32_t *pn_heart_rate, int8_t  *pch_hr_valid)
/**
* \brief        Calculate the heart rate and SpO2 level
* \par          Details
*               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the ratio for the SPO2 is computed.
*               Since this algorithm is aiming for Arm M0/M3. formaula for SPO2 did not achieve the accuracy due to register overflow.
*               Thus, accurate SPO2 is precalculated and save longo uch_spo2_table[] per each ratio.
*
* \param[in]    *pun_ir_buffer           - IR sensor data buffer
* \param[in]    n_ir_buffer_length      - IR sensor data buffer length
* \param[in]    *pun_red_buffer          - Red sensor data buffer
* \param[out]    *pn_spo2                - Calculated SpO2 value
* \param[out]    *pch_spo2_valid         - 1 if the calculated SpO2 value is valid
* \param[out]    *pn_heart_rate          - Calculated heart rate value
* \param[out]    *pch_hr_valid           - 1 if the calculated heart rate value is valid
*
* \retval       None
*/
{
    uint32_t un_ir_mean ,un_only_once ;
    int32_t k ,n_i_ratio_count;
    int32_t i, s, m, n_exact_ir_valley_locs_count ,n_middle_idx;
    int32_t n_th1, n_npks,n_c_min;      
    int32_t an_ir_valley_locs[15] ;
    int32_t an_exact_ir_valley_locs[15] ;
    int32_t an_dx_peak_locs[15] ;
    int32_t n_peak_interval_sum;
    
    int32_t n_y_ac, n_x_ac;
    int32_t n_spo2_calc; 
    int32_t n_y_dc_max, n_x_dc_max; 
    int32_t n_y_dc_max_idx, n_x_dc_max_idx; 
    int32_t an_ratio[5],n_ratio_average; 
    int32_t n_nume,  n_denom ;
    // 平均值,用于去直流    
    un_ir_mean =0; 
    for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
    un_ir_mean =un_ir_mean/n_ir_buffer_length ;
    for (k=0 ; k<n_ir_buffer_length ; k++ )  an_x[k] =  pun_ir_buffer[k] - un_ir_mean ; 
    
    // 4点移动平均,用于滤波
    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
        n_denom= ( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3]);
        an_x[k]=  n_denom/(int32_t)4; 
    }

    // 一阶差分
    
    for( k=0; k<BUFFER_SIZE-MA4_SIZE-1;  k++)
        an_dx[k]= (an_x[k+1]- an_x[k]);

    // 差分后的值做2点移动平均
    for(k=0; k< BUFFER_SIZE-MA4_SIZE-2; k++){
        an_dx[k] =  ( an_dx[k]+an_dx[k+1])/2 ;
    }
    
    // hamming window 构造窗函数
    // 减少频谱泄露
    for ( i=0 ; i<BUFFER_SIZE-HAMMING_SIZE-MA4_SIZE-2 ;i++){
        s= 0;
        for( k=i; k<i+ HAMMING_SIZE ;k++){
            s -= an_dx[k] *auw_hamm[k-i] ; 
                     }
        an_dx[i]= s/ (int32_t)1146; // divide by sum of auw_hamm 
    }

 
    n_th1=0; // 波形提取特征点的阈值
    for ( k=0 ; k<BUFFER_SIZE-HAMMING_SIZE ;k++){
        n_th1 += ((an_dx[k]>0)? an_dx[k] : ((int32_t)0-an_dx[k])) ;
    }
    n_th1= n_th1/ ( BUFFER_SIZE-HAMMING_SIZE);
    // peak location is acutally index for sharpest location of raw signal since we flipped the signal         
    maxim_find_peaks( an_dx_peak_locs, &n_npks, an_dx, BUFFER_SIZE-HAMMING_SIZE, n_th1, 8, 5 );//peak_height, peak_distance, max_num_peaks 

    n_peak_interval_sum =0;                                //波峰间隔平均值
    if (n_npks>=2){
        for (k=1; k<n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k]-an_dx_peak_locs[k -1]);
        n_peak_interval_sum=n_peak_interval_sum/(n_npks-1);
//		printf("n_peak_interval_sum=%d\n", n_peak_interval_sum);
        *pn_heart_rate=(int32_t)(6000/n_peak_interval_sum);//  采样率为100Hz，一分钟采样6000个，6000/平均波峰间隔即为心率值
        *pch_hr_valid  = 1;
    }
    else  {
        *pn_heart_rate = -999;
        *pch_hr_valid  = 0;
    }
    //初始数据波谷位置修正         
    for ( k=0 ; k<n_npks ;k++)
        an_ir_valley_locs[k]=an_dx_peak_locs[k]+HAMMING_SIZE/2; 

    // raw value : RED(=y) and IR(=X)
    // we need to assess DC and AC value of ir and red PPG. 
    for (k=0 ; k<n_ir_buffer_length ; k++ )  {
        an_x[k] =  pun_ir_buffer[k] ; 
        an_y[k] =  pun_red_buffer[k] ; 
    }

    // find precise min near an_ir_valley_locs
	//精确的查找位置减小spo2误差
    n_exact_ir_valley_locs_count =0; 
    for(k=0 ; k<n_npks ;k++){
        un_only_once =1;
        m=an_ir_valley_locs[k];
        n_c_min= 16777216;//2^24;
        if (m+5 <  BUFFER_SIZE-HAMMING_SIZE  && m-5 >0){
            for(i= m-5;i<m+5; i++)
                if (an_x[i]<n_c_min){
                    if (un_only_once >0){
                       un_only_once =0;
                   } 
                   n_c_min= an_x[i] ;
                   an_exact_ir_valley_locs[k]=i;
                }
            if (un_only_once ==0)
                n_exact_ir_valley_locs_count ++ ;
        }
    }
    if (n_exact_ir_valley_locs_count <2 ){
       *pn_spo2 =  -999 ; // do not use SPO2 since signal ratio is out of range
       *pch_spo2_valid  = 0; 
       return;
    }

    // 4 pt MA
    for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
        an_x[k]=( an_x[k]+an_x[k+1]+ an_x[k+2]+ an_x[k+3])/(int32_t)4;
        an_y[k]=( an_y[k]+an_y[k+1]+ an_y[k+2]+ an_y[k+3])/(int32_t)4;
    }

    //using an_exact_ir_valley_locs , find ir-red DC andir-red AC for SPO2 calibration ratio
    //finding AC/DC maximum of raw ir * red between two valley locations
    n_ratio_average =0; 
    n_i_ratio_count =0; 
    
    for(k=0; k< 5; k++) an_ratio[k]=0;
    for (k=0; k< n_exact_ir_valley_locs_count; k++){
        if (an_exact_ir_valley_locs[k] > BUFFER_SIZE ){             
            *pn_spo2 =  -999 ; // do not use SPO2 since valley loc is out of range
            *pch_spo2_valid  = 0; 
            return;
        }
    }
    // find max between two valley locations 
    // and use ratio betwen AC compoent of Ir & Red and DC compoent of Ir & Red for SPO2 

    for (k=0; k< n_exact_ir_valley_locs_count-1; k++){
        n_y_dc_max= -16777216 ; 
        n_x_dc_max= - 16777216; 
        if (an_exact_ir_valley_locs[k+1]-an_exact_ir_valley_locs[k] >10){
            for (i=an_exact_ir_valley_locs[k]; i< an_exact_ir_valley_locs[k+1]; i++){
                if (an_x[i]> n_x_dc_max) {n_x_dc_max =an_x[i];n_x_dc_max_idx =i; }
                if (an_y[i]> n_y_dc_max) {n_y_dc_max =an_y[i];n_y_dc_max_idx=i;}
            }
            n_y_ac= (an_y[an_exact_ir_valley_locs[k+1]] - an_y[an_exact_ir_valley_locs[k] ] )*(n_y_dc_max_idx -an_exact_ir_valley_locs[k]); //red
            n_y_ac=  an_y[an_exact_ir_valley_locs[k]] + n_y_ac/ (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k])  ; 
        
        
            n_y_ac=  an_y[n_y_dc_max_idx] - n_y_ac;    // subracting linear DC compoenents from raw 
            n_x_ac= (an_x[an_exact_ir_valley_locs[k+1]] - an_x[an_exact_ir_valley_locs[k] ] )*(n_x_dc_max_idx -an_exact_ir_valley_locs[k]); // ir
            n_x_ac=  an_x[an_exact_ir_valley_locs[k]] + n_x_ac/ (an_exact_ir_valley_locs[k+1] - an_exact_ir_valley_locs[k]); 
            n_x_ac=  an_x[n_y_dc_max_idx] - n_x_ac;      // subracting linear DC compoenents from raw 
            n_nume=( n_y_ac *n_x_dc_max)>>7 ; //prepare X100 to preserve floating value
            n_denom= ( n_x_ac *n_y_dc_max)>>7;
            if (n_denom>0  && n_i_ratio_count <5 &&  n_nume != 0)
            {   
                an_ratio[n_i_ratio_count]= (n_nume*20)/n_denom ; //formular is ( n_y_ac *n_x_dc_max) / ( n_x_ac *n_y_dc_max) ;  ///*************************n_nume原来是*100************************//
                n_i_ratio_count++;
            }
        }
    }

    maxim_sort_ascend(an_ratio, n_i_ratio_count);
    n_middle_idx= n_i_ratio_count/2;

    if (n_middle_idx >1)
        n_ratio_average =( an_ratio[n_middle_idx-1] +an_ratio[n_middle_idx])/2; // use median
    else
        n_ratio_average = an_ratio[n_middle_idx ];

    if( n_ratio_average>2 && n_ratio_average <184){
        n_spo2_calc= uch_spo2_table[n_ratio_average] ;
        *pn_spo2 = n_spo2_calc ;
        *pch_spo2_valid  = 1;//  float_SPO2 =  -45.060*n_ratio_average* n_ratio_average/10000 + 30.354 *n_ratio_average/100 + 94.845 ;  // for comparison with table
    }
    else{
        *pn_spo2 =  -999 ; // do not use SPO2 since signal ratio is out of range
        *pch_spo2_valid  = 0; 
    }
}


void maxim_find_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_size, int32_t n_min_height, int32_t n_min_distance, int32_t n_max_num)
/**
* \brief        Find peaks
* \par          Details
*               Find at most MAX_NUM peaks above MIN_HEIGHT separated by at least MIN_DISTANCE
*               n_min_height：波峰的最小高度阈值，只有高于此值的波峰才会被记录。
*               n_min_distance：波峰之间的最小距离，低于此距离的波峰会被移除。
*               n_max_num：最多允许记录的波峰数量。
* \retval       None
*/
{
    maxim_peaks_above_min_height( pn_locs, pn_npks, pn_x, n_size, n_min_height );
    maxim_remove_close_peaks( pn_locs, pn_npks, pn_x, n_min_distance );
    *pn_npks = min( *pn_npks, n_max_num );  //限制波峰数量，如果波峰数量超过n_max_num，只保留前n_max_num个波峰
}

void maxim_peaks_above_min_height(int32_t *pn_locs, int32_t *pn_npks, int32_t  *pn_x, int32_t n_size, int32_t n_min_height)
/**
* \brief        Find peaks above n_min_height
* \par          Details
*               Find all peaks above MIN_HEIGHT
*
* \retval       None
*/
{
    int32_t i = 1, n_width;
    *pn_npks = 0;
    
    while (i < n_size-1){
        if (pn_x[i] > n_min_height && pn_x[i] > pn_x[i-1]){            // 遍历信号，找到波峰左边缘
            n_width = 1;
            while (i+n_width < n_size && pn_x[i] == pn_x[i+n_width])    // 处理平顶波峰，如果多个连续点相等，计算平顶宽度
                n_width++;
            if (pn_x[i] > pn_x[i+n_width] && (*pn_npks) < 15 ){         // 找到波峰右边缘
                pn_locs[(*pn_npks)++] = i;        
                // for flat peaks, peak location is left edge
                i += n_width+1;
            }
            else
                i += n_width;
        }
        else
            i++;
    }
}


void maxim_remove_close_peaks(int32_t *pn_locs, int32_t *pn_npks, int32_t *pn_x, int32_t n_min_distance)
/**
* \brief        Remove peaks
* \par          Details
*               Remove peaks separated by less than MIN_DISTANCE
*
* \retval       None
*/
{
    
    int32_t i, j, n_old_npks, n_dist;
    
    /* Order peaks from large to small */
    maxim_sort_indices_descend( pn_x, pn_locs, *pn_npks );

    for ( i = -1; i < *pn_npks; i++ ){
        n_old_npks = *pn_npks;
        *pn_npks = i+1;
        for ( j = i+1; j < n_old_npks; j++ ){
            n_dist =  pn_locs[j] - ( i == -1 ? -1 : pn_locs[i] ); // lag-zero peak of autocorr is at index -1
            if ( n_dist > n_min_distance || n_dist < -n_min_distance )
                pn_locs[(*pn_npks)++] = pn_locs[j];
        }
    }

    // Resort indices longo ascending order
    maxim_sort_ascend( pn_locs, *pn_npks );
}

void maxim_sort_ascend(int32_t *pn_x,int32_t n_size) 
/**
* \brief        Sort array
* \par          Details
*               Sort array in ascending order (insertion sort algorithm)
*
* \retval       None
*/
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_x[i];
        for (j = i; j > 0 && n_temp < pn_x[j-1]; j--)
            pn_x[j] = pn_x[j-1];
        pn_x[j] = n_temp;
    }
}

void maxim_sort_indices_descend(int32_t *pn_x, int32_t *pn_indx, int32_t n_size)
/**
* \brief        Sort indices
* \par          Details
*               Sort indices according to descending order (insertion sort algorithm)
*
* \retval       None
*/ 
{
    int32_t i, j, n_temp;
    for (i = 1; i < n_size; i++) {
        n_temp = pn_indx[i];
        for (j = i; j > 0 && pn_x[n_temp] > pn_x[pn_indx[j-1]]; j--)
            pn_indx[j] = pn_indx[j-1];
        pn_indx[j] = n_temp;
    }
}


float maxim_blood_flow_rate(uint32_t *pun_ir_buffer,  int32_t n_ir_buffer_length)
/*
* \brief        Calculate the blood flow rate	
* \par          Details	
*               By detecting  peaks of PPG cycle and corresponding AC/DC of red/infra-red signal, the ratio for the BFR is computed.
*               Since this algorithm is aiming for Arm M0/M3. formaula for BFR did not achieve the accuracy due to register overflow.
*               Thus, accurate BFR is precalculated and save longo uch_bfr_table[] per each ratio.
*
* \param[in]    *pun_ir_buffer           - IR sensor data buffer
* \param[in]    n_ir_buffer_length      - IR sensor data buffer length
* \param[in]    *pun_red_buffer          - Red sensor data buffer
* \param[out]    *pn_bfr                - Calculated BFR value
* \param[out]    *pch_bfr_valid         - 1 if the calculated BFR value is valid
*
*
* \retval       None
*/
{
	double pwtt;
	uint32_t pwtt_mean;
    double a = -0.72099;
	double b = 251.8364;
	double c = 0.56908;
	double d = 5.39439;
	double Ps, Pd, CO; 
	double BV;
	uint8_t S = 500;

	uint32_t un_ir_mean ,un_only_once ;
	int32_t k ,n_i_ratio_count;
	int32_t i, s, m, n_exact_ir_valley_locs_count ,n_middle_idx;
	int32_t n_th1, n_npks,n_c_min;      
	int32_t an_ir_valley_locs[15] ;
	int32_t an_exact_ir_valley_locs[15] ;
	int32_t an_dx_peak_locs[15] ;
	int32_t n_peak_interval_sum;
	int32_t an_dx_zero_locs[15] ;
	
	int32_t n_y_ac, n_x_ac;
	int32_t n_bfr_calc; 
	int32_t n_y_dc_max, n_x_dc_max; 
	int32_t n_y_dc_max_idx, n_x_dc_max_idx; 
	int32_t an_ratio[5],n_ratio_average; 
	int32_t n_nume,  n_denom ;
	// 平均值,用于去直流    
	un_ir_mean =0; 
	for (k=0 ; k<n_ir_buffer_length ; k++ ) un_ir_mean += pun_ir_buffer[k] ;
	un_ir_mean =un_ir_mean/n_ir_buffer_length ;
	for (k=0 ; k<n_ir_buffer_length ; k++ )  bn_x[k] =  pun_ir_buffer[k] - un_ir_mean ; 
	
	// 4点移动平均,用于滤波
	for(k=0; k< BUFFER_SIZE-MA4_SIZE; k++){
		n_denom= ( bn_x[k]+ bn_x[k+1]+ bn_x[k+2]+ bn_x[k+3]);
		bn_x[k]=  n_denom/(int32_t)4; 
	}

	// 一阶差分
	for( k=0; k<BUFFER_SIZE-MA4_SIZE-1;  k++)
		bn_dx[k]= (bn_x[k+1]- bn_x[k]);

	// 差分后的值做2点移动平均
	for(k=0; k< BUFFER_SIZE-MA4_SIZE-2; k++){
		bn_dx[k] =  ( bn_dx[k]+bn_dx[k+1])/2 ;
	}
	
	// 二阶差分
	for( k=0; k<BUFFER_SIZE-MA4_SIZE-3;  k++)
		bn_dx[k]= (bn_dx[k+1]- bn_dx[k]);

	// 差分后的值做2点移动平均
	for(k=0; k< BUFFER_SIZE-MA4_SIZE-4; k++){
		bn_dx[k] =  ( bn_dx[k]+bn_dx[k+1])/2 ;
	}

	// hamming window 构造窗函数
	// 减少频谱泄露
	for ( i=0 ; i<BUFFER_SIZE-HAMMING_SIZE-MA4_SIZE-2 ;i++){
		s= 0;
		for( k=i; k<i+ HAMMING_SIZE ;k++){
			s -= bn_dx[k] *auw_hamm[k-i] ; 
						}
		bn_dx[i]= s/ (int32_t)1146; // divide by sum of auw_hamm 
	}

	n_th1=0; // 波形提取特征点的阈值
    for ( k=0 ; k<BUFFER_SIZE-HAMMING_SIZE ;k++){
        n_th1 += ((bn_dx[k]>0)? bn_dx[k] : ((int32_t)0-bn_dx[k])) ;
    }
    n_th1= n_th1/ ( BUFFER_SIZE-HAMMING_SIZE);
    // peak location is acutally index for sharpest location of raw signal since we flipped the signal         
    maxim_find_peaks( an_dx_peak_locs, &n_npks, bn_dx, BUFFER_SIZE-HAMMING_SIZE, n_th1, 8, 5 );//peak_height, peak_distance, max_num_peaks 

//  printf("an_dx_peak_locs[0] = %d\n",an_dx_peak_locs[0]);
//	printf("an_dx_peak_locs[1] = %d\n",an_dx_peak_locs[1]);
//	printf("an_dx_peak_locs[2] = %d\n",an_dx_peak_locs[2]);
//	printf("an_dx_peak_locs[3] = %d\n",an_dx_peak_locs[3]);
//	printf("an_dx_peak_locs[4] = %d\n",an_dx_peak_locs[4]);
//	printf("andx[0] = %d\n",an_dx[an_dx_peak_locs[0]]);
//	printf("andx[1] = %d\n",an_dx[an_dx_peak_locs[1]]);
//	printf("andx[2] = %d\n",an_dx[an_dx_peak_locs[2]]);
//	printf("andx[3] = %d\n",an_dx[an_dx_peak_locs[3]]);
//	printf("andx[4] = %d\n",an_dx[an_dx_peak_locs[4]]);

   n_peak_interval_sum =0;                                //波峰间隔平均值
    if (n_npks>=2){
        for (k=1; k<n_npks; k++)
            n_peak_interval_sum += (an_dx_peak_locs[k]-an_dx_peak_locs[k -1]);
        n_peak_interval_sum=n_peak_interval_sum/(n_npks-1);
	}		

	pwtt = n_peak_interval_sum * 0.01 ; //pwtt单位为秒;
//	printf("pwtt = %f\n",pwtt);

// 计算血流速度
	Ps = a*pwtt + b;
	Pd = c*Ps + d;
	CO = 17*(Ps-Pd)*4;
	BV = CO/S;
//	printf("bv = %f\n",BV);
	
	return BV; //返回血流速度

}

