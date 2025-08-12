#include "gy906.h"
#include "bflb_mtimer.h"

static struct bflb_device_s *gpio;
/*******************************************************************************
*函数名称:SMBus_StartBit
*说明:在SMBus上生成START起始条件
*输入:无
*输出:无
*返回:无
*******************************************************************************/
void SMBus_StartBit(void)
{
    SMBus_SDA_OUT();     //sda线输出
    SMBUS_SDA_H();		//释放SDA，将其拉高
    bflb_mtimer_delay_us(5);	    // 等待几微秒
    SMBUS_SCL_H();		// 释放SCL，将其拉高
    bflb_mtimer_delay_us(5);	    //在Stop信号之间生成总线空闲时间
    SMBUS_SDA_L();		// 拉低SDA
    bflb_mtimer_delay_us(5);	    // Hold time after (Repeated) Start
    // 条件：在此周期后，生成第一个时钟。
    //(Thd:sta=4.0us min)在SCK=1时，检测到SDA由1到0表示通信开始（下降沿）
    SMBUS_SCL_L();	    // 拉低SCL
    bflb_mtimer_delay_us(5);	    //等待几微秒
}
 
/*******************************************************************************
*函数名称：SMBus_StopBit
*描述：在SMBus上产生停止条件
*输入：无
*输出：无
*返回值：无
*******************************************************************************/
void SMBus_StopBit(void)
{
    SMBus_SDA_OUT();//sda线输出
	/***先拉低SDA再释放SCL和SDA***/
    SMBUS_SCL_L();		
    bflb_mtimer_delay_us(5);	
    SMBUS_SDA_L();		
    bflb_mtimer_delay_us(5);	
    SMBUS_SCL_H();	
    bflb_mtimer_delay_us(5);	  
    SMBUS_SDA_H();// Set SDA line在SCK=1时，检测到SDA由0到1表示通信结束（上升沿）
}

/*******************************************************************************
*函数名称：SMBus_SendByte
*描述：在SMBus上发送一个字节
*输入：Tx_buffer（要发送的字节）
*输出：无
*返回值：无
*******************************************************************************/
uint8_t SMBus_SendByte(uint8_t Tx_buffer)
{
    uint8_t	Bit_counter;
    uint8_t	Ack_bit;
    uint8_t	bit_out;
 
    for(Bit_counter=8; Bit_counter; Bit_counter--)
    {
        if (Tx_buffer&0x80)//与10000000。就是最高跟1与
        {
            bit_out=1;   // I如果Tx_buffer当前位为1，则将bit_out设置为1，否则将其设置为0。
        }
        else
        {
            bit_out=0;  // 否则将bit_out清零
        }
        SMBus_SendBit(bit_out);		// 发送SDA上的当前位
        Tx_buffer<<=1;				//获取下一个位以进行检查
    }
 
    Ack_bit=SMBus_ReceiveBit();		// 获取确认位
    return	Ack_bit;
}

/*******************************************************************************
函数名  : SMBus_SendBit
描述    : //发送一位
输入    : bit_out（要发送的位）
输出    : 无
返回    : 无
*******************************************************************************/
void SMBus_SendBit(uint8_t bit_out)
{
    SMBus_SDA_OUT();
    if(bit_out==0)
    {
        SMBUS_SDA_L();
    }
    else
    {
        SMBUS_SDA_H();
    }
    bflb_mtimer_delay_us(2);					//数据输入到时钟变化之间的最短延迟时间为250纳秒（了确保在输入数据变化后足够的时间内，系统可以采集和处理这些数据，以确保数据的准确性和稳定性）
    SMBUS_SCL_H();					// 拉高SCL
    bflb_mtimer_delay_us(6);					// 时钟脉冲的高电平
    SMBUS_SCL_L();					// 拉低SCL
    bflb_mtimer_delay_us(3);					//
//	SMBUS_SDA_H();				    // 拉高SDA
    return;
}

/*******************************************************************************
函数名：SMBus_ReceiveByte
描述：在SMBus上接收一个字节(byte)
输入：ack_nack
输出：无
返回值：RX_buffer
该函数用于在SMBus上接收一个字节(byte)。输入参数ack_nack表示是否返回应答位，取值为0或1，0表示不返回应答位，1表示返回应答位。
输出参数为空。返回值为RX_buffer，表示已成功接收一个字节并返回接收到的数据。
*******************************************************************************/
uint8_t SMBus_ReceiveByte(uint8_t ack_nack)
{
    uint8_t RX_buffer;
    uint8_t	Bit_Counter;
 
    for(Bit_Counter=8; Bit_Counter; Bit_Counter--)
    {
        if(SMBus_ReceiveBit())			//从SDA线获取一个位
        {
            RX_buffer <<= 1;			// 如果位为1，则在RX_buffer中保存1。
            RX_buffer |=0x01;
        }
        else
        {
            RX_buffer <<= 1;			// 如果位为0，则在RX_buffer中保存0。
            RX_buffer &=0xfe;
        }
    }
    SMBus_SendBit(ack_nack);			// 发送确认比特。
    return RX_buffer;
}

/*******************************************************************************
*函数名：SMBus_ReceiveBit
*描述：在SMBus上接收一个位(bit)
*输入：无
*输出：无
*返回值：Ack_bit
该函数用于在SMBus上接收一个位(bit)。输入和输出参数都为空，返回值为Ack_bit，表示已成功接收一个位并返回应答位的值。
*******************************************************************************/
uint8_t SMBus_ReceiveBit(void)
{
    uint8_t Ack_bit;
    SMBus_SDA_IN();
 
    SMBUS_SDA_H();          //引脚靠外部电阻上拉，当作输入
	bflb_mtimer_delay_us(2);			// 
    SMBUS_SCL_H();			// 拉高SCL
    bflb_mtimer_delay_us(5);			//时钟脉冲的高电平
    if (SMBUS_SDA_PIN())
    {
        Ack_bit=1;
    }
    else
    {
        Ack_bit=0;
    }
    SMBUS_SCL_L();			// Clear SCL line
    bflb_mtimer_delay_us(3);			// Low Level of Clock Pulse
 
    return	Ack_bit;
}

//GY906引脚输出模式控制
void SMBus_SDA_OUT(void)//SDA输出方向配置
{
	gpio = bflb_device_get_by_name("gpio");		
	bflb_gpio_init(gpio, SMBUS_SDA, GPIO_OUTPUT| GPIO_FLOAT| GPIO_SMT_EN | GPIO_DRV_1);				

}

void SMBus_SDA_IN(void)//SDA输入方向配置
{
	gpio = bflb_device_get_by_name("gpio");		
	bflb_gpio_init(gpio, SMBUS_SDA, GPIO_INPUT| GPIO_PULLUP| GPIO_SMT_EN | GPIO_DRV_1);				
}

/*******************************************************************************
*函数名称  : SMBus_Init
*描述          : SMBus初始化
*输入          : 无
*输出          : 无
*返回值        : 无
*******************************************************************************/
void SMBus_Init()
{
    gpio = bflb_device_get_by_name("gpio");
    printf("gpio output gy906\r\n");
    /* I2C0_SDA */
    bflb_gpio_init(gpio, SMBUS_SDA, GPIO_OUTPUT| GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_1);
    /* I2C0_SCL */
    bflb_gpio_init(gpio, SMBUS_SCL, GPIO_OUTPUT| GPIO_FLOAT | GPIO_SMT_EN | GPIO_DRV_1);
 
    SMBUS_SCL_H();
    SMBUS_SDA_H();
}

/*******************************************************************************
*函数名称  : SMBus_ReadMemory
*描述          : 从RAM/EEPROM读取数据
*输入          : 从器件地址和命令字
*输出          : 无
*返回值        : 数据
*******************************************************************************/
uint16_t SMBus_ReadMemory(uint8_t slaveAddress, uint8_t command)
{
    uint16_t data;			//数据存储（DataH:DataL）
    uint8_t Pec;				// PEC字节存储
    uint8_t DataL=0;			// 低数据字节存储
    uint8_t DataH=0;			// 高数据字节存储。
    uint8_t arr[6];			// 发送字节的缓冲区
    uint8_t PecReg;			// 计算的PEC字节存储
    uint8_t ErrorCounter;	// 定义与MLX90614通信的尝试次数
 
    ErrorCounter=0x00;				// 初始化ErrorCounte
	slaveAddress <<= 1;	//2-7位表示从机地址
	
    do
    {
repeat:
        SMBus_StopBit();			    //如果从设备发送NACK停止通信
        --ErrorCounter;				    //预减ErrorCounter
        if(!ErrorCounter) 			    //ErrorCounter= 0？
        {
            break;					    //是的，退出do-while {}
        }
 
        SMBus_StartBit();				//起始条件
        if(SMBus_SendByte(slaveAddress))//发送SlaveAddress 最低位Wr=0表示接下来写命令
        {
            goto	repeat;			    //再次重复通信
        }
        if(SMBus_SendByte(command))	    //发送命令
        {
            goto	repeat;		    	//再次重复通信
        }
 
        SMBus_StartBit();					//重复起始条件
        if(SMBus_SendByte(slaveAddress+1))	//发送地址最低位Rd=1表示接下来读数据
        {
            goto	repeat;             	//再次重复通信
        }
 
        DataL = SMBus_ReceiveByte(ACK);	//读取低字节数据，主设备必须发送ACK应答
        DataH = SMBus_ReceiveByte(ACK); //取高字节数据，主设备必须发送ACK应答
        Pec = SMBus_ReceiveByte(NACK);	//读取PEC字节，主设备必须发送NACK非应答
        SMBus_StopBit();				//停止条件
        arr[5] = slaveAddress;		//
        arr[4] = command;			//
        arr[3] = slaveAddress+1;	//加载数组arr
        arr[2] = DataL;				//
        arr[1] = DataH;				//
        arr[0] = 0;					//
        PecReg=PEC_Calculation(arr);//计算CRC
    }
    while(PecReg != Pec);		//计算CRC
 
	data = (DataH<<8) | DataL;	//data=DataH:DataL
    return data;
}

/*******************************************************************************
*函数名称：PEC_calculation
*描述：计算接收字节的PEC
*输入：pec []
*输出：无
*返回值：pec [0]-此字节包含计算的crc值
*******************************************************************************/
uint8_t PEC_Calculation(uint8_t pec[])
{
    uint8_t crc[6];
    uint8_t	BitPosition=47;
    uint8_t	shift;
    uint8_t	i;
    uint8_t	j;
    uint8_t	temp;
 
    do
    {
        /*加载模式值 0x000000000107*/
        crc[5]=0;
        crc[4]=0;
        crc[3]=0;
        crc[2]=0;
        crc[1]=0x01;
        crc[0]=0x07;
 
        /*加载模式值0x000000000107*/
        BitPosition=47;
 
        /*将位移位置设为0*/
        shift=0;
 
        /*从MSByte字节5开始寻找传输消息中的第一个“1”。*/
        i=5;
        j=0;
        while((pec[i]&(0x80>>j))==0 && i>0)
        {
            BitPosition--;
            if(j<7)
            {
                j++;
            }
            else
            {
                j=0x00;
                i--;
            }
        }/*终止循环*/
 
        /*获取模式值的位移值*/
        shift=BitPosition-8;
 
        /*位移模式值 */
        while(shift)
        {
            for(i=5; i<0xFF; i--)
            {
                if((crc[i-1]&0x80) && (i>0))
                {
                    temp=1;
                }
                else
                {
                    temp=0;
                }
                crc[i]<<=1;
                crc[i]+=temp;
            }/*终止for循环*/
            shift--;
        }/*终止while循环*/
 
        /*PEC和CRC之间的异或运算*/
        for(i=0; i<=5; i++)
        {
            pec[i] ^=crc[i];
        }/*终止for循环*/
    }
    while(BitPosition>8); /*终止do-while*/
 
    return pec[0];
}

/*******************************************************************************
*函数名称：SMBus_ReadTemp
*描述：计算并返回温度值
*输入：无
*输出：无
*返回值：SMBus_ReadMemory(0x00, 0x07)*0.02-273.15
*******************************************************************************/
float SMBus_ReadTemp(void)
{   
	float temp;
	temp = SMBus_ReadMemory(SA, RAM_ACCESS|RAM_TOBJ1)*0.02-273.15;
	return temp;
}
 
/*********************************END OF FILE*********************************/


