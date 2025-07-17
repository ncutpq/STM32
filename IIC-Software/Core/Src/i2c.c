/*
 * @Author: dcr
 * @Date: 2025-06-03 20:40:13
 * @Description: 软件模拟实现I2C操作
 */

#include "i2c.h"

/**
 * @description: 以阻塞的方式延时微秒
 * @param {uint32_t} t 延迟时间
 * @return {*}
 */
void delay_us(uint32_t t) {
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000000);
    HAL_Delay(t - 1);
    HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);
}


//初始化IIC
void IIC_Init(void)
{					     
	// 初始化GPIOC (已在main函数中完成)
	//MX_GPIO_Init();
	
	// 总线空闲状态下输出高电平
	I2C_SDA_WritePin(1);
  I2C_SCL_WritePin(1);
}

/**
 * @description: I2C起始信号
 * @param {*}
 * @return {*}
 */
void IIC_Start(void)
{
	SDA_OUT();     //SDA线输出
	
	I2C_SDA_WritePin(1);
  I2C_SCL_WritePin(1);
	delay_us(4);

	I2C_SDA_WritePin(0);//START:when CLK is high,DATA change form high to low
	delay_us(4);
	I2C_SCL_WritePin(0);//钳住I2C总线，准备发送或接收数据 
}	  

/**
 * @description: I2C停止信号
 * @param {*}
 * @return {*}
 */
void IIC_Stop(void)
{
	SDA_OUT();//SDA线输出

	I2C_SCL_WritePin(0);
  I2C_SDA_WritePin(0);//STOP:when CLK is high， DATA change form low to high
 	delay_us(4);
	I2C_SCL_WritePin(1);
  I2C_SDA_WritePin(1);//发送I2C总线结束信号
	delay_us(4);							   	
}

/**
 * @description: 等待应答信号到来
 * @param {*}
 * @return 0：接收应答成功；
					 1：接收应答失败；
 */
u8 IIC_Wait_Ack(void)
{
	u8 ucErrTime=0;
	SDA_IN();      //SDA设置为输入  

	I2C_SDA_WritePin(1);
	delay_us(1);	   

	I2C_SCL_WritePin(1);
	delay_us(1);	 
	while(I2C_SDA_ReadPin())
	{
		ucErrTime++;
		if(ucErrTime>250)
		{
			IIC_Stop();
			return 1;
		}
	} 
	I2C_SCL_WritePin(0);  //时钟输出0 
	return 0;  
} 


/**
 * @description: 产生ACK应答
 * @param {*}
 * @return {*}
 */
void IIC_Ack(void)
{
	I2C_SCL_WritePin(0);
	SDA_OUT();
	I2C_SDA_WritePin(0);
	delay_us(2);
	I2C_SCL_WritePin(1);
	delay_us(2);
	I2C_SCL_WritePin(0);	
}


/**
 * @description: 不产生ACK应答
 * @param {*}
 * @return {*}
 */
void IIC_NAck(void)
{
	I2C_SCL_WritePin(0);
	SDA_OUT();
	I2C_SDA_WritePin(1);
	delay_us(2);
	I2C_SCL_WritePin(1);
	delay_us(2);
	I2C_SCL_WritePin(0);
}					 				     

/**
 * @description: I2C发送一个字节
 * @param txd:待发送的字节数据
 * @return {*}
 */
void IIC_Send_Byte(u8 txd)
{                        
  u8 t;   
	SDA_OUT(); 	    
	I2C_SCL_WritePin(0);//拉低时钟开始数据传输
	for(t=0;t<8;t++)
	{              
		I2C_SDA_WritePin((txd&0x80)>>7);  // MSB先行
		txd<<=1; 	  
		delay_us(2);   //对TEA5767这三个延时都是必须的
		I2C_SCL_WritePin(1);
		delay_us(2); 
		I2C_SCL_WritePin(0);		
		delay_us(2);
	}	 
} 	    

/**
 * @description: I2C读取1个字节，然后发送相应的ACK、nACK信号
 * @param  ack:待发送的ack信号
 * @return receive：读取成功的字符数量
 */
u8 IIC_Read_Byte(unsigned char ack)
{
	unsigned char i,receive=0;
	SDA_IN();//SDA设置为输入
  for(i=0;i<8;i++ )
	{
		I2C_SCL_WritePin(0);
    delay_us(2);
		I2C_SCL_WritePin(1);
		receive<<=1;
		if(I2C_SDA_ReadPin())receive++;   
		delay_us(1); 
  }					 
	if (!ack)
		IIC_NAck();//发送nACK
	else
		IIC_Ack(); //发送ACK   
	return receive;
}

