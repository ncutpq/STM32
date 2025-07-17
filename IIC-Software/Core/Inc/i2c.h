/*
 * @Author: dcr
 * @Date: 2025-06-03 20:40:13
 * @Description: 软件模拟实现I2C操作
 */


#ifndef __SOFTWARE_I2C_H
#define __SOFTWARE_I2C_H

//修改为所用型号
#include "stm32f1xx_hal.h"
#include "stm32f1xx_hal_gpio.h"

typedef uint32_t  u32;
typedef uint16_t u16;
typedef uint8_t  u8;

#define I2C_GPIO_PORT GPIOC
#define I2C_SCL_PIN GPIO_PIN_12  // SCL引脚
#define I2C_SDA_PIN GPIO_PIN_11  // SDA引脚


//IO方向设置
//#define SDA_IN()  {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=8<<12;} // 上下拉输入模式
//#define SDA_OUT() {GPIOC->CRH&=0XFFFF0FFF;GPIOC->CRH|=3<<12;} // 推挽输出模式
#define SDA_IN()  {I2C_GPIO_PORT->CRH&=0XFFFF0FFF;I2C_GPIO_PORT->CRH|=8<<12;} // 上下拉输入模式
#define SDA_OUT() {I2C_GPIO_PORT->CRH&=0XFFFF0FFF;I2C_GPIO_PORT->CRH|=3<<12;} // 推挽输出模式

//IO操作函数	 
//SCL 输出
#define I2C_SCL_WritePin(_x)                                        \
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SCL_PIN, \
                      ((_x) ? GPIO_PIN_SET : GPIO_PIN_RESET))
//SDA	输出
#define I2C_SDA_WritePin(_x)                                        \
    HAL_GPIO_WritePin(I2C_GPIO_PORT, I2C_SDA_PIN, \
                      ((_x) ? GPIO_PIN_SET : GPIO_PIN_RESET))
//SDA 输入
#define I2C_SDA_ReadPin() \
    HAL_GPIO_ReadPin(I2C_GPIO_PORT, I2C_SDA_PIN)

void delay_us(uint32_t t);

//IIC所有操作函数
void IIC_Init(void);                //初始化IIC的IO口				 
void IIC_Start(void);				//发送IIC开始信号
void IIC_Stop(void);	  			//发送IIC停止信号
void IIC_Send_Byte(u8 txd);			//IIC发送一个字节
u8 IIC_Read_Byte(unsigned char ack);//IIC读取一个字节
u8 IIC_Wait_Ack(void); 				//IIC等待ACK信号
void IIC_Ack(void);					//IIC发送ACK信号
void IIC_NAck(void);				//IIC不发送ACK信号

#endif

