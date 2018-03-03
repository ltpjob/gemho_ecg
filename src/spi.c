#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include "spi.h"

#define SPIx            SPI2
#define SPIGPIOGROUP    GPIOB
#define SPICS           GPIO_Pin_12

void SPI_Configuration(void)
{
    SPI_InitTypeDef SPI_InitStruct;

    SPI_InitStruct.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStruct.SPI_Direction= SPI_Direction_2Lines_FullDuplex;
    SPI_InitStruct.SPI_Mode = SPI_Mode_Master;
    SPI_InitStruct.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStruct.SPI_CPOL = SPI_CPOL_Low;
    SPI_InitStruct.SPI_CPHA = SPI_CPHA_1Edge;
    SPI_InitStruct.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStruct.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_InitStruct.SPI_CRCPolynomial = 7;
    
    SPI_Init(SPIx, &SPI_InitStruct);
}

void SPI_start()
{
    SPI_SSOutputCmd(SPIx, ENABLE);
    SPI_Cmd(SPIx, ENABLE);
}
/**
  * @brief  写1字节数据到SPI总线
  * @param  TxData 写到总线的数据
  * @retval None
  */
void SPI_WriteByte(uint8_t TxData)
{                
    while((SPIx->SR&SPI_I2S_FLAG_TXE)==0);   //等待发送区空          
    SPIx->DR=TxData;                                         //发送一个byte 
    while((SPIx->SR&SPI_I2S_FLAG_RXNE)==0); //等待接收完一个byte  
    SPIx->DR;        
}
/**
  * @brief  从SPI总线读取1字节数据
  * @retval 读到的数据
  */
uint8_t SPI_ReadByte(void)
{            
    while((SPIx->SR&SPI_I2S_FLAG_TXE)==0);   //等待发送区空              
    SPIx->DR=0xFF;                                               //发送一个空数据产生输入数据的时钟 
    while((SPIx->SR&SPI_I2S_FLAG_RXNE)==0); //等待接收完一个byte  
    return SPIx->DR;                             
}
/**
  * @brief  进入临界区
  * @retval None
  */
void SPI_CrisEnter(void)
{
//    __set_PRIMASK(1);
}
/**
  * @brief  退出临界区
  * @retval None
  */
void SPI_CrisExit(void)
{
//    __set_PRIMASK(0);
}
 
/**
  * @brief  片选信号输出低电平
  * @retval None
  */
void SPI_CS_Select(void)
{
    GPIO_ResetBits(SPIGPIOGROUP, SPICS);
}
/**
  * @brief  片选信号输出高电平
  * @retval None
  */
void SPI_CS_Deselect(void)
{
    GPIO_SetBits(SPIGPIOGROUP, SPICS);
}
/*********************************END OF FILE**********************************/