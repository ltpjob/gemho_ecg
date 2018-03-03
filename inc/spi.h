void SPI_Configuration(void);

void SPI_start(void);
/**
  * @brief  写1字节数据到SPI总线
  * @param  TxData 写到总线的数据
  * @retval None
  */
void SPI_WriteByte(uint8_t TxData);
/**
  * @brief  从SPI总线读取1字节数据
  * @retval 读到的数据
  */
uint8_t SPI_ReadByte(void);
/**
  * @brief  进入临界区
  * @retval None
  */
void SPI_CrisEnter(void);
/**
  * @brief  退出临界区
  * @retval None
  */
void SPI_CrisExit(void);
 
/**
  * @brief  片选信号输出低电平
  * @retval None
  */
void SPI_CS_Select(void);
/**
  * @brief  片选信号输出高电平
  * @retval None
  */
void SPI_CS_Deselect(void);