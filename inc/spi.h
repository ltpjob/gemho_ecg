void SPI_Configuration(void);

void SPI_start(void);
/**
  * @brief  д1�ֽ����ݵ�SPI����
  * @param  TxData д�����ߵ�����
  * @retval None
  */
void SPI_WriteByte(uint8_t TxData);
/**
  * @brief  ��SPI���߶�ȡ1�ֽ�����
  * @retval ����������
  */
uint8_t SPI_ReadByte(void);
/**
  * @brief  �����ٽ���
  * @retval None
  */
void SPI_CrisEnter(void);
/**
  * @brief  �˳��ٽ���
  * @retval None
  */
void SPI_CrisExit(void);
 
/**
  * @brief  Ƭѡ�ź�����͵�ƽ
  * @retval None
  */
void SPI_CS_Select(void);
/**
  * @brief  Ƭѡ�ź�����ߵ�ƽ
  * @retval None
  */
void SPI_CS_Deselect(void);