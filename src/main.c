#include "stm32f10x_conf.h"
#include "stm32f10x.h"
#include <stdio.h>
#include "spi.h"
#include "wizchip_conf.h"
#include "socket.h"

#define _NETSERVICE_DEBUG_

#define RXBUFFERSIZE 256
__IO uint16_t RXBuffer0[RXBUFFERSIZE];
__IO uint16_t RXBuffer1[RXBUFFERSIZE];
__IO uint8_t g_using_buf0 = 1;
__IO uint8_t g_recv_flag = 0;

wiz_NetInfo WIZNETINFO = {.mac = {0x00, 0x08, 0xdc,0x00, 0xab, 0x99},
                          .ip = {192, 168, 88, 6},
                          .sn = {255,255,255,0},
                          .gw = {192, 168, 88, 1},
                          .dns = {0,0,0,0},
                          .dhcp = NETINFO_STATIC };

void RCC_Configuration(void)
{

  RCC_ADCCLKConfig(RCC_PCLK2_Div8); 
  /* Enable peripheral clocks ------------------------------------------------*/
  /* Enable DMA1 clock */
  RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA1, ENABLE);

  /* Enable ADC1 and GPIOC clock */
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_ADC1 | RCC_APB2Periph_GPIOA | RCC_APB2Periph_GPIOB, ENABLE);
  RCC_APB1PeriphClockCmd(RCC_APB1Periph_SPI2 | RCC_APB1Periph_I2C2, ENABLE);
  
}

void W5500_config()
{
  SPI_Configuration();
  
  // First of all, Should register SPI callback functions implemented by user for accessing WIZCHIP 
  /* Critical section callback */
  reg_wizchip_cris_cbfunc(SPI_CrisEnter, SPI_CrisExit);   //×¢²áÁÙ½çÇøº¯Êý
  /* Chip selection call back */
#if   _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_VDM_
  reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);//×¢²áSPIÆ¬Ñ¡ÐÅºÅº¯Êý
#elif _WIZCHIP_IO_MODE_ == _WIZCHIP_IO_MODE_SPI_FDM_
  reg_wizchip_cs_cbfunc(SPI_CS_Select, SPI_CS_Deselect);  // CS must be tried with LOW.
#else
#if (_WIZCHIP_IO_MODE_ & _WIZCHIP_IO_MODE_SIP_) != _WIZCHIP_IO_MODE_SIP_
#error "Unknown _WIZCHIP_IO_MODE_"
#else
  reg_wizchip_cs_cbfunc(wizchip_select, wizchip_deselect);
#endif
#endif
  /* SPI Read & Write callback function */
  reg_wizchip_spi_cbfunc(SPI_ReadByte, SPI_WriteByte);    //×¢²á¶ÁÐ´º¯Êý
  
  SPI_start();
}

void GPIO_Configuration(void)
{
  GPIO_InitTypeDef GPIO_InitStructure;

  /* Configure PC.04 (ADC Channel14) as analog input -------------------------*/
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_1;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AIN;
  GPIO_Init(GPIOA, &GPIO_InitStructure);
  
  //spi2
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_13 | GPIO_Pin_15;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  /* Confugure MISO pin as Input Floating  */
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_14;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IN_FLOATING;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  
  GPIO_InitStructure.GPIO_Pin = GPIO_Pin_12;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
  GPIO_Init(GPIOB, &GPIO_InitStructure);
  GPIO_SetBits(GPIOB, GPIO_Pin_12);
}

void network_init(wiz_NetInfo *winfo)
{
#ifdef _NETSERVICE_DEBUG_
  uint8_t tmpstr[6];
#endif
  ctlnetwork(CN_SET_NETINFO, winfo);
  ctlnetwork(CN_GET_NETINFO, winfo);
  
  // Display Network Information
  
#ifdef _NETSERVICE_DEBUG_
  ctlwizchip(CW_GET_ID,(void*)tmpstr);
  printf("\r\n=== %s NET CONF ===\r\n",(char*)tmpstr);
  printf("MAC: %02X:%02X:%02X:%02X:%02X:%02X\r\n",winfo->mac[0],winfo->mac[1],winfo->mac[2],
         winfo->mac[3],winfo->mac[4],winfo->mac[5]);
  printf("SIP: %d.%d.%d.%d\r\n", winfo->ip[0],winfo->ip[1],winfo->ip[2],winfo->ip[3]);
  printf("GAR: %d.%d.%d.%d\r\n", winfo->gw[0],winfo->gw[1],winfo->gw[2],winfo->gw[3]);
  printf("SUB: %d.%d.%d.%d\r\n", winfo->sn[0],winfo->sn[1],winfo->sn[2],winfo->sn[3]);
  printf("DNS: %d.%d.%d.%d\r\n", winfo->dns[0],winfo->dns[1],winfo->dns[2],winfo->dns[3]);
  uint32_t cpuid[3];
  cpuid[0]=*(vu32*)(0x1ffff7e8);
  cpuid[1]=*(vu32*)(0x1ffff7ec);
  cpuid[2]=*(vu32*)(0x1ffff7f0);
  printf("CPUID: %x %x %x\r\n", cpuid[0], cpuid[1], cpuid[2]);
  printf("======================\r\n");
#endif
}

int32_t net_service()
{
  int32_t ret;
  uint16_t size = 0, sentsize=0;
  uint8_t sn = 0;
  uint16_t port = 5566;
  uint8_t *pData = NULL;

  
#ifdef _NETSERVICE_DEBUG_
  uint8_t destip[4];
  uint16_t destport;
#endif
  
  switch(getSn_SR(sn))
  {
  case SOCK_ESTABLISHED :
    if(getSn_IR(sn) & Sn_IR_CON)
    {
#ifdef _NETSERVICE_DEBUG_
      getSn_DIPR(sn, destip);
      destport = getSn_DPORT(sn);
      
      printf("%d:Connected - %d.%d.%d.%d : %d\r\n",sn, destip[0], destip[1], destip[2], destip[3], destport);
#endif
      setSn_IR(sn, Sn_IR_CON);
    }
    
    if(g_recv_flag>1)
    {
#ifdef _NETSERVICE_DEBUG_
      printf("bad recv value!!!  %d\n", g_recv_flag);
#endif
      g_recv_flag=0;
    }
    else if(g_recv_flag == 1)
    {      
      size = RXBUFFERSIZE*2;
      sentsize = 0;
      if(g_using_buf0 == 0)
        pData = (uint8_t *)RXBuffer0;
      else
        pData = (uint8_t *)RXBuffer1;
      while(size != sentsize)
      {
        ret = send(sn, pData+sentsize, size-sentsize);
        if(ret < 0)
        {
          close(sn);
          return ret;
        }
        sentsize += ret;
      }
      
      if(g_recv_flag>1)
      {
#ifdef _NETSERVICE_DEBUG_
        printf("bug happened\n");
#endif
      }
      
      g_recv_flag--;
    }
    
    break;
  case SOCK_CLOSE_WAIT :
#ifdef _NETSERVICE_DEBUG_
    printf("%d:CloseWait\r\n",sn);
#endif
    if((ret = disconnect(sn)) != SOCK_OK) return ret;
#ifdef _NETSERVICE_DEBUG_
    printf("%d:Socket Closed\r\n", sn);
#endif
    break;
  case SOCK_INIT :
#ifdef _NETSERVICE_DEBUG_
    printf("%d:Listen, TCP server loopback, port [%d]\r\n", sn, port);
#endif
    if( (ret = listen(sn)) != SOCK_OK) return ret;
    break;
  case SOCK_CLOSED:
#ifdef _NETSERVICE_DEBUG_
    printf("%d:TCP server loopback start\r\n",sn);
#endif
    if((ret = socket(sn, Sn_MR_TCP, port, 0x00)) != sn) return ret;
#ifdef _NETSERVICE_DEBUG_
    printf("%d:Socket opened\r\n",sn);
#endif
    break;
  default:
    break;
  }
  return 1;
}

int main(void)
{
  ADC_InitTypeDef ADC_InitStructure;
  DMA_InitTypeDef DMA_InitStructure;
  NVIC_InitTypeDef NVIC_InitStructure;

  SystemInit();
  
  RCC_Configuration();

  /* GPIO configuration ------------------------------------------------------*/
  GPIO_Configuration();
  
  NVIC_PriorityGroupConfig(NVIC_PriorityGroup_3);
  NVIC_InitStructure.NVIC_IRQChannel = DMA1_Channel1_IRQn;   
  NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 2; 
  NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
  NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
  NVIC_Init(&NVIC_InitStructure);

  /* DMA1 channel1 configuration ----------------------------------------------*/
  DMA_DeInit(DMA1_Channel1);
  DMA_Cmd(DMA1_Channel1, DISABLE); 
  DMA_InitStructure.DMA_PeripheralBaseAddr = (uint32_t)&(ADC1->DR);;
  DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)RXBuffer0;
  DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
  DMA_InitStructure.DMA_BufferSize = RXBUFFERSIZE;
  DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
  DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
  DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_HalfWord;
  DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_HalfWord;
  DMA_InitStructure.DMA_Mode = DMA_Mode_Circular;
  DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
  DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
  DMA_Init(DMA1_Channel1, &DMA_InitStructure);
  
  DMA_ClearFlag(DMA1_FLAG_TC1);                                 
  DMA_ITConfig(DMA1_Channel1, DMA_IT_TC, ENABLE);      
  
  /* Enable DMA1 channel1 */
  DMA_Cmd(DMA1_Channel1, ENABLE);
  
  /* ADC1 configuration ------------------------------------------------------*/
  ADC_InitStructure.ADC_Mode = ADC_Mode_Independent;
  ADC_InitStructure.ADC_ScanConvMode = ENABLE;
  ADC_InitStructure.ADC_ContinuousConvMode = ENABLE;
  ADC_InitStructure.ADC_ExternalTrigConv = ADC_ExternalTrigConv_None;
  ADC_InitStructure.ADC_DataAlign = ADC_DataAlign_Right;
  ADC_InitStructure.ADC_NbrOfChannel = 1;
  ADC_Init(ADC1, &ADC_InitStructure);

  /* ADC1 regular channel14 configuration */ 
  ADC_RegularChannelConfig(ADC1, ADC_Channel_1, 1, ADC_SampleTime_239Cycles5);

  /* Enable ADC1 DMA */
  ADC_DMACmd(ADC1, ENABLE);
  
  /* Enable ADC1 */
  ADC_Cmd(ADC1, ENABLE);

  /* Enable ADC1 reset calibration register */   
  ADC_ResetCalibration(ADC1);
  /* Check the end of ADC1 reset calibration register */
  while(ADC_GetResetCalibrationStatus(ADC1));

  /* Start ADC1 calibration */
  ADC_StartCalibration(ADC1);
  /* Check the end of ADC1 calibration */
  while(ADC_GetCalibrationStatus(ADC1));
     
  /* Start ADC1 Software Conversion */ 
  ADC_SoftwareStartConvCmd(ADC1, ENABLE);
  
  
  W5500_config();
  wizchip_sw_reset();
  network_init(&WIZNETINFO);
  
  while (1)
  {
    net_service();
  }
}

void DMA1_Channel1_IRQHandler(void)
{
  if(DMA_GetITStatus(DMA1_FLAG_TC1))
  {
    if(g_using_buf0 ==0)
    {
      DMA1_Channel1->CMAR = (uint32_t)RXBuffer0;
      g_using_buf0 = 1;
    }
    else
    {
      DMA1_Channel1->CMAR = (uint32_t)RXBuffer1;
      g_using_buf0 = 0;
    }
    
    g_recv_flag++;
    
    DMA_ClearITPendingBit(DMA1_IT_TC1);
  }
}

