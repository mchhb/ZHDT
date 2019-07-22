#include "ADXL345.h"
#include "delay.h"
void SPI_init(void)
{
 
  GPIO_InitTypeDef GPIO_InitStructure;
  SPI_InitTypeDef    SPI_InitStructure;  
  RCC_APB2PeriphClockCmd(RCC_APB2Periph_AFIO | RCC_APB2Periph_GPIOA |
                         RCC_APB2Periph_GPIOB| RCC_APB2Periph_GPIOC, ENABLE);
    
 
 
  RCC_APB2PeriphClockCmd(ADXL345_SPI_CLK ,ENABLE);
                          
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin =ADXL345_SPI_CS_PIN ;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_Out_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ADXL345_SPI_CS_GPIO_PORT, &GPIO_InitStructure);
 
  GPIO_SetBits(ADXL345_SPI_CS_GPIO_PORT,ADXL345_SPI_CS_PIN);
 
  GPIO_StructInit(&GPIO_InitStructure);
  GPIO_InitStructure.GPIO_Pin = ADXL345_SPI_SCK_PIN|ADXL345_SPI_MISO_PIN|ADXL345_SPI_MOSI_PIN;
  GPIO_InitStructure.GPIO_Mode=GPIO_Mode_AF_PP;
  GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
  GPIO_Init(ADXL345_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure); 

   
   SPI_InitStructure.SPI_Direction=SPI_Direction_2Lines_FullDuplex;
   SPI_InitStructure.SPI_Mode=SPI_Mode_Master;
   SPI_InitStructure.SPI_DataSize = SPI_DataSize_16b;
   SPI_InitStructure.SPI_CPOL=SPI_CPOL_High;
   SPI_InitStructure.SPI_CPHA=SPI_CPHA_2Edge;
   SPI_InitStructure.SPI_NSS=SPI_NSS_Soft;
   SPI_InitStructure.SPI_BaudRatePrescaler=SPI_BaudRatePrescaler_256;
   SPI_InitStructure.SPI_FirstBit=SPI_FirstBit_MSB;
   SPI_InitStructure.SPI_CRCPolynomial=7;
   SPI_Init(ADXL345_SPI, &SPI_InitStructure); 

   SPI_Cmd(ADXL345_SPI,ENABLE);  
}

void ADXL345_init(void)
{
    SPI_init();   
 
   ADXL345_write_byte(0x1E,0x00); 
   ADXL345_write_byte(0x1F,0x00); 
   ADXL345_write_byte(0x20,0x00); 
   ADXL345_write_byte(0x21,0x00);  
   ADXL345_write_byte(0x22,0x00);  
   ADXL345_write_byte(0x23,0x00);  
 
   ADXL345_write_byte(0x24,0x01);  
   ADXL345_write_byte(0x25,0x01);  
   ADXL345_write_byte(0x26,0x2B);  
   ADXL345_write_byte(0x27,0x00);  
   ADXL345_write_byte(0x28,0x09);  
   ADXL345_write_byte(0x29,0xFF);  
   ADXL345_write_byte(0x2A,0x80);  
   //ADXL345_read_byte(0x2B);    
   ADXL345_write_byte(0x2C,0x0F); 
   ADXL345_write_byte(0x2D,0x08); 
   ADXL345_write_byte(0x2E,0x80); 
   ADXL345_write_byte(0x2F,0x00);
   //ADXL345_read_byte(0x30);    
   ADXL345_write_byte(0x31,0X0B); 
   ADXL345_write_byte(0x38,0x00);  
  //ADXL345_read_byte(0x39);    
 
 }
 
 
 uint8_t ADXL345_read_byte(uint8_t add)

{
     GPIO_ResetBits(ADXL345_SPI_CS_GPIO_PORT ,ADXL345_SPI_CS_PIN);
     SPI_I2S_SendData(ADXL345_SPI,(add|0x80)<<8|0x00);
 
    while(SPI_I2S_GetFlagStatus(ADXL345_SPI,SPI_I2S_FLAG_TXE)==RESET);
   
    while(SPI_I2S_GetFlagStatus(ADXL345_SPI, SPI_I2S_FLAG_RXNE)==RESET);
   
    GPIO_SetBits(ADXL345_SPI_CS_GPIO_PORT ,ADXL345_SPI_CS_PIN);
   
    return SPI_I2S_ReceiveData(ADXL345_SPI)&0xff;
 
  }
void ADXL345_write_byte(uint8_t add,uint8_t val)

{
     GPIO_ResetBits(ADXL345_SPI_CS_GPIO_PORT ,ADXL345_SPI_CS_PIN);

     SPI_I2S_SendData(ADXL345_SPI,add<<8|val);
 
    while(SPI_I2S_GetFlagStatus(ADXL345_SPI,SPI_I2S_FLAG_TXE)==RESET);
   
    while(SPI_I2S_GetFlagStatus(ADXL345_SPI, SPI_I2S_FLAG_RXNE)==RESET);
   
    GPIO_SetBits(ADXL345_SPI_CS_GPIO_PORT ,ADXL345_SPI_CS_PIN);
     SPI_I2S_ReceiveData(ADXL345_SPI)&0xff;
 
  }
void ADXL345_ReadXYZ(float *g)
{ 
    uint8_t BUF[6];  
    int16_t temp;
   
    BUF[0] = ADXL345_read_byte(0x32);  
    BUF[1] = ADXL345_read_byte(0x33);
    delay_ms(1);   
    BUF[2] = ADXL345_read_byte(0x34); 
    BUF[3] = ADXL345_read_byte(0x35);
    delay_ms(1);    
    BUF[4] = ADXL345_read_byte(0x36);  
    BUF[5] = ADXL345_read_byte(0x37);   
    delay_ms(1);
   
    temp = (BUF[1] << 8) + BUF[0];
    if(temp < 0)
        temp = -temp;
    g[0] = (float)(temp * 3.9);  
   
    temp = (BUF[3] << 8) + BUF[2];
    if(temp < 0)
        temp = -temp;
    g[1] = (float)(temp * 3.9);  

    temp = (BUF[5] << 8) + BUF[4];
    if(temp < 0)
        temp = -temp;
    g[2] = (float)(temp * 3.9); 
}



