#ifndef __ADXL345__H
#define __ADXL345__H

#include "stm32f10x.h"

#define ADXL345_FLAG_TIMEOUT             ((uint32_t)0x1000)


#define ADXL345_SPI                       SPI1
#define ADXL345_SPI_CLK                   RCC_APB2Periph_SPI1

#define ADXL345_SPI_SCK_PIN               GPIO_Pin_5                 
#define ADXL345_SPI_SCK_GPIO_PORT         GPIOA                      
#define ADXL345_SPI_SCK_GPIO_CLK          RCC_APB2Periph_GPIOA
#define ADXL345_SPI_SCK_SOURCE            GPIO_PinSource5
#define ADXL345_SPI_SCK_AF                GPIO_AF_5

#define ADXL345_SPI_MISO_PIN              GPIO_Pin_6                 
#define ADXL345_SPI_MISO_GPIO_PORT        GPIOA                      
#define ADXL345_SPI_MISO_GPIO_CLK         RCC_APB2Periph_GPIOA
#define ADXL345_SPI_MISO_SOURCE           GPIO_PinSource6
#define ADXL345_SPI_MISO_AF               GPIO_AF_5

#define ADXL345_SPI_MOSI_PIN              GPIO_Pin_7                 
#define ADXL345_SPI_MOSI_GPIO_PORT        GPIOA                      
#define ADXL345_SPI_MOSI_GPIO_CLK         RCC_APB2Periph_GPIOA
#define ADXL345_SPI_MOSI_SOURCE           GPIO_PinSource7
#define ADXL345_SPI_MOSI_AF               GPIO_AF_5

#define ADXL345_SPI_CS_PIN                GPIO_Pin_4                 
#define ADXL345_SPI_CS_GPIO_PORT          GPIOC                      
#define ADXL345_SPI_CS_GPIO_CLK           RCC_APB2Periph_GPIOC




#define ADXL345_WHO_AM_I_ADDR          0x0F 
#define ADXL345_CTRL_REG1_ADDR         0x20 
#define ADXL345_CTRL_REG2_ADDR         0x21 
#define ADXL345_CTRL_REG3_ADDR         0x22 
#define ADXL345_CTRL_REG4_ADDR         0x23 
#define ADXL345_CTRL_REG5_ADDR         0x24 
#define ADXL345_REFERENCE_REG_ADDR     0x25 
#define ADXL345_OUT_TEMP_ADDR          0x26 
#define ADXL345_STATUS_REG_ADDR        0x27 
#define ADXL345_OUT_X_L_ADDR           0x28 
#define ADXL345_OUT_X_H_ADDR           0x29 
#define ADXL345_OUT_Y_L_ADDR           0x2A 
#define ADXL345_OUT_Y_H_ADDR           0x2B 
#define ADXL345_OUT_Z_L_ADDR           0x2C 
#define ADXL345_OUT_Z_H_ADDR           0x2D 
#define ADXL345_FIFO_CTRL_REG_ADDR     0x2E 
#define ADXL345_FIFO_SRC_REG_ADDR      0x2F 

#define ADXL345_INT1_CFG_ADDR          0x30 
#define ADXL345_INT1_SRC_ADDR          0x31 
#define ADXL345_INT1_TSH_XH_ADDR       0x32 
#define ADXL345_INT1_TSH_XL_ADDR       0x33 
#define ADXL345_INT1_TSH_YH_ADDR       0x34 
#define ADXL345_INT1_TSH_YL_ADDR       0x35 
#define ADXL345_INT1_TSH_ZH_ADDR       0x36 
#define ADXL345_INT1_TSH_ZL_ADDR       0x37 
#define ADXL345_INT1_DURATION_ADDR     0x38 

 

#define I_AM_ADXL345      ((uint8_t)0xD4)


#define ADXL345_MODE_POWERDOWN       ((uint8_t)0x00)
#define ADXL345_MODE_ACTIVE          ((uint8_t)0x08)


#define ADXL345_OUTPUT_DATARATE_1    ((uint8_t)0x00)
#define ADXL345_OUTPUT_DATARATE_2    ((uint8_t)0x40)
#define ADXL345_OUTPUT_DATARATE_3    ((uint8_t)0x80)
#define ADXL345_OUTPUT_DATARATE_4    ((uint8_t)0xC0)

#define ADXL345_X_ENABLE            ((uint8_t)0x02)
#define ADXL345_Y_ENABLE            ((uint8_t)0x01)
#define ADXL345_Z_ENABLE            ((uint8_t)0x04)
#define ADXL345_AXES_ENABLE         ((uint8_t)0x07)
#define ADXL345_AXES_DISABLE        ((uint8_t)0x00)


#define ADXL345_BANDWIDTH_1         ((uint8_t)0x00)
#define ADXL345_BANDWIDTH_2         ((uint8_t)0x10)
#define ADXL345_BANDWIDTH_3         ((uint8_t)0x20)
#define ADXL345_BANDWIDTH_4         ((uint8_t)0x30)


#define ADXL345_FULLSCALE_250               ((uint8_t)0x00)
#define ADXL345_FULLSCALE_500               ((uint8_t)0x10)
#define ADXL345_FULLSCALE_2000              ((uint8_t)0x20)

 
 
#define ADXL345_BlockDataUpdate_Continous   ((uint8_t)0x00)
#define ADXL345_BlockDataUpdate_Single      ((uint8_t)0x80)

 
 
#define ADXL345_BLE_LSB                     ((uint8_t)0x00)
#define ADXL345_BLE_MSB                    ((uint8_t)0x40)

 
  
#define ADXL345_HIGHPASSFILTER_DISABLE      ((uint8_t)0x00)
#define ADXL345_HIGHPASSFILTER_ENABLE      ((uint8_t)0x10)

  
#define ADXL345_INT1INTERRUPT_DISABLE       ((uint8_t)0x00)
#define ADXL345_INT1INTERRUPT_ENABLE    ((uint8_t)0x80)

  
#define ADXL345_INT2INTERRUPT_DISABLE       ((uint8_t)0x00)
#define ADXL345_INT2INTERRUPT_ENABLE    ((uint8_t)0x08)

  
#define ADXL345_INT1INTERRUPT_LOW_EDGE      ((uint8_t)0x20)
#define ADXL345_INT1INTERRUPT_HIGH_EDGE     ((uint8_t)0x00)

 

#define ADXL345_BOOT_NORMALMODE             ((uint8_t)0x00)
#define ADXL345_BOOT_REBOOTMEMORY           ((uint8_t)0x80)
 
 
  
#define ADXL345_HPM_NORMAL_MODE_RES         ((uint8_t)0x00)
#define ADXL345_HPM_REF_SIGNAL              ((uint8_t)0x10)
#define ADXL345_HPM_NORMAL_MODE             ((uint8_t)0x20)
#define ADXL345_HPM_AUTORESET_INT           ((uint8_t)0x30)

  
#define ADXL345_HPFCF_0              0x00
#define ADXL345_HPFCF_1              0x01
#define ADXL345_HPFCF_2              0x02
#define ADXL345_HPFCF_3              0x03
#define ADXL345_HPFCF_4              0x04
#define ADXL345_HPFCF_5              0x05
#define ADXL345_HPFCF_6              0x06
#define ADXL345_HPFCF_7              0x07
#define ADXL345_HPFCF_8              0x08
#define ADXL345_HPFCF_9              0x09



#define ADXL345_CS_LOW()       GPIO_ResetBits(ADXL345_SPI_CS_GPIO_PORT, ADXL345_SPI_CS_PIN)
#define ADXL345_CS_HIGH()      GPIO_SetBits(ADXL345_SPI_CS_GPIO_PORT, ADXL345_SPI_CS_PIN)

 
void ADXL345_init(void);
void ADXL345_write_byte(uint8_t add,uint8_t val);
void ADXL345_ReadXYZ(float *g);
uint8_t ADXL345_read_byte(uint8_t add);

#endif

