/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.h
* Author             : MCD Application Team
* Version            : V3.3.0
* Date               : 21-March-2011
* Description        : Hardware Configuration & Setup
********************************************************************************
* THE PRESENT FIRMWARE WHICH IS FOR GUIDANCE ONLY AIMS AT PROVIDING CUSTOMERS
* WITH CODING INFORMATION REGARDING THEIR PRODUCTS IN ORDER FOR THEM TO SAVE TIME.
* AS A RESULT, STMICROELECTRONICS SHALL NOT BE HELD LIABLE FOR ANY DIRECT,
* INDIRECT OR CONSEQUENTIAL DAMAGES WITH RESPECT TO ANY CLAIMS ARISING FROM THE
* CONTENT OF SUCH FIRMWARE AND/OR THE USE MADE BY CUSTOMERS OF THE CODING
* INFORMATION CONTAINED HEREIN IN CONNECTION WITH THEIR PRODUCTS.
*******************************************************************************/

/* Define to prevent recursive inclusion -------------------------------------*/
#ifndef __HW_CONFIG_H
#define __HW_CONFIG_H

/* Includes ------------------------------------------------------------------*/
#include "usb_type.h"
#include "display.h"

//#include "TscHal.h"
//#include "LcdHal.h"
/* Exported types ------------------------------------------------------------*/
/* Exported constants --------------------------------------------------------*/
/* Exported macro ------------------------------------------------------------*/
/* Exported define -----------------------------------------------------------*/
#define MASS_MEMORY_START     0x04002000
#define BULK_MAX_PACKET_SIZE  0x00000040
#define LED_ON                0xF0
#define LED_OFF               0xFF

#define USART_RX_DATA_SIZE   2048
/* Exported functions ------------------------------------------------------- */
void Set_System();
void Set_USBClock();
void Enter_LowPowerMode();
void Leave_LowPowerMode();
void USB_Interrupts_Config();
void USB_Cable_Config(FunctionalState NewState);
void USART_Config_Default();
bool USART_Config();
void Handle_USBAsynchXfer();
void Get_SerialNum();

void HWConfig_SetHardwareParams();

/* External variables --------------------------------------------------------*/

#define TSC_I2C_DEVICE_REGISTER     0x82

/* Touchscreen Controller DEFINES for STM3210C-EVAL */
#ifdef USE_STM3210C_EVAL
#define TSC_I2C_PORT              I2C1
#define TSC_I2C_CLK               RCC_APB1Periph_I2C1
#define TSC_I2C_SDA_GPIO_PIN      GPIO_Pin_6
#define TSC_I2C_SDA_GPIO_PORT     GPIOB
#define TSC_I2C_SDA_GPIO_CLK      RCC_APB2Periph_GPIOB
#define TSC_I2C_SDA_SOURCE        GPIO_PinSource6
#define TSC_I2C_SDA_AF            GPIO_Remap_I2C1
#define TSC_I2C_SCL_GPIO_PIN      GPIO_Pin_7
#define TSC_I2C_SCL_GPIO_PORT     GPIOB
#define TSC_I2C_SCL_GPIO_CLK      RCC_APB2Periph_GPIOB
#define TSC_I2C_SCL_SOURCE        GPIO_PinSource7
#define TSC_I2C_SCL_AF            GPIO_Remap_I2C1
#define TSC_GPIO_PIN_SOURCE       GPIO_PinSource14
#define TSC_IT_EXTI_PIN_SOURCE    GPIO_PinSource14
#define TSC_GPIO_PORT_SOURCE      GPIO_PortSourceGPIOB
#define TSC_EXTI_IRQ_CHANNEL      EXTI15_10_IRQn
#define TSC_EXTI_LINE             EXTI_Line14
#define TSC_IT_GPIO_CLK           RCC_APB2Periph_GPIOB
#define TSC_IT_GPIO_PORT          GPIOB
#define TSC_IT_GPIO_PIN           GPIO_Pin_14

/* Touchscreen Controller DEFINES for STM32100E-EVAL */
#elif USE_STM32100E_EVAL
#define TSC_I2C_PORT              I2C2
#define TSC_I2C_CLK               RCC_APB1Periph_I2C2
#define TSC_I2C_SDA_GPIO_PIN      GPIO_Pin_11
#define TSC_I2C_SDA_GPIO_PORT     GPIOB
#define TSC_I2C_SDA_GPIO_CLK      RCC_APB2Periph_GPIOB
#define TSC_I2C_SDA_SOURCE        GPIO_PinSource11
#define TSC_I2C_SDA_AF            NULL

#define TSC_I2C_SCL_GPIO_PIN      GPIO_Pin_10
#define TSC_I2C_SCL_GPIO_PORT     GPIOB
#define TSC_I2C_SCL_GPIO_CLK      RCC_APB2Periph_GPIOB
#define TSC_I2C_SCL_SOURCE        GPIO_PinSource10
#define TSC_I2C_SCL_AF            NULL

#define TSC_GPIO_PIN_SOURCE       GPIO_PinSource12
#define TSC_IT_EXTI_PIN_SOURCE    GPIO_PinSource12
#define TSC_GPIO_PORT_SOURCE      GPIO_PortSourceGPIOA
#define TSC_EXTI_IRQ_CHANNEL      EXTI15_10_IRQn
#define JOY_EXTI_IRQ_CHANNEL      NULL
#define TSC_EXTI_LINE             EXTI_Line12
#define TSC_IT_GPIO_CLK           RCC_APB2Periph_GPIOA
#define TSC_IT_GPIO_PORT          GPIOA
#define TSC_IT_GPIO_PIN           GPIO_Pin_12
#define JOY_EXTI_LINE             NULL

/* Touchscreen Controller DEFINES for STM322xG-EVAL */
#elif defined(USE_STM322xG_EVAL)
#define TSC_I2C_PORT              I2C1
#define TSC_I2C_CLK               RCC_APB1Periph_I2C1
#define TSC_I2C_SDA_GPIO_PIN      GPIO_Pin_6
#define TSC_I2C_SDA_GPIO_PORT     GPIOB
#define TSC_I2C_SDA_GPIO_CLK      RCC_AHB1Periph_GPIOB
#define TSC_I2C_SDA_SOURCE        GPIO_PinSource6
#define TSC_I2C_SDA_AF            GPIO_AF_I2C1
#define TSC_I2C_SCL_GPIO_PIN      GPIO_Pin_9
#define TSC_I2C_SCL_GPIO_PORT     GPIOB
#define TSC_I2C_SCL_GPIO_CLK      RCC_AHB1Periph_GPIOB
#define TSC_I2C_SCL_SOURCE        GPIO_PinSource9
#define TSC_I2C_SCL_AF            GPIO_AF_I2C1
#define TSC_GPIO_PIN_SOURCE       GPIO_PinSource2
#define TSC_IT_EXTI_PIN_SOURCE    EXTI_PinSource2
#define TSC_GPIO_PORT_SOURCE      EXTI_PortSourceGPIOI
#define TSC_EXTI_IRQ_CHANNEL      EXTI2_IRQn
#define TSC_EXTI_LINE             EXTI_Line2
#define TSC_IT_GPIO_CLK           RCC_AHB1Periph_GPIOI
#define TSC_IT_GPIO_PORT          GPIOI
#define TSC_IT_GPIO_PIN           GPIO_Pin_2
#endif

#ifdef USE_STM3210C_EVAL
/* LCD Controller DEFINES for STM3210C-EVAL board */
#define LCD_CTRL_PORT_NCS            GPIOB
#define LCD_GPIO_DATA_PORT           GPIOC
#define LCD_CTRL_PIN_NCS             GPIO_Pin_2
#define LCD_GPIO_PIN_SCK             GPIO_Pin_10
#define LCD_GPIO_PIN_MISO            GPIO_Pin_11
#define LCD_GPIO_PIN_MOSI            GPIO_Pin_12
#define LCD_GPIO_RCC_BUS_PERIPH      RCC_APB2Periph_GPIOC
#define LCD_GPIO_RCC_BUS_PERIPH_NCS  RCC_APB2Periph_GPIOB
#define LCD_GPIO_REMAP_PORT          GPIO_Remap_SPI3
#define LCD_RCC_BUS_PERIPH           RCC_APB1Periph_SPI3
#define LCD_SPI_PORT                 SPI3
#define LCD_CONNECTION_MODE          GL_SPI

#elif USE_STM3210B_EVAL

/* LCD Controller DEFINES for STM3210B-EVAL board */
#define LCD_CTRL_PORT_NCS            GPIOB
#define LCD_GPIO_DATA_PORT           GPIOB
#define LCD_CTRL_PIN_NCS             GPIO_Pin_2
#define LCD_GPIO_PIN_SCK             GPIO_Pin_13
#define LCD_GPIO_PIN_MISO            GPIO_Pin_14
#define LCD_GPIO_PIN_MOSI            GPIO_Pin_15
#define LCD_GPIO_RCC_BUS_PERIPH      RCC_APB2Periph_GPIOB
#define LCD_GPIO_RCC_BUS_PERIPH_NCS  RCC_APB2Periph_GPIOB
#define LCD_RCC_BUS_PERIPH           RCC_APB1Periph_SPI2
#define LCD_SPI_PORT                 SPI2
#define LCD_GPIO_REMAP_PORT          ((uint32_t)0)
#define LCD_CONNECTION_MODE          GL_SPI

#elif USE_STM32100B_EVAL

/* LCD Controller DEFINES for STM32100B-EVAL board  */
#define LCD_CTRL_PORT_NCS            GPIOB
#define LCD_GPIO_DATA_PORT           GPIOB
#define LCD_CTRL_PIN_NCS             GPIO_Pin_2
#define LCD_GPIO_PIN_SCK             GPIO_Pin_13
#define LCD_GPIO_PIN_MISO            GPIO_Pin_14
#define LCD_GPIO_PIN_MOSI            GPIO_Pin_15
#define LCD_GPIO_RCC_BUS_PERIPH      RCC_APB2Periph_GPIOB
#define LCD_GPIO_RCC_BUS_PERIPH_NCS  RCC_APB2Periph_GPIOB
#define LCD_RCC_BUS_PERIPH           RCC_APB1Periph_SPI2
#define LCD_SPI_PORT                 SPI2
#define LCD_GPIO_REMAP_PORT          ((uint32_t)0)
#define LCD_CONNECTION_MODE          GL_SPI

#elif USE_STM32L152_EVAL

/* LCD Controller DEFINES for STM32L152-EVAL board */
#define LCD_CTRL_PORT_NCS            GPIOH
#define LCD_GPIO_DATA_PORT           GPIOE
#define LCD_CTRL_PIN_NCS             GPIO_Pin_2
#define LCD_GPIO_PIN_SCK             GPIO_Pin_13
#define LCD_GPIO_PIN_MISO            GPIO_Pin_14
#define LCD_GPIO_PIN_MOSI            GPIO_Pin_15
#define LCD_GPIO_RCC_BUS_PERIPH      RCC_AHBPeriph_GPIOE
#define LCD_GPIO_RCC_BUS_PERIPH_NCS  RCC_AHBPeriph_GPIOH
#define LCD_RCC_BUS_PERIPH           RCC_APB2Periph_SPI1
#define LCD_SPI_PORT                 SPI1
#define LCD_GPIO_REMAP_PORT          ((uint32_t)0)
#define LCD_CONNECTION_MODE          GL_SPI

#elif defined(USE_STM322xG_EVAL)
/* LCD Controller DEFINES for STM322xG-EVAL board */
#define LCD_RCC_BUS_PERIPH           RCC_AHB3Periph_FSMC
#define LCD_CONNECTION_MODE          GL_FSMC
#else
/* LCD Controller DEFINES for STM3210E-EVAL & STM32100E-EVAL boards */
#define LCD_RCC_BUS_PERIPH           RCC_AHBPeriph_FSMC
#define LCD_CONNECTION_MODE          GL_FSMC
#endif

extern uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE];
extern uint32_t USART_Rx_ptr_in;
extern uint32_t USART_Rx_ptr_out;
extern uint32_t USART_Rx_length;
extern TSC_HW_Parameters_TypeDef pTscHwParam;
extern LCD_HW_Parameters_TypeDef pLcdHwParam;

TSC_HW_Parameters_TypeDef* NewTscHwParamObj();

LCD_HW_Parameters_TypeDef* NewLcdHwParamObj();

#endif  /*__HW_CONFIG_H*/
/******************* (C) COPYRIGHT 2011 STMicroelectronics *****END OF FILE****/
