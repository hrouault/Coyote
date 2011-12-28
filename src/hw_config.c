/******************** (C) COPYRIGHT 2011 STMicroelectronics ********************
* File Name          : hw_config.c
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

/* Includes ------------------------------------------------------------------*/
#ifdef STM32L1XX_MD
#include "stm32l1xx_it.h"
#else
#include "stm32f10x_it.h"
#endif /* STM32L1XX_MD */


#include "usb_lib.h"
#include "usb_prop.h"
#include "usb_desc.h"
#include "hw_config.h"
#include "platform_config.h"
#include "usb_pwr.h"
#include "stm32_eval.h"
//#include "TscHal.h"
//#include "LcdHal.h"
#include "display.h"

/* Private typedef -----------------------------------------------------------*/
/* Private define ------------------------------------------------------------*/
/* Private macro -------------------------------------------------------------*/
/* Private variables ---------------------------------------------------------*/
ErrorStatus HSEStartUpStatus;
USART_InitTypeDef USART_InitStructure;

uint8_t  USART_Rx_Buffer [USART_RX_DATA_SIZE];
uint32_t USART_Rx_ptr_in = 0;
uint32_t USART_Rx_ptr_out = 0;
uint32_t USART_Rx_length  = 0;

uint8_t  USB_Tx_State = 0;
static void IntToUnicode(uint32_t value , uint8_t * pbuf , uint8_t len);
/* Extern variables ----------------------------------------------------------*/

extern LINE_CODING linecoding;

/* Private function prototypes -----------------------------------------------*/
static void HWConfig_TSCParamInit();
static void HWConfig_LCDParamInit();
/* Private functions ---------------------------------------------------------*/
/*******************************************************************************
* Function Name  : Set_System
* Description    : Configures Main system clocks & power
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_System(void)
{
#if !defined(STM32F10X_CL) && !defined(STM32L1XX_MD)
    GPIO_InitTypeDef GPIO_InitStructure;
#endif /* STM32F10X_CL && STM32L1XX_MD */
#if defined(USB_USE_EXTERNAL_PULLUP)
    GPIO_InitTypeDef  GPIO_InitStructure;
#endif /* USB_USE_EXTERNAL_PULLUP */
    /*!< At this stage the microcontroller clock setting is already configured,
         this is done through SystemInit() function which is called from startup
         file (startup_stm32f10x_xx.s) before to branch to application main.
         To reconfigure the default setting of SystemInit() function, refer to
         system_stm32f10x.c file
       */
#ifdef STM32L1XX_MD
    /* Enable the SYSCFG module clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_SYSCFG, ENABLE);
#endif /* STM32L1XX_MD */
#if !defined(STM32F10X_CL) && !defined(STM32L1XX_MD)
    /* Enable USB_DISCONNECT GPIO clock */
    RCC_APB2PeriphClockCmd(RCC_APB2Periph_GPIO_DISCONNECT, ENABLE);
    /* Configure USB pull-up pin */
    GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_OD;
    GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* STM32F10X_CL && STM32L1XX_MD */
#if defined(USB_USE_EXTERNAL_PULLUP)
    /* Enable the USB disconnect GPIO clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_GPIO_DISCONNECT, ENABLE);
    /* USB_DISCONNECT used as USB pull-up */
    GPIO_InitStructure.GPIO_Pin = USB_DISCONNECT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_OUT;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_2MHz;
    GPIO_InitStructure.GPIO_OType = GPIO_OType_PP;
    GPIO_InitStructure.GPIO_PuPd = GPIO_PuPd_NOPULL;
    GPIO_Init(USB_DISCONNECT, &GPIO_InitStructure);
#endif /* USB_USE_EXTERNAL_PULLUP */
}

/*******************************************************************************
* Function Name  : Set_USBClock
* Description    : Configures USB Clock input (48MHz)
* Input          : None.
* Return         : None.
*******************************************************************************/
void Set_USBClock(void)
{
#if defined(STM32L1XX_MD)
    /* Enable USB clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#elif defined(STM32F10X_CL)
    /* Select USBCLK source */
    RCC_OTGFSCLKConfig(RCC_OTGFSCLKSource_PLLVCO_Div3);
    /* Enable the USB clock */
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_OTG_FS, ENABLE) ;
#else
    /* Select USBCLK source */
    RCC_USBCLKConfig(RCC_USBCLKSource_PLLCLK_1Div5);
    /* Enable the USB clock */
    RCC_APB1PeriphClockCmd(RCC_APB1Periph_USB, ENABLE);
#endif /* STM32F10X_CL */
}

/*******************************************************************************
* Function Name  : Enter_LowPowerMode
* Description    : Power-off system clocks and power while entering suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Enter_LowPowerMode(void)
{
    /* Set the device state to suspend */
    bDeviceState = SUSPENDED;
}

/*******************************************************************************
* Function Name  : Leave_LowPowerMode
* Description    : Restores system clocks and power while exiting suspend mode
* Input          : None.
* Return         : None.
*******************************************************************************/
void Leave_LowPowerMode(void)
{
    DEVICE_INFO * pInfo = &Device_Info;
    /* Set the device state to the correct state */
    if (pInfo->Current_Configuration != 0) {
        /* Device configured */
        bDeviceState = CONFIGURED;
    } else {
        bDeviceState = ATTACHED;
    }
}

/*******************************************************************************
* Function Name  : USB_Interrupts_Config
* Description    : Configures the USB interrupts
* Input          : None.
* Return         : None.
*******************************************************************************/
void USB_Interrupts_Config(void)
{
    NVIC_InitTypeDef NVIC_InitStructure;
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_1);
#ifdef STM32L1XX_MD
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#elif defined(STM32F10X_CL)
    /* Enable the USB Interrupts */
    NVIC_InitStructure.NVIC_IRQChannel = OTG_FS_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#else
    NVIC_InitStructure.NVIC_IRQChannel = USB_LP_CAN1_RX0_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 1;
    NVIC_InitStructure.NVIC_IRQChannelSubPriority = 0;
    NVIC_InitStructure.NVIC_IRQChannelCmd = ENABLE;
    NVIC_Init(&NVIC_InitStructure);
#endif /* STM32L1XX_MD */
    /* Enable USART Interrupt */
    NVIC_InitStructure.NVIC_IRQChannel = EVAL_COM1_IRQn;
    NVIC_InitStructure.NVIC_IRQChannelPreemptionPriority = 0;
    NVIC_Init(&NVIC_InitStructure);
}

/*******************************************************************************
* Function Name  : USB_Cable_Config
* Description    : Software Connection/Disconnection of USB Cable
* Input          : None.
* Return         : Status
*******************************************************************************/
void USB_Cable_Config(FunctionalState NewState)
{
#ifdef STM32L1XX_MD
    if (NewState != DISABLE) {
        STM32L15_USB_CONNECT;
    } else {
        STM32L15_USB_DISCONNECT;
    }
#elif defined(USE_STM3210C_EVAL)
    if (NewState != DISABLE) {
        USB_DevConnect();
    } else {
        USB_DevDisconnect();
    }
#else /* USE_STM3210B_EVAL or USE_STM3210E_EVAL */
    if (NewState != DISABLE) {
        GPIO_ResetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
    } else {
        GPIO_SetBits(USB_DISCONNECT, USB_DISCONNECT_PIN);
    }
#endif /* USE_STM3210C_EVAL */
}

/*******************************************************************************
* Function Name  : Handle_USBAsynchXfer.
* Description    : send data to USB.
* Input          : None.
* Return         : none.
*******************************************************************************/
void Handle_USBAsynchXfer(void)
{
    uint16_t USB_Tx_ptr;
    uint16_t USB_Tx_length;
    if (USB_Tx_State != 1) {
        if (USART_Rx_ptr_out == USART_RX_DATA_SIZE) {
            USART_Rx_ptr_out = 0;
        }
        if (USART_Rx_ptr_out == USART_Rx_ptr_in) {
            USB_Tx_State = 0;
            return;
        }
        if (USART_Rx_ptr_out > USART_Rx_ptr_in) { /* rollback */
            USART_Rx_length = USART_RX_DATA_SIZE - USART_Rx_ptr_out;
        } else {
            USART_Rx_length = USART_Rx_ptr_in - USART_Rx_ptr_out;
        }
        if (USART_Rx_length > VIRTUAL_COM_PORT_DATA_SIZE) {
            USB_Tx_ptr = USART_Rx_ptr_out;
            USB_Tx_length = VIRTUAL_COM_PORT_DATA_SIZE;
            USART_Rx_ptr_out += VIRTUAL_COM_PORT_DATA_SIZE;
            USART_Rx_length -= VIRTUAL_COM_PORT_DATA_SIZE;
        } else {
            USB_Tx_ptr = USART_Rx_ptr_out;
            USB_Tx_length = USART_Rx_length;
            USART_Rx_ptr_out += USART_Rx_length;
            USART_Rx_length = 0;
        }
        USB_Tx_State = 1;
#ifdef USE_STM3210C_EVAL
        USB_SIL_Write(EP1_IN, &USART_Rx_Buffer[USB_Tx_ptr], USB_Tx_length);
#else
        UserToPMABufferCopy(&USART_Rx_Buffer[USB_Tx_ptr], ENDP1_TXADDR, USB_Tx_length);
        SetEPTxCount(ENDP1, USB_Tx_length);
        SetEPTxValid(ENDP1);
#endif /* USE_STM3210C_EVAL */
    }
}

/*******************************************************************************
* Function Name  : Get_SerialNum.
* Description    : Create the serial number string descriptor.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void Get_SerialNum(void)
{
    uint32_t Device_Serial0, Device_Serial1, Device_Serial2;
#ifdef STM32L1XX_MD
    Device_Serial0 = *(uint32_t *)(0x1FF80050);
    Device_Serial1 = *(uint32_t *)(0x1FF80054);
    Device_Serial2 = *(uint32_t *)(0x1FF80064);
#else
    Device_Serial0 = *(__IO uint32_t *)(0x1FFFF7E8);
    Device_Serial1 = *(__IO uint32_t *)(0x1FFFF7EC);
    Device_Serial2 = *(__IO uint32_t *)(0x1FFFF7F0);
#endif /* STM32L1XX_MD */
    Device_Serial0 += Device_Serial2;
    if (Device_Serial0 != 0) {
        IntToUnicode(Device_Serial0, &Virtual_Com_Port_StringSerial[2] , 8);
        IntToUnicode(Device_Serial1, &Virtual_Com_Port_StringSerial[18], 4);
    }
}

/*******************************************************************************
* Function Name  : HexToChar.
* Description    : Convert Hex 32Bits value into char.
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
static void IntToUnicode(uint32_t value , uint8_t * pbuf , uint8_t len)
{
    uint8_t idx = 0;
    for (idx = 0 ; idx < len ; idx ++) {
        if (((value >> 28)) < 0xA) {
            pbuf[ 2 * idx] = (value >> 28) + '0';
        } else {
            pbuf[2 * idx] = (value >> 28) + 'A' - 10;
        }
        value = value << 4;
        pbuf[ 2 * idx + 1] = 0;
    }
}
#ifdef STM32F10X_CL
/*******************************************************************************
* Function Name  : USB_OTG_BSP_uDelay.
* Description    : provide delay (usec).
* Input          : None.
* Output         : None.
* Return         : None.
*******************************************************************************/
void USB_OTG_BSP_uDelay(const uint32_t usec)
{
    RCC_ClocksTypeDef  RCC_Clocks;
    /* Configure HCLK clock as SysTick clock source */
    SysTick_CLKSourceConfig(SysTick_CLKSource_HCLK);
    RCC_GetClocksFreq(&RCC_Clocks);
    SysTick_Config(usec * (RCC_Clocks.HCLK_Frequency / 1000000));
    SysTick->CTRL  &= ~SysTick_CTRL_TICKINT_Msk ;
    while (!(SysTick->CTRL & SysTick_CTRL_COUNTFLAG_Msk));
}
#endif /* STM32F10X_CL */

void HWConfig_SetHardwareParams(void)
{
  HWConfig_TSCParamInit();
  HWConfig_LCDParamInit();
}

static void HWConfig_TSCParamInit(void)
{
#if defined(USE_STM3210C_EVAL) || defined(USE_STM32100E_EVAL) || defined(USE_STM322xG_EVAL)
    TSC_HW_Parameters_TypeDef* pTscParam = NewTscHwParamObj ();

    /* Assign the following values for Touchscreen Controller Parameters Structure */
    pTscParam->TSC_I2C_Clk                = TSC_I2C_CLK;
    pTscParam->TSC_I2C_Sda_Gpio_Pin       = TSC_I2C_SDA_GPIO_PIN;
    pTscParam->TSC_I2C_Sda_Gpio_Port      = TSC_I2C_SDA_GPIO_PORT;
    pTscParam->TSC_I2C_Sda_Gpio_Clk       = TSC_I2C_SDA_GPIO_CLK;
    pTscParam->TSC_I2C_Sda_PinSource      = TSC_I2C_SDA_SOURCE;
    pTscParam->TSC_I2C_Sda_AltFunc        = (uint32_t)TSC_I2C_SDA_AF;
    pTscParam->TSC_I2C_Scl_Gpio_Pin       = TSC_I2C_SCL_GPIO_PIN;
    pTscParam->TSC_I2C_Scl_Gpio_Port      = TSC_I2C_SCL_GPIO_PORT;
    pTscParam->TSC_I2C_Scl_Gpio_Clk       = TSC_I2C_SCL_GPIO_CLK;
    pTscParam->TSC_I2C_Scl_PinSource      = TSC_I2C_SCL_SOURCE;
    pTscParam->TSC_I2C_Scl_AltFunc        = (uint32_t)TSC_I2C_SCL_AF;
    pTscParam->TSC_I2C_Clk                = TSC_I2C_CLK;
    pTscParam->TSC_IT_Exti_Pin_Source     = TSC_IT_EXTI_PIN_SOURCE;
    pTscParam->TSC_IT_Gpio_Clk            = TSC_IT_GPIO_CLK;
    pTscParam->TSC_PortSource             = TSC_GPIO_PORT_SOURCE;
    pTscParam->TSC_PinSource              = TSC_GPIO_PIN_SOURCE;
    pTscParam->TSC_Exti_IrqChannel        = TSC_EXTI_IRQ_CHANNEL;
    pTscParam->TSC_Exti_Line              = TSC_EXTI_LINE;
    pTscParam->TSC_IT_Gpio_Port           = TSC_IT_GPIO_PORT;
    pTscParam->TSC_IT_Gpio_Pin            = TSC_IT_GPIO_PIN;
    pTscParam->TSC_DeviceRegister         = TSC_I2C_DEVICE_REGISTER;
    pTscParam->TSC_Bus_Port               = TSC_I2C_PORT;
#endif

}

static void HWConfig_LCDParamInit()
{
  LCD_HW_Parameters_TypeDef* pLcdParam = NewLcdHwParamObj ();

  /* Assign the following values for LCD Controller Parameters Structure */
  pLcdParam->LCD_Connection_Mode        = LCD_CONNECTION_MODE;
  pLcdParam->LCD_Rcc_Bus_Periph         = LCD_RCC_BUS_PERIPH;

  /* Configuration for SPI interfaced LCDs */
#if defined(USE_STM3210C_EVAL) || defined(USE_STM3210B_EVAL) ||\
  defined(USE_STM32100B_EVAL) ||   defined(USE_STM32L152_EVAL)
  pLcdParam->LCD_Ctrl_Port_NCS          = LCD_CTRL_PORT_NCS;
  pLcdParam->LCD_Gpio_Data_Port         = LCD_GPIO_DATA_PORT;
  pLcdParam->LCD_Ctrl_Pin_NCS           = LCD_CTRL_PIN_NCS;
  pLcdParam->LCD_Gpio_Pin_SCK           = LCD_GPIO_PIN_SCK;
  pLcdParam->LCD_Gpio_Pin_MISO          = LCD_GPIO_PIN_MISO;
  pLcdParam->LCD_Gpio_Pin_MOSI          = LCD_GPIO_PIN_MOSI;
  pLcdParam->LCD_Gpio_RemapPort         = LCD_GPIO_REMAP_PORT;
  pLcdParam->LCD_Rcc_BusPeriph_GPIO     = LCD_GPIO_RCC_BUS_PERIPH;
  pLcdParam->LCD_Rcc_BusPeriph_GPIO_Ncs = LCD_GPIO_RCC_BUS_PERIPH_NCS;
  pLcdParam->LCD_Bus_Port               = LCD_SPI_PORT;
#endif
}

void USART_Config_Default()
{
}

bool USART_Config()
{
    return TRUE;
}

TSC_HW_Parameters_TypeDef pTscHwParam;

TSC_HW_Parameters_TypeDef* NewTscHwParamObj ()
{
  return &pTscHwParam;
}

LCD_HW_Parameters_TypeDef pLcdHwParam;

LCD_HW_Parameters_TypeDef* NewLcdHwParamObj()
{
  return &pLcdHwParam;
}
