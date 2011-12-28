/* Copyright (c) 2010,2011, Hervé Rouault
 * All rights reserved.
 *
 * This file is part of Coyotte
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 * * Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the following disclaimer.  * Redistributions in
 * binary form must reproduce the above copyright notice, this list of
 * conditions and the following disclaimer in the documentation and/or other
 * materials provided with the distribution.  * Neither the name of the
 * <organization> nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written
 * permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF
 * THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 */

#ifndef DISPLAY_H
#define DISPLAY_H

#include <stdio.h>
#include <string.h>

#define PAGE_HOME       0xFF

void display_init();
void InputInterface_Init();
void ShowLoadingLogo();
void Show_HomeScreen();

#define LCD_NCS_PIN             GPIO_Pin_2
#define LCD_NCS_GPIO_PORT       GPIOB
#define LCD_NCS_GPIO_CLK        RCC_APB2Periph_GPIOB

/**
  * @brief  LCD SPI Interface pins
  */
#define LCD_SPI			            SPI3
#define LCD_SPI_CLK		          RCC_APB1Periph_SPI3
#define LCD_SPI_SCK_PIN         GPIO_Pin_10
#define LCD_SPI_SCK_GPIO_PORT   GPIOC
#define LCD_SPI_SCK_GPIO_CLK    RCC_APB2Periph_GPIOC
#define LCD_SPI_MISO_PIN        GPIO_Pin_11
#define LCD_SPI_MISO_GPIO_PORT  GPIOC
#define LCD_SPI_MISO_GPIO_CLK   RCC_APB2Periph_GPIOC
#define LCD_SPI_MOSI_PIN        GPIO_Pin_12
#define LCD_SPI_MOSI_GPIO_PORT  GPIOC
#define LCD_SPI_MOSI_GPIO_CLK   RCC_APB2Periph_GPIOC

/**
  * @brief  LCD Registers
  */
#define LCD_REG_0             0x00
#define LCD_REG_1             0x01
#define LCD_REG_2             0x02
#define LCD_REG_3             0x03
#define LCD_REG_4             0x04
#define LCD_REG_5             0x05
#define LCD_REG_6             0x06
#define LCD_REG_7             0x07
#define LCD_REG_8             0x08
#define LCD_REG_9             0x09
#define LCD_REG_10            0x0A
#define LCD_REG_12            0x0C
#define LCD_REG_13            0x0D
#define LCD_REG_14            0x0E
#define LCD_REG_15            0x0F
#define LCD_REG_16            0x10
#define LCD_REG_17            0x11
#define LCD_REG_18            0x12
#define LCD_REG_19            0x13
#define LCD_REG_20            0x14
#define LCD_REG_21            0x15
#define LCD_REG_22            0x16
#define LCD_REG_23            0x17
#define LCD_REG_24            0x18
#define LCD_REG_25            0x19
#define LCD_REG_26            0x1A
#define LCD_REG_27            0x1B
#define LCD_REG_28            0x1C
#define LCD_REG_29            0x1D
#define LCD_REG_30            0x1E
#define LCD_REG_31            0x1F
#define LCD_REG_32            0x20
#define LCD_REG_33            0x21
#define LCD_REG_34            0x22
#define LCD_REG_36            0x24
#define LCD_REG_37            0x25
#define LCD_REG_40            0x28
#define LCD_REG_41            0x29
#define LCD_REG_43            0x2B
#define LCD_REG_45            0x2D
#define LCD_REG_48            0x30
#define LCD_REG_49            0x31
#define LCD_REG_50            0x32
#define LCD_REG_51            0x33
#define LCD_REG_52            0x34
#define LCD_REG_53            0x35
#define LCD_REG_54            0x36
#define LCD_REG_55            0x37
#define LCD_REG_56            0x38
#define LCD_REG_57            0x39
#define LCD_REG_59            0x3B
#define LCD_REG_60            0x3C
#define LCD_REG_61            0x3D
#define LCD_REG_62            0x3E
#define LCD_REG_63            0x3F
#define LCD_REG_64            0x40
#define LCD_REG_65            0x41
#define LCD_REG_66            0x42
#define LCD_REG_67            0x43
#define LCD_REG_68            0x44
#define LCD_REG_69            0x45
#define LCD_REG_70            0x46
#define LCD_REG_71            0x47
#define LCD_REG_72            0x48
#define LCD_REG_73            0x49
#define LCD_REG_74            0x4A
#define LCD_REG_75            0x4B
#define LCD_REG_76            0x4C
#define LCD_REG_77            0x4D
#define LCD_REG_78            0x4E
#define LCD_REG_79            0x4F
#define LCD_REG_80            0x50
#define LCD_REG_81            0x51
#define LCD_REG_82            0x52
#define LCD_REG_83            0x53
#define LCD_REG_96            0x60
#define LCD_REG_97            0x61
#define LCD_REG_106           0x6A
#define LCD_REG_118           0x76
#define LCD_REG_128           0x80
#define LCD_REG_129           0x81
#define LCD_REG_130           0x82
#define LCD_REG_131           0x83
#define LCD_REG_132           0x84
#define LCD_REG_133           0x85
#define LCD_REG_134           0x86
#define LCD_REG_135           0x87
#define LCD_REG_136           0x88
#define LCD_REG_137           0x89
#define LCD_REG_139           0x8B
#define LCD_REG_140           0x8C
#define LCD_REG_141           0x8D
#define LCD_REG_143           0x8F
#define LCD_REG_144           0x90
#define LCD_REG_145           0x91
#define LCD_REG_146           0x92
#define LCD_REG_147           0x93
#define LCD_REG_148           0x94
#define LCD_REG_149           0x95
#define LCD_REG_150           0x96
#define LCD_REG_151           0x97
#define LCD_REG_152           0x98
#define LCD_REG_153           0x99
#define LCD_REG_154           0x9A
#define LCD_REG_157           0x9D
#define LCD_REG_192           0xC0
#define LCD_REG_193           0xC1
#define LCD_REG_229           0xE5

#define	CUR_READ_DRAW_CUR     0x06

#define LCD_DIR_HORIZONTAL       0x0000
#define LCD_DIR_VERTICAL         0x0001

#define LCD_PIXEL_WIDTH          0x0140
#define LCD_PIXEL_HEIGHT         0x00F0

#define LCD_ILI9320        0x9320
#define LCD_HX8312         0x8312
#define LCD_SPFD5408       0x5408
#define START_BYTE         0x70
#define SET_INDEX          0x00
#define READ_STATUS        0x01
#define LCD_WRITE_REG      0x02
#define LCD_READ_REG       0x03

#define TS_ReadCalibrationVaraible(offset)  (*(__IO uint32_t*)(CalibrationAddr + offset))

#ifdef USE_TIMEOUT_USER_CALLBACK
uint8_t IOE_TimeoutUserCallback(void);
#else
#define IOE_TimeoutUserCallback()  IOE_TIMEOUT
#endif /* USE_TIMEOUT_USER_CALLBACK */

#define p_strncpy(oBuf, iBuf, Len)  strncpy((char*)oBuf, (char*)iBuf, (int)Len)
#define p_strlen(iBuf)              strlen((char*)iBuf)
#define min(x,y)                    ((x<y)? x:y)

#define START_BYTE         0x70

#define GL_FONT_BIG           0x00
#define GL_FONT_SMALL         0x01
#define GL_FONT_BIG_WIDTH       16
#define GL_FONT_BIG_HEIGHT      24
#define GL_FONT_SMALL_WIDTH      8
#define GL_FONT_SMALL_HEIGHT    12

#define FONT_LENGTH                     8
#define BUTTON_SLICE_LENGTH             8
#define SLIDEBAR_CURSOR_LENGTH          6
#define SLIDEBAR_CENTRAL_LENGTH         27
#define SLIDEBAR_OFFSET_LENGTH          4
#define SLIDEBAR_PIECE_LENGTH           4
#define SLIDEBAR_HEIGHT                 18
#define BUTTON_HEIGHT                   26
#define BUTTON_PIECE_LENGTH             8
#define RBUTTON_OPT_SIZE                20
#define RADIO_BUTTON_RADIUS             9
#define COMBOBOX_SIZE                   22
#define CHECKBOX_SIZE                   20
#define PAGE_MAX_NUM                    50
#define TIMEOUT                         1000000



extern __IO uint8_t GL_Font;
extern __IO uint8_t GL_FontWidth;
extern __IO uint8_t GL_FontHeight;

extern uint16_t LCD_Height;
extern uint16_t LCD_Width;

typedef enum {
    GL_HORIZONTAL = 0,
    GL_LEFT_VERTICAL,
    GL_RIGHT_VERTICAL
} GL_Direction;

/**
  * @brief  GL_ButtonStatus enumeration definition
  */
typedef enum {
    UNSELECTED = 0,
    SELECTED = 1,
    GL_UNKNOWN = 2
} GL_ButtonStatus;

/**
  * @brief  GL_ObjType enumeration definition
  */
typedef enum {
    GL_BUTTON = 1,
    GL_RADIO_BUTTON = 2,
    GL_CHECKBOX = 3,
    GL_LABEL = 4,
    GL_SWITCH = 5,
    GL_ICON = 6,
    GL_COMBOBOX = 7,
    GL_SLIDEBAR = 8,
    GL_HISTOGRAM = 9,
    GL_GRAPH_CHART = 10
} GL_ObjType;

/**
  * @brief  GL_Coordinate struct definition
  */
typedef struct {
    uint16_t MaxX;
    uint16_t MinX;
    uint16_t MaxY;
    uint16_t MinY;
} GL_Coordinate_TypeDef;

typedef enum {
    GL_ERROR = 0,
    GL_OK = !GL_ERROR
} GL_ErrStatus;

typedef struct GL_PageControlsObj GL_PageControls_TypeDef;

struct GL_PageControlsObj {
    uint16_t               ID;
    void         *         objPTR;
    GL_Coordinate_TypeDef  objCoordinates;
    GL_ObjType             objType;
    GL_ErrStatus(*SetObjVisible)(GL_PageControls_TypeDef * pThis, GL_Coordinate_TypeDef objCoordinates);
};

typedef struct GL_PageObj GL_Page_TypeDef;

struct GL_PageObj {
    uint8_t                  objName[20];
    uint8_t                  Page_Active;
    uint8_t                  Page_Visible;
    uint16_t                 ControlCount;
    GL_ErrStatus(*ShowPage)(GL_Page_TypeDef * pThis, uint8_t bVal);
    GL_PageControls_TypeDef * PageControls[30];
    GL_ErrStatus(*SetPage)(GL_Page_TypeDef * pThis, uint8_t bVal);
    uint8_t (*GetObjStatus)(GL_Page_TypeDef * pThis, uint16_t ID);
    GL_Coordinate_TypeDef(*GetObjCoordinates)(GL_Page_TypeDef * pThis, uint16_t ID);
};

typedef enum {
    BitReset = 0,
    BitSet = 1
} IOE_BitValue_TypeDef;

typedef struct {
    uint16_t Height;
    uint16_t Length;
} GL_ObjDimensions_TypeDef;

typedef struct GL_LabelObj GL_Label_TypeDef;

/**
  * @brief  GL_LabelObj struct definition
  */
struct GL_LabelObj {
    uint16_t          ID;
    uint8_t           label[20];
    __IO uint8_t      FontSize;
    uint8_t           Control_Visible;
    uint16_t          Colour;
    GL_Direction      Direction;
};

/* Forward declaration for circular typedefs */
typedef struct GL_ButtonObj GL_Button_TypeDef;

/**
  * @brief  GL_ButtonObj struct definition
  */
struct GL_ButtonObj {
    uint16_t          ID;
#ifndef USE_2D_OBJECTS
    uint8_t     *     ImageClickedPTR;
    uint8_t     *     ImageUnClickedPTR;
#endif
    uint8_t           label[20];
    uint8_t           isObjectTouched;
    uint8_t           Control_Visible;
    void (*EventHandler)(void);
};

typedef enum {
    GL_LOW   = Bit_RESET,
    GL_HIGH  = Bit_SET
} GL_SignalActionType;

typedef FLASH_Status      TSC_FLASH_Status;

typedef enum {
    IOE_DMA_TX = 0,
    IOE_DMA_RX = 1
} IOE_DMADirection_TypeDef;

typedef enum {
    IOE_OK = 0,
    IOE_FAILURE,
    IOE_TIMEOUT,
    PARAM_ERROR,
    IOE1_NOT_OPERATIONAL,
    IOE2_NOT_OPERATIONAL
} IOE_Status_TypDef;

typedef enum {GL_SPI = 0, GL_FSMC = 1, GL_OTHER = 2} GL_BusType;

typedef struct {
    uint8_t          TSC_PortSource;
    uint16_t         TSC_PinSource;
    uint32_t         TSC_I2C_Clk;
    uint16_t         TSC_I2C_Sda_Gpio_Pin;
    GPIO_TypeDef  *  TSC_I2C_Sda_Gpio_Port;
    uint32_t         TSC_I2C_Sda_Gpio_Clk;
    uint16_t         TSC_I2C_Sda_PinSource;
    uint32_t         TSC_I2C_Sda_AltFunc;
    uint16_t         TSC_I2C_Scl_Gpio_Pin;
    GPIO_TypeDef  *  TSC_I2C_Scl_Gpio_Port;
    uint32_t         TSC_I2C_Scl_Gpio_Clk;
    uint16_t         TSC_I2C_Scl_PinSource;
    uint32_t         TSC_I2C_Scl_AltFunc;
    uint8_t          TSC_Exti_IrqChannel;
    uint32_t         TSC_Exti_Line;
    uint16_t         TSC_IT_Exti_Pin_Source;
    uint32_t         TSC_IT_Gpio_Clk;
    GPIO_TypeDef  *  TSC_IT_Gpio_Port;
    uint16_t         TSC_IT_Gpio_Pin;
    uint32_t         TSC_DeviceRegister;
    uint32_t         TSC_RegisterAddress;
    I2C_TypeDef   *  TSC_Bus_Port;
} TSC_HW_Parameters_TypeDef;

typedef struct {
    GPIO_TypeDef * LCD_Ctrl_Port_NCS;
    GPIO_TypeDef * LCD_Gpio_Data_Port;
    uint16_t       LCD_Ctrl_Pin_NCS;
    uint16_t       LCD_Gpio_Pin_SCK;
    uint16_t       LCD_Gpio_Pin_MISO;
    uint16_t       LCD_Gpio_Pin_MOSI;
    uint32_t       LCD_Rcc_BusPeriph_GPIO;
    uint32_t       LCD_Rcc_BusPeriph_GPIO_Ncs;
    uint32_t       LCD_Gpio_RemapPort;
    uint32_t       LCD_Rcc_Bus_Periph;
    SPI_TypeDef  * LCD_Bus_Port;
    GL_BusType     LCD_Connection_Mode;
} LCD_HW_Parameters_TypeDef;

typedef struct GL_CheckboxObj GL_Checkbox_TypeDef;
typedef struct GL_RadioButtonGrp GL_RadioButtonGrp_TypeDef;
typedef struct GL_SlidebarObj GL_Slidebar_TypeDef;
typedef struct GL_SwitchObj GL_Switch_TypeDef;

#define IOE_I2C                          I2C2
#define IOE_I2C_CLK                      RCC_APB1Periph_I2C2
#define IOE_I2C_SCL_PIN                  GPIO_Pin_10
#define IOE_I2C_SCL_GPIO_PORT            GPIOB
#define IOE_I2C_SCL_GPIO_CLK             RCC_APB2Periph_GPIOB
#define IOE_I2C_SDA_PIN                  GPIO_Pin_11
#define IOE_I2C_SDA_GPIO_PORT            GPIOB
#define IOE_I2C_SDA_GPIO_CLK             RCC_APB2Periph_GPIOB
#define IOE_I2C_DR                       ((uint32_t)0x40005810)
#define IOE_I2C_SPEED                    300000

/**
  * @brief  IOE DMA definitions
  */
#define IOE_DMA                          DMA1
#define IOE_DMA_CLK                      RCC_AHBPeriph_DMA1
#define IOE_DMA_TX_CHANNEL               DMA1_Channel4
#define IOE_DMA_RX_CHANNEL               DMA1_Channel5
#define IOE_DMA_TX_TCFLAG                DMA1_FLAG_TC4
#define IOE_DMA_RX_TCFLAG                DMA1_FLAG_TC5


/**
  * @brief  IO Expander Interrupt line on EXTI
  */
#define IOE_IT_PIN                       GPIO_Pin_12
#define IOE_IT_GPIO_PORT                 GPIOA
#define IOE_IT_GPIO_CLK                  RCC_APB2Periph_GPIOA
#define IOE_IT_EXTI_PORT_SOURCE          GPIO_PortSourceGPIOA
#define IOE_IT_EXTI_PIN_SOURCE           GPIO_PinSource12
#define IOE_IT_EXTI_LINE                 EXTI_Line12
#define IOE_IT_EXTI_IRQn                 EXTI15_10_IRQn


/**
  * @brief  The 7 bits IO Expanders adresses and chip IDs
  */
#define IOE_1_ADDR                 0x82
#define STMPE811_ID                0x0811


/*------------------------------------------------------------------------------
    Functional and Interrupt Management
------------------------------------------------------------------------------*/
/**
  * @brief  IO Expander Functionalities definitions
  */
#define IOE_ADC_FCT              0x01
#define IOE_TS_FCT               0x02
#define IOE_IO_FCT               0x04
#define IOE_TEMPSENS_FCT         0x08

/**
  * @brief  Interrupt source configuration definitons
  */
#define IOE_ITSRC_TSC           0x01  /* IO_Exapnder 1 */
#define IOE_ITSRC_TEMPSENS      0x08  /* IO_Exapnder 1 */

/**
  * @brief  Glaobal Interrupts definitions
  */
#define IOE_GIT_GPIO             0x80
#define IOE_GIT_ADC              0x40
#define IOE_GIT_TEMP             0x20
#define IOE_GIT_FE               0x10
#define IOE_GIT_FF               0x08
#define IOE_GIT_FOV              0x04
#define IOE_GIT_FTH              0x02
#define IOE_GIT_TOUCH            0x01

/**
  * @brief IO Exapanders Pins definition
  */
#define IO1_IN_ALL_PINS          (uint32_t)(MEMS_INT1_PIN | MEMS_INT2_PIN)


/*------------------------------------------------------------------------------
    STMPE811 device register definition
------------------------------------------------------------------------------*/
/**
  * @brief  Identification registers
  */
#define IOE_REG_CHP_ID             0x00
#define IOE_REG_ID_VER             0x02

/**
  * @brief  General Control Registers
  */
#define IOE_REG_SYS_CTRL1          0x03
#define IOE_REG_SYS_CTRL2          0x04
#define IOE_REG_SPI_CFG            0x08

/**
  * @brief  Interrupt Control register
  */
#define IOE_REG_INT_CTRL           0x09
#define IOE_REG_INT_EN             0x0A
#define IOE_REG_INT_STA            0x0B
#define IOE_REG_GPIO_INT_EN        0x0C
#define IOE_REG_GPIO_INT_STA       0x0D

/**
  * @brief  GPIO Registers
  */
#define IOE_REG_GPIO_SET_PIN       0x10
#define IOE_REG_GPIO_CLR_PIN       0x11
#define IOE_REG_GPIO_MP_STA        0x12
#define IOE_REG_GPIO_DIR           0x13
#define IOE_REG_GPIO_ED            0x14
#define IOE_REG_GPIO_RE            0x15
#define IOE_REG_GPIO_FE            0x16
#define IOE_REG_GPIO_AF            0x17

/**
  * @brief  ADC Registers
  */
#define IOE_REG_ADC_INT_EN         0x0E
#define IOE_REG_ADC_INT_STA        0x0F
#define IOE_REG_ADC_CTRL1          0x20
#define IOE_REG_ADC_CTRL2          0x21
#define IOE_REG_ADC_CAPT           0x22
#define IOE_REG_ADC_DATA_CH0       0x30 /* 16-Bit register */
#define IOE_REG_ADC_DATA_CH1       0x32 /* 16-Bit register */
#define IOE_REG_ADC_DATA_CH2       0x34 /* 16-Bit register */
#define IOE_REG_ADC_DATA_CH3       0x36 /* 16-Bit register */
#define IOE_REG_ADC_DATA_CH4       0x38 /* 16-Bit register */
#define IOE_REG_ADC_DATA_CH5       0x3A /* 16-Bit register */
#define IOE_REG_ADC_DATA_CH6       0x3B /* 16-Bit register */
#define IOE_REG_ADC_DATA_CH7       0x3C /* 16-Bit register */

/**
  * @brief  TouchScreen Registers
  */
#define IOE_REG_TSC_CTRL           0x40
#define IOE_REG_TSC_CFG            0x41
#define IOE_REG_WDM_TR_X           0x42
#define IOE_REG_WDM_TR_Y           0x44
#define IOE_REG_WDM_BL_X           0x46
#define IOE_REG_WDM_BL_Y           0x48
#define IOE_REG_FIFO_TH            0x4A
#define IOE_REG_FIFO_STA           0x4B
#define IOE_REG_FIFO_SIZE          0x4C
#define IOE_REG_TSC_DATA_X         0x4D
#define IOE_REG_TSC_DATA_Y         0x4F
#define IOE_REG_TSC_DATA_Z         0x51
#define IOE_REG_TSC_DATA_XYZ       0x52
#define IOE_REG_TSC_FRACT_XYZ      0x56
#define IOE_REG_TSC_DATA           0x57
#define IOE_REG_TSC_I_DRIVE        0x58
#define IOE_REG_TSC_SHIELD         0x59

/**
  * @brief  Temperature Sensor registers
  */
#define IOE_REG_TEMP_CTRL          0x60
#define IOE_REG_TEMP_DATA          0x61
#define IOE_REG_TEMP_TH            0x62

/*------------------------------------------------------------------------------
    Functions parameters defines
------------------------------------------------------------------------------*/
/**
  * @brief Touch Screen Pins definition
  */
#define TOUCH_YD                    IO_Pin_1 /* IO_Exapnader_1 */ /* Input */
#define TOUCH_XD                    IO_Pin_2 /* IO_Exapnader_1 */ /* Input */
#define TOUCH_YU                    IO_Pin_3 /* IO_Exapnader_1 */ /* Input */
#define TOUCH_XU                    IO_Pin_4 /* IO_Exapnader_1 */ /* Input */
#define TOUCH_IO_ALL                (uint32_t)(IO_Pin_1 | IO_Pin_2 | IO_Pin_3 | IO_Pin_4)

/**
  * @brief  IO Pins
  */
#define IO_Pin_0                 0x01
#define IO_Pin_1                 0x02
#define IO_Pin_2                 0x04
#define IO_Pin_3                 0x08
#define IO_Pin_4                 0x10
#define IO_Pin_5                 0x20
#define IO_Pin_6                 0x40
#define IO_Pin_7                 0x80
#define IO_Pin_ALL               0xFF

/**
  * @brief  IO Pin directions
  */
#define Direction_IN             0x00
#define Direction_OUT            0x01

/**
  * @brief  Interrupt Line output parameters
  */
#define Polarity_Low             0x00
#define Polarity_High            0x04
#define Type_Level               0x00
#define Type_Edge                0x02

/**
  * @brief IO Interrupts
  */
#define IO_IT_0                  0x01
#define IO_IT_1                  0x02
#define IO_IT_2                  0x04
#define IO_IT_3                  0x08
#define IO_IT_4                  0x10
#define IO_IT_5                  0x20
#define IO_IT_6                  0x40
#define IO_IT_7                  0x80
#define ALL_IT                   0xFF
#define IOE_TS_IT                (uint8_t)(IO_IT_0 | IO_IT_1 | IO_IT_2)

/**
  * @brief  Edge detection value
  */
#define EDGE_FALLING              0x01
#define EDGE_RISING               0x02

/**
  * @brief  Global interrupt Enable bit
  */
#define IOE_GIT_EN                0x01




void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue);
void LCD_SetCursor(uint8_t Xpos, uint16_t Ypos);
void LCD_Clear();
void LCD_Setup();
void TS_CheckCalibration();
FlagStatus TS_IsCalibrationDone();
void TSC_Init();
void ProcessInputData();
ErrorStatus CursorInit(uint8_t * PointerMark);
void TimeOutCalculate();
void TSC_Read();
void CursorDraw(uint16_t X, uint16_t Y, uint8_t DrawPhase);
void GL_Delay(uint32_t nTime);
void Set_LastFlashMemoryAddress(uint32_t address);
GL_ErrStatus Create_PageObj(GL_Page_TypeDef * pThis);
GL_PageControls_TypeDef * NewLabel(uint16_t ID, const uint8_t * label, GL_Direction direction, __IO uint8_t FontSize, __IO uint16_t Colour);
GL_PageControls_TypeDef * NewButton(uint16_t ID, const uint8_t * label, void (*pEventHandler)(void));
GL_ErrStatus AddPageControlObj(uint16_t PosX, uint16_t PosY, GL_PageControls_TypeDef * objPTR, GL_Page_TypeDef * pagePTR);
void GL_SetTextColor(__IO uint16_t GL_NewTextColor);
void GL_SetBackColor(__IO uint16_t GL_NewBackColor);
void GL_SetFont(uint8_t uFont);
void GL_DisplayAdjStringLine(uint16_t Line, uint16_t Column, uint8_t * ptr, uint8_t Transparent_Flag);
void GL_DrawButtonBMP(uint16_t maxX, uint16_t minX, uint16_t maxY, uint16_t minY, uint8_t * ptrBitmap);
static void IOE_GPIO_Config();
static void IOE_I2C_Config();
uint8_t IOE_IsOperational(uint8_t DeviceAddr);
uint8_t IOE_Reset(uint8_t DeviceAddr);
uint16_t IOE_ReadID(uint8_t DeviceAddr);
uint8_t IOE_FnctCmd(uint8_t DeviceAddr, uint8_t Fct, FunctionalState NewState);
uint8_t IOE_IOPinConfig(uint8_t DeviceAddr, uint8_t IO_Pin, uint8_t Direction);
uint8_t IOE_TS_Config();
void LCD_nCS_StartByte(uint8_t Start_Byte);
void LCD_CtrlLinesConfig(void);
void LCD_CtrlLinesWrite(GPIO_TypeDef * GPIOx, uint16_t CtrlPins, BitAction BitVal);
void LCD_SPIConfig(void);
static void PutPixel(int16_t x, int16_t y);
void TS_Calibration();
static void TS_SaveCalibrationVariables();
void TS_CheckCalibration();
uint8_t IOE_IOAFConfig(uint8_t DeviceAddr, uint8_t IO_Pin, FunctionalState NewState);
uint8_t IOE_IOEdgeConfig(uint8_t DeviceAddr, uint8_t IO_Pin, uint8_t Edge);
uint8_t IOE_WriteIOPin(uint8_t IO_Pin, IOE_BitValue_TypeDef BitVal);
static GL_ObjDimensions_TypeDef GetObjSize(GL_PageControls_TypeDef * pPageControl);
uint8_t CompareCoordinates(uint16_t u16_XMax, uint16_t u16_XMin, uint16_t u16_YMax, uint16_t u16_YMin);
static void CallPreEvents(GL_PageControls_TypeDef * pControl);
static void CallEvent(GL_PageControls_TypeDef * pControl);
uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr);
uint16_t I2C_ReadDataBuffer(uint8_t DeviceAddr, uint32_t RegisterAddr);
uint8_t I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue);
uint16_t getDisplayCoordinateX(uint16_t x_touch, uint16_t y_touch);
uint16_t getDisplayCoordinateY(uint16_t x_touch, uint16_t y_touch);
void LCD_PutPixel(uint16_t Xpos, uint16_t Ypos, uint16_t Color, uint8_t PixelSpec);
uint16_t LCD_GetPixel(uint16_t Xpos, uint16_t Ypos);
static void GL_SetStringFieldValue(uint8_t * dBuf, uint8_t * sBuf, uint32_t MaxLength);
static GL_ErrStatus Create_Label(GL_Label_TypeDef * pThis);
static GL_ErrStatus Create_Button(GL_Button_TypeDef * pThis);
void LCD_SetTextColor(__IO uint16_t Color);
void LCD_SetBackColor(__IO uint16_t Color);
void GL_LCD_CtrlLinesWrite(GPIO_TypeDef * GPIOx, uint16_t CtrlPins, GL_SignalActionType BitVal);
void GL_LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii, uint8_t Transparent_Flag);
void GL_SetDisplayWindow(uint16_t Xpos, uint16_t Ypos, uint16_t Height, uint16_t Width);
void GL_DrawBMP(uint8_t * ptrBitmap);
void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void GL_Clear(uint16_t Color);
void GL_Cross(uint16_t Ypos, uint16_t Xpos);
TSC_FLASH_Status TSC_WriteDataToNVM(uint32_t FlashFree_Address, int32_t * Data, uint32_t Size);
void TSC_FLASH_ClearFlag(uint32_t FLASH_FLAG);
static void IOE_DMA_Config(IOE_DMADirection_TypeDef Direction, uint8_t * buffer);
void LCD_WriteRAM_Prepare();
void LCD_WriteRAM(uint16_t RGB_Code);
uint16_t LCD_ReadReg(uint8_t LCD_Reg);
uint16_t GL_LCD_ReadRAM();
void GL_LCD_DrawCharTransparent(uint16_t Xpos, uint16_t Ypos, const uint16_t * c);
void GL_LCD_DrawChar(uint8_t Xpos, uint16_t Ypos, const uint16_t * c);
void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width);
void LCD_WriteRAMWord(uint16_t RGB_Code);
void GL_DrawLine(uint16_t Ypos, uint16_t Xpos, uint16_t Length, uint8_t Direction);
void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction);
void TimingDelay_Decrement();
TSC_FLASH_Status TSC_FLASH_ProgramWord(uint32_t Address, uint32_t Data);
TSC_FLASH_Status TSC_FLASH_ErasePage(uint32_t Page_Address);


#endif /* DISPLAY_H */
