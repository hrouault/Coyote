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
 */

#include <stdio.h>
#include "stm32_eval.h"
#include "stm32f10x.h"
#include "display.h"
#include "hw_config.h"
#include "adc.h"
#include "pictures.h"
#include "images.h"
#include "gl_fonts.h"

#define LAST_FLASH_MEMORY_ADDRESS	((uint32_t)0x08040000)

#define RESCALE_FACTOR 1000000
#define CalibrationDone_Offset  ((uint8_t)0 * sizeof(uint32_t))
#define A2_Offset               ((uint8_t)1 * sizeof(uint32_t))
#define B2_Offset               ((uint8_t)2 * sizeof(uint32_t))
#define C2_Offset               ((uint8_t)3 * sizeof(uint32_t))
#define D2_Offset               ((uint8_t)4 * sizeof(uint32_t))
#define E2_Offset               ((uint8_t)5 * sizeof(uint32_t))
#define F2_Offset               ((uint8_t)6 * sizeof(uint32_t))
/* Private macros ------------------------------------------------------------*/
#define TS_ReadCalibrationVaraible(offset)  (*(__IO uint32_t*)(CalibrationAddr + offset))
/* Private variables ---------------------------------------------------------*/
int32_t A2 = 0, B2 = 0, C2 = 0, D2 = 0, E2 = 0, F2 = 0;

/* General Control Registers */
#define GL_SYS_CTRL1        IOE_REG_SYS_CTRL1
#define GL_SYS_CTRL2        IOE_REG_SYS_CTRL2

/* Interrupt Control register */
#define GL_SPI_CFG          IOE_REG_SPI_CFG
#define GL_INT_CTRL         IOE_REG_INT_CTRL
#define GL_INT_EN           IOE_REG_INT_EN
#define GL_INT_STA          IOE_REG_INT_STA
#define GL_GPIO_INT_EN      IOE_REG_GPIO_INT_EN
#define GL_GPIO_INT_STA     IOE_REG_GPIO_INT_STA
/* GPIO Registers */
#define GL_GPIO_SET_PIN     IOE_REG_GPIO_SET_PIN
#define GL_GPIO_CLR_PIN     IOE_REG_GPIO_CLR_PIN
#define GL_GPIO_MP_STA      IOE_REG_GPIO_MP_STA
#define GL_GPIO_DIR         IOE_REG_GPIO_DIR
#define GL_GPIO_ED          IOE_REG_GPIO_ED
#define GL_GPIO_RE          IOE_REG_GPIO_RE
#define GL_GPIO_FE          IOE_REG_GPIO_FE
#define GL_GPIO_AF          IOE_REG_GPIO_AF

/* ADC Registers */
#define GL_ADC_CTRL         IOE_REG_ADC_CTRL1
/*........ */
/* TouchScreen Registers */
#define GL_TSC_CTRL         IOE_REG_TSC_CTRL
#define GL_TSC_CFG          IOE_REG_TSC_CFG
#define GL_WDM_TR_X         IOE_REG_WDM_TR_X
#define GL_WDM_TR_Y         IOE_REG_WDM_TR_Y
#define GL_WDM_BL_X         IOE_REG_WDM_BL_X
#define GL_WDM_BL_Y         IOE_REG_WDM_BL_Y
#define GL_FIFO_TH          IOE_REG_FIFO_TH
#define GL_FIFO_CTRL_STA    IOE_REG_FIFO_STA
#define GL_FIFO_SIZE        IOE_REG_FIFO_SIZE
#define GL_TSC_DATA_X       IOE_REG_TSC_DATA_X
#define GL_TSC_DATA_Y       IOE_REG_TSC_DATA_Y
#define GL_TSC_DATA_Z       IOE_REG_TSC_DATA_Z


/* For Flashing */
#define FLASH_PAGE_SIZE          ((uint16_t)0x800)
#define TSC_FLASH_COMPLETE       FLASH_COMPLETE

#define TSC_FLASH_FLAG_PGERR     ((uint32_t)FLASH_FLAG_PGERR)  /*!< FLASH Program error flag */
#define TSC_FLASH_FLAG_WRPRTERR  ((uint32_t)FLASH_FLAG_WRPRTERR)  /*!< FLASH Write protected error flag */

__IO uint8_t calibration_done = 0;

#define TIMEOUT_MAX             0x1000   /*<! The value of the maximal timeout for I2C waiting loops */

uint32_t IOE_TimeOut = TIMEOUT_MAX; /*<! Value of Timeout when I2C communication fails */

uint32_t CalibrationAddr = 0;

uint32_t FlashFree_Address = 0;

uint32_t TSC_Value_X;
uint32_t TSC_Value_Y;

typedef struct {
    uint16_t TouchDetected;
    uint16_t X;
    uint16_t Y;
    uint16_t Z;
} TS_STATE;

typedef enum {FAILED = 0, PASSED = !FAILED} TSC_Flash_TestStatus;

__IO TSC_Flash_TestStatus TSC_MemoryProgramStatus;

TS_STATE TS_State;

static uint16_t PageCount = 0;
GL_Page_TypeDef * PagesList[20];

static __IO uint16_t TextColor = 0x0000, BackColor = 0xFFFF;
__IO uint16_t          GL_BackColor;
__IO uint16_t          GL_TextColor;

#define	CUR_READ_BEH_MASK     0x02
#define CUR_RELEASE	      0x20
#define IO2_OUT_ALL_PINS         (uint32_t)(AUDIO_RESET_PIN | MII_INT_PIN)

#define IOE_2_ADDR                 0x88

typedef struct {
    uint16_t xpos;
    uint8_t ypos;
    char text[20];
} Label;

typedef struct {
    uint16_t xpos;
    uint8_t ypos;
    char text[20];
} Button;

#define MEMS_INT1_PIN               IO_Pin_3 /* IO_Exapnader_1 */ /* Input */
#define MEMS_INT2_PIN               IO_Pin_2 /* IO_Exapnader_1 */ /* Input */

uint32_t EndAddr = 0;

static uint8_t CursorPointer[] = {
    0x08,   /*Height of cursor symbol*/
    0x0B,   /*Width of cursor symbol*/
    0x35,   /*Count of pixels of cursor symbol*/
    0x80, 0xC0, 0xE0, 0xF0, 0xF8, 0xFC, 0xFE, 0xFF, 0xFF, 0xF8, 0xD8
};

typedef struct GL_IconObj GL_Icon_TypeDef;


#define MAX_BUTTON_LABEL_LENGTH              16
#define MAX_CTRL_X_PAGE                      30
#define MAX_LABEL_LENGTH                     46


#define GL_Horizontal         0x00
#define GL_Vertical           0x01

#define SinglePixel           0x08
#define FirstPixel            0x01
#define MiddlePixel           0x02
#define LastPixel             0x04

#define GL_SPI_CFG          IOE_REG_SPI_CFG


#define AUDIO_RESET_PIN             IO_Pin_2 /* IO_Exapnader_2 */ /* Output */

#define	CUR_DRAW_CUR          0x08
#define CUR_DRAW_CUR_MASK     0x04

#define VBAT_DIV_PIN                IO_Pin_0 /* IO_Exapnader_1 */ /* Output */
#define IO1_OUT_ALL_PINS         (uint32_t)(VBAT_DIV_PIN)

#define CSW_CMD_PASSED                0x00

#define TOUCH_DELAY     			0x10

#define	CUR_DRAW_BEH	        0x01
#define	CUR_DRAW_BEH_MASK     0x01

__IO uint32_t vu32_gTimeOutCount = 0;
__IO uint8_t vu8_gTouchEnable = 1;
__IO uint8_t vu8_gSleepState = 0;
uint8_t ValidTouchDetected = 0;
/* Set to 0 to disable The Automatic Backlight Switch Off */
__IO uint8_t vu8_gPowerSaveOption = 1;

__IO uint32_t u32_TSXCoordinate;
__IO uint32_t u32_TSYCoordinate;

typedef struct GL_ComboBoxGrp    GL_ComboBoxGrp_TypeDef;

#define MII_INT_PIN                 IO_Pin_0 /* IO_Exapnader_2 */ /* Output */
#define JOY_IO_PINS                  (uint32_t)(IO_Pin_3 | IO_Pin_4 | IO_Pin_5 | IO_Pin_6 | IO_Pin_7)

typedef enum {GL_RESET = 0, GL_SET = !GL_RESET} GL_FlagStatus, GL_ITStatus;

__IO uint8_t touch_done = 0;


__IO TSC_FLASH_Status TSC_FlashStatus;


static void Show_Target();

static GL_Coordinate_TypeDef GetObjCoordinates(GL_Page_TypeDef * pPage, uint16_t ID)
{
    GL_Coordinate_TypeDef null;
    uint32_t index = 0;
    /* Init the structure with zeros */
    memset(&null, 0x00, sizeof(GL_Coordinate_TypeDef));
    if (!pPage) {
        return null;
    }
    while (index < pPage->ControlCount) { /* search for the required object */
        if (pPage->PageControls[index]->objType == GL_LABEL) {
            GL_Label_TypeDef * pTmp;
            pTmp = (GL_Label_TypeDef *)(pPage->PageControls[index]->objPTR);
            if (pTmp->ID == ID) {
                return pPage->PageControls[index]->objCoordinates;
            }
            index++;
        }
        if (pPage->PageControls[index]->objType == GL_BUTTON) {
            GL_Button_TypeDef * pTmp;
            pTmp = (GL_Button_TypeDef *)(pPage->PageControls[index]->objPTR);
            if (pTmp->ID == ID) {
                return pPage->PageControls[index]->objCoordinates;
            }
            index++;
        }
    }
    return null;
}


void LCD_SetCursor(uint8_t Xpos, uint16_t Ypos)
{
    LCD_WriteReg(LCD_REG_32, Xpos);
    LCD_WriteReg(LCD_REG_33, Ypos);
}

void display_init()
{
    RCC_AHBPeriphClockCmd(RCC_AHBPeriph_DMA2, ENABLE);
    RCC_ClocksTypeDef RCC_Clocks;
    /* Set the Vector Table base location at 0x08000000 */
    NVIC_SetVectorTable(NVIC_VectTab_FLASH, 0x0000);
    NVIC_PriorityGroupConfig(NVIC_PriorityGroup_2);
    /* Setup SysTick Timer for 10 msec interrupts  */
    RCC_GetClocksFreq(&RCC_Clocks);
    if (SysTick_Config(RCC_Clocks.SYSCLK_Frequency / 100)) {
        /* Capture error */
        while (1);
    }
    /* configure Systick priority */
    NVIC_SetPriority(SysTick_IRQn, 0x0B);
    /* Set HW structure parameters */
    HWConfig_SetHardwareParams();
    /* Initialize the LCD */
    LCD_Setup();
    LCD_SetCursor(0x00, 0x013F);
    LCD_Clear();
    LCD_WriteReg(R3, 0x1028);
    InputInterface_Init();
    ShowLoadingLogo();
    /* Check if Touchscreen Calibration has been done*/
    TS_CheckCalibration();
    //    TSC_FLASH_ClearFlag( TSC_FLASH_FLAG_BSY | TSC_FLASH_FLAG_EOP | TSC_FLASH_FLAG_PGERR );
    //    TS_Calibration();
    TSC_Init();
    /*Initialize cursor*/
    LCD_Clear();
    /* Menu Initialisation*/
    Show_HomeScreen();
    GL_Delay(20);
}

void InputInterface_Init()
{
    /* Set the last Flash Memory address */
    Set_LastFlashMemoryAddress(LAST_FLASH_MEMORY_ADDRESS);
    /* Touch Screen Init */
    TSC_Init();
}


__IO uint8_t vu8_gPageCurrent = 0;
GL_Page_TypeDef * pageStart;

uint16_t behindcurs[88];

static __IO uint32_t TimingDelay;

GL_ErrStatus ShowPage(GL_Page_TypeDef * pPage, uint8_t bVal)
{
    uint32_t i = 0;
    if (!pPage) {
        return GL_ERROR;
    }
    pPage->Page_Visible = bVal;
    pPage->SetPage(pPage, bVal);
    if (bVal == 0x01) {
        while (i < pPage->ControlCount) { /* search for the required button */
            pPage->PageControls[i]->SetObjVisible(pPage->PageControls[i], pPage->PageControls[i]->objCoordinates);
            i++;
        }
    } else {
        LCD_Clear();
    }
    pPage->SetPage(pPage, bVal);
    return GL_OK;
}

static GL_ErrStatus SetPage(GL_Page_TypeDef * pPage, uint8_t bVal)
{
    if (!pPage) {
        return GL_ERROR;
    }
    pPage->Page_Active = bVal;
    return GL_OK;
}

uint8_t GetObjStatus(GL_Page_TypeDef * pPage, uint16_t ID)
{
    uint32_t index = 0;
    if (pPage) {
        while (index < pPage->ControlCount) { /* search for the required object */
            if (pPage->PageControls[index]->objType == GL_BUTTON) {
                GL_Button_TypeDef * pTmp;
                pTmp = (GL_Button_TypeDef *)(pPage->PageControls[index]->objPTR);
                if (pTmp->ID == ID) {
                    return pTmp->isObjectTouched;
                }
            }
            index++;
        }
    }
    return 0x00;
}

static GL_ErrStatus SetLabelVisible(GL_PageControls_TypeDef * pTmp, GL_Coordinate_TypeDef objCoordinates)
{
    GL_Label_TypeDef * pThis = (GL_Label_TypeDef *)(pTmp->objPTR);
    if (!pThis) {
        return GL_ERROR;
    }
    pThis->Control_Visible = 1;
    if (pTmp->objType == GL_LABEL) {
        GL_SetTextColor(pThis->Colour);
        if (pThis->FontSize == GL_FONT_SMALL) {
            GL_SetFont(GL_FONT_SMALL);
            if (pThis->Direction == GL_HORIZONTAL) {
                GL_DisplayAdjStringLine((uint8_t)(objCoordinates.MinY), (uint16_t)(objCoordinates.MaxX),
                                        (uint8_t *)pThis->label,
                                        TRUE);
            }
            GL_SetFont(GL_FONT_BIG);
        } else {
            GL_SetFont(GL_FONT_BIG);
            if (pThis->Direction == GL_HORIZONTAL) {
                GL_DisplayAdjStringLine((uint8_t)(objCoordinates.MinY), (uint16_t)(objCoordinates.MaxX), (uint8_t *)pThis->label, TRUE);
            }
        }
    }
    return GL_OK;
}



void Show_HomeScreen()
{
    //    char tempstring[20];
    /*Resetting all the menu parameters*/
    vu8_gPageCurrent = PAGE_HOME;
    GL_PageControls_TypeDef * Maintemp;
    GL_PageControls_TypeDef * Targettemp;
    GL_PageControls_TypeDef * Activate;
    GL_PageControls_TypeDef * Standby;
    GL_PageControls_TypeDef * Settemp;
    GL_PageControls_TypeDef * SetPID;
    /*     GL_PageControls_TypeDef* CoefP;
     *     GL_PageControls_TypeDef* CoefI;
     *     GL_PageControls_TypeDef* CoefD;
     *     GL_PageControls_TypeDef* Standby;
     *     GL_PageControls_TypeDef* Settemp;
     *     GL_PageControls_TypeDef* SetPID;
     */
    /* ************** GRAPHIC LIBRARY - PAGE 0 ************** */
    GL_Page_TypeDef pageStartobj;
    GL_Page_TypeDef * pageStart = &pageStartobj;
    Create_PageObj(pageStart);
    //    siprintf((char *)tempstring, "%i.%i°C", adc_curtemp/1000, adc_curtemp%1000);
    Maintemp = NewLabel(10, (uint8_t *)"18.5°C", GL_HORIZONTAL, 0x00, 0x0000);
    Targettemp = NewLabel(11, (uint8_t *)"Target: 20.0°C", GL_HORIZONTAL, 0x01, 0x0000);
    Activate = NewButton(12, (uint8_t *)"Activate", Show_Target);
    Standby = NewButton(13, (uint8_t *)"Standby", Show_Target);
    Settemp = NewButton(14, (uint8_t *)"Target", Show_Target);
    SetPID = NewButton(15, (uint8_t *)"Set PID", Show_Target);
    AddPageControlObj((uint16_t)30, (uint16_t)100, Targettemp, pageStart);
    AddPageControlObj((uint16_t)60, (uint16_t)50, Maintemp, pageStart);
    AddPageControlObj((uint16_t)100, (uint16_t)200, Activate, pageStart);
    AddPageControlObj((uint16_t)180, (uint16_t)200, Standby, pageStart);
    AddPageControlObj((uint16_t)280, (uint16_t)80, Settemp, pageStart);
    AddPageControlObj((uint16_t)280, (uint16_t)140, SetPID, pageStart);
    LCD_Clear();
    //    GL_SetTextColor(0x0000);
    pageStart->ShowPage(pageStart, 1);
}

GL_Page_TypeDef * pageTarget;
static void Show_Target()
{
    //    char tempstring[20];
    /*Resetting all the menu parameters*/
    vu8_gPageCurrent = 2;
    GL_PageControls_TypeDef * Maintemp;
    GL_PageControls_TypeDef * Targettemp;
    GL_Page_TypeDef pageTargetobj;
    GL_Page_TypeDef * pageTarget = &pageTargetobj;
    Create_PageObj(pageStart);
    Maintemp = NewLabel(10, (uint8_t *)"18.5 C", GL_HORIZONTAL, 0x00, 0x00);
    Targettemp = NewLabel(11, (uint8_t *)"Target: 20.0 C", GL_HORIZONTAL, 0x01, 0x00);
    AddPageControlObj((uint16_t)30, (uint16_t)100, Targettemp, pageStart);
    AddPageControlObj((uint16_t)60, (uint16_t)50, Maintemp, pageStart);
    LCD_Clear();
    //    GL_SetTextColor(0x0000);
    pageTarget->ShowPage(pageTarget, 1);
}


void ShowLoadingLogo(void)
{
    GL_SetTextColor(0x0000);
    GL_SetBackColor(0xFFFF);
    GL_SetFont(0x01);
    GL_DisplayAdjStringLine(190, 10, (uint8_t *)"Copyright (c) 2010, 2011 H. Rouault", FALSE);
    GL_DisplayAdjStringLine(210, 10, (uint8_t *)"All rights reserved", FALSE);
    GL_DrawButtonBMP(236, 84, 162, 10, (uint8_t *)pictures_labpicrgb_bmp);
    GL_Delay(200);
}

__IO uint8_t GL_Font = GL_FONT_BIG;
__IO uint8_t GL_FontWidth = GL_FONT_BIG_WIDTH;
__IO uint8_t GL_FontHeight = GL_FONT_BIG_HEIGHT;

uint16_t LCD_Height = 240;
uint16_t LCD_Width  = 320;

static GL_ErrStatus SetButtonVisible(GL_PageControls_TypeDef * pTmp, GL_Coordinate_TypeDef objCoordinates)
{
    uint8_t btn_length = 0;
    uint32_t LabelLength = 0;
    GL_Button_TypeDef * pThis = (GL_Button_TypeDef *)(pTmp->objPTR);
#ifndef USE_2D_OBJECTS
    uint32_t n = 1;
    uint8_t * ptrBitmapLeft = NULL;
    uint8_t * ptrBitmapCenter = NULL;
    uint8_t * ptrBitmapRight = NULL;
#endif
    if (!pThis) {
        return GL_ERROR;
    }
    pThis->Control_Visible = 0x01;
    LabelLength = p_strlen(pThis->label);
    if (pThis->isObjectTouched == 0x00) {
#ifndef USE_2D_OBJECTS
        if (pThis->ImageUnClickedPTR == BtnNormal) {
            ptrBitmapLeft = (uint8_t *)BtnNormalLeft;
            ptrBitmapCenter = pThis->ImageUnClickedPTR;
            ptrBitmapRight = (uint8_t *)BtnNormalRight;
        } else {
            ptrBitmapLeft = (uint8_t *)BtnPressedLeft;
            ptrBitmapCenter = pThis->ImageUnClickedPTR;
            ptrBitmapRight = (uint8_t *)BtnPressedRight;
        }
#else
        GL_SetTextColor(GL_Black);
        if (LabelLength > MAX_BUTTON_LABEL_LENGTH)
            GL_DrawFilledRectangle((uint16_t)(objCoordinates.MaxX),
                                   (uint16_t)(objCoordinates.MaxX - ((MAX_BUTTON_LABEL_LENGTH - 1)*FONT_LENGTH)),
                                   (uint8_t)(objCoordinates.MaxY),
                                   (uint8_t)(objCoordinates.MinY), GL_Blue);
        else
            GL_DrawFilledRectangle((uint16_t)(objCoordinates.MaxX),
                                   (uint16_t)(objCoordinates.MaxX - ((LabelLength + 1)*FONT_LENGTH)),
                                   (uint8_t)(objCoordinates.MaxY),
                                   (uint8_t)(objCoordinates.MinY), GL_Blue);
#endif
    } else if (pThis->isObjectTouched == 0x01) {
#ifndef USE_2D_OBJECTS
        if (pThis->ImageClickedPTR == BtnPressed) {
            ptrBitmapLeft = (uint8_t *)BtnPressedLeft;
            ptrBitmapCenter = pThis->ImageClickedPTR;
            ptrBitmapRight = (uint8_t *)BtnPressedRight;
        } else {
            ptrBitmapLeft = (uint8_t *)BtnNormalLeft;
            ptrBitmapCenter = pThis->ImageClickedPTR;
            ptrBitmapRight = (uint8_t *)BtnNormalRight;
        }
#else
        GL_SetTextColor(GL_Black);
        if (LabelLength > MAX_BUTTON_LABEL_LENGTH)
            GL_DrawFilledRectangle((uint16_t)(objCoordinates.MaxX),
                                   (uint16_t)(objCoordinates.MaxX - ((MAX_BUTTON_LABEL_LENGTH - 1)*FONT_LENGTH)),
                                   (uint8_t)(objCoordinates.MaxY),
                                   (uint8_t)(objCoordinates.MinY), GL_Grey);
        else
            GL_DrawFilledRectangle((uint16_t)(objCoordinates.MaxX),
                                   (uint16_t)(objCoordinates.MaxX - ((LabelLength + 1)*FONT_LENGTH)),
                                   (uint8_t)(objCoordinates.MaxY),
                                   (uint8_t)(objCoordinates.MinY), GL_Grey);
#endif
    }
#ifndef USE_2D_OBJECTS
    GL_DrawButtonBMP((uint16_t)(objCoordinates.MaxX),
                     (uint16_t)(objCoordinates.MaxX - BUTTON_SLICE_LENGTH),
                     (uint8_t)(objCoordinates.MaxY),
                     (uint8_t)(objCoordinates.MinY),
                     ptrBitmapRight);
    if (LabelLength < 10) {
        for (; n < LabelLength && (objCoordinates.MaxX - ((n + 1)*BUTTON_SLICE_LENGTH)) > BUTTON_SLICE_LENGTH + 1; n++) {
            GL_DrawButtonBMP((uint16_t)(objCoordinates.MaxX - (n * BUTTON_SLICE_LENGTH)),
                             (uint16_t)(objCoordinates.MaxX - ((n + 1)*BUTTON_SLICE_LENGTH)),
                             (uint8_t)(objCoordinates.MaxY),
                             (uint8_t)(objCoordinates.MinY),
                             ptrBitmapCenter);
        }
    } else {
        for (; n < LabelLength - 1 && (objCoordinates.MaxX - ((n + 1)*BUTTON_SLICE_LENGTH)) > BUTTON_SLICE_LENGTH + 1; n++) {
            GL_DrawButtonBMP((uint16_t)(objCoordinates.MaxX - (n * BUTTON_SLICE_LENGTH)),
                             (uint16_t)(objCoordinates.MaxX - ((n + 1)*BUTTON_SLICE_LENGTH)),
                             (uint8_t)(objCoordinates.MaxY), (uint8_t)(objCoordinates.MinY),
                             ptrBitmapCenter);
        }
    }
    if ((objCoordinates.MaxX - ((n + 1)*BUTTON_SLICE_LENGTH)) > 0) {
        GL_DrawButtonBMP((uint16_t)(objCoordinates.MaxX - (n * BUTTON_SLICE_LENGTH)),
                         (uint16_t)(objCoordinates.MaxX - ((n + 1)*BUTTON_SLICE_LENGTH)),
                         (uint8_t)(objCoordinates.MaxY), (uint8_t)(objCoordinates.MinY),
                         ptrBitmapLeft);
    }
#endif
    GL_SetTextColor(0xFFFF);
    GL_SetFont(GL_FONT_SMALL);
    btn_length = LabelLength * FONT_LENGTH + 2 * FONT_LENGTH;
    if (LabelLength < 7) {
        GL_DisplayAdjStringLine((uint8_t)(objCoordinates.MaxY) - 18,
                                (uint16_t)(objCoordinates.MaxX) - ((btn_length + (LabelLength - 1)*FONT_LENGTH) / 2) + 2,
                                (uint8_t *)pThis->label,
                                1);
    } else {
        GL_DisplayAdjStringLine((uint8_t)(objCoordinates.MaxY) - 18,
                                (uint16_t)(objCoordinates.MaxX) - ((btn_length + (LabelLength - 1)*FONT_LENGTH) / 2) + 4,
                                (uint8_t *)pThis->label,
                                0x01);
    }
    GL_SetFont(GL_FONT_BIG);
    GL_SetBackColor(0xFFFF);
    GL_SetTextColor(0x0000);
    return GL_OK;
}


void LCD_WriteRegIndex(uint8_t LCD_Reg)
{
    /* Reset LCD control line(/CS) and Send Start-Byte */
    LCD_nCS_StartByte(START_BYTE | SET_INDEX);
    /* Write 16-bit Reg Index (High Byte is 0) */
    SPI_I2S_SendData(LCD_SPI, 0x00);
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
    SPI_I2S_SendData(LCD_SPI, LCD_Reg);
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}

void LCD_WriteReg(uint8_t LCD_Reg, uint16_t LCD_RegValue)
{
    /* Write 16-bit Index (then Write Reg) */
    LCD_WriteRegIndex(LCD_Reg);
    /* Write 16-bit Reg */
    /* Reset LCD control line(/CS) and Send Start-Byte */
    LCD_nCS_StartByte(START_BYTE | LCD_WRITE_REG);
    SPI_I2S_SendData(LCD_SPI, LCD_RegValue >> 8);
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
    SPI_I2S_SendData(LCD_SPI, (LCD_RegValue & 0xFF));
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}

void LCD_Setup(void)
{
    /* Configure the LCD Control pins --------------------------------------------*/
    LCD_CtrlLinesConfig();
    /* Configure the LCD_SPI interface ----------------------------------------------*/
    LCD_SPIConfig();
    GL_Delay(5); /* Delay 50 ms */
    /* Start Initial Sequence ------------------------------------------------*/
    LCD_WriteReg(LCD_REG_229, 0x8000); /* Set the internal vcore voltage */
    LCD_WriteReg(LCD_REG_0,  0x0001); /* Start internal OSC. */
    LCD_WriteReg(LCD_REG_1,  0x0100); /* set SS and SM bit */
    LCD_WriteReg(LCD_REG_2,  0x0700); /* set 1 line inversion */
    LCD_WriteReg(LCD_REG_3,  0x1030); /* set GRAM write direction and BGR=1. */
    LCD_WriteReg(LCD_REG_4,  0x0000); /* Resize register */
    LCD_WriteReg(LCD_REG_8,  0x0202); /* set the back porch and front porch */
    LCD_WriteReg(LCD_REG_9,  0x0000); /* set non-display area refresh cycle ISC[3:0] */
    LCD_WriteReg(LCD_REG_10, 0x0000); /* FMARK function */
    LCD_WriteReg(LCD_REG_12, 0x0000); /* RGB interface setting */
    LCD_WriteReg(LCD_REG_13, 0x0000); /* Frame marker Position */
    LCD_WriteReg(LCD_REG_15, 0x0000); /* RGB interface polarity */
    /* Power On sequence -----------------------------------------------------*/
    LCD_WriteReg(LCD_REG_16, 0x0000); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(LCD_REG_17, 0x0000); /* DC1[2:0], DC0[2:0], VC[2:0] */
    LCD_WriteReg(LCD_REG_18, 0x0000); /* VREG1OUT voltage */
    LCD_WriteReg(LCD_REG_19, 0x0000); /* VDV[4:0] for VCOM amplitude */
    GL_Delay(20);                 /* Dis-charge capacitor power voltage (200ms) */
    LCD_WriteReg(LCD_REG_16, 0x17B0); /* SAP, BT[3:0], AP, DSTB, SLP, STB */
    LCD_WriteReg(LCD_REG_17, 0x0137); /* DC1[2:0], DC0[2:0], VC[2:0] */
    GL_Delay(5);                  /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_18, 0x0139); /* VREG1OUT voltage */
    GL_Delay(5);                  /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_19, 0x1d00); /* VDV[4:0] for VCOM amplitude */
    LCD_WriteReg(LCD_REG_41, 0x0013); /* VCM[4:0] for VCOMH */
    GL_Delay(5);                  /* Delay 50 ms */
    LCD_WriteReg(LCD_REG_32, 0x0000); /* GRAM horizontal Address */
    LCD_WriteReg(LCD_REG_33, 0x0000); /* GRAM Vertical Address */
    /* Adjust the Gamma Curve ------------------------------------------------*/
    LCD_WriteReg(LCD_REG_48, 0x0006);
    LCD_WriteReg(LCD_REG_49, 0x0101);
    LCD_WriteReg(LCD_REG_50, 0x0003);
    LCD_WriteReg(LCD_REG_53, 0x0106);
    LCD_WriteReg(LCD_REG_54, 0x0b02);
    LCD_WriteReg(LCD_REG_55, 0x0302);
    LCD_WriteReg(LCD_REG_56, 0x0707);
    LCD_WriteReg(LCD_REG_57, 0x0007);
    LCD_WriteReg(LCD_REG_60, 0x0600);
    LCD_WriteReg(LCD_REG_61, 0x020b);
    /* Set GRAM area ---------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_80, 0x0000); /* Horizontal GRAM Start Address */
    LCD_WriteReg(LCD_REG_81, 0x00EF); /* Horizontal GRAM End Address */
    LCD_WriteReg(LCD_REG_82, 0x0000); /* Vertical GRAM Start Address */
    LCD_WriteReg(LCD_REG_83, 0x013F); /* Vertical GRAM End Address */
    LCD_WriteReg(LCD_REG_96,  0x2700); /* Gate Scan Line */
    LCD_WriteReg(LCD_REG_97,  0x0001); /* NDL,VLE, REV */
    LCD_WriteReg(LCD_REG_106, 0x0000); /* set scrolling line */
    /* Partial Display Control -----------------------------------------------*/
    LCD_WriteReg(LCD_REG_128, 0x0000);
    LCD_WriteReg(LCD_REG_129, 0x0000);
    LCD_WriteReg(LCD_REG_130, 0x0000);
    LCD_WriteReg(LCD_REG_131, 0x0000);
    LCD_WriteReg(LCD_REG_132, 0x0000);
    LCD_WriteReg(LCD_REG_133, 0x0000);
    /* Panel Control ---------------------------------------------------------*/
    LCD_WriteReg(LCD_REG_144, 0x0010);
    LCD_WriteReg(LCD_REG_146, 0x0000);
    LCD_WriteReg(LCD_REG_147, 0x0003);
    LCD_WriteReg(LCD_REG_149, 0x0110);
    LCD_WriteReg(LCD_REG_151, 0x0000);
    LCD_WriteReg(LCD_REG_152, 0x0000);
    /* Set GRAM write direction and BGR = 1 */
    /* I/D=01 (Horizontal : increment, Vertical : decrement) */
    /* AM=1 (address is updated in vertical writing direction) */
    LCD_WriteReg(LCD_REG_3, 0x1018);
    LCD_WriteReg(LCD_REG_7, 0x0173); /* 262K color and display ON */
}

void TS_CheckCalibration()
{
    /* Clear All pending flags */
#if defined(USE_STM3210C_EVAL) || defined(USE_STM32100E_EVAL)
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR);
#endif
    /* Calculate the address of the Penultimate Flash Memory Page, where the calibration parameters will be saved. */
    /*  CalibrationAddr = (uint32_t)(EndAddr-0x404); */
    if ((TS_ReadCalibrationVaraible(CalibrationDone_Offset) & 0x000000FF) != 1) {
        TS_Calibration();
    } else {
        calibration_done = 1;
    }
    /* Reading Calibration Parameters from the Flash Memory */
    A2 = (TS_ReadCalibrationVaraible(A2_Offset));
    B2 = (TS_ReadCalibrationVaraible(B2_Offset));
    C2 = (TS_ReadCalibrationVaraible(C2_Offset));
    D2 = (TS_ReadCalibrationVaraible(D2_Offset));
    E2 = (TS_ReadCalibrationVaraible(E2_Offset));
    F2 = (TS_ReadCalibrationVaraible(F2_Offset));
}

FlagStatus TS_IsCalibrationDone()
{
    return (calibration_done == 1) ? SET : RESET ;
}

void TSC_Init()
{
    /* Configure the needed pins */
    IOE_GPIO_Config();
    /* Configure the I2C peripheral */
    IOE_I2C_Config();
    /* Read IO Expander 1 ID  */
    if (IOE_IsOperational(IOE_1_ADDR)) {
        //    return IOE1_NOT_OPERATIONAL;
        return;
    }
    if (IOE_IsOperational(IOE_2_ADDR)) {
        //    return IOE2_NOT_OPERATIONAL;
        return;
    }
    /* Generate IOExpander Software reset */
    IOE_Reset(IOE_1_ADDR);
    IOE_Reset(IOE_2_ADDR);
    /* ---------------------- IO Expander 1 configuration --------------------- */
    /* Enable the GPIO, Touch Screen and ADC functionalities */
    IOE_FnctCmd(IOE_1_ADDR, IOE_IO_FCT | IOE_TS_FCT | IOE_ADC_FCT, ENABLE);
    /* Configure the VBAT pin in output mode pin*/
    IOE_IOPinConfig(IOE_1_ADDR, VBAT_DIV_PIN , Direction_OUT);
    /* ENABLE the alternate function for IN1 pin */
    IOE_IOAFConfig(IOE_1_ADDR, VBAT_DIV_PIN, ENABLE);
    /* Apply the default state for the out pins */
    IOE_WriteIOPin(VBAT_DIV_PIN, BitReset);
    /* Configure the MEMS interrupt pins in Input mode */
    IOE_IOPinConfig(IOE_2_ADDR, (uint32_t)(MEMS_INT1_PIN | MEMS_INT2_PIN), Direction_IN);
    /* ENABLE the alternate function for the Joystick pins */
    IOE_IOAFConfig(IOE_2_ADDR, (uint32_t)(MEMS_INT1_PIN | MEMS_INT2_PIN), ENABLE);
    /* Configure the IOs to detect Falling and Rising Edges */
    IOE_IOEdgeConfig(IOE_2_ADDR, (uint32_t)(MEMS_INT1_PIN | MEMS_INT2_PIN), (uint32_t)(EDGE_FALLING | EDGE_RISING));
    /* Touch Screen controller configuration */
    IOE_TS_Config();
    /* ------------------------------------------------------------------------ */
    /* ---------------------- IO Expander 2 configuration --------------------- */
    /* Enable the GPIO, Temperature Sensor and ADC functionalities */
    IOE_FnctCmd(IOE_2_ADDR, IOE_IO_FCT | IOE_TEMPSENS_FCT | IOE_ADC_FCT, ENABLE);
    /* Configure the Audio Codec Reset pin in output mode pin*/
    IOE_IOPinConfig(IOE_2_ADDR, (uint32_t)(AUDIO_RESET_PIN), Direction_OUT);
    IOE_IOPinConfig(IOE_2_ADDR, (uint32_t)(MII_INT_PIN), Direction_IN);
    /* ENABLE the alternate function for IN1 pin */
    IOE_IOAFConfig(IOE_2_ADDR, (uint32_t)(AUDIO_RESET_PIN | MII_INT_PIN), ENABLE);
    /* Apply the default state for the out pins */
    IOE_WriteIOPin(AUDIO_RESET_PIN, BitReset);
    IOE_WriteIOPin(MII_INT_PIN, BitReset);
    /* Configure the Joystick pins in Input mode */
    IOE_IOPinConfig(IOE_2_ADDR, JOY_IO_PINS , Direction_IN);
    /* ENABLE the alternate function for the Joystick pins */
    IOE_IOAFConfig(IOE_2_ADDR, JOY_IO_PINS, ENABLE);
    /* Configure the IOs to detect Falling and Rising Edges */
    IOE_IOEdgeConfig(IOE_2_ADDR, JOY_IO_PINS, (uint8_t)(EDGE_FALLING | EDGE_RISING));
    /* Temperature Sensor module configuration */
    //  IOE_TempSens_Config();
    /* ------------------------------------------------------------------------ */
    /* Configuration is OK */
    //  return IOE_OK;
    return;
}

void ProcessInputData()
{
    uint16_t p_index, c_index;
    GL_Coordinate_TypeDef tmpCoord;
    GL_ObjDimensions_TypeDef tmpSize;
    if (touch_done) {
        if (vu8_gSleepState == 1) {
            //            GL_BackLightSwitch(GL_ON);
            vu8_gSleepState = 0;
        } else if ((vu8_gTouchEnable == 1) && (vu8_gSleepState == 0)) {
            vu32_gTimeOutCount = 0;
            for (p_index = 0; p_index < PageCount; p_index++) {
                if (PagesList[p_index]->Page_Active == 1) {
                    for (c_index = 0; c_index < PagesList[p_index]->ControlCount; c_index++) {
                        tmpCoord = PagesList[p_index]->PageControls[c_index]->objCoordinates;
                        tmpSize = GetObjSize(PagesList[p_index]->PageControls[c_index]);
                        if (CompareCoordinates(tmpCoord.MaxX, tmpCoord.MaxX - tmpSize.Length + 1, tmpCoord.MaxY, tmpCoord.MaxY - tmpSize.Height)) {
                            CallPreEvents(PagesList[p_index]->PageControls[c_index]);
                            CallEvent(PagesList[p_index]->PageControls[c_index]);
                            u32_TSYCoordinate = 0;
                            touch_done = 0;
                            GL_Delay(15);
                            break;
                        }
                    }
                }
            }
        }
        touch_done = 0;
    }
}

void TimeOutCalculate()
{
    if (vu8_gSleepState == 0) {
        if (vu8_gPowerSaveOption == 1) {
            vu32_gTimeOutCount++;
            if (vu32_gTimeOutCount > TIMEOUT) {
                //        GL_BackLightSwitch(GL_OFF);
                vu8_gSleepState = 1;
                vu32_gTimeOutCount = 0;
            }
        }
    }
}

void TSC_Read()
{
    TSC_Value_X = 0x00;
    TSC_Value_Y = 0x00;
    if ((I2C_ReadDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_TSC_CTRL) & 0x80)) {
        GL_Delay(TOUCH_DELAY);
        /* Stop touchscreen controller */
        I2C_WriteDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_TSC_CTRL, 0x00);
        TSC_Value_X = I2C_ReadDataBuffer(pTscHwParam.TSC_DeviceRegister, GL_TSC_DATA_Y);
        TSC_Value_Y = I2C_ReadDataBuffer(pTscHwParam.TSC_DeviceRegister, GL_TSC_DATA_X);
        I2C_ReadDataBuffer(pTscHwParam.TSC_DeviceRegister, GL_TSC_DATA_Z);
        u32_TSXCoordinate = getDisplayCoordinateX(TSC_Value_X, TSC_Value_Y);
        u32_TSYCoordinate = getDisplayCoordinateY(TSC_Value_X, TSC_Value_Y);
        touch_done = 1;
        /* Clear the interrupt pending bit and enable the FIFO again */
        I2C_WriteDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_FIFO_CTRL_STA, 0x01);
        I2C_WriteDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_INT_STA, IOE_TS_IT);
        I2C_WriteDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_FIFO_CTRL_STA, 0x00);
        GL_Delay(0x02);
        /* Enable touchscreen controller */
        I2C_WriteDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_TSC_CTRL, 0x01);
        GL_Delay(0x05);
        /* check if the FIFO is not empty */
        if (I2C_ReadDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_FIFO_CTRL_STA) != 0x20) {
            /* Flush the FIFO */
            I2C_WriteDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_FIFO_CTRL_STA, 0x01);
            I2C_WriteDeviceRegister(pTscHwParam.TSC_DeviceRegister, GL_FIFO_CTRL_STA, 0x00);
        }
    } else {
        GL_Delay(1);
    }
}

/* unit nTime = 10ms */
void GL_Delay(uint32_t nTime)
{
    TimingDelay = nTime;
    while (TimingDelay != 0);
}

void Set_LastFlashMemoryAddress(uint32_t address)
{
    EndAddr = address;
    /* Calculate the address of the Penultimate Flash Memory Page, where the calibration parameters will be saved. */
    CalibrationAddr = (uint32_t)(EndAddr - 0x800);
}

GL_ErrStatus Create_PageObj(GL_Page_TypeDef * pThis)
{
    if (!pThis) {
        return GL_ERROR;
    } else {
        pThis->Page_Active       = 0x00;
        pThis->Page_Visible      = 0x00;
        /* Assign member functions */
        pThis->ShowPage          = ShowPage;
        pThis->SetPage           = SetPage;
        pThis->GetObjStatus      = GetObjStatus;
        pThis->GetObjCoordinates = GetObjCoordinates;
        pThis->ControlCount      = 0;
        //    memset(pThis->PageControls,0x00,sizeof(GL_PageControls_TypeDef*)*MAX_CTRL_X_PAGE);
        /* pThis->GetObjSize = GetObjSize; */
        pThis->SetPage(pThis, 0x00);
        if (PageCount < PAGE_MAX_NUM) {
            PagesList[PageCount] = pThis;
            PageCount++;
        } else {
            return GL_ERROR;
        }
    }
    return GL_OK;
}



GL_PageControls_TypeDef * NewLabel(uint16_t ID, const uint8_t * label, GL_Direction direction, __IO uint8_t FontSize, __IO uint16_t Colour)
{
    GL_Label_TypeDef * pControlObj = NULL;
    GL_PageControls_TypeDef * pPageControlObj = NULL;
    //  pControlObj = (GL_Label_TypeDef *)malloc(sizeof(GL_Label_TypeDef));
    if (pControlObj) {
        pControlObj->ID = ID;
        GL_SetStringFieldValue(pControlObj->label, (uint8_t *)label, MAX_LABEL_LENGTH);
        pControlObj->FontSize = FontSize;
        pControlObj->Colour = Colour;
        pControlObj->Direction = direction;
        /* Create the Label object */
        Create_Label(pControlObj);
        //    pPageControlObj = (GL_PageControls_TypeDef*)malloc(sizeof(GL_PageControls_TypeDef));
        if (pPageControlObj) {
            pPageControlObj->objPTR = (void *)pControlObj;
            pPageControlObj->objType = GL_LABEL;
        } else {
            //      free(pControlObj);
            pControlObj = NULL;
        }
    }
    return pPageControlObj;
}

GL_PageControls_TypeDef * NewButton(uint16_t ID, const uint8_t * label, void (*pEventHandler)(void))
{
    GL_PageControls_TypeDef * pPageControlObj = NULL;
    GL_Button_TypeDef * pControlObj = NULL;
    //  pControlObj = (GL_Button_TypeDef *)malloc(sizeof(GL_Button_TypeDef));
    if (pControlObj) {
        pControlObj->ID = ID;
#ifndef USE_2D_OBJECTS
        pControlObj->ImageUnClickedPTR  = (uint8_t *)BtnNormal;
        pControlObj->ImageClickedPTR    = (uint8_t *)BtnPressed;
#endif
        GL_SetStringFieldValue(pControlObj->label, (uint8_t *)label, MAX_BUTTON_LABEL_LENGTH);
        pControlObj->EventHandler = pEventHandler;
        /* Create the Button object */
        Create_Button(pControlObj);
        //    pPageControlObj = (GL_PageControls_TypeDef*)malloc(sizeof(GL_PageControls_TypeDef));
        if (pPageControlObj) {
            pPageControlObj->objPTR = (void *)pControlObj;
            pPageControlObj->objType = GL_BUTTON;
        } else {
            //      free(pControlObj);
            pControlObj = NULL;
        }
    }
    return pPageControlObj;
}

GL_ErrStatus AddPageControlObj(uint16_t PosX, uint16_t PosY, GL_PageControls_TypeDef * objPTR, GL_Page_TypeDef * pagePTR)
{
    if (pagePTR->ControlCount < MAX_CTRL_X_PAGE) {
        GL_Coordinate_TypeDef objCoordinates;
        //    memset(&objCoordinates, 0x00, sizeof(GL_Coordinate_TypeDef));
        if ((PosX > 320) || (PosY > 240)) {
            return GL_ERROR;
        }
        objCoordinates.MaxX = PosX;
        objCoordinates.MinY = PosY;
        if (objPTR->objType == GL_LABEL) {
            GL_Label_TypeDef * pTmp = ((GL_Label_TypeDef *)(objPTR->objPTR));
            objPTR->SetObjVisible = SetLabelVisible;
            objPTR->ID = pTmp->ID;
            if (pTmp->Direction == GL_LEFT_VERTICAL || pTmp->Direction == GL_RIGHT_VERTICAL) {
                objCoordinates.MaxX = PosY;
                objCoordinates.MinY = PosX;
            }
        } else {
            if (objPTR->objType == GL_BUTTON || objPTR->objType == GL_SWITCH) {
                objCoordinates.MinX = PosX - BUTTON_PIECE_LENGTH;
                objCoordinates.MaxY = PosY + BUTTON_HEIGHT;
                if (objPTR->objType == GL_BUTTON) {
                    GL_Button_TypeDef * pTmp = ((GL_Button_TypeDef *)(objPTR->objPTR));
                    objPTR->SetObjVisible = SetButtonVisible;
                    objPTR->ID = pTmp->ID;
                }
            }
        }
        objPTR->objCoordinates = objCoordinates;
        pagePTR->PageControls[pagePTR->ControlCount] = objPTR;
        pagePTR->ControlCount++;
        return 0;
    } else
        return 1;
}

void GL_SetTextColor(__IO uint16_t GL_NewTextColor)
{
    GL_TextColor = GL_NewTextColor;
    TextColor = GL_TextColor;
}

void GL_SetBackColor(__IO uint16_t GL_NewBackColor)
{
    GL_BackColor = GL_NewBackColor;
    BackColor = GL_BackColor;
}

void GL_SetFont(uint8_t uFont)
{
    GL_Font = uFont;
    switch (uFont) {
        case GL_FONT_BIG:
            GL_FontWidth  = GL_FONT_BIG_WIDTH;
            GL_FontHeight = GL_FONT_BIG_HEIGHT;
            break;
        case GL_FONT_SMALL:
            GL_FontWidth  = GL_FONT_SMALL_WIDTH;
            GL_FontHeight = GL_FONT_SMALL_HEIGHT;
            break;
        default:
            break;
    } /* End switch */
}

void GL_DisplayAdjStringLine(uint16_t Line, uint16_t Column, uint8_t * ptr, uint8_t Transparent_Flag)
{
    uint32_t index = 0;
    uint32_t iMaxChar = ((LCD_Width - Column) / (GL_FontWidth - 1));
    /* Send the string character by character on lCD */
    while ((*ptr != 0) & (index < iMaxChar)) {
        /* Display one character on LCD */
        GL_LCD_DisplayChar(Line, Column, *ptr, Transparent_Flag);
        /* Decrement the column position by GL_FontWidth */
        if (*ptr == 'A' || *ptr == 'G' || *ptr == 'M' || *ptr == 'O' || *ptr == 'Q' || *ptr == 'X' || *ptr == 'm')
            Column += (GL_FontWidth);
        else
            Column += (GL_FontWidth - 1);
        /* Point on the next character */
        ptr++;
        /* Increment the character counter */
        index++;
    }
}

void GL_DrawButtonBMP(uint16_t maxX, uint16_t minX, uint16_t maxY, uint16_t minY, uint8_t * ptrBitmap)
{
    LCD_WriteReg(R3, 0x10A8);
    LCD_SetDisplayWindow(maxY, maxX, maxY - minY, maxX - minX);
    GL_DrawBMP(ptrBitmap);
    LCD_SetDisplayWindow(LCD_Height - 1, LCD_Width - 1, LCD_Height, LCD_Width);
    LCD_WriteReg(R3, 0x1028);
}

static void IOE_GPIO_Config(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* Enable IOE_I2C and IOE_I2C_PORT & Alternate Function clocks */
    RCC_APB1PeriphClockCmd(IOE_I2C_CLK, ENABLE);
    RCC_APB2PeriphClockCmd(IOE_I2C_SCL_GPIO_CLK | IOE_I2C_SDA_GPIO_CLK | IOE_IT_GPIO_CLK
                           | RCC_APB2Periph_AFIO, ENABLE);
    /* Reset IOE_I2C IP */
    RCC_APB1PeriphResetCmd(IOE_I2C_CLK, ENABLE);
    /* Release reset signal of IOE_I2C IP */
    RCC_APB1PeriphResetCmd(IOE_I2C_CLK, DISABLE);
    /* IOE_I2C SCL and SDA pins configuration */
    GPIO_InitStructure.GPIO_Pin = IOE_I2C_SCL_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_10MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_OD;
    GPIO_Init(IOE_I2C_SCL_GPIO_PORT, &GPIO_InitStructure);
    /* IOE_I2C SCL and SDA pins configuration */
    GPIO_InitStructure.GPIO_Pin = IOE_I2C_SDA_PIN;
    GPIO_Init(IOE_I2C_SDA_GPIO_PORT, &GPIO_InitStructure);
    /* Set EXTI pin as Input PullUp - IO_Expander_INT */
    GPIO_InitStructure.GPIO_Pin = IOE_IT_PIN;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_IPU;
    GPIO_Init(IOE_IT_GPIO_PORT, &GPIO_InitStructure);
    /* Connect IO Expander IT line to EXTI line */
    GPIO_EXTILineConfig(IOE_IT_EXTI_PORT_SOURCE, IOE_IT_EXTI_PIN_SOURCE);
}


/**
  * @brief  Configure the I2C Peripheral used to communicate with IO_Expanders.
  * @param  None
  * @retval None
  */
static void IOE_I2C_Config(void)
{
    I2C_InitTypeDef I2C_InitStructure;
    /* IOE_I2C configuration */
    I2C_InitStructure.I2C_Mode = I2C_Mode_I2C;
    I2C_InitStructure.I2C_DutyCycle = I2C_DutyCycle_2;
    I2C_InitStructure.I2C_OwnAddress1 = 0x00;
    I2C_InitStructure.I2C_Ack = I2C_Ack_Enable;
    I2C_InitStructure.I2C_AcknowledgedAddress = I2C_AcknowledgedAddress_7bit;
    I2C_InitStructure.I2C_ClockSpeed = IOE_I2C_SPEED;
    I2C_Init(IOE_I2C, &I2C_InitStructure);
}

uint8_t IOE_IsOperational(uint8_t DeviceAddr)
{
    /* Return Error if the ID is not correct */
    if (IOE_ReadID(DeviceAddr) != (uint16_t)STMPE811_ID) {
        /* Check if a Timeout occured */
        if (IOE_TimeOut == 0) {
            return (IOE_TimeoutUserCallback());
        } else {
            return IOE_FAILURE; /* ID is not Correct */
        }
    } else {
        return IOE_OK; /* ID is correct */
    }
}

/**
  * @brief  Resets the IO Expander by Software (SYS_CTRL1, RESET bit).
  * @param  DeviceAddr: The address of the IOExpander, could be : IOE_1_ADDR
  *         or IOE_2_ADDR.
  * @retval IOE_OK: if all initializations are OK. Other value if error.
  */
uint8_t IOE_Reset(uint8_t DeviceAddr)
{
    /* Power Down the IO_Expander */
    I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_SYS_CTRL1, 0x02);
    /* wait for a delay to insure registers erasing */
    GL_Delay(2);
    /* Power On the Codec after the power off => all registers are reinitialized*/
    I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_SYS_CTRL1, 0x00);
    /* If all OK return IOE_OK */
    return IOE_OK;
}

uint16_t IOE_ReadID(uint8_t DeviceAddr)
{
    uint16_t tmp = 0;
    /* Read device ID  */
    tmp = I2C_ReadDeviceRegister(DeviceAddr, 0);
    tmp = (uint32_t)(tmp << 8);
    tmp |= (uint32_t)I2C_ReadDeviceRegister(DeviceAddr, 1);
    /* Return the ID */
    return (uint16_t)tmp;
}

/**
  * @brief  Configures the selcted IO Expander functionalities.
  * @param  DeviceAddr: The address of the IOExpander, could be : IOE_1_ADDR
  *         or IOE_2_ADDR.
  * @param  IOE_TEMPSENS_FCT: the functions to be configured. could be any
  *         combination of the following values:
  *   @arg  IOE_IO_FCT : IO function
  *   @arg  IOE_TS_FCT : Touch Screen function
  *   @arg  IOE_ADC_FCT : ADC function
  *   @arg  IOE_TEMPSENS_FCT : Tempreature Sensor function
  * @retval IOE_OK: if all initializations are OK. Other value if error.
  */
uint8_t IOE_FnctCmd(uint8_t DeviceAddr, uint8_t Fct, FunctionalState NewState)
{
    uint8_t tmp = 0;
    /* Get the register value */
    tmp = I2C_ReadDeviceRegister(DeviceAddr, IOE_REG_SYS_CTRL2);
    if (NewState != DISABLE) {
        /* Set the Functionalities to be Enabled */
        tmp &= ~(uint8_t)Fct;
    } else {
        /* Set the Functionalities to be Disabled */
        tmp |= (uint8_t)Fct;
    }
    /* Set the register value */
    I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_SYS_CTRL2, tmp);
    /* If all OK return IOE_OK */
    return IOE_OK;
}

/**
  * @brief  Configures the selected pin direction (to be an input or an output)
  * @param  DeviceAddr: The address of the IOExpander, could be : IOE_1_ADDR
  *         or IOE_2_ADDR.
  * @param  IO_Pin: IO_Pin_x: Where x can be from 0 to 7.
  * @param  Direction: could be Direction_IN or Direction_OUT.
  * @retval IOE_OK: if all initializations are OK. Other value if error.
  */
uint8_t IOE_IOPinConfig(uint8_t DeviceAddr, uint8_t IO_Pin, uint8_t Direction)
{
    uint8_t tmp = 0;
    /* Get all the Pins direction */
    tmp = I2C_ReadDeviceRegister(DeviceAddr, IOE_REG_GPIO_DIR);
    if (Direction != Direction_IN) {
        tmp |= (uint8_t)IO_Pin;
    } else {
        tmp &= ~(uint8_t)IO_Pin;
    }
    /* Write the register new value */
    I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_GPIO_DIR, tmp);
    /* If all OK return IOE_OK */
    return IOE_OK;
}

uint8_t IOE_TS_Config()
{
    uint8_t tmp = 0;
    /* Enable TSC Fct: already done in IOE_Config */
    tmp = I2C_ReadDeviceRegister(IOE_1_ADDR, IOE_REG_SYS_CTRL2);
    tmp &= ~(uint32_t)(IOE_TS_FCT | IOE_ADC_FCT);
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_SYS_CTRL2, tmp);
    /* Enable the TSC gloabl interrupts */
    tmp = I2C_ReadDeviceRegister(IOE_1_ADDR, IOE_REG_INT_EN);
    tmp |= (uint32_t)(IOE_GIT_TOUCH | IOE_GIT_FTH | IOE_GIT_FOV);
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_INT_EN, tmp);
    /* Select Sample Time, bit number and ADC Reference */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_ADC_CTRL1, 0x49);
    /* Wait for ~20 ms */
    GL_Delay(2);
    /* Select the ADC clock speed: 3.25 MHz */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_ADC_CTRL2, 0x01);
    /* Select TSC pins in non default mode */
    tmp = I2C_ReadDeviceRegister(IOE_1_ADDR, IOE_REG_GPIO_AF);
    tmp &= ~(uint8_t)TOUCH_IO_ALL;
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_GPIO_AF, tmp);
    /* Select 2 nF filter capacitor */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_TSC_CFG, 0x9A);
    /* Select single point reading  */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_FIFO_TH, 0x01);
    /* Write 0x01 to clear the FIFO memory content. */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_FIFO_STA, 0x01);
    /* Write 0x00 to put the FIFO back into operation mode  */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_FIFO_STA, 0x00);
    /* set the data format for Z value: 7 fractional part and 1 whole part */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_TSC_FRACT_XYZ, 0x01);
    /* set the driving capability of the device for TSC pins: 50mA */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_TSC_I_DRIVE, 0x01);
    /* Use no tracking index, touchscreen controller operation mode (XYZ) and
       enable the TSC */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_TSC_CTRL, 0x01);
    /*  Clear all the status pending bits */
    I2C_WriteDeviceRegister(IOE_1_ADDR, IOE_REG_INT_STA, 0xFF);
    /* Initialize the TS structure to their default values */
    TS_State.TouchDetected = TS_State.X = TS_State.Y = TS_State.Z = 0;
    /* All configuration done */
    return IOE_OK;
}

void LCD_nCS_StartByte(uint8_t Start_Byte)
{
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_RESET);
    SPI_I2S_SendData(LCD_SPI, Start_Byte);
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
}

void LCD_CtrlLinesConfig(void)
{
    GPIO_InitTypeDef GPIO_InitStructure;
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(LCD_NCS_GPIO_CLK, ENABLE);
    /* Configure NCS in Output Push-Pull mode */
    GPIO_InitStructure.GPIO_Pin = LCD_NCS_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_Out_PP;
    GPIO_Init(LCD_NCS_GPIO_PORT, &GPIO_InitStructure);
}

/**
  * @brief  Sets or reset LCD control lines.
  * @param  GPIOx: where x can be B or D to select the GPIO peripheral.
  * @param  CtrlPins: the Control line. This parameter can be:
  *     @arg LCD_NCS_PIN: Chip Select pin
  * @param  BitVal: specifies the value to be written to the selected bit.
  *   This parameter can be:
  *     @arg Bit_RESET: to clear the port pin
  *     @arg Bit_SET: to set the port pin
  * @retval None
  */
void LCD_CtrlLinesWrite(GPIO_TypeDef * GPIOx, uint16_t CtrlPins, BitAction BitVal)
{
    /* Set or Reset the control line */
    GPIO_WriteBit(GPIOx, CtrlPins, BitVal);
}


/**
  * @brief  Configures the LCD_SPI interface.
  * @param  None
  * @retval None
  */
void LCD_SPIConfig(void)
{
    SPI_InitTypeDef    SPI_InitStructure;
    GPIO_InitTypeDef   GPIO_InitStructure;
    /* Enable GPIO clock */
    RCC_APB2PeriphClockCmd(LCD_SPI_SCK_GPIO_CLK | LCD_SPI_MISO_GPIO_CLK | LCD_SPI_MOSI_GPIO_CLK
                           | RCC_APB2Periph_AFIO, ENABLE);
    GPIO_PinRemapConfig(GPIO_Remap_SPI3, ENABLE);
    /* Enable SPI clock  */
    RCC_APB1PeriphClockCmd(LCD_SPI_CLK, ENABLE);
    /* Configure SPI pins: SCK, MISO and MOSI */
    GPIO_InitStructure.GPIO_Pin = LCD_SPI_SCK_PIN;
    GPIO_InitStructure.GPIO_Speed = GPIO_Speed_50MHz;
    GPIO_InitStructure.GPIO_Mode = GPIO_Mode_AF_PP;
    GPIO_Init(LCD_SPI_SCK_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LCD_SPI_MISO_PIN;
    GPIO_Init(LCD_SPI_MISO_GPIO_PORT, &GPIO_InitStructure);
    GPIO_InitStructure.GPIO_Pin = LCD_SPI_MOSI_PIN;
    GPIO_Init(LCD_SPI_MOSI_GPIO_PORT, &GPIO_InitStructure);
    SPI_I2S_DeInit(LCD_SPI);
    /* SPI Config */
    SPI_InitStructure.SPI_Direction = SPI_Direction_2Lines_FullDuplex;
    SPI_InitStructure.SPI_Mode = SPI_Mode_Master;
    SPI_InitStructure.SPI_DataSize = SPI_DataSize_8b;
    SPI_InitStructure.SPI_CPOL = SPI_CPOL_High;
    SPI_InitStructure.SPI_CPHA = SPI_CPHA_2Edge;
    SPI_InitStructure.SPI_NSS = SPI_NSS_Soft;
    SPI_InitStructure.SPI_BaudRatePrescaler = SPI_BaudRatePrescaler_2;
    SPI_InitStructure.SPI_FirstBit = SPI_FirstBit_MSB;
    SPI_Init(LCD_SPI, &SPI_InitStructure);
    /* SPI enable */
    SPI_Cmd(LCD_SPI, ENABLE);
}

/**
  * @brief  Displays a pixel.
  * @param  x: pixel x.
  * @param  y: pixel y.
  * @retval None
  */
static void PutPixel(int16_t x, int16_t y)
{
    if (x < 0 || x > 239 || y < 0 || y > 319) {
        return;
    }
    LCD_DrawLine(x, y, 1, LCD_DIR_HORIZONTAL);
}

void TS_Calibration(void)
{
    uint32_t coordinate_X1a = 0, coordinate_X2a = 0, coordinate_X3a = 0, coordinate_X4a = 0, coordinate_X5a = 0;
    uint32_t coordinate_Y1a = 0, coordinate_Y2a = 0, coordinate_Y3a = 0, coordinate_Y4a = 0, coordinate_Y5a = 0;
    uint32_t coordinate_X1b = 0, coordinate_X2b = 0, coordinate_X3b = 0, coordinate_X4b = 0, coordinate_X5b = 0;
    uint32_t coordinate_Y1b = 0, coordinate_Y2b = 0, coordinate_Y3b = 0, coordinate_Y4b = 0, coordinate_Y5b = 0;
    uint32_t coordinate_X1 = 0, coordinate_X2 = 0, coordinate_X3 = 0, coordinate_X4 = 0, coordinate_X5 = 0;
    uint32_t coordinate_Y1 = 0, coordinate_Y2 = 0, coordinate_Y3 = 0, coordinate_Y4 = 0, coordinate_Y5 = 0;
    uint16_t Xd1 = (LCD_Width / 2), Xd2 = 1 * (LCD_Width / 5), Xd3 = 4 * (LCD_Width / 5), Xd4 = 4 * (LCD_Width / 5), Xd5 = 1 * (LCD_Width / 5);
    uint16_t Yd1 = (LCD_Height / 2), Yd2 = 1 * (LCD_Height / 5), Yd3 = 1 * (LCD_Height / 5), Yd4 = 4 * (LCD_Height / 5), Yd5 = 4 * (LCD_Height / 5);
    double A = 0.0, B = 0.0, C = 0.0, D = 0.0, E = 0.0, F = 0.0;
    double d = 0.0, dx1 = 0.0, dx2 = 0.0, dx3 = 0.0, dy1 = 0.0, dy2 = 0.0, dy3 = 0.0;
    uint32_t X2_1 = 0, X2_2 = 0, X2_3 = 0, X2_4 = 0, X2_5 = 0;
    uint32_t Y2_1 = 0, Y2_2 = 0, Y2_3 = 0, Y2_4 = 0, Y2_5 = 0;
    uint32_t XxY_1 = 0, XxY_2 = 0, XxY_3 = 0, XxY_4 = 0, XxY_5 = 0;
    uint32_t XxXd_1 = 0, XxXd_2 = 0, XxXd_3 = 0, XxXd_4 = 0, XxXd_5 = 0;
    uint32_t YxXd_1 = 0, YxXd_2 = 0, YxXd_3 = 0, YxXd_4 = 0, YxXd_5 = 0;
    uint32_t XxYd_1 = 0, XxYd_2 = 0, XxYd_3 = 0, XxYd_4 = 0, XxYd_5 = 0;
    uint32_t YxYd_1 = 0, YxYd_2 = 0, YxYd_3 = 0, YxYd_4 = 0, YxYd_5 = 0;
    uint32_t alfa = 0, beta = 0, chi = 0, Kx = 0, Ky = 0, Lx = 0, Ly = 0;
    uint16_t epsilon = 0, fi = 0, Mx = 0, My = 0;
    GL_SetBackColor(0xFFFF);
    GL_SetTextColor(0x0000);
    LCD_Clear();
    GL_DisplayAdjStringLine(3 * (LCD_Height / 7), 25, "Run Calibration.", 0);
    GL_Delay(40);
    GL_DisplayAdjStringLine(3 * (LCD_Height / 7), 25, "Run Calibration..", 0);
    GL_Delay(40);
    GL_DisplayAdjStringLine(3 * (LCD_Height / 7), 25, "Run Calibration...", 0);
    GL_Delay(30);
    touch_done = 0;
    LCD_Clear();
    GL_Cross((LCD_Height / 2), (LCD_Width / 2));   /* Absolute Central Point */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X1a = TSC_Value_X;
    coordinate_Y1a = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross(1 * (LCD_Height / 5), 1 * (LCD_Width / 5)); /* Nord-East Corner point */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X2a = TSC_Value_X;
    coordinate_Y2a = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross(1 * (LCD_Height / 5), 4 * (LCD_Width / 5)); /* Nord-West Corner */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X3a = TSC_Value_X;
    coordinate_Y3a = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross(4 * (LCD_Height / 5), 4 * (LCD_Width / 5)); /* Sud-West Corner */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X4a = TSC_Value_X;
    coordinate_Y4a = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross(4 * (LCD_Height / 5), 1 * (LCD_Width / 5)); /* Sud-East Corner point */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X5a = TSC_Value_X;
    coordinate_Y5a = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross((LCD_Height / 2), (LCD_Width / 2));   /* Absolute Central Point */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X1b = TSC_Value_X;
    coordinate_Y1b = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross(1 * (LCD_Height / 5), 1 * (LCD_Width / 5)); /* Nord-East Corner point */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X2b = TSC_Value_X;
    coordinate_Y2b = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross(1 * (LCD_Height / 5), 4 * (LCD_Width / 5)); /* Nord-West Corner */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X3b = TSC_Value_X;
    coordinate_Y3b = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross(4 * (LCD_Height / 5), 4 * (LCD_Width / 5)); /* Sud-West Corner */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X4b = TSC_Value_X;
    coordinate_Y4b = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    LCD_Clear();
    GL_Cross(4 * (LCD_Height / 5), 1 * (LCD_Width / 5)); /* Sud-East Corner point */
    while (touch_done == 0) {
        TSC_Read();
    }
    coordinate_X5b = TSC_Value_X;
    coordinate_Y5b = TSC_Value_Y;
    GL_Delay(90); /* This is to catch only one touch event */
    TSC_Init();
    touch_done = 0;
    /* Average between X and Y coupled Touchscreen values */
    coordinate_X1 = (coordinate_X1a + coordinate_X1b) / 2;
    coordinate_X2 = (coordinate_X2a + coordinate_X2b) / 2;
    coordinate_X3 = (coordinate_X3a + coordinate_X3b) / 2;
    coordinate_X4 = (coordinate_X4a + coordinate_X4b) / 2;
    coordinate_X5 = (coordinate_X5a + coordinate_X5b) / 2;
    coordinate_Y1 = (coordinate_Y1a + coordinate_Y1b) / 2;
    coordinate_Y2 = (coordinate_Y2a + coordinate_Y2b) / 2;
    coordinate_Y3 = (coordinate_Y3a + coordinate_Y3b) / 2;
    coordinate_Y4 = (coordinate_Y4a + coordinate_Y4b) / 2;
    coordinate_Y5 = (coordinate_Y5a + coordinate_Y5b) / 2;
    X2_1 = (coordinate_X1 * coordinate_X1);
    X2_2 = (coordinate_X2 * coordinate_X2);
    X2_3 = (coordinate_X3 * coordinate_X3);
    X2_4 = (coordinate_X4 * coordinate_X4);
    X2_5 = (coordinate_X5 * coordinate_X5);
    Y2_1 = (coordinate_Y1 * coordinate_Y1);
    Y2_2 = (coordinate_Y2 * coordinate_Y2);
    Y2_3 = (coordinate_Y3 * coordinate_Y3);
    Y2_4 = (coordinate_Y4 * coordinate_Y4);
    Y2_5 = (coordinate_Y5 * coordinate_Y5);
    XxY_1 = (coordinate_X1 * coordinate_Y1);
    XxY_2 = (coordinate_X2 * coordinate_Y2);
    XxY_3 = (coordinate_X3 * coordinate_Y3);
    XxY_4 = (coordinate_X4 * coordinate_Y4);
    XxY_5 = (coordinate_X5 * coordinate_Y5);
    XxXd_1 = (coordinate_X1 * Xd1);
    XxXd_2 = (coordinate_X2 * Xd2);
    XxXd_3 = (coordinate_X3 * Xd3);
    XxXd_4 = (coordinate_X4 * Xd4);
    XxXd_5 = (coordinate_X5 * Xd5);
    YxXd_1 = (coordinate_Y1 * Xd1);
    YxXd_2 = (coordinate_Y2 * Xd2);
    YxXd_3 = (coordinate_Y3 * Xd3);
    YxXd_4 = (coordinate_Y4 * Xd4);
    YxXd_5 = (coordinate_Y5 * Xd5);
    XxYd_1 = (coordinate_X1 * Yd1);
    XxYd_2 = (coordinate_X2 * Yd2);
    XxYd_3 = (coordinate_X3 * Yd3);
    XxYd_4 = (coordinate_X4 * Yd4);
    XxYd_5 = (coordinate_X5 * Yd5);
    YxYd_1 = (coordinate_Y1 * Yd1);
    YxYd_2 = (coordinate_Y2 * Yd2);
    YxYd_3 = (coordinate_Y3 * Yd3);
    YxYd_4 = (coordinate_Y4 * Yd4);
    YxYd_5 = (coordinate_Y5 * Yd5);
    alfa = X2_1 + X2_2 + X2_3 + X2_4 + X2_5;
    beta = Y2_1 + Y2_2 + Y2_3 + Y2_4 + Y2_5;
    chi = XxY_1 + XxY_2 + XxY_3 + XxY_4 + XxY_5;
    epsilon = coordinate_X1 + coordinate_X2 + coordinate_X3 + coordinate_X4 + coordinate_X5;
    fi = coordinate_Y1 + coordinate_Y2 + coordinate_Y3 + coordinate_Y4 + coordinate_Y5;
    Kx = XxXd_1 + XxXd_2 + XxXd_3 + XxXd_4 + XxXd_5;
    Ky = XxYd_1 + XxYd_2 + XxYd_3 + XxYd_4 + XxYd_5;
    Lx = YxXd_1 + YxXd_2 + YxXd_3 + YxXd_4 + YxXd_5;
    Ly = YxYd_1 + YxYd_2 + YxYd_3 + YxYd_4 + YxYd_5;
    Mx = Xd1 + Xd2 + Xd3 + Xd4 + Xd5;
    My = Yd1 + Yd2 + Yd3 + Yd4 + Yd5;
    d = 5 * (((double)alfa * beta) - ((double)chi * chi)) + 2 * ((double)chi * epsilon * fi) - ((double)alfa * fi * fi) - ((double)beta * epsilon * epsilon);
    dx1 = 5 * (((double)Kx * beta) - ((double)Lx * chi)) + ((double)fi * (((double)Lx * epsilon) - ((double)Kx * fi))) + ((double)Mx * (((double)chi * fi) - ((double)beta * epsilon)));
    dx2 = 5 * (((double)Lx * alfa) - ((double)Kx * chi)) + ((double)epsilon * (((double)Kx * fi) - ((double)Lx * epsilon))) + ((double)Mx * (((double)chi * epsilon) - ((double)alfa * fi)));
    dx3 = ((double)Kx * (((double)chi * fi) - ((double)beta * epsilon))) + ((double)Lx * (((double)chi * epsilon) - ((double)alfa * fi))) + ((double)Mx * (((double)alfa * beta) - ((double)chi * chi)));
    dy1 = 5 * (((double)Ky * beta) - ((double)Ly * chi)) + ((double)fi * (((double)Ly * epsilon) - ((double)Ky * fi))) + ((double)My * (((double)chi * fi) - ((double)beta * epsilon)));
    dy2 = 5 * (((double)Ly * alfa) - ((double)Ky * chi)) + ((double)epsilon * (((double)Ky * fi) - ((double)Ly * epsilon))) + ((double)My * (((double)chi * epsilon) - ((double)alfa * fi)));
    dy3 = ((double)Ky * (((double)chi * fi) - ((double)beta * epsilon))) + ((double)Ly * (((double)chi * epsilon) - ((double)alfa * fi))) + ((double)My * (((double)alfa * beta) - ((double)chi * chi)));
    A = dx1 / d;
    B = dx2 / d;
    C = dx3 / d;
    D = dy1 / d;
    E = dy2 / d;
    F = dy3 / d;
    /* To avoid computation with "double" variables A, B, C, D, E, F, we use the s32 variables
       A2, B2, C2, D2, E2, F2, multiplied for a Scale Factor equal to 100000 to retain the precision*/
    A2 = (int32_t)(A * RESCALE_FACTOR);
    B2 = (int32_t)(B * RESCALE_FACTOR);
    C2 = (int32_t)(C * RESCALE_FACTOR);
    D2 = (int32_t)(D * RESCALE_FACTOR);
    E2 = (int32_t)(E * RESCALE_FACTOR);
    F2 = (int32_t)(F * RESCALE_FACTOR);
    LCD_Clear();
    GL_DisplayAdjStringLine(3 * (LCD_Height / 7), LCD_Width / 11, "Calibration done!", 0);
    GL_Delay(25); /* Now show HOME Menu */
    LCD_Clear();
    calibration_done = 1;
    TS_SaveCalibrationVariables();
}

/**
  * @brief  Save TouchScreen 5 points calibration coordinates
  * @param  None
  * @retval None
  */
static void TS_SaveCalibrationVariables(void)
{
    int32_t Data[7];
    /********************* FLASH PROGRAMMING FOR SAVING "calibration_done" variable **********************/
    TSC_FlashStatus = TSC_FLASH_COMPLETE;
    TSC_MemoryProgramStatus = PASSED;
    FlashFree_Address = CalibrationAddr;
    Data[0] = calibration_done;
    Data[1] = A2;
    Data[2] = B2;
    Data[3] = C2;
    Data[4] = D2;
    Data[5] = E2;
    Data[6] = F2;
    TSC_FlashStatus = TSC_WriteDataToNVM(FlashFree_Address, Data, sizeof(Data));
    /* Reading Calibration Flag from the Flash Memory */
    calibration_done = (*(__IO uint32_t *) FlashFree_Address) & 0x000000FF;
}

uint8_t IOE_IOAFConfig(uint8_t DeviceAddr, uint8_t IO_Pin, FunctionalState NewState)
{
    uint8_t tmp = 0;
    /* Get the current state of the GPIO_AF register */
    tmp = I2C_ReadDeviceRegister(DeviceAddr, IOE_REG_GPIO_AF);
    if (NewState != DISABLE) {
        /* Enable the selected pins alternate function */
        tmp |= (uint8_t)IO_Pin;
    } else {
        /* Disable the selected pins alternate function */
        tmp &= ~(uint8_t)IO_Pin;
    }
    /* Write back the new valu in GPIO_AF register */
    I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_GPIO_AF, tmp);
    /* If all OK return IOE_OK */
    return IOE_OK;
}

/**
  * @brief  Configures the Edge for which a transition is detectable for the
  *         the selected pin.
  * @param  DeviceAddr: The address of the IOExpander, could be : IOE_1_ADDR
  *         or IOE_2_ADDR.
  * @param  IO_Pin: IO_Pin_x, Where x can be from 0 to 7.
  * @param  Edge: The edge which will be detected. This parameter can be one or a
  *         a combination of follwing values: EDGE_FALLING and EDGE_RISING .
  * @retval IOE_OK: if all initializations are OK. Other value if error.
  */
uint8_t IOE_IOEdgeConfig(uint8_t DeviceAddr, uint8_t IO_Pin, uint8_t Edge)
{
    uint8_t tmp1 = 0, tmp2 = 0;
    /* Get the registers values */
    tmp1 = I2C_ReadDeviceRegister(DeviceAddr, IOE_REG_GPIO_FE);
    tmp2 = I2C_ReadDeviceRegister(DeviceAddr, IOE_REG_GPIO_RE);
    /* Disable the Falling Edge */
    tmp1 &= ~(uint8_t)IO_Pin;
    /* Disable the Falling Edge */
    tmp2 &= ~(uint8_t)IO_Pin;
    /* Enable the Falling edge if selected */
    if (Edge & EDGE_FALLING) {
        tmp1 |= (uint8_t)IO_Pin;
    }
    /* Enable the Rising edge if selected */
    if (Edge & EDGE_RISING) {
        tmp2 |= (uint8_t)IO_Pin;
    }
    /* Write back the registers values */
    I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_GPIO_FE, tmp1);
    I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_GPIO_RE, tmp2);
    /* if OK return 0 */
    return IOE_OK;
}

uint8_t IOE_WriteIOPin(uint8_t IO_Pin, IOE_BitValue_TypeDef BitVal)
{
    uint8_t DeviceAddr = 0;
    /* Get the IO expander Address according to which pin is to be controlled */
    if (IO_Pin & IO1_OUT_ALL_PINS) {
        DeviceAddr = IOE_1_ADDR;
    } else if (IO_Pin & IO2_OUT_ALL_PINS) {
        DeviceAddr = IOE_2_ADDR;
    } else {
        return PARAM_ERROR;
    }
    /* Apply the bit value to the selected pin */
    if (BitVal == BitReset) {
        /* Set the register */
        I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_GPIO_CLR_PIN, IO_Pin);
    } else {
        /* Set the register */
        I2C_WriteDeviceRegister(DeviceAddr, IOE_REG_GPIO_SET_PIN, IO_Pin);
    }
    return IOE_OK;
}

static GL_ObjDimensions_TypeDef GetObjSize(GL_PageControls_TypeDef * pPageControl)
{
    GL_ObjDimensions_TypeDef dimensions;
    //  memset(&dimensions, 0x00, sizeof(GL_ObjDimensions_TypeDef));
    if (pPageControl->objType == GL_BUTTON) {
        GL_Button_TypeDef * pTmp;
        pTmp = (GL_Button_TypeDef *)(pPageControl->objPTR);
        dimensions.Length = BUTTON_SLICE_LENGTH * 2 + (BUTTON_SLICE_LENGTH * (p_strlen((pTmp->label)) - 1));
        dimensions.Height  = BUTTON_HEIGHT;
    }
    return dimensions;
}

uint8_t CompareCoordinates(uint16_t u16_XMax, uint16_t u16_XMin, uint16_t u16_YMax, uint16_t u16_YMin)
{
    if ((u32_TSXCoordinate <= u16_XMin) || (u32_TSXCoordinate >= u16_XMax - 5)) {
        ValidTouchDetected = 0;
        return 0;
    } else if ((u32_TSYCoordinate < u16_YMin) || (u32_TSYCoordinate > u16_YMax)) {
        ValidTouchDetected = 0;
        return 0;
    } else {
        ValidTouchDetected = 1;
        return 1;
    }
}

static void CallPreEvents(GL_PageControls_TypeDef * pControl)
{
    uint32_t index = 0;
    uint32_t active_index = 0; /* needed for ComboBox */
    uint8_t * ptrBitmap = 0x00;
    void * pTmp;
    switch (pControl->objType) {
        case GL_BUTTON:
            pTmp = (GL_Button_TypeDef *)(pControl->objPTR);
            ((GL_Button_TypeDef *)(pTmp))->isObjectTouched = 1;
            pControl->SetObjVisible(pControl, pControl->objCoordinates);
            while (index < 800000)
                index++;
            ((GL_Button_TypeDef *)(pTmp))->isObjectTouched = 0;
            pControl->SetObjVisible(pControl, pControl->objCoordinates);
            break;
        default:
            break;
    }
}

static void CallEvent(GL_PageControls_TypeDef * pControl)
{
    uint32_t index = 0;
    void * pTmp;
    switch (pControl->objType) {
        case GL_BUTTON:
            pTmp = (GL_Button_TypeDef *)(pControl->objPTR);
            ((GL_Button_TypeDef *)pTmp)->EventHandler();
            break;
        default:
            break;
    }
}

uint8_t I2C_ReadDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr)
{
    uint8_t IOE_BufferRX[2] = {0x00, 0x00};
    /* Configure DMA Peripheral */
    IOE_DMA_Config(IOE_DMA_RX, (uint8_t *)IOE_BufferRX);
    /* Enable DMA NACK automatic generation */
    I2C_DMALastTransferCmd(IOE_I2C, ENABLE);
    /* Enable the I2C peripheral */
    I2C_GenerateSTART(IOE_I2C, ENABLE);
    /* Test on SB Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_SB)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send device address for write */
    I2C_Send7bitAddress(IOE_I2C, DeviceAddr, I2C_Direction_Transmitter);
    /* Test on ADDR Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_CheckEvent(IOE_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send the device's internal address to write to */
    I2C_SendData(IOE_I2C, RegisterAddr);
    /* Test on TXE FLag (data dent) */
    IOE_TimeOut = TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_BTF))) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send START condition a second time */
    I2C_GenerateSTART(IOE_I2C, ENABLE);
    /* Test on SB Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_SB)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send IOExpander address for read */
    I2C_Send7bitAddress(IOE_I2C, DeviceAddr, I2C_Direction_Receiver);
    /* Test on ADDR Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_CheckEvent(IOE_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Enable I2C DMA request */
    I2C_DMACmd(IOE_I2C, ENABLE);
    /* Enable DMA RX Channel */
    DMA_Cmd(IOE_DMA_RX_CHANNEL, ENABLE);
    /* Wait until DMA Transfer Complete */
    IOE_TimeOut = 2 * TIMEOUT_MAX;
    while (!DMA_GetFlagStatus(IOE_DMA_RX_TCFLAG)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send STOP Condition */
    I2C_GenerateSTOP(IOE_I2C, ENABLE);
    /* Disable DMA RX Channel */
    DMA_Cmd(IOE_DMA_RX_CHANNEL, DISABLE);
    /* Disable I2C DMA request */
    I2C_DMACmd(IOE_I2C, DISABLE);
    /* Clear DMA RX Transfer Complete Flag */
    DMA_ClearFlag(IOE_DMA_RX_TCFLAG);
    /* return a pointer to the IOE_Buffer */
    return (uint8_t)IOE_BufferRX[0];
}


/**
  * @brief  Reads a buffer of 2 bytes from the device registers.
  * @param  DeviceAddr: The address of the device, could be : IOE_1_ADDR
  *         or IOE_2_ADDR.
  * @param  RegisterAddr: The target register adress (between 00x and 0x24)
  * @retval A pointer to the buffer containing the two returned bytes (in halfword).
  */
uint16_t I2C_ReadDataBuffer(uint8_t DeviceAddr, uint32_t RegisterAddr)
{
    uint8_t tmp = 0;
    uint8_t IOE_BufferRX[2] = {0x00, 0x00};
    /* Configure DMA Peripheral */
    IOE_DMA_Config(IOE_DMA_RX, (uint8_t *)IOE_BufferRX);
    /* Enable DMA NACK automatic generation */
    I2C_DMALastTransferCmd(IOE_I2C, ENABLE);
    /* Enable the I2C peripheral */
    I2C_GenerateSTART(IOE_I2C, ENABLE);
    /* Test on SB Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_SB)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send device address for write */
    I2C_Send7bitAddress(IOE_I2C, DeviceAddr, I2C_Direction_Transmitter);
    /* Test on ADDR Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_CheckEvent(IOE_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send the device's internal address to write to */
    I2C_SendData(IOE_I2C, RegisterAddr);
    /* Test on TXE FLag (data dent) */
    IOE_TimeOut = TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_BTF))) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send START condition a second time */
    I2C_GenerateSTART(IOE_I2C, ENABLE);
    /* Test on SB Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_SB)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send IOExpander address for read */
    I2C_Send7bitAddress(IOE_I2C, DeviceAddr, I2C_Direction_Receiver);
    /* Test on ADDR Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_CheckEvent(IOE_I2C, I2C_EVENT_MASTER_RECEIVER_MODE_SELECTED)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Enable I2C DMA request */
    I2C_DMACmd(IOE_I2C, ENABLE);
    /* Enable DMA RX Channel */
    DMA_Cmd(IOE_DMA_RX_CHANNEL, ENABLE);
    /* Wait until DMA Transfer Complete */
    IOE_TimeOut = 2 * TIMEOUT_MAX;
    while (!DMA_GetFlagStatus(IOE_DMA_RX_TCFLAG)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Send STOP Condition */
    I2C_GenerateSTOP(IOE_I2C, ENABLE);
    /* Disable DMA RX Channel */
    DMA_Cmd(IOE_DMA_RX_CHANNEL, DISABLE);
    /* Disable I2C DMA request */
    I2C_DMACmd(IOE_I2C, DISABLE);
    /* Clear DMA RX Transfer Complete Flag */
    DMA_ClearFlag(IOE_DMA_RX_TCFLAG);
    /* Reorganize received data */
    tmp = IOE_BufferRX[0];
    IOE_BufferRX[0] = IOE_BufferRX[1];
    IOE_BufferRX[1] = tmp;
    /* return a pointer to the IOE_Buffer */
    return *(uint16_t *)IOE_BufferRX;
}

uint8_t I2C_WriteDeviceRegister(uint8_t DeviceAddr, uint8_t RegisterAddr, uint8_t RegisterValue)
{
    uint32_t read_verif = 0;
    uint8_t IOE_BufferTX = 0;
    /* Get Value to be written */
    IOE_BufferTX = RegisterValue;
    /* Configure DMA Peripheral */
    IOE_DMA_Config(IOE_DMA_TX, (uint8_t *)(&IOE_BufferTX));
    /* Enable the I2C peripheral */
    I2C_GenerateSTART(IOE_I2C, ENABLE);
    /* Test on SB Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_SB) == RESET) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Transmit the slave address and enable writing operation */
    I2C_Send7bitAddress(IOE_I2C, DeviceAddr, I2C_Direction_Transmitter);
    /* Test on ADDR Flag */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!I2C_CheckEvent(IOE_I2C, I2C_EVENT_MASTER_TRANSMITTER_MODE_SELECTED)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Transmit the first address for r/w operations */
    I2C_SendData(IOE_I2C, RegisterAddr);
    /* Test on TXE FLag (data dent) */
    IOE_TimeOut = TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_TXE)) && (!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_BTF))) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Enable I2C DMA request */
    I2C_DMACmd(IOE_I2C, ENABLE);
    /* Enable DMA TX Channel */
    DMA_Cmd(IOE_DMA_TX_CHANNEL, ENABLE);
    /* Wait until DMA Transfer Complete */
    IOE_TimeOut = TIMEOUT_MAX;
    while (!DMA_GetFlagStatus(IOE_DMA_TX_TCFLAG)) {
        if (IOE_TimeOut-- == 0) return(IOE_TimeoutUserCallback());
    }
    /* Wait until BTF Flag is set before generating STOP */
    IOE_TimeOut = 2 * TIMEOUT_MAX;
    while ((!I2C_GetFlagStatus(IOE_I2C, I2C_FLAG_BTF))) {
    }
    /* Send STOP Condition */
    I2C_GenerateSTOP(IOE_I2C, ENABLE);
    /* Disable DMA TX Channel */
    DMA_Cmd(IOE_DMA_TX_CHANNEL, DISABLE);
    /* Disable I2C DMA request */
    I2C_DMACmd(IOE_I2C, DISABLE);
    /* Clear DMA TX Transfer Complete Flag */
    DMA_ClearFlag(IOE_DMA_TX_TCFLAG);
#ifdef VERIFY_WRITTENDATA
    /* Verify (if needed) that the loaded data is correct  */
    /* Read the just written register*/
    read_verif = I2C_ReadDeviceRegister(DeviceAddr, RegisterAddr);
    /* Load the register and verify its value  */
    if (read_verif != RegisterValue) {
        /* Control data wrongly tranfered */
        read_verif = IOE_FAILURE;
    } else {
        /* Control data correctly transfered */
        read_verif = 0;
    }
#endif
    /* Return the verifying value: 0 (Passed) or 1 (Failed) */
    return read_verif;
}

uint16_t getDisplayCoordinateX(uint16_t x_touch, uint16_t y_touch)
{
    uint16_t Xd;
    float temp;
    temp = (A2 * x_touch + B2 * y_touch + C2) / RESCALE_FACTOR;
    Xd = (uint16_t)(temp);
    if (Xd > 60000) {
        /* this to avoid negative value */
        Xd = 0;
    }
    return Xd;
}

/**
  * @brief  Returns the Display x-axis Coordinate corresponding to Touch x-axis coordinate.
  *         X and Y are inverted because the display is meant in Landscape
  * @param  x_touch: X coordinate of the Touch Panel
  * @param  y_touch: Y coordinate of the Touch Panel
  * @retval uint16_t - Yd: Coordinate X of the LCD Panel (Y-axis Coordinate)
  */
uint16_t getDisplayCoordinateY(uint16_t x_touch, uint16_t y_touch)
{
    uint16_t Yd;
    float temp;
    temp = (D2 * x_touch + E2 * y_touch + F2) / RESCALE_FACTOR;
    Yd = (uint16_t)(temp);
    if (Yd > 60000) {
        /*  this to avoid negative value */
        Yd = 0;
    }
    return Yd;
}

void LCD_PutPixel(uint16_t Xpos, uint16_t Ypos, uint16_t Color, uint8_t PixelSpec)
{
    /*Start part of put pixel for first pixel of block and for single one.
      It consists of set the cursor's position and GRAM prepare sequence.*/
    if ((PixelSpec & (FirstPixel | SinglePixel)) != GL_RESET) {
        LCD_SetCursor(Xpos, Ypos);
        LCD_WriteRAM_Prepare();
    }
    /*Write pixel's color to GRAM. Obligatory for all modes of PutPixel call.*/
    LCD_WriteRAM(Color);
    /*End part of putting pixel for last pixel of block or single pixel.
      Close usage of display by setting CS high.*/
    if ((PixelSpec & (LastPixel | SinglePixel)) != GL_RESET) {
        GL_LCD_CtrlLinesWrite(pLcdHwParam.LCD_Ctrl_Port_NCS, pLcdHwParam.LCD_Ctrl_Pin_NCS, GL_HIGH);
    }
}

static void GL_SetStringFieldValue(uint8_t * dBuf, uint8_t * sBuf, uint32_t MaxLength)
{
    uint32_t tmp_length = 0;
    tmp_length = min(p_strlen(sBuf), MaxLength - 1);
    if (tmp_length > 0) {
        p_strncpy(dBuf, sBuf, tmp_length);
        dBuf[tmp_length] = 0;
    } else {
        dBuf[0] = 0;
    }
}

static GL_ErrStatus Create_Label(GL_Label_TypeDef * pThis)
{
    if (!pThis) {
        return GL_ERROR;
    }
    pThis->Control_Visible = 1;
    return GL_OK;
}

static GL_ErrStatus Create_Button(GL_Button_TypeDef * pThis)
{
    if (!pThis) {
        return GL_ERROR;
    }
    pThis->isObjectTouched = 0;
    pThis->Control_Visible = 1;
    return GL_OK;
}

void GL_LCD_CtrlLinesWrite(GPIO_TypeDef * GPIOx, uint16_t CtrlPins, GL_SignalActionType BitVal)
{
    /* Set or Reset the control line */
    LCD_CtrlLinesWrite(GPIOx, CtrlPins, (BitAction)BitVal);
}

void GL_LCD_DisplayChar(uint16_t Line, uint16_t Column, uint8_t Ascii, uint8_t Transparent_Flag)
{
    Ascii -= 32;
    switch (GL_Font) {
        case 0x00:
            if (Transparent_Flag == 1)
                GL_LCD_DrawCharTransparent(Line, Column, &GL_Font16x24.table[Ascii * GL_FontHeight]);
            else
                GL_LCD_DrawChar(Line, Column, (&GL_Font16x24.table[Ascii * GL_FontHeight]));
            break;
        case 0x01:
            if (Transparent_Flag == 1)
                GL_LCD_DrawCharTransparent(Line, Column, &GL_Font8x12_bold.table[Ascii * GL_FontHeight]);
            else
                GL_LCD_DrawChar(Line, Column, &GL_Font8x12_bold.table[Ascii * GL_FontHeight]);
            break;
        default:
            break;
    }
}

void GL_DrawBMP(uint8_t * ptrBitmap)
{
    uint32_t uDataAddr = *(uint32_t *)(ptrBitmap + 10);
    uint32_t uBmpSize = *(uint32_t *)(ptrBitmap + 2);
    uBmpSize -= uDataAddr;
    DMA_InitTypeDef DMA_InitStructure_lcd;
    DMA_DeInit(DMA2_Channel2);
    DMA_InitStructure_lcd.DMA_PeripheralBaseAddr = &(SPI3->DR);
    DMA_InitStructure_lcd.DMA_MemoryBaseAddr = (uint32_t)((uint8_t *)ptrBitmap + uDataAddr);
    DMA_InitStructure_lcd.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure_lcd.DMA_BufferSize = uBmpSize;
    DMA_InitStructure_lcd.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure_lcd.DMA_MemoryInc = DMA_MemoryInc_Enable;
    DMA_InitStructure_lcd.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure_lcd.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure_lcd.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure_lcd.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure_lcd.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel2, &DMA_InitStructure_lcd);
    DMA_Cmd(DMA2_Channel2, ENABLE);
    /* Enable DMA Channel2 */
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
    while (!(SPI3->SR & SPI_I2S_FLAG_TXE));
    while (SPI3->SR & SPI_I2S_FLAG_BSY);
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, DISABLE);
    SPI3->SR &= !SPI_I2S_FLAG_TXE;
    GL_LCD_CtrlLinesWrite(pLcdHwParam.LCD_Ctrl_Port_NCS, pLcdHwParam.LCD_Ctrl_Pin_NCS, GL_HIGH);
}

void LCD_Clear()
{
    uint16_t white = 0xFF;
    DMA_InitTypeDef DMA_InitStructure_lcd;
    DMA_DeInit(DMA2_Channel2);
    DMA_InitStructure_lcd.DMA_PeripheralBaseAddr = &(SPI3->DR);
    DMA_InitStructure_lcd.DMA_MemoryBaseAddr = (uint32_t)(&white);
    DMA_InitStructure_lcd.DMA_DIR = DMA_DIR_PeripheralDST;
    DMA_InitStructure_lcd.DMA_BufferSize = LCD_Width * LCD_Height * 2;
    DMA_InitStructure_lcd.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    DMA_InitStructure_lcd.DMA_MemoryInc = DMA_MemoryInc_Disable;
    DMA_InitStructure_lcd.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    DMA_InitStructure_lcd.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    DMA_InitStructure_lcd.DMA_Mode = DMA_Mode_Normal;
    DMA_InitStructure_lcd.DMA_Priority = DMA_Priority_VeryHigh;
    DMA_InitStructure_lcd.DMA_M2M = DMA_M2M_Disable;
    DMA_Init(DMA2_Channel2, &DMA_InitStructure_lcd);
    DMA_Cmd(DMA2_Channel2, ENABLE);
    /* Enable DMA Channel2 */
    LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, ENABLE);
    while (!(SPI3->SR & SPI_I2S_FLAG_TXE));
    while (SPI3->SR & SPI_I2S_FLAG_BSY);
    SPI_I2S_DMACmd(SPI3, SPI_I2S_DMAReq_Tx, DISABLE);
    SPI3->SR &= !SPI_I2S_FLAG_TXE;
    GL_LCD_CtrlLinesWrite(pLcdHwParam.LCD_Ctrl_Port_NCS, pLcdHwParam.LCD_Ctrl_Pin_NCS, GL_HIGH);
}

void LCD_DrawLine(uint8_t Xpos, uint16_t Ypos, uint16_t Length, uint8_t Direction)
{
    uint32_t i = 0;
    LCD_SetCursor(Xpos, Ypos);
    if (Direction == LCD_DIR_HORIZONTAL) {
        LCD_WriteRAM_Prepare(); /* Prepare to write GRAM */
        for (i = 0; i < Length; i++) {
            LCD_WriteRAM(TextColor);
        }
        LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
    } else {
        for (i = 0; i < Length; i++) {
            LCD_WriteRAMWord(TextColor);
            Xpos++;
            LCD_SetCursor(Xpos, Ypos);
        }
    }
}

void GL_Cross(uint16_t Ypos, uint16_t Xpos)
{
    LCD_DrawLine(Ypos, Xpos - 30, 60, GL_Horizontal); /* Horizontal Line */
    LCD_DrawLine(Ypos - 30, Xpos, 60, GL_Vertical);   /* Vertical Line   */
}

TSC_FLASH_Status TSC_WriteDataToNVM(uint32_t FlashFree_Address, int32_t * Data, uint32_t Size)
{
    TSC_FLASH_Status TSC_FlashStatus = TSC_FLASH_COMPLETE;
    uint32_t words = (Size / sizeof(uint32_t)) + ((Size % sizeof(uint32_t)) ? 1 : 0);
    uint32_t index = 0;
    /* Unlock the Flash Program Erase controller */
    FLASH_Unlock();
    /* Erase Flash sectors ******************************************************/
    /* Clear All pending flags */
    FLASH_ClearFlag(FLASH_FLAG_BSY | FLASH_FLAG_EOP | FLASH_FLAG_PGERR | FLASH_FLAG_WRPRTERR);
    /* Erase Last Flash Page */
    TSC_FlashStatus = TSC_FLASH_ErasePage(FlashFree_Address);
    for (index = 0; index < words; index++) {
        /* Writing calibration parameters to the Flash Memory */
        TSC_FlashStatus = TSC_FLASH_ProgramWord(FlashFree_Address, Data[index]);
        /* Increasing Flash Memory Page Address */
        FlashFree_Address = FlashFree_Address + 4;
    }
    return TSC_FlashStatus;
}

static void IOE_DMA_Config(IOE_DMADirection_TypeDef Direction, uint8_t * buffer)
{
    DMA_InitTypeDef DMA_InitStructure;
    RCC_AHBPeriphClockCmd(IOE_DMA_CLK, ENABLE);
    /* Initialize the DMA_PeripheralBaseAddr member */
    DMA_InitStructure.DMA_PeripheralBaseAddr = IOE_I2C_DR;
    /* Initialize the DMA_MemoryBaseAddr member */
    DMA_InitStructure.DMA_MemoryBaseAddr = (uint32_t)buffer;
    /* Initialize the DMA_PeripheralInc member */
    DMA_InitStructure.DMA_PeripheralInc = DMA_PeripheralInc_Disable;
    /* Initialize the DMA_MemoryInc member */
    DMA_InitStructure.DMA_MemoryInc = DMA_MemoryInc_Enable;
    /* Initialize the DMA_PeripheralDataSize member */
    DMA_InitStructure.DMA_PeripheralDataSize = DMA_PeripheralDataSize_Byte;
    /* Initialize the DMA_MemoryDataSize member */
    DMA_InitStructure.DMA_MemoryDataSize = DMA_MemoryDataSize_Byte;
    /* Initialize the DMA_Mode member */
    DMA_InitStructure.DMA_Mode = DMA_Mode_Normal;
    /* Initialize the DMA_Priority member */
    DMA_InitStructure.DMA_Priority = DMA_Priority_VeryHigh;
    /* Initialize the DMA_M2M member */
    DMA_InitStructure.DMA_M2M = DMA_M2M_Disable;
    /* If using DMA for Reception */
    if (Direction == IOE_DMA_RX) {
        /* Initialize the DMA_DIR member */
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralSRC;
        /* Initialize the DMA_BufferSize member */
        DMA_InitStructure.DMA_BufferSize = 2;
        DMA_DeInit(IOE_DMA_RX_CHANNEL);
        DMA_Init(IOE_DMA_RX_CHANNEL, &DMA_InitStructure);
    }
    /* If using DMA for Transmission */
    else if (Direction == IOE_DMA_TX) {
        /* Initialize the DMA_DIR member */
        DMA_InitStructure.DMA_DIR = DMA_DIR_PeripheralDST;
        /* Initialize the DMA_BufferSize member */
        DMA_InitStructure.DMA_BufferSize = 1;
        DMA_DeInit(IOE_DMA_TX_CHANNEL);
        DMA_Init(IOE_DMA_TX_CHANNEL, &DMA_InitStructure);
    }
}

void LCD_WriteRAM_Prepare()
{
    LCD_WriteRegIndex(LCD_REG_34); /* Select GRAM Reg */
    /* Reset LCD control line(/CS) and Send Start-Byte */
    LCD_nCS_StartByte(START_BYTE | LCD_WRITE_REG);
}

void LCD_WriteRAM(uint16_t RGB_Code)
{
    SPI_I2S_SendData(LCD_SPI, RGB_Code >> 8);
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
    SPI_I2S_SendData(LCD_SPI, RGB_Code & 0xFF);
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
}

uint16_t LCD_ReadReg(uint8_t LCD_Reg)
{
    uint16_t tmp = 0;
    uint8_t i = 0;
    /* LCD_SPI prescaler: 4 */
    LCD_SPI->CR1 &= 0xFFC7;
    LCD_SPI->CR1 |= 0x0008;
    /* Write 16-bit Index (then Read Reg) */
    LCD_WriteRegIndex(LCD_Reg);
    /* Read 16-bit Reg */
    /* Reset LCD control line(/CS) and Send Start-Byte */
    LCD_nCS_StartByte(START_BYTE | LCD_READ_REG);
    for (i = 0; i < 5; i++) {
        SPI_I2S_SendData(LCD_SPI, 0xFF);
        while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
        }
        /* One byte of invalid dummy data read after the start byte */
        while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
        }
        SPI_I2S_ReceiveData(LCD_SPI);
    }
    SPI_I2S_SendData(LCD_SPI, 0xFF);
    /* Read upper byte */
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
    /* Read lower byte */
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
    }
    tmp = SPI_I2S_ReceiveData(LCD_SPI);
    SPI_I2S_SendData(LCD_SPI, 0xFF);
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_BSY) != RESET) {
    }
    /* Read lower byte */
    while (SPI_I2S_GetFlagStatus(LCD_SPI, SPI_I2S_FLAG_RXNE) == RESET) {
    }
    tmp = ((tmp & 0xFF) << 8) | SPI_I2S_ReceiveData(LCD_SPI);
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
    /* LCD_SPI prescaler: 2 */
    LCD_SPI->CR1 &= 0xFFC7;
    return tmp;
}

uint16_t GL_LCD_ReadRAM()
{
    __IO uint16_t tmp = 0;
#if defined(GL_LCD_BASE)
    /* Write 16-bit Index (then Read Reg) */
    GL_LCD->LCD_REG = R34; /* Select GRAM Reg */
    /* Read 16-bit Reg */
    tmp = GL_LCD->LCD_RAM;
    tmp = GL_LCD->LCD_RAM; /* This line must be removed to work with some LCD controller */
#else
    tmp = LCD_ReadReg(R34);
#endif
    return tmp;
}

void GL_LCD_DrawCharTransparent(uint16_t Xpos, uint16_t Ypos, const uint16_t * c) /* 16bit char */
{
    uint32_t line_index = 0, pixel_index = 0;
    uint8_t Xaddress = 0;
    uint16_t Yaddress = 0;
    Xaddress = Xpos;
    Yaddress = Ypos;
    for (line_index = 0; line_index < GL_FontHeight; line_index++) {
        for (pixel_index = 0; pixel_index < GL_FontWidth; pixel_index++) {
            /* SmallFonts have bytes in reverse order */
            if ((GL_Font == GL_FONT_BIG   && (((const uint16_t *)c)[line_index] & (1 << pixel_index)) == 0x00) ||
                    (GL_Font == GL_FONT_SMALL && (((const uint16_t *)c)[line_index] & (0x80 >> pixel_index)) == 0x00)) {
                Yaddress++;
            } else {
                LCD_PutPixel(Xaddress, Yaddress++, GL_TextColor, SinglePixel);
            }
        }
        Xaddress++;
        Yaddress = Ypos;
    }
}

void GL_LCD_DrawChar(uint8_t Xpos, uint16_t Ypos, const uint16_t * c) /* 16bit char */
{
    uint32_t line_index = 0, pixel_index = 0;
    uint8_t Xaddress = 0;
    uint16_t Yaddress = 0;
    __IO uint16_t tmp_color = 0;
    Xaddress = Xpos;
    Yaddress = Ypos;
    for (line_index = 0; line_index < GL_FontHeight; line_index++) {
        /* SmallFonts have bytes in reverse order */
        if ((GL_Font == GL_FONT_BIG   && (((const uint16_t *)c)[line_index] & (1 << 0)) == 0x00) ||
                (GL_Font == GL_FONT_SMALL && (((const uint16_t *)c)[line_index] & (0x80 >> 0)) == 0x00)) {
            tmp_color = GL_BackColor;
        } else {
            tmp_color = GL_TextColor;
        }
        LCD_PutPixel(Xaddress, Yaddress++, tmp_color, FirstPixel);
        for (pixel_index = 1; pixel_index < GL_FontWidth - 1; pixel_index++) {
            /* SmallFonts have bytes in reverse order */
            if ((GL_Font == GL_FONT_BIG   && (((const uint16_t *)c)[line_index] & (1 << pixel_index)) == 0x00) ||
                    (GL_Font == GL_FONT_SMALL && (((const uint16_t *)c)[line_index] & (0x80 >> pixel_index)) == 0x00)) {
                tmp_color = GL_BackColor;
            } else {
                tmp_color = GL_TextColor;
            }
            LCD_PutPixel(Xaddress, Yaddress++, tmp_color, MiddlePixel);
        }
        pixel_index++;
        /* SmallFonts have bytes in reverse order */
        if ((GL_Font == GL_FONT_BIG   && (((const uint16_t *)c)[line_index] & (1 << pixel_index)) == 0x00) ||
                (GL_Font == GL_FONT_SMALL && (((const uint16_t *)c)[line_index] & (0x80 >> pixel_index)) == 0x00)) {
            tmp_color = GL_BackColor;
        } else {
            tmp_color = GL_TextColor;
        }
        LCD_PutPixel(Xaddress, Yaddress++, tmp_color, LastPixel);
        Xaddress++;
        Yaddress = Ypos;
    }
}

void LCD_SetDisplayWindow(uint8_t Xpos, uint16_t Ypos, uint8_t Height, uint16_t Width)
{
    /* Horizontal GRAM Start Address */
    LCD_WriteReg(LCD_REG_80, Xpos);
    /* Horizontal GRAM End Address */
    LCD_WriteReg(LCD_REG_81, Xpos + Height - 1);
    /* Vertical GRAM Start Address */
    LCD_WriteReg(LCD_REG_82, Ypos);
    /* Vertical GRAM End Address */
    LCD_WriteReg(LCD_REG_83, Ypos + Width - 1);
    LCD_SetCursor(Xpos, Ypos);
}

void LCD_WriteRAMWord(uint16_t RGB_Code)
{
    LCD_WriteRAM_Prepare();
    LCD_WriteRAM(RGB_Code);
    LCD_CtrlLinesWrite(LCD_NCS_GPIO_PORT, LCD_NCS_PIN, Bit_SET);
}

void TimingDelay_Decrement()
{
  if (TimingDelay != 0x00)
  {
    TimingDelay--;
  }
}

TSC_FLASH_Status TSC_FLASH_ProgramWord(uint32_t Address, uint32_t Data)
{
  return FLASH_ProgramWord(Address, Data);
}

TSC_FLASH_Status TSC_FLASH_ErasePage(uint32_t Page_Address)
{
  TSC_FLASH_Status TSC_FlashStatus = TSC_FLASH_COMPLETE;
#ifndef STM32F2XX  
   TSC_FlashStatus = FLASH_ErasePage(Page_Address);
#else
   uint8_t index = 0;
   uint32_t size = 0;
   
   while(size < Page_Address && index < 12)
   {
     size += FLASH_Sectors[1][index];
     index++;
   }
   
   if(size != Page_Address && index != 0)
   {
     index--;
   }
   TSC_FlashStatus = FLASH_EraseSector(FLASH_Sectors[0][index], VoltageRange_3);

#endif   
   return TSC_FlashStatus;
}
