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
#include "LcdHal.h"
#include "TscHal.h"
#include "JoyHal.h"
#include "touchscreen.h"
#include "cursor.h"
#include "display.h"
#include "hw_config.h"
#include "adc.h"
#include "pictures.h"

#if defined(USE_STM32100E_EVAL)
#define LAST_FLASH_MEMORY_ADDRESS	((uint32_t)0x08080000)
#elif defined(USE_STM322xG_EVAL)
#define LAST_FLASH_MEMORY_ADDRESS	((uint32_t)0x08100000)
#elif defined(USE_STM3210C_EVAL)
#define LAST_FLASH_MEMORY_ADDRESS	((uint32_t)0x08040000)
#endif



void display_init()
{
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
    GL_LCD_Init();
    GL_Clear(GL_White);
    InputInterface_Init();
    ShowLoadingLogo();
    /* Check if Touchscreen Calibration has been done*/
    TS_CheckCalibration();
    /*Initialize cursor*/
    GL_Clear(White);
    CursorInit(GL_NULL);
    /* Menu Initialisation*/
    Show_HomeScreen();
    CursorShow(195, 80);
}

void InputInterface_Init(void)
{
    /* Set the last Flash Memory address */
    Set_LastFlashMemoryAddress(LAST_FLASH_MEMORY_ADDRESS);
    /* Touch Screen Init */
    TSC_Init();
}


__IO uint8_t vu8_gPageCurrent = 0;
GL_Page_TypeDef * pageStart;


void Show_HomeScreen()
{
    char tempstring[20];
    /*Resetting all the menu parameters*/
    vu8_gPageCurrent = PAGE_HOME;
    GL_PageControls_TypeDef * Maintemp;
    /*     GL_PageControls_TypeDef* Targettemp;
     *     GL_PageControls_TypeDef* CoefP;
     *     GL_PageControls_TypeDef* CoefI;
     *     GL_PageControls_TypeDef* CoefD;
     *     GL_PageControls_TypeDef* Activate;
     *     GL_PageControls_TypeDef* Standby;
     *     GL_PageControls_TypeDef* Settemp;
     *     GL_PageControls_TypeDef* SetPID;
     */
    /* ************** GRAPHIC LIBRARY - PAGE 0 ************** */
    GL_Page_TypeDef pageStartobj;
    GL_Page_TypeDef * pageStart = &pageStartobj;
    Create_PageObj(pageStart);
    siprintf((char *)tempstring, "%i.%i°C", adc_curtemp/1000, adc_curtemp%1000);
    Maintemp = NewLabel(10, (uint8_t *)"18.5°C", GL_HORIZONTAL, GL_FONT_BIG, GL_Black);
    AddPageControlObj((uint16_t)10, (uint16_t)205, Maintemp, pageStart);
    GL_Clear(White);
    GL_SetTextColor(GL_Blue);
    pageStart->ShowPage(pageStart, GL_TRUE);
}


void ShowLoadingLogo(void)
{
    GL_SetTextColor(Black);
    GL_SetBackColor(White);
    GL_DrawButtonBMP(210, 110, (LCD_Height / 10) * 2 + 100, (LCD_Height / 10) * 2, (uint8_t *) ___pictures_labpic_bmp);
    GL_DisplayAdjStringLine(3 * (LCD_Height / 5), (LCD_Width / 3) * 2 + 6, (uint8_t *)"Copyright (c) 2010,2011, Hervé Rouault", GL_FALSE);
    GL_DisplayAdjStringLine(4 * (LCD_Height / 5), (LCD_Width / 3) * 2 + 6, (uint8_t *)"All rights reserved.", GL_FALSE);
    GL_Delay(30);
    GL_Delay(30);
    GL_Delay(30);
    GL_Delay(30);
    GL_Delay(15);
}
