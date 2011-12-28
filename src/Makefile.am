# Copyright (c) 2010,2011, Hervé Rouault All rights reserved.
# 
# This file is part of Coyotte
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
# * Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.  * Redistributions in binary
# form must reproduce the above copyright notice, this list of conditions and
# the following disclaimer in the documentation and/or other materials provided
# with the distribution.  * Neither the name of the <organization> nor the
# names of its contributors may be used to endorse or promote products derived
# from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
# DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
# (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
# LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
# ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
# (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
# SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
#
## Process this file with automake to produce Makefile.in

#SUBDIRS = #stm32usb usbvirtual stm32base

# Board to use:
# USE_STM3210B_EVAL
# USE_STM32100B_EVAL
# USE_STM3210E_EVAL
BOARD = USE_STM3210C_EVAL
# USE_STM32L152_EVAL
# USE_STM32100E_EVAL

# MCU line
# STM32F10X_LD     # !< STM32F10X_LD: STM32 Low density devices
# STM32F10X_LD_VL  # !< STM32F10X_LD_VL: STM32 Low density Value Line devices
# STM32F10X_MD # !< STM32F10X_MD: STM32 Medium density devices
# STM32F10X_MD_VL  # !< STM32F10X_MD_VL: STM32 Medium density Value Line devices
# STM32F10X_HD     # !< STM32F10X_HD: STM32 High density devices
# STM32F10X_HD_VL  # !< STM32F10X_HD_VL: STM32 High density value line devices
# STM32F10X_XL     # !< STM32F10X_XL: STM32 XL-density devices
MCULINE = STM32F10X_CL     # !< STM32F10X_CL: STM32 Connectivity line devices

ST_URL = http://www.st.com/internet/com/SOFTWARE_RESOURCES/SW_COMPONENT/FIRMWARE

STM32_USB_FILENAME = um0424.zip
STM32_USB_URL = $(ST_URL)/$(STM32_USB_FILENAME)

STM32_GUI_FILENAME = stm32_gui_lib.zip
STM32_GUI_URL = $(ST_URL)/$(STM32_GUI_FILENAME)


STEVALBOARD_FOLDER = stm32_gui_lib/Utilities/STM32_EVAL
STEVALBOARD3210C_FOLDER = $(STEVALBOARD_FOLDER)/STM3210C_EVAL
STGUILIB_FOLDER = stm32_gui_lib/Libraries/Embedded_GUI_Library
STGUILIB_INC = $(STGUILIB_FOLDER)/inc
STGUIHAL_FOLDER = stm32_gui_lib/Libraries/Embedded_GUI_HAL
STGUIHAL_INC = $(STGUIHAL_FOLDER)/inc

STLIB_FOLDER = STM32_USB-FS-Device_Lib_V3.3.0/Libraries
STLIB_INC = $(STLIB_FOLDER)/STM32F10x_StdPeriph_Driver/inc
STUSBLIB_INC = $(STLIB_FOLDER)/STM32_USB-FS-Device_Driver/inc
STVCP_INC = STM32_USB-FS-Device_Lib_V3.3.0/Project/Virtual_COM_Port/inc

STLIBEVALBOARD_FOLDER = STM32_USB-FS-Device_Lib_V3.3.0/Utilities/STM32_EVAL

CMSIS = $(STLIB_FOLDER)/CMSIS/CM3
CORESUPPORT = $(CMSIS)/CoreSupport
DEVICESUPPORT = $(CMSIS)/DeviceSupport/ST/STM32F10x

stdperi_src = $(STLIB_FOLDER)/STM32F10x_StdPeriph_Driver/src
stdperi = $(stdperi_src)/stm32f10x_gpio.c\
		  $(stdperi_src)/stm32f10x_rcc.c\
		  $(stdperi_src)/stm32f10x_dma.c\
		  $(stdperi_src)/stm32f10x_tim.c\
		  $(stdperi_src)/stm32f10x_rtc.c\
		  $(stdperi_src)/stm32f10x_usart.c\
		  $(stdperi_src)/stm32f10x_spi.c\
		  $(stdperi_src)/stm32f10x_exti.c\
		  $(stdperi_src)/stm32f10x_i2c.c\
		  $(stdperi_src)/stm32f10x_adc.c\
		  $(stdperi_src)/stm32f10x_flash.c\
		  $(stdperi_src)/misc.c

usbperi_src = $(STLIB_FOLDER)/STM32_USB-FS-Device_Driver/src
usbperi = $(usbperi_src)/usb_init.c\
		  $(usbperi_src)/usb_regs.c\
		  $(usbperi_src)/usb_core.c\
		  $(usbperi_src)/usb_mem.c\
		  $(usbperi_src)/usb_int.c\
		  $(usbperi_src)/usb_sil.c\
		  $(usbperi_src)/otgd_fs_dev.c\
		  $(usbperi_src)/otgd_fs_cal.c\
		  $(usbperi_src)/otgd_fs_pcd.c

#guilib_src = $(STGUILIB_FOLDER)/src
#guilib = $(guilib_src)/cursor.c\
# 		 $(guilib_src)/gl_fonts.c\
#   		 $(guilib_src)/graphicObject.c\
#		 $(guilib_src)/images.c
#guihal_src = $(STGUIHAL_FOLDER)/src
#guihal = $(guihal_src)/touchscreen.c\
#		 $(guihal_src)/LcdHal.c\
#		 $(guihal_src)/TscHal.c


evalboard = $(STEVALBOARD3210C_FOLDER)/stm3210c_eval.c\
			$(STEVALBOARD3210C_FOLDER)/stm3210c_eval_lcd.c\
			$(STEVALBOARD3210C_FOLDER)/stm3210c_eval_ioe.c

STVCP_SRC = STM32_USB-FS-Device_Lib_V3.3.0/Project/Virtual_COM_Port/src
usbvirtual = $(STVCP_SRC)/usb_pwr.c\
			 $(STVCP_SRC)/usb_prop.c\
			 $(STVCP_SRC)/usb_desc.c\
			 $(STVCP_SRC)/usb_istr.c\
			 $(STVCP_SRC)/usb_endp.c

$(stdperi):
	curl -LO $(STM32_USB_URL)
	unzip -q $(STM32_USB_FILENAME)

#$(guilib):
#	curl -LO $(STM32_GUI_URL)
#	mkdir stm32_gui_lib
#	unzip -q $(STM32_GUI_FILENAME) -d stm32_gui_lib

cmsis = $(CMSIS)/CoreSupport/core_cm3.c\
		$(CMSIS)/DeviceSupport/ST/STM32F10x/system_stm32f10x.c\
		$(CMSIS)/DeviceSupport/ST/STM32F10x/startup/gcc_ride7/startup_stm32f10x_cl.s

bin_PROGRAMS = tempcontrol

tempcontrol_SOURCES = $(stdperi)\
					  $(cmsis)\
					  stm32f10x_it.c\
					  syscalls.c\
					  main.c\
					  hw_config.c hw_config.h\
					  display.c display.h\
					  pictures.c pictures.h\
					  images.c images.h\
					  gl_fonts.c gl_fonts.h

#					  $(evalboard)\
#					  $(guilib)\
#					  $(guihal)\
#					  $(usbperi)\
#					  $(usbvirtual)\
#					  adc.c adc.h\
#					  pwm.c pwm.h\
#					  rtc.c rtc.h
#
tempcontrol_CPPFLAGS = -D$(BOARD) -D$(MCULINE) -DUSE_STDPERIPH_DRIVER\
					   -I$(CORESUPPORT)\
					   -I$(DEVICESUPPORT)\
					   -I$(STLIBEVALBOARD_FOLDER) -I$(STEVALBOARD3210C_FOLDER)\
					   -I$(STLIB_INC) -I$(STUSBLIB_INC) -I$(STVCP_INC)
#					   -I$(STGUILIB_INC) -I$(STGUIHAL_INC)
tempcontrol_CFLAGS = -mcpu=cortex-m3 -mthumb -msoft-float -fno-common\
					 -ggdb -O0 -Wall -fdata-sections -ffunction-sections
tempcontrol_CCASFLAGS = -mcpu=cortex-m3 -mthumb -msoft-float -fno-common\
					 -ggdb -O0 -Wall -fdata-sections -ffunction-sections
tempcontrol_LDFLAGS = -nostdlib -nodefaultlibs -nostartfiles\
					  -T$(top_srcdir)/linker/stm3210c_flash.ld\
					  -Wl,--gc-sections
