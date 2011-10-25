# Copyright (c) 2010,2011, Hervé Rouault
# All rights reserved.
# 
# This file is part of Coyotte
# 
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#     * Redistributions of source code must retain the above copyright
#       notice, this list of conditions and the following disclaimer.
#     * Redistributions in binary form must reproduce the above copyright
#       notice, this list of conditions and the following disclaimer in the
#       documentation and/or other materials provided with the distribution.
#     * Neither the name of the <organization> nor the
#       names of its contributors may be used to endorse or promote products
#       derived from this software without specific prior written permission.
# 
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
# ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
# WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL <COPYRIGHT HOLDER> BE LIABLE FOR ANY
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
BOARD = USE_STM3210B_EVAL
# USE_STM32100B_EVAL
# USE_STM3210E_EVAL
# USE_STM3210C_EVAL
# USE_STM32L152_EVAL
# USE_STM32100E_EVAL

# MCU line
# STM32F10X_LD     # !< STM32F10X_LD: STM32 Low density devices
# STM32F10X_LD_VL  # !< STM32F10X_LD_VL: STM32 Low density Value Line devices
MCULINE = STM32F10X_MD # !< STM32F10X_MD: STM32 Medium density devices
# STM32F10X_MD_VL  # !< STM32F10X_MD_VL: STM32 Medium density Value Line devices
# STM32F10X_HD     # !< STM32F10X_HD: STM32 High density devices
# STM32F10X_HD_VL  # !< STM32F10X_HD_VL: STM32 High density value line devices
# STM32F10X_XL     # !< STM32F10X_XL: STM32 XL-density devices
# STM32F10X_CL     # !< STM32F10X_CL: STM32 Connectivity line devices

STM32_USB_FILENAME = um0424.zip
STM32_USB_URL = http://www.st.com/internet/com/SOFTWARE_RESOURCES/SW_COMPONENT/FIRMWARE/$(STM32_USB_FILENAME)

STLIB_FOLDER = STM32_USB-FS-Device_Lib_V3.3.0/Libraries
STLIB_INC = $(STLIB_FOLDER)/STM32F10x_StdPeriph_Driver/inc
STUSBLIB_INC = $(STLIB_FOLDER)/STM32_USB-FS-Device_Driver/inc
STVCP_INC = STM32_USB-FS-Device_Lib_V3.3.0/Project/Virtual_COM_Port/inc

CMSIS = $(STLIB_FOLDER)/CMSIS/CM3
CORESUPPORT = $(CMSIS)/CoreSupport
DEVICESUPPORT = $(CMSIS)/DeviceSupport/ST/STM32F10x
BOARDHEADER = STM32_USB-FS-Device_Lib_V3.3.0/Utilities/STM32_EVAL

stdperi_src = $(STLIB_FOLDER)/STM32F10x_StdPeriph_Driver/src
stdperi = $(stdperi_src)/stm32f10x_gpio.c $(stdperi_src)/stm32f10x_rcc.c $(stdperi_src)/stm32f10x_dma.c\
			 $(stdperi_src)/stm32f10x_tim.c $(stdperi_src)/stm32f10x_rtc.c\
			 $(stdperi_src)/stm32f10x_usart.c\
			 $(stdperi_src)/stm32f10x_spi.c\
			 $(stdperi_src)/stm32f10x_exti.c\
			 $(stdperi_src)/stm32f10x_i2c.c\
			 $(stdperi_src)/stm32f10x_adc.c\
			 $(stdperi_src)/misc.c

usbperi_src = $(STLIB_FOLDER)/STM32_USB-FS-Device_Driver/src
usbperi = $(usbperi_src)/usb_init.c $(usbperi_src)/usb_regs.c $(usbperi_src)/usb_core.c\
			 $(usbperi_src)/usb_mem.c $(usbperi_src)/usb_int.c $(usbperi_src)/usb_sil.c

evalboard = STM32_USB-FS-Device_Lib_V3.3.0/Utilities/STM32_EVAL/STM3210B_EVAL/stm3210b_eval.c

STVCP_SRC = STM32_USB-FS-Device_Lib_V3.3.0/Project/Virtual_COM_Port/src
usbvirtual = $(STVCP_SRC)/hw_config.c $(STVCP_SRC)/usb_pwr.c $(STVCP_SRC)/usb_prop.c\
				 $(STVCP_SRC)/usb_desc.c $(STVCP_SRC)/usb_istr.c $(STVCP_SRC)/usb_endp.c

startup = startup/startup_stm32f10x_md.c

um0424.zip:
	curl -LO $(STM32_USB_URL)

STM32_USB-FS-Device_Lib_V3.3.0: $(STM32_USB_FILENAME)
	unzip -q $(STM32_USB_FILENAME)

cmsis = $(CMSIS)/CoreSupport/core_cm3.c


bin_PROGRAMS = tempcontrol

libstm.c: STM32_USB-FS-Device_Lib_V3.3.0

tempcontrol_SOURCES = $(cmsis)\
							 $(stdperi)\
							 $(usbperi)\
							 $(usbvirtual)\
							 $(evalboard)\
							 $(startup)\
							 syscalls.c\
							 main.c adc.c adc.h pwm.c pwm.h
tempcontrol_CPPFLAGS = -D$(BOARD) -D$(MCULINE) -DUSE_STDPERIPH_DRIVER\
							  -I$(CORESUPPORT) -I$(DEVICESUPPORT) -I$(BOARDHEADER)\
							  -I$(STLIB_INC) -I$(STUSBLIB_INC) -I$(STVCP_INC)

#tempcontrol_CFLAGS = -I $(srcdir)/usbvirtual -I $(srcdir)/stm32usb -I $(srcdir)/stm32stdperiph -I $(srcdir)/stm32base
#tempcontrol_LDADD = stm32stdperiph/*.o usbvirtual/*.o stm32usb/*.o stm32base/*.o

