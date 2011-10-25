Overview
========

This project provides a way to control temperature on a microscope stage for live imaging.

It consists in electronic circuit maps for the control of a Peltier module as well as the source code to use with a STM32 microcontroller.

Electronics
===========

Microcontroller program
=======================

Installation
============

Requirements
------------

* [STM32F10x standard peripheral library](http://www.st.com/internet/com/SOFTWARE_RESOURCES/SW_COMPONENT/FIRMWARE/stm32f10x_stdperiph_lib.zip)
* [STM32F10x USB full-speed device library](http://www.st.com/internet/com/SOFTWARE_RESOURCES/SW_COMPONENT/FIRMWARE/um0424.zip)
* [STM32 Virtual COM Port Driver](http://www.st.com/internet/com/SOFTWARE_RESOURCES/SW_COMPONENT/SW_DRIVER/stm32_vcp.zip)
* A working gcc arm eabi toolchain. Use for instance [this Makefile](https://github.com/jsnyder/arm-eabi-toolchain). 

Compilation
-----------

The program need to be cross-compiled

```bash
mkdir build
cd build
../configure --host arm-none-eabi
make
```

You don't need to run `make install` since you will upload it onto the board instead of installing it.

Third party software
====================

This program includes a script which automatically the *STM32 USB FileSystem Device Library*. This library is not open-source and you have to agree with the provided license in order to use the code.

