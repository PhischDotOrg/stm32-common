# stm32f4-common
This repository contains code for STM32F4-based boards that I'm using in some of my projects.

It is intended to be used as a git [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) by a top-level project.

See [https://github.com/PhischDotOrg/stm32f4-minimal](https://github.com/PhischDotOrg/stm32f4-minimal) for an example.

# How to check out
This project makes use of git's [submodules](https://git-scm.com/book/en/v2/Git-Tools-Submodules) feature. This repository is therefore more of a front-end for other code parts. In order to obtain all the needed code, please check out with the `--recursive` flag.

Example:

```
$ git clone git@github.com:PhischDotOrg/stm32f4-common.git --recursive
```

# How to build
These instructions apply to all top-level projects that use this repository as a sub-module.

## Pre-requisites
In order to build the code, you need these packages installed on your computer:
- [CMake](https://cmake.org/download/)
- [GNU Arm Embedded Toolchain](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm)
  - Other C++11 capable compilers might work, too, but I haven't tested them.
- _Optional_: [OpenOCD](http://openocd.org/getting-openocd/)
  - If OpenOCD is found, the `make flash` target will allow you to flash the firmware onto the embedded device.
- _Optional_: [Doxygen](https://www.doxygen.nl/download.html)
  - If Doxygen is found, the `make doxygen` target will allow you to create the Doxygen documentation.

## Building in Visual Studio Code
The easiest way to build this project is to load the [Visual Studio Code](https://code.visualstudio.com) Workspace:

[https://github.com/PhischDotOrg/stm32f4-usbdevice/blob/master/stm32f4-usbdevice.code-workspace](https://github.com/PhischDotOrg/stm32f4-usbdevice/blob/master/stm32f4-usbdevice.code-workspace)

The Workspace should set up the CMake Kits to offer _Generic STM32F4_ and _Generic STM32F4 (Windows)_. Please chose the one that is suitable for you.

## Building via Command Line
The code can be built via Command Line, e.g. like this:

```
$ mkdir build
$ cd build
$ cmake -DCMAKE_TOOLCHAIN_FILE=../common/Generic_STM32F4.ctools -DSTM32F4_BOARD=STM32F4_Discovery ..
$ make
```

_Please note:_ CMake apparently uses [Ninja](https://ninja-build.org) as the default Build Tool on some platforms (e.g. recent Ubuntu). In that case, the `make` command should be `ninja` instead.

# How to flash
The CMake build files include a `flash` target which uses OpenOCD to flash the binary to the controller. This is probably the most convenient way to flash:

```
$ make flash
```

# Build Types
The `CMakeLists.txt` file uses the [CMAKE_BUILD_TYPE](https://cmake.org/cmake/help/latest/variable/CMAKE_BUILD_TYPE.html) variable as follows:
  - _Debug_: Compile with `-O0 -g`.
  - _Release_: Compile with `-O3`.
  - _RelWithDebInfo_: Compile with `-O2 -Og -g`.
  - _MinSizeRel_: Compile with `-Os -g`.

# Supported Architectures
The `CMakeLists.txt` file uses [CMAKE_SYSTEM_NAME](https://cmake.org/cmake/help/latest/variable/CMAKE_SYSTEM_NAME.html) variable
to determine whether the code is being built for the Host computer or for an embedded Target Device:
  - _Generic_: Code is being cross-compiled for an Embedded Target.
  - Otherwise, it is assumed that the Code is compiled for the Host computer. This is useful for Unit Tests. In that case,
    the `HOSTBUILD` macro is added, so C/C++ code can use the preprocessor to check whether the code is being built for
    a non-embedded system (e.g. like `#if defined(HOSTBUILD)`).

# Supported Embedded Boards
The CMake Variable `STM32F4_BOARD` variable is also evaluated in the [CMakeLists.txt](https://github.com/PhischDotOrg/stm32f4-common/blob/master/CMakeLists.txt) file and allows for the following options:
  - _STM32F4 Discovery_: Sets `STM32F4_BOARD=STM32F4_Discovery` to build for the [STM32F4 Discovery Board](https://www.st.com/en/evaluation-tools/stm32f4discovery.html).
  - _STM32F4 Nucleo F411RE_: Sets `STM32F4_BOARD=STM32F4_Nucleo_F411RE` to build for the [STM32F4 Nucleo F411RE Board](https://www.st.com/en/evaluation-tools/nucleo-f411re.html).

Note that this can also be set from the command line or a top-level `CMakeLists.txt` file.

# Build Targets
- _all_: Default pseudo-target building all executables, most notably `firmware.elf`
- _flash_: Build the binary and flash it to the controller using OpenOCD.
- _doxygen_: Build the Doxygen documentation for the C++ Code.
- _bin_: Build a `.bin`, `.s19`and `.hex` file from the `.elf` file. Also generate an ASCII dump from the `.bin` file.
  - This may be useful if you need to flash using other tools. The ASCII dump may come in handy if you need to compare binary changes between two builds.

# Limitations
I have not tested this much on the STM32F4 Nucleo Board, so this might not work at all.
