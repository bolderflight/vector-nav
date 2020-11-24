# vector_nav
Driver for VectorNav Inertial Measurement Unit (IMU) and Inertial Navigation System (INS) sensors.
   * [License](LICENSE.md)
   * [Changelog](CHANGELOG.md)
   * [Contributing guide](CONTRIBUTING.md)

## Description
VectorNav produces a line of high accuracy IMU and INS sensors. The MEMS sensors in these units are temperature calibrated and an integrated microcontroller provides real-time Extended Kalman Filtering (EKF). The VN-100 is an IMU and Attitude and Heading Reference System (AHRS) providing IMU data and an estimate of the vehicle's attitude and heading. The VN-200 and VN-300 include an integrated GNSS receiver, providing GNSS data and extending the sensor to an Inertial Navigation System (INS), providing high rate estimates of the vehicle's velocity and position in addition to attitude. The VN-300 utilizes two GNSS receivers to enhance heading accuracy. This library communicates with the VN-100, VN-200, and VN-300 sensors using SPI communication.

## Installation
CMake is used to build this library, which is exported as a library target called *vector_nav*. The header is added as:

```
#include "vector_nav/vector_nav.h"
```
Note that you'll need CMake version 3.13 or above; it is recommended to build and install CMake from source, directions are located in the [CMake GitLab repository](https://github.com/Kitware/CMake).

The library can be also be compiled stand-alone using the CMake idiom of creating a *build* directory and then, from within that directory issuing:

```
cmake .. -DMCU=MK66FX1M0
make
```

This will build the library and an example executables called *spi_example*. The example executable source files are located at *examples/spi_example.cc*. This code is built and tested on AARCH64 and AMD64 systems running Linux and AMD64 systems running the Windows Subsystem for Linux (WSL). The [arm-none-eabi](https://developer.arm.com/tools-and-software/open-source-software/developer-tools/gnu-toolchain/gnu-rm/downloads) toolchain must be installed in your Linux environment.

Notice that the *cmake* command includes a define specifying the microcontroller the code is being compiled for. This is required to correctly configure the code, CPU frequency, and compile/linker options. The available MCUs are:
   * MK20DX128
   * MK20DX256
   * MK64FX512
   * MK66FX1M0
   * MKL26Z64

These are known to work with the same packages used in Teensy products. Also switching the MK66FX1M0 or MK64FX512 from BGA to LQFP packages is known to work well. Swapping packages of other chips is probably fine, as long as it's only a package change.

The *spi_example* target creates an executable for communicating with the sensor using SPI communication. The target also has a *_hex* for creating the hex file and a *_upload* to upload the software to the microcontroller. 

## Namespace
This library is within the namespace *sensors*.

## Registers
*registers.h*, within the namespace *vector_nav* defines all of the VectorNav configuration and data registers. Registers that are common across VectorNav's product line are within the namespace *common*, registers for the VN-100 are within the namespace *vn100*, registers for the VN-200 are within the namespace *vn200*, and registers for the VN-300 are within the namespace *vn300*. Each register is defined as a struct, whose name matches the register name within the VectorNav User Manuals. The structs define the register ID, the register size, whether it is read only, and the register payload. The payload contains the register fields that are written to or read from.

The concept is the *VectorNav* class initializes communication with the sensor and provides methods for writing and reading these register structs to/from the sensor. The *VectorNav* class, including structs defined in *registers.h*, enables using any of the sensor's available functionality. The *Vn100*, *Vn200*, and *Vn300* classes wrap around the *VectorNav* class to provide convenience methods for the most common configuration and data collection functions. Whereas these classes provide limited functionality, they provide a more intuitive interface for the majority of use cases.

## Classes
   * [VectorNav](VECTOR_NAV_README.md): enables initializing communication and writing / reading register structs.
   * [Vn100](VN100_README.md): provides convenience methods for the VN-100 IMU / AHRS.
   * [Vn200](VN200_README.md): provides convenience methods for the VN-200 GNSS-aided INS.
   * [Vn300](VN300_README.md): provides convenience methods for the VN-300 GNSS-aided INS.
