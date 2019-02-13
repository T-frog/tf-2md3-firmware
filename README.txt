Contributer: Atsushi Watanabe at Tsukuba University

# Compile Firmware

## Setting Development Environment

This is a quick guide which contains all the tools you'll need to install on your development environment.

* samba
```
$ git clone https://github.com/at-wat/samba.git
$ cd samba
$ ./configure
$ make
$ sudo make install
```

* tf2md3_flash
```
$ cd ../
$ git clone https://github.com/T-frog/tf2md3_flash.git
$ cd tf2md3_flash
$ sudo make install
```

* gcc-arm-none-eabi
```
$ wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/6-2017q2/gcc-arm-none-eabi-6-2017-q2-update-linux.tar.bz2?product=GNU%20ARM%20Embedded%20Toolchain,64-bit,,Linux,6-2017-q2-update > gcc-arm-none-eabi.tar.bz2
$ sudo tar xjf gcc-arm-none-eabi.tar.bz2 --strip-components 1 -C /usr
```

## Compile

```
$ cd tfrog-motordriver
$ make
```

# Flash Firmware

**DEVICE_PATH**: Full path to a mounted device
**FIRMWARE**: A path to a compiled firmware

```
$ tf2md3_flash DEVICE_PATH FIRMWARE.bin
```

# LICENSE

Deliverables of tf-2md3-firmware code are inherited from at91lib license as below.

/* ----------------------------------------------------------------------------
 *         ATMEL Microcontroller Software Support 
 * ----------------------------------------------------------------------------
 * Copyright (c) 2008, Atmel Corporation
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * - Redistributions of source code must retain the above copyright notice,
 * this list of conditions and the disclaimer below.
 *
 * Atmel's name may not be used to endorse or promote products derived from
 * this software without specific prior written permission.
 *
 * DISCLAIMER: THIS SOFTWARE IS PROVIDED BY ATMEL "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NON-INFRINGEMENT ARE
 * DISCLAIMED. IN NO EVENT SHALL ATMEL BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 * ----------------------------------------------------------------------------
 */
