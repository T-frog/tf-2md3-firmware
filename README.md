Contributer: Atsushi Watanabe at Tsukuba University

## Compile Firmware

### Setting Development Environment

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

### Compile

```
$ cd tfrog-motordriver
$ make
```

## Flash Firmware

**DEVICE_PATH**: Full path to a mounted device
**FIRMWARE**: A path to a compiled firmware

```
$ tf2md3_flash DEVICE_PATH FIRMWARE.bin
```

## LICENSE

Source code derived from ATMEL at91lib is licensed under [the at91lib license](LICENSE.at91lib).
Other source code is licensed under [the Apache License Version 2.0](LICENSE).
