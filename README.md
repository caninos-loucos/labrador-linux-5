# labrador-linux-5

Linux 5 kernel source code for Caninos Labrador.

## About

This repository contains the source code for both the Caninos Labrador's
 64-bit and 32-bit Linux kernel drivers, along with Debian-specific patches and
 security updates.

## Clean Build

Before compilation, please ensure that you have the following libraries and/or
 tools installed on your machine:
 
1) GNU Binutils environment and Native GCC compiler
2) GCC cross-compiler targeting "gcc-aarch64-linux-gnu"
3) GCC cross-compiler targeting "gcc-arm-linux-gnueabihf"
4) Make build tool
5) Git client
6) Bison and Flex development libraries
7) NCurses development libraries
8) LibSSL development libraries
9) U-Boot Tools libraries

* For Debian-based distros
```
sudo apt-get update
sudo apt-get install build-essential binutils make git \
                       gcc-arm-linux-gnueabihf gcc-aarch64-linux-gnu \
                       bison flex libncurses-dev libssl-dev u-boot-tools
```
* For Arch-based distros
```
sudo pacman -Sy git u-boot-tools make bc hostname \
                aarch64-linux-gnu-gcc ncurses openssl base-devel
yay -Sy arm-linux-gnueabihf-gcc-stage1 
yay -S arm-linux-gnueabihf-gcc-stage2
yay -S arm-linux-gnueabihf-gcc
```

> NOTE: You don't need to install the **gnueabihf** packages if you are not compiling for 32-bits Labrador.

After installing these tools, clone this repository to your computer.
 Then, navigate to its main directory and execute its makefile.

```
git clone https://github.com/caninos-loucos/labrador-linux-5.git
cd labrador-linux-5
make
make all32
```

## Custom Build (32-bit/64-bit)

If you want to perform an custom build, follow these steps:

1) To load the configuration:

```
make config32
make config
```

> Note: This will overwrite any previously configured settings.

2) To customize which modules are compiled into your kernel image:

```
make menuconfig32
make menuconfig
```

3.1) To compile ONLY the device tree binary blob:

```
make dtbs32
make dtbs
```

3.2) To compile your kernel image and device trees:

```
make kernel32
make kernel
```

4) To reset the whole build:

```
make clean32
make clean
```

## After compilation

After successful compilation, the kernel image, device tree binaries and kernel modules should be located in the "output" folder for 64-bit architectures or "output32" folder for 32-bit architecture.

To copy the kernel onto an SD card that contains a Labrador system, execute  
``` 
sh install.sh
```  
for 64-bit architecture, or  
```
sh install32.sh
```   
for 32-bit architecture.

 > Note: If you are using a custom system or different distro, be sure to **label** the partitions to **BOOT and SYSTEM** for 32-bit systems and **linux64** for 64-bit, as the install script does not guess where the SD card is mounted.

## Installing the Bootloaders

The **bootloaders and configs** are located in the folders **boot and boot32**. You will have to figure your SD card's **mountpoint** and what is its respective **device file** by using 
```
lsblk
```
and guessing by the device size. The **device file** will generally be ```/dev/mmcblk0``` but can be something like ```/dev/sdb``` or ```/dev/sdc```.

* For 64-bits, with **device file**=/dev/mmcblk0 and **mountpoint**=/media/user/linux64
```
sudo dd if=boot/bootloader.bin of=/dev/mmcblk0 seek=1 bs=512 conv=notrunc
sudo cp boot/config.json /media/user/linux64/boot
```

* For 32-bits, with **device file**=/dev/mmcblk0 and **mountpoint**=/media/user/BOOT
```
sudo dd if=boot32/bootloader.bin of=/dev/mmcblk0 seek=1 bs=512 conv=notrunc
sudo cp boot32/uEnv.txt /media/user/BOOT
```


## Contributing

**Caninos Loucos Forum: <https://forum.caninosloucos.org.br/>**

**Caninos Loucos Website: <https://caninosloucos.org/>**

**Caninos Loucos Wiki: <https://wiki.caninosloucos.org/>**
