#!/bin/bash
set -e

function flash() {
	./../esptool-master_FOR_8689/esptool.py -p /dev/ttyUSB0 -b $((115200*2)) write_flash -fs 16m \
		0x040000 ${BIN_PATH}/irom0_flash.bin \
		0x004000 ${BIN_PATH}/irom1.bin \

#		0x140000 smscart.img
}

function flashretry() {
	flash $1 $2 || flash $1 $2
}

#./../esptool-master_FOR_8689/esptool.py -p /dev/ttyUSB0 -b $((115200*2)) write_flash -fs 16m 0x80000 smscart.img



make clean
make
#./../esptool-master_FOR_8689/esptool.py -p /dev/ttyUSB0 -b $((115200)) write_flash 0x0 ${BIN_PATH}/boot.bin
flash || flash