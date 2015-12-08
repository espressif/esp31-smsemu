#!/bin/bash
set -e

function flash() {
	./../esptool-master_FOR_8689/esptool.py -p /dev/ttyUSB0 -b $((115200*2)) write_flash -fs 16m \
		0x40000 ${BIN_PATH}/irom0_flash.bin \
		0x4000 ${BIN_PATH}/irom1.bin \

}

function flashretry() {
	flash $1 $2 || flash $1 $2
}

make clean
make
#./../esptool-master_FOR_8689/esptool.py -p /dev/ttyUSB0 -b $((115200)) write_flash 0x0 ${BIN_PATH}/boot.bin
flash || flash