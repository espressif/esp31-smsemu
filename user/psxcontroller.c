/*
 * ESPRSSIF MIT License
 *
 * Copyright (c) 2015 <ESPRESSIF SYSTEMS (SHANGHAI) PTE LTD>
 *
 * Permission is hereby granted for use on ESPRESSIF SYSTEMS ESP8266 only, in which case,
 * it is free of charge, to any person obtaining a copy of this software and associated
 * documentation files (the "Software"), to deal in the Software without restriction, including
 * without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense,
 * and/or sell copies of the Software, and to permit persons to whom the Software is furnished
 * to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all copies or
 * substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS
 * FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR
 * COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER
 * IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN
 * CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
 *
 */


#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "../smsplus/shared.h"
#include "esp_common.h"
#include "esp32/esp32.h"

#include "gpio.h"
#include "psxcontroller.h"

#define PSX_CLK 14
#define PSX_DAT 25
#define PSX_ATT 16
#define PSX_CMD 17

/* Sends and receives a byte from/to the PSX controller using SPI */
int psxSendRecv(int send) {
	int x;
	int ret=0;
	volatile int delay;
	GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<PSX_ATT));
	for (delay=0; delay<100; delay++);
	for (x=0; x<8; x++) {
		if (send&1) {
			GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<PSX_CMD));
		} else {
			GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<PSX_CMD));
		}
		GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<PSX_CLK));
		for (delay=0; delay<100; delay++);
		GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<PSX_CLK));
		for (delay=0; delay<100; delay++);
		ret>>=1;
		send>>=1;
		if (GPIO_REG_READ(GPIO_IN)&(1<<PSX_DAT)) ret|=128;
	}
	return ret;
}

int psxDone() {
	GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<PSX_ATT));
}


void psxReadInput() {
	int b1, b2;
	int smsButtons;
	int smsSystem;

	psxSendRecv(0x01); //wake up
	psxSendRecv(0x42); //get data
	psxSendRecv(0xff); //should return 0x5a
	b1=psxSendRecv(0xff); //buttons byte 1
	b2=psxSendRecv(0xff); //buttons byte 2
	psxDone();

	//Convert PSX buttons to SMS buttons
	smsButtons=0; smsSystem=0;
	if (!(b1&(1<<4))) smsButtons|=INPUT_UP;
	if (!(b1&(1<<6))) smsButtons|=INPUT_DOWN;
	if (!(b1&(1<<7))) smsButtons|=INPUT_LEFT;
	if (!(b1&(1<<5))) smsButtons|=INPUT_RIGHT;
	if (!(b2&(1<<5))) smsButtons|=INPUT_BUTTON1;
	if (!(b2&(1<<6))) smsButtons|=INPUT_BUTTON2;
	if (!(b1&(1<<3))) smsSystem|=INPUT_START;
	if (!(b1&(1<<0))) smsSystem|=INPUT_PAUSE;
	if (!(b2&(1<<4))) smsSystem|=INPUT_SOFT_RESET;
	if (!(b2&(1<<7))) smsSystem|=INPUT_HARD_RESET;
	input.pad[0]=smsButtons;
	input.system=smsSystem;
}


void psxcontrollerInit() {
	volatile int delay;
	int t;
	GPIO_ConfigTypeDef gpioconf[2]={
		{(1<<PSX_CLK)|(1<<PSX_CMD)|(1<<PSX_ATT), 0, GPIO_Mode_Output, GPIO_PullUp_DIS, GPIO_PullDown_DIS, GPIO_PIN_INTR_DISABLE},
		{(1<<PSX_DAT), 0, GPIO_Mode_Input, GPIO_PullUp_DIS, GPIO_PullDown_EN, GPIO_PIN_INTR_DISABLE},
	};
	gpio_config(&gpioconf[0]);
	gpio_config(&gpioconf[1]);
	
	//Send a few dummy bytes to clean the pipes.
	psxSendRecv(0);
	psxDone();
	for (delay=0; delay<500; delay++);
	psxSendRecv(0);
	psxDone();
	for (delay=0; delay<500; delay++);
	//Try and detect the type of controller, so we can give the user some diagnostics.
	psxSendRecv(0x01);
	t=psxSendRecv(0x00);
	psxDone();
	if (t==0 || t==0xff) {
		printf("No PSX/PS2 controller detected (0x%X). You will not be able to control the game.\n", t);
	} else {
		printf("PSX controller type 0x%X\n", t);
	}
}