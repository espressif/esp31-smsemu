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

#include "i2s_freertos.h"

#include "psxcontroller.h"
#include "i2s_freertos.h"


//Read an unaligned byte.
char unalChar(const char *adr) {
	//See if the byte is in memory that can be read unaligned anyway.
	if (!(((int)adr)&0x40000000)) return *adr;
	//Nope: grab a word and distill the byte.
	int *p=(int *)((int)adr&0xfffffffc);
	int v=*p;
	int w=((int)adr&3);
	if (w==0) return ((v>>0)&0xff);
	if (w==1) return ((v>>8)&0xff);
	if (w==2) return ((v>>16)&0xff);
	if (w==3) return ((v>>24)&0xff);
}


//These are the arrays we store in the app cpu IRAM. We don't use the app
//processor, so we can freely use its IRAM for other purposes.
typedef struct {
	unsigned char videodata[256*192];
	unsigned char sram[0x8000];
} AppIramData;

AppIramData *appIramData=(AppIramData*)0x3ffa8000;

void system_load_sram(void) {
}


#define SMS_FPS 60
#define SNDRATE 22050

static void smsemu(void *arg) {
	int frameno, frameTgt;
	int lastTickCnt, tickCnt;
	int didFrame;
	int x;
	short sndleft[(SNDRATE/SMS_FPS)], sndright[(SNDRATE/SMS_FPS)];
	sms.use_fm=0;
	sms.country=TYPE_OVERSEAS;
	sms.dummy=appIramData->videodata; //A normal cart shouldn't access this memory ever. Point it to vram just in case.
	sms.sram=appIramData->sram;
	bitmap.data=appIramData->videodata;
	bitmap.width=256;
	bitmap.height=192;
	bitmap.pitch=256;
	bitmap.depth=8;
	
	cart.pages=((512*1024)/0x4000);
	cart.rom=(char*)0x40180000;
	cart.type=TYPE_SMS;

	lcdInit();

	system_init(SNDRATE);
	printf("Sound buffer: %d samples, enabled=%d.\n", snd.bufsize, snd.enabled);
	lastTickCnt=0;
	while(1) {
		//tickCnt would be 100tick/sec, but because we're running at double the clock speed without informing
		//the RTOS, it's 200tick/sec
		for (frameno=0; frameno<SMS_FPS; frameno++) {
			tickCnt=xTaskGetTickCount();
			if (tickCnt==lastTickCnt) tickCnt++;
			frameTgt=((tickCnt-lastTickCnt)*SMS_FPS)/200;
			frameTgt+=didFrame; //Try do diffuse frames a bit... otherwise, because of the low
				//granularity of the FreeRTOS tick variable, the drawn frames will be 'clumped together'.
			if (frameTgt<=frameno) {
				psxReadInput();
				sms_frame(0);
				lcdWriteSMSFrame();
				didFrame=3;
				printf("1");
			} else {
				sms_frame(1);
				if (didFrame!=0) didFrame--;
				printf("0");
			}
			for (x=0; x<snd.bufsize; x++) {
				i2sPushSample((snd.buffer[0][x]<<16)+snd.buffer[1][x]);
			}
		}
		printf("\n");
		tickCnt=xTaskGetTickCount();
		if (tickCnt==lastTickCnt) tickCnt++;
		printf("fps=%d\n", (SMS_FPS*200)/(tickCnt-lastTickCnt));
		lastTickCnt=tickCnt;
	}
}

/******************************************************************************
 * FunctionName : user_init
 * Description  : entry of user application, init user function here
 * Parameters   : none
 * Returns      : none
*******************************************************************************/
void user_init(void)
{
	//Clock CPU at 160MHz. We kinda need the speed here...
	SET_PERI_REG_MASK(CPU_PER_CONF_REG, PRODPORT_CPUPERIOD_SEL);

	printf("SDK version:%s\n", system_get_sdk_version());
	xTaskCreate(smsemu, "smsemu"  , 2048, NULL, 3, NULL);
	psxcontrollerInit();
	i2sInit();
	i2sSetRate(SNDRATE, 0);
}

