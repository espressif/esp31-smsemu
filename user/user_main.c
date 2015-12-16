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

#include "psxcontroller.h"

char unalChar(const char *adr) {
	if (!(((int)adr)&0x40000000)) return *adr;
	int *p=(int *)((int)adr&0xfffffffc);
	int v=*p;
	int w=((int)adr&3);
	if (w==0) return ((v>>0)&0xff);
	if (w==1) return ((v>>8)&0xff);
	if (w==2) return ((v>>16)&0xff);
	if (w==3) return ((v>>24)&0xff);
}



//Abuse app iram as mem buffer
char *videodata=(char*)0x3ffa8000;


void system_load_sram(void) {
}


#define SMS_FPS 60

static void smsemu(void *arg) {
	int frameno, frameTgt;
	int lastTickCnt, tickCnt;
	int didFrame;
	sms.use_fm=0;
	sms.country=TYPE_OVERSEAS;
	sms.dummy=(uint8*)0x3ffa8000; //Redirect dummy accesses to ROM. Write is a no-op and reads don't matter.
	sms.sram=&videodata[256*192]; //We put the SRAM in the app cpu icache, after the videodata.
	bitmap.data=videodata;
	bitmap.width=256;
	bitmap.height=192;
	bitmap.pitch=256;
	bitmap.depth=8;
	
	cart.pages=((512*1024)/0x4000);
	cart.rom=(char*)0x40180000;
	cart.type=TYPE_SMS;
//	snd.bufsize=16;
	
	lcdInit();

	system_init(0);
	sms_init();
	lastTickCnt=0;
	while(1) {
		//tickCnt would be 100tick/sec, but because we're running at double the clock speed without informing
		//the RTOS, it's 200tick/sec
		for (frameno=0; frameno<SMS_FPS; frameno++) {
			tickCnt=xTaskGetTickCount();
			if (tickCnt==lastTickCnt) tickCnt++;
			frameTgt=((tickCnt-lastTickCnt)*SMS_FPS)/200;
			frameTgt+=didFrame; //Try do diffuse frames a bit...
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
}

