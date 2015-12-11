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
#include "esp_common.h"
#include "gpio.h"

#include "../smsplus/shared.h"

#define SPI_RST 18
#define SPI_CS 19
#define SPI_CLK 20
#define SPI_DAT 21

void lcdSpiWrite(int data) {
	int bit;
	GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<SPI_CS));
	for (bit=0x100; bit!=0; bit>>=1) {
		if (data&bit) {
			GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<SPI_DAT));
		} else {
			GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<SPI_DAT));
		}
		GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<SPI_CLK));
		GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<SPI_CLK));
	}
}

void lcdRaiseCs() {
	GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<SPI_CS));
}

void lcdLowerCs() {
	GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<SPI_CS));
}

#define SPI_WriteCMD(cmd) lcdSpiWrite(cmd)
#define SPI_WriteDAT(dat) lcdSpiWrite(dat|0x100)


void lcdInit() {
	printf("LCD init\n");
	GPIO_ConfigTypeDef gpioconf={
		(1<<SPI_CS)|(1<<SPI_CLK)|(1<<SPI_DAT)|(1<<SPI_RST), 0, GPIO_Mode_Output, GPIO_PullUp_DIS, GPIO_PullDown_DIS, GPIO_PIN_INTR_DISABLE
	};
	gpio_config(&gpioconf);
	
	lcdRaiseCs();
	vTaskDelay(10);

	GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<SPI_RST));
	vTaskDelay(150);
	GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<SPI_RST));
	vTaskDelay(10);

	SPI_WriteCMD(0x11); //Sleep Out
	vTaskDelay(120);
	//SPI_WriteCMD(0x36);
	//SPI_WriteDAT(0x40);
	SPI_WriteCMD(0xB4);
	SPI_WriteDAT(0x10);
	SPI_WriteCMD(0xB3);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x20);
	//SPI_WriteCMD(0xC0);
	//SPI_WriteDAT(0x04);
	SPI_WriteCMD(0xC5);
	SPI_WriteDAT(0x07);
	SPI_WriteCMD(0xC8); //Set Gamma
	SPI_WriteDAT(0x01);
	SPI_WriteDAT(0x36);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x02);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x1C);
	SPI_WriteDAT(0x77);
	SPI_WriteDAT(0x14);
	SPI_WriteDAT(0x67);
	SPI_WriteDAT(0x20);
	SPI_WriteDAT(0x0E);
	SPI_WriteDAT(0x00);
	SPI_WriteCMD(0xD0);   //Set Power
	SPI_WriteDAT(0x44);   //DDVDH
	SPI_WriteDAT(0x41);
	SPI_WriteDAT(0x08);   //VREG1
	SPI_WriteDAT(0xC2);
	SPI_WriteCMD(0xD1);   //Set VCOM
	SPI_WriteDAT(0x50);   //VCOMH
	SPI_WriteDAT(0x11);   //VCOML
	SPI_WriteCMD(0xD2);   //Set NOROW
	SPI_WriteDAT(0x05);   //SAP
	SPI_WriteDAT(0x12);   //DC10
	SPI_WriteCMD(0xE9); //Set Panel
	SPI_WriteDAT(0x09);
	SPI_WriteCMD(0xEA); //Set STBA
	SPI_WriteDAT(0x03);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteCMD(0xEE); //Set EQ
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteCMD(0xED); //Set DIR TIM
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x9A);
	SPI_WriteDAT(0x9A);
	SPI_WriteDAT(0x9B);
	SPI_WriteDAT(0x9B);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0x00);
	SPI_WriteDAT(0xAE);
	SPI_WriteDAT(0xAE);
	SPI_WriteDAT(0x01);
	SPI_WriteDAT(0x9B);
	SPI_WriteDAT(0x00);

#if 0
SPI_WriteCMD(0xC6);//set rgb interface
SPI_WriteDAT(0x83);//
SPI_WriteCMD(0x29); //Display On
	vTaskDelay(50);
 
/////////////////////////////////////////////////////
SPI_WriteCMD(0x36);
SPI_WriteDAT(0x48);//
SPI_WriteCMD(0x3A);//
SPI_WriteDAT(0x55);//
#else
	SPI_WriteCMD(0xC6);//set rgb interface (this one is undocumented?)
	SPI_WriteDAT(0x83);//
	SPI_WriteCMD(0x29);//Display On - Do Not Set 
	vTaskDelay(50);
	 
	/////////////////////////////////////////////////////
	SPI_WriteCMD(0x36);
	SPI_WriteDAT(0x68);//
	SPI_WriteCMD(0x3A);//
	SPI_WriteDAT(0x55);//
#endif

	lcdRaiseCs();
	printf("Init done.\n");
}

void lcdWriteSMSFrame() {
	int index;
	int col;
	int x, y;
	int pal565[32];
	char *p=bitmap.data;

	//Convert RGB palette to 565 data as required by the LCD beforehand.
	for (x=0; x<32; x++) {
		pal565[x]=((bitmap.pal.color[x][2]>>3)<<11)|((bitmap.pal.color[x][1]>>2)<<5)|(bitmap.pal.color[x][0]>>3);
	}

	lcdLowerCs();
	SPI_WriteCMD(0x2A); //Column address set
	SPI_WriteDAT(0);
	SPI_WriteDAT(0);
	SPI_WriteDAT(0);
	SPI_WriteDAT(255);
	SPI_WriteCMD(0x2B); //Page address set
	SPI_WriteDAT(0);
	SPI_WriteDAT(0);
	SPI_WriteDAT(0);
	SPI_WriteDAT(192);
	SPI_WriteCMD(0x2C); //Memory write
	for (y=0; y<192; y++) {
		for (x=0; x<256; x++) {
			index=(*p++)&31;
			col=pal565[index];
			SPI_WriteDAT((col>>8)&0xff);
			SPI_WriteDAT(col&0xff);
		}
	}
	lcdRaiseCs();
}
