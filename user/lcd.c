#pragma GCC optimize ("O2")

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


/*
LCD driver for ILI9341 on TM022 LCD
*/

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_common.h"
#include "gpio.h"
#include "spi_register.h"

#include "../smsplus/shared.h"

#define SPI_RST 18
#define SPI_CS 19
#define SPI_CLK 20
#define SPI_DAT 21
#define SPI_DC 22


uint32_t lcdData[16]; //can contain (16*32/9=)56 9-bit data words.
int lcdDataPos=0;

#define SPIDEV 3

void lcdSpiSend(int isCmd) {
	int x=0;

	while(READ_PERI_REG(SPI_CMD(SPIDEV)) & SPI_USR);
	if (isCmd) {
		GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<SPI_DC));
	} else {
		GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<SPI_DC));
	}

	for (x=0; x<16; x++) {
		WRITE_PERI_REG(SPI_W0(SPIDEV)+(x*4), lcdData[x]);
		lcdData[x]=0;
	}
	WRITE_PERI_REG(SPI_USER1(SPIDEV), (((lcdDataPos*8)-1)<<SPI_USR_MOSI_BITLEN_S));
	WRITE_PERI_REG(SPI_CMD(SPIDEV), SPI_USR);
	lcdDataPos=0;
}

void lcdSpiWrite(int data) {
	int bytePos=(lcdDataPos/4);
	if ((lcdDataPos&3)==0) lcdData[bytePos]|=data<<24;
	if ((lcdDataPos&3)==1) lcdData[bytePos]|=data<<16;
	if ((lcdDataPos&3)==2) lcdData[bytePos]|=data<<8;
	if ((lcdDataPos&3)==3) lcdData[bytePos]|=data<<0;
	lcdDataPos++;
}

void SPI_WriteCMD(int cmd) {
	if (lcdDataPos!=0) lcdSpiSend(0);
	lcdSpiWrite(cmd);
	lcdSpiSend(1);
}

void SPI_WriteDAT(int dat) {
	lcdSpiWrite(dat&0xff);
}




int lcdXpos=0;
int lcdYpos=0;
int pal565[32];
char *bmData;

//We use a timer to poke us when the SPI transmission is done. This define indicates when that happens.
#define SENDTICKS 3200

void lcdPumpPixels() {
	int col;
	if (lcdYpos==192) {
		//ack int
		xthal_set_ccompare(1, xthal_get_ccount()-1);
		//disable int
		xt_ints_off(1<<XCHAL_TIMER_INTERRUPT(1));
		return;
	}

	do {
		col=pal565[(*bmData++)&31];
		SPI_WriteDAT((col>>8));
		SPI_WriteDAT(col);
		lcdXpos++;
	} while (lcdXpos&31);
	lcdSpiSend(0);
	if (lcdXpos==256) {
		lcdXpos=0;
		lcdYpos++;
	}
	xthal_set_ccompare(1, xthal_get_ccount()+SENDTICKS);
}


void lcdWriteSMSFrame() {
	int index;
	int col;
	int x, y;

	//Convert RGB palette to 565 data as required by the LCD beforehand.
	for (x=0; x<32; x++) {
		pal565[x]=((bitmap.pal.color[x][0]>>3)<<11)|((bitmap.pal.color[x][1]>>2)<<5)|(bitmap.pal.color[x][2]>>3);
	}
	///Center image
	#define XSTART 32
	#define YSTART 24

	SPI_WriteCMD(0x2A); //Column address set
	SPI_WriteDAT((XSTART)>>8);
	SPI_WriteDAT((XSTART)&0xff);
	SPI_WriteDAT((XSTART+255)>>8);
	SPI_WriteDAT((XSTART+255)&0xff);
	SPI_WriteCMD(0x2B); //Page address set
	SPI_WriteDAT((YSTART)>>8);
	SPI_WriteDAT((YSTART)&0xff);
	SPI_WriteDAT((YSTART+192)>>8);
	SPI_WriteDAT((YSTART+192)&0xff);
	SPI_WriteCMD(0x2C); //Memory write
	bmData=bitmap.data;
	lcdXpos=0;
	lcdYpos=0;

	xt_ints_on(1<<XCHAL_TIMER_INTERRUPT(1));
	lcdPumpPixels();
}


void lcdInit() {
	int x;
	printf("LCD init\n");
	//Init reset
	GPIO_ConfigTypeDef gpioconf={(1<<SPI_RST)|(1<<SPI_DC), 0, GPIO_Mode_Output, GPIO_PullUp_DIS, GPIO_PullDown_DIS, GPIO_PIN_INTR_DISABLE };
	gpio_config(&gpioconf);

	//SPI clk = 40MHz
	WRITE_PERI_REG(SPI_CLOCK(SPIDEV), (0<<SPI_CLKDIV_PRE_S) |
		(1<<SPI_CLKCNT_N_S)|(1<<SPI_CLKCNT_L_S)|(0<<SPI_CLKCNT_H_S));
	WRITE_PERI_REG(SPI_CTRL(SPIDEV), 0);//SPI_WR_BIT_ORDER);
	WRITE_PERI_REG(SPI_USER(SPIDEV), SPI_CS_SETUP|SPI_CS_HOLD|SPI_USR_MOSI|SPI_WR_BYTE_ORDER);
	WRITE_PERI_REG(SPI_USER1(SPIDEV), (9<<SPI_USR_MOSI_BITLEN_S));

	//Route SPI to pins
	WRITE_PERI_REG(GPIO_ENABLE,0xfffffff);//ENABLE gpio19 gpio4 oe_enable
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL4,GPIO_GPIO_FUNC19_OUT_SEL, VSPICS0_OUT_IDX, GPIO_GPIO_FUNC19_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO19_U, MCU_SEL, 0, MCU_SEL_S);
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL5,GPIO_GPIO_FUNC20_OUT_SEL, VSPICLK_OUT_MUX_IDX, GPIO_GPIO_FUNC20_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO20_U, MCU_SEL, 0, MCU_SEL_S);
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL5,GPIO_GPIO_FUNC21_OUT_SEL, VSPID_OUT_IDX, GPIO_GPIO_FUNC21_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO21_U, MCU_SEL, 0, MCU_SEL_S);

	lcdSpiWrite(0); //dummy
	lcdSpiSend(0);

	vTaskDelay(10);
	GPIO_REG_WRITE(GPIO_OUT_W1TC, (1<<SPI_RST));
	vTaskDelay(150);
	GPIO_REG_WRITE(GPIO_OUT_W1TS, (1<<SPI_RST));
	vTaskDelay(10);

	SPI_WriteCMD(0xCB);  
	SPI_WriteDAT(0x39); 
	SPI_WriteDAT(0x2C); 
	SPI_WriteDAT(0x00); 
	SPI_WriteDAT(0x34); 
	SPI_WriteDAT(0x02); 

	SPI_WriteCMD(0xCF);  
	SPI_WriteDAT(0x00); 
	SPI_WriteDAT(0XC1); 
	SPI_WriteDAT(0X30); 
 
	SPI_WriteCMD(0xE8);  
	SPI_WriteDAT(0x85); 
	SPI_WriteDAT(0x00); 
	SPI_WriteDAT(0x78); 
 
	SPI_WriteCMD(0xEA);  
	SPI_WriteDAT(0x00); 
	SPI_WriteDAT(0x00); 
 
	SPI_WriteCMD(0xED);  
	SPI_WriteDAT(0x64); 
	SPI_WriteDAT(0x03); 
	SPI_WriteDAT(0X12); 
	SPI_WriteDAT(0X81); 

	SPI_WriteCMD(0xF7);  
	SPI_WriteDAT(0x20); 
  
	SPI_WriteCMD(0xC0);    //Power control 
	SPI_WriteDAT(0x23);   //VRH[5:0] 
 
	SPI_WriteCMD(0xC1);    //Power control 
	SPI_WriteDAT(0x10);   //SAP[2:0];BT[3:0] 
 
	SPI_WriteCMD(0xC5);    //VCM control 
	SPI_WriteDAT(0x3e);
	SPI_WriteDAT(0x28); 
 
	SPI_WriteCMD(0xC7);    //VCM control2 
	SPI_WriteDAT(0x86);  //--
 
	SPI_WriteCMD(0x36);    // Memory Access Control 
	SPI_WriteDAT(0x28);

	SPI_WriteCMD(0x3A);    
	SPI_WriteDAT(0x55); 

	SPI_WriteCMD(0xB1);    
	SPI_WriteDAT(0x00);  
	SPI_WriteDAT(0x18); 
 
	SPI_WriteCMD(0xB6);    // Display Function Control 
	SPI_WriteDAT(0x08); 
	SPI_WriteDAT(0x82);
	SPI_WriteDAT(0x27);  
 
	SPI_WriteCMD(0xF2);    // 3Gamma Function Disable 
	SPI_WriteDAT(0x00); 
 
	SPI_WriteCMD(0x26);    //Gamma curve selected 
	SPI_WriteDAT(0x01); 
 
	SPI_WriteCMD(0xE0);    //Set Gamma 
	SPI_WriteDAT(0x0F); 
	SPI_WriteDAT(0x31); 
	SPI_WriteDAT(0x2B); 
	SPI_WriteDAT(0x0C); 
	SPI_WriteDAT(0x0E); 
	SPI_WriteDAT(0x08); 
	SPI_WriteDAT(0x4E); 
	SPI_WriteDAT(0xF1); 
	SPI_WriteDAT(0x37); 
	SPI_WriteDAT(0x07); 
	SPI_WriteDAT(0x10); 
	SPI_WriteDAT(0x03); 
	SPI_WriteDAT(0x0E); 
	SPI_WriteDAT(0x09); 
	SPI_WriteDAT(0x00); 

	SPI_WriteCMD(0XE1);    //Set Gamma 
	SPI_WriteDAT(0x00); 
	SPI_WriteDAT(0x0E); 
	SPI_WriteDAT(0x14); 
	SPI_WriteDAT(0x03); 
	SPI_WriteDAT(0x11); 
	SPI_WriteDAT(0x07); 
	SPI_WriteDAT(0x31); 
	SPI_WriteDAT(0xC1); 
	SPI_WriteDAT(0x48); 
	SPI_WriteDAT(0x08); 
	SPI_WriteDAT(0x0F); 
	SPI_WriteDAT(0x0C); 
	SPI_WriteDAT(0x31); 
	SPI_WriteDAT(0x36); 
	SPI_WriteDAT(0x0F); 
 
	SPI_WriteCMD(0x11);    //Exit Sleep 
	vTaskDelay(10);
				
	SPI_WriteCMD(0x29);    //Display on 
	SPI_WriteCMD(0x2c); 
	lcdSpiSend(0);

	xt_set_interrupt_handler(XCHAL_TIMER_INTERRUPT(1), lcdPumpPixels, NULL);
//	xt_ints_on(1<<XCHAL_TIMER_INTERRUPT(1));

	printf("Init done.\n");
}

