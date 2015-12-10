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

#define DR_REG_RMT_BASE 0x60016000
#define DR_REG_APB_CTRL_BASE 0x60025000
#include "espressif/esp_common.h"
#include "apb_ctrl_reg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "../smsplus/types.h"
#include "../smsplus/sound.h"
#include "../smsplus/system.h"

#include "xtensa/hal.h"

#define RMTDIV 3


#define ETS_INTR_ENABLE(inum) \
    ets_isr_unmask((1<<inum))

#define ETS_INTR_DISABLE(inum) \
    ets_isr_mask((1<<inum))


/*
We will simulate a 640x480@60Hz signal here. This is a VESA industry-standard signal.
We will run the RMT clock at slightly more than the pixel clock (80MHz/3=26.6MHz, VGA=25.1MHz) which will compress
the image slightly. The horizontal timings will be in spec.
*/

#define VGA_TICK_CLK_KHZ 80000  //=CPU clock
#define VGA_PIXEL_CLK_KHZ 25175
#define PIX_TO_TICK(x) ((VGA_TICK_CLK_KHZ*(x))/VGA_PIXEL_CLK_KHZ)
#define VGA_FP_PIXELS 16
#define VGA_SYNC_PIXELS 96
#define VGA_BP_PIXELS 48
#define VGA_VIS_PIXELS 640
#define VGA_FP_LINES 10
#define VGA_SYNC_LINES 2
#define VGA_BP_LINES 33
#define VGA_VIS_LINES 480



void video(void *arg) {
	int x;
	for (x=0; x<0x80; x++) {
		WRITE_PERI_REG(RMT_CH0DATA, 0x80010001);
	}
	for (x=0; x<0x70; x++) {
		WRITE_PERI_REG(RMT_CH0DATA, 0x80010001);
	}
	WRITE_PERI_REG(RMT_CH0DATA, 0x00000000);
	SET_PERI_REG_MASK(RMT_CH0CONF1, RMT_TX_START_CH0);
	printf("Start transmit...\n");

	while(1) {
//		while ((READ_PERI_REG(RMT_CH0STATUS)&(1<<28)))  printf(".");
//		printf("S");
//		printf("St %x\n", READ_PERI_REG(RMT_CH0ADDR));
//		WRITE_PERI_REG(RMT_CH0DATA, 0x80010001);
//		SET_PERI_REG_MASK(RMT_CH0CONF1, RMT_TX_START_CH0);
	}

}


static int vga_ypos;
static int dosync;

void timer_isr() {
	int x;
	if (dosync) {
		//All the colorburst things are done. Reload the RMT memory.
		xthal_set_ccompare(1, xthal_get_ccompare(1)+PIX_TO_TICK(VGA_SYNC_PIXELS+VGA_BP_PIXELS));
		if ((int)xthal_get_ccompare(1)-(int)xthal_get_ccount()<0) xthal_set_ccompare(1, xthal_get_ccount()+10000);

		WRITE_PERI_REG(RMT_CH0CONF1, RMT_REF_CNT_RST_CH0|RMT_APB_MEM_RST_CH0|RMT_MEM_RD_RST_CH0|RMT_MEM_WR_RST_CH0);
		WRITE_PERI_REG(RMT_CH2CONF1, RMT_REF_CNT_RST_CH0|RMT_APB_MEM_RST_CH0|RMT_MEM_RD_RST_CH1|RMT_MEM_WR_RST_CH1);
		WRITE_PERI_REG(RMT_CH4CONF1, RMT_REF_CNT_RST_CH0|RMT_APB_MEM_RST_CH0|RMT_MEM_RD_RST_CH2|RMT_MEM_WR_RST_CH2);
		WRITE_PERI_REG(RMT_CH6CONF1, RMT_REF_CNT_RST_CH0|RMT_APB_MEM_RST_CH0|RMT_MEM_RD_RST_CH3|RMT_MEM_WR_RST_CH3);
//		WRITE_PERI_REG(RMT_CH0ADDR, 0x000);
//		WRITE_PERI_REG(RMT_CH1ADDR, 0x080);
//		WRITE_PERI_REG(RMT_CH2ADDR, 0x100);

		for (x=0; x<0x78; x++) {
			WRITE_PERI_REG(RMT_CH0DATA, 0x00028002);
			WRITE_PERI_REG(RMT_CH2DATA, 0x00028002);
			WRITE_PERI_REG(RMT_CH4DATA, 0x00028002);
		}
		WRITE_PERI_REG(RMT_CH0DATA, 0x00000000);
		WRITE_PERI_REG(RMT_CH2DATA, 0x00000000);
		WRITE_PERI_REG(RMT_CH4DATA, 0x00000000);

		vga_ypos++;
		if (vga_ypos==(VGA_FP_LINES+VGA_SYNC_LINES+VGA_BP_LINES+VGA_VIS_LINES)) {
			vga_ypos=0;
//			printf("r\n");
		}
		dosync=0;
	} else {
		//We need to kick off the RMT color outputs
		xthal_set_ccompare(1, xthal_get_ccompare(1)+PIX_TO_TICK(VGA_VIS_PIXELS+VGA_FP_PIXELS));
		if ((int)xthal_get_ccompare(1)-(int)xthal_get_ccount()<0) xthal_set_ccompare(1, xthal_get_ccount()+10000);
		SET_PERI_REG_MASK(RMT_CH0CONF1, RMT_TX_START_CH0);
		SET_PERI_REG_MASK(RMT_CH2CONF1, RMT_TX_START_CH2);
		SET_PERI_REG_MASK(RMT_CH4CONF1, RMT_TX_START_CH4);
		SET_PERI_REG_MASK(RMT_CH6CONF1, RMT_TX_START_CH6);
		dosync=1;
	}
}



//XCHAL_TIMER_INTERRUPT(XT_TIMER_INDEX) = 15
//XCHAL_INT_LEVEL(XT_TIMER_INTNUM) should be <= XCHAL_EXCM_LEVEL(3). 0 is already used.

void video_setup() {
	WRITE_PERI_REG(GPIO_ENABLE,0xfffffff);//ENABLE gpio19 gpio4 oe_enable
       
       //87 rmt_ch0 send
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL4,GPIO_GPIO_FUNC19_OUT_SEL, 87+0, GPIO_GPIO_FUNC19_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO19_U, MCU_SEL, 0, MCU_SEL_S);

	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL5,GPIO_GPIO_FUNC20_OUT_SEL, 87+2, GPIO_GPIO_FUNC20_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO20_U, MCU_SEL, 0, MCU_SEL_S);

	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL5,GPIO_GPIO_FUNC21_OUT_SEL, 87+4, GPIO_GPIO_FUNC21_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO21_U, MCU_SEL, 0, MCU_SEL_S);

	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL5,GPIO_GPIO_FUNC22_OUT_SEL, 87+6, GPIO_GPIO_FUNC22_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO22_U, MCU_SEL, 0, MCU_SEL_S);

	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL5,GPIO_GPIO_FUNC23_OUT_SEL, 87+7, GPIO_GPIO_FUNC23_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO23_U, MCU_SEL, 0, MCU_SEL_S);

	//RMT 80MHz input
	WRITE_PERI_REG(APB_CTRL_PLL_TICK_CONF, 1);

	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL4,GPIO_GPIO_FUNC19_OUT_SEL, 87, GPIO_GPIO_FUNC19_OUT_SEL_S);//GPIO19
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO19_U, MCU_SEL, 0, MCU_SEL_S);

	//Channels 0-4 are r, g, b, hsync, vsync respectively.
	WRITE_PERI_REG(RMT_CH0CONF0, (2<<RMT_MEM_SIZE_CH0_S)|(RMTDIV<<RMT_DIV_CNT_CH0_S)|RMT_CARRIER_EN_CH0);
	WRITE_PERI_REG(RMT_CH0CONF1,  RMT_MEM_RD_RST_CH0|RMT_MEM_WR_RST_CH0);
	WRITE_PERI_REG(RMT_CH0CARRIER_DUTY, 0x01000000);

	WRITE_PERI_REG(RMT_CH2CONF0, (2<<RMT_MEM_SIZE_CH2_S)|(RMTDIV<<RMT_DIV_CNT_CH2_S)|RMT_CARRIER_EN_CH2);
	WRITE_PERI_REG(RMT_CH2CONF1,  RMT_MEM_RD_RST_CH2|RMT_MEM_WR_RST_CH2);
	WRITE_PERI_REG(RMT_CH2CARRIER_DUTY, 0x01000000);

	WRITE_PERI_REG(RMT_CH4CONF0, (2<<RMT_MEM_SIZE_CH4_S)|(RMTDIV<<RMT_DIV_CNT_CH4_S)|RMT_CARRIER_EN_CH4);
	WRITE_PERI_REG(RMT_CH4CONF1,  RMT_MEM_RD_RST_CH4|RMT_MEM_WR_RST_CH4);
	WRITE_PERI_REG(RMT_CH4CARRIER_DUTY, 0x01000000);

	WRITE_PERI_REG(RMT_CH6CONF0, (1<<RMT_MEM_SIZE_CH6_S)|(RMTDIV<<RMT_DIV_CNT_CH6_S)|RMT_CARRIER_EN_CH6);
	WRITE_PERI_REG(RMT_CH6CONF1,  RMT_MEM_RD_RST_CH6|RMT_MEM_WR_RST_CH6);
	WRITE_PERI_REG(RMT_CH6CARRIER_DUTY, 0x01000000);

	WRITE_PERI_REG(RMT_CH7CONF0, (1<<RMT_MEM_SIZE_CH7_S)|(RMTDIV<<RMT_DIV_CNT_CH7_S)|RMT_CARRIER_EN_CH7);
	WRITE_PERI_REG(RMT_CH7CONF1,  RMT_MEM_RD_RST_CH7|RMT_MEM_WR_RST_CH7);
	WRITE_PERI_REG(RMT_CH7CARRIER_DUTY, 0x01000000);

	//Ch 6 is fixed because Hsync. We can write the data now and never bother with it again.
	WRITE_PERI_REG(RMT_CH6DATA, ((0x8000|((VGA_BP_PIXELS+VGA_VIS_PIXELS+VGA_FP_PIXELS)/2))<<16)|(VGA_SYNC_PIXELS/2));
	WRITE_PERI_REG(RMT_CH6DATA, 0);

	//Schedule timer at random point in the future to kick things off
	xthal_set_ccompare(1, xthal_get_ccount()+80000000L);
	xt_set_interrupt_handler(XCHAL_TIMER_INTERRUPT(1), timer_isr, NULL);
	xt_ints_on(1<<XCHAL_TIMER_INTERRUPT(1));
	printf("Video initialized.\n");
}


//unsigned char videodata[256*192];
unsigned char videodata[1];

/*
void system_manage_sram(uint8 *sram, int slot, int mode) {

}
*/

static void smsemu(void *arg) {
	snd.sample_rate=0;
	bitmap.data=videodata;
	bitmap.width=256;
	bitmap.height=192;
	bitmap.pitch=256;
	bitmap.depth=8;
	bitmap.granularity=1;
	
	
	system_init();
	system_poweron();
	while(1) {
		system_frame(0);
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
    printf("SDK version:%s\n", system_get_sdk_version());
	video_setup();
//	xTaskCreate(video, "video"  , 2048, NULL, 3, NULL);
//	xTaskCreate(smsemu, "smsemu"  , 2048, NULL, 3, NULL);
}

