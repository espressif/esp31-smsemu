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

#define RMTDIV 2


#define ETS_INTR_ENABLE(inum) \
    ets_isr_unmask((1<<inum))

#define ETS_INTR_DISABLE(inum) \
    ets_isr_mask((1<<inum))


void intr_matrix_set(uint32 model_num, uint32 intr_num)
{
	uint32 addr = INTR_MAP_REG_A + (model_num / 6) * 4;
	uint32 shift = (model_num % 6) * 5;

	if (shift >= 15) {
		shift++;
	}

	SET_PERI_REG_BITS(addr, 0x1f, intr_num, shift);//0x80
}


void timer1_schedule(int tickCount) {
	xthal_set_ccompare(1, xthal_get_ccount()+tickCount);
}


void video(void *arg) {
	int x;
	SET_PERI_REG_BITS(0x60009034, MCU_SEL, 2, MCU_SEL_S);//MIDI_MCU_SEL
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL3,GPIO_GPIO_FUNC14_OUT_SEL, 80, GPIO_GPIO_FUNC14_OUT_SEL_S);//GPIO14
	SET_PERI_REG_BITS(0x60009030, MCU_SEL, 2, MCU_SEL_S);//MTMS_MCU_SEL
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL3,GPIO_GPIO_FUNC15_OUT_SEL, 81, GPIO_GPIO_FUNC15_OUT_SEL_S);//GPIO15
       SET_PERI_REG_BITS(0x6000903c, MCU_SEL, 2, MCU_SEL_S);//MTDO_MCU_SEL
       WRITE_PERI_REG(GPIO_ENABLE,0xffffff);//ENABLE gpio19 gpio4 oe_enable
       
       //87 rmt_ch0 send
       SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL4,GPIO_GPIO_FUNC19_OUT_SEL, 87, GPIO_GPIO_FUNC19_OUT_SEL_S);//GPIO19
       SET_PERI_REG_BITS(0x60009074, MCU_SEL, 0, MCU_SEL_S);//
	//rmt_ch1 receive 84
	SET_PERI_REG_BITS(GPIO_FUNC_IN_SEL16,GPIO_GPIO_FUNC84_IN_SEL, 27, GPIO_GPIO_FUNC84_IN_SEL_S);//GPIO27
	SET_PERI_REG_MASK(0x6000902c, FUN_IE);//ie
	//SET_PERI_REG_MASK(0x6000902c, FUN_PU);//
	SET_PERI_REG_MASK(0x6000902c, FUN_PD);//
	   
       //WRITE_PERI_REG(GPIO_ENABLE,((0x1<<19)||(0x1<<4)));//ENABLE gpio19 gpio4 oe_enable
       
       //DEBUG GPIO
//	SET_PERI_REG_BITS(GPIO_PIN_REG_4,MCU_SEL,0,MCU_SEL_S);
//	SET_PERI_REG_MASK(GPIO_ENABLE,BIT(4));
//	SET_PERI_REG_BITS(GPIO_PIN_REG_0,MCU_SEL,0,MCU_SEL_S);
//	SET_PERI_REG_MASK(GPIO_ENABLE,BIT(0));
	//clear gpio0 out
//	CLEAR_PERI_REG_MASK(GPIO_OUT,BIT(4));
	//SET_PERI_REG_MASK(GPIO_OUT,BIT(4));  
//	CLEAR_PERI_REG_MASK(GPIO_OUT,BIT(0));
	
        //enable channel interrupt
//	 SET_PERI_REG_MASK(RMT_INT_ENA, RMT_RMT_CH0_TX_END_INT_ENA); 
//	 SET_PERI_REG_MASK(RMT_INT_ENA, RMT_RMT_CH1_RX_END_INT_ENA); 
//	 SET_PERI_REG_MASK(RMT_INT_ENA, RMT_RMT_CH1_ERR_INT_ENA);	 
	 
         //xtal 26mhz /(PB_CTRL_APB_CTRL_XTAL_TICK_NUM+1)
//         SET_PERI_REG_BITS(RTC_CLK_CONF, RTC_CNTL_SOC_CLK_SEL, 0, RTC_CNTL_SOC_CLK_SEL_S);
//	  SET_PERI_REG_BITS(APB_CTRL_XTAL_TICK_CONF,APB_CTRL_APB_CTRL_XTAL_TICK_NUM,25,APB_CTRL_APB_CTRL_XTAL_TICK_NUM_S);


  
         //pll 160mhz /(PB_CTRL_APB_CTRL_XTAL_TICK_NUM+1)
//         SET_PERI_REG_BITS(RTC_CLK_CONF, RTC_CNTL_SOC_CLK_SEL, 1, RTC_CNTL_SOC_CLK_SEL_S);
//	  SET_PERI_REG_BITS(APB_CTRL_PLL_TICK_CONF,APB_CTRL_APB_CTRL_PLL_TICK_NUM,159,APB_CTRL_APB_CTRL_PLL_TICK_NUM_S);

	WRITE_PERI_REG(APB_CTRL_PLL_TICK_CONF, 1);

	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL4,GPIO_GPIO_FUNC19_OUT_SEL, 87, GPIO_GPIO_FUNC19_OUT_SEL_S);//GPIO19
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO19_U, MCU_SEL, 0, MCU_SEL_S);

	WRITE_PERI_REG(RMT_CH0CONF0, (4<<RMT_MEM_SIZE_CH0_S)|(RMTDIV<<RMT_DIV_CNT_CH0_S)|RMT_CARRIER_EN_CH0);
	WRITE_PERI_REG(RMT_CH0CONF1,  RMT_TX_CONTI_MODE_CH0|RMT_MEM_RD_RST_CH0|RMT_MEM_WR_RST_CH0);
	WRITE_PERI_REG(RMT_CH0CARRIER_DUTY, 0x01000000);
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


void timer_isr() {
	printf("timer\n");
	timer1_schedule(80000000);
}



//XCHAL_TIMER_INTERRUPT(XT_TIMER_INDEX) = 15
//XCHAL_INT_LEVEL(XT_TIMER_INTNUM) should be <= XCHAL_EXCM_LEVEL(3). 0 is already used.

void video_setup() {
	xt_set_interrupt_handler(XCHAL_TIMER_INTERRUPT(1), timer_isr, NULL);
	xt_ints_on(1<<XCHAL_TIMER_INTERRUPT(1));
	timer1_schedule(80000000);
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
	xTaskCreate(video, "video"  , 2048, NULL, 3, NULL);
//	xTaskCreate(smsemu, "smsemu"  , 2048, NULL, 3, NULL);
}

