/******************************************************************************
 * Copyright 2013-2015 Espressif Systems
 *
 * FileName: i2s_freertos.c
 *
 * Description: I2S output routines for a FreeRTOS system. Uses DMA and a queue
 * to abstract away the nitty-gritty details.
 *
 * Modification history:
 *     2015/06/01, v1.0 File created.
 *     2015/12/17, Adapted for ESP31
*******************************************************************************/

/*
How does this work? Basically, to get sound, you need to:
- Connect an I2S codec to the I2S pins on the ESP.
- Start up a thread that's going to do the sound output
- Call I2sInit()
- Call I2sSetRate() with the sample rate you want.
- Generate sound and call i2sPushSample() with 32-bit samples.
The 32bit samples basically are 2 16-bit signed values (the analog values for
the left and right channel) concatenated as (Rout<<16)+Lout

I2sPushSample will block when you're sending data too quickly, so you can just
generate and push data as fast as you can and I2sPushSample will regulate the
speed.
*/

#include "espressif/esp_common.h"
#include "espressif/esp32/i2s/i2s_reg.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"

#include "i2s_freertos.h"

struct sdio_queue {
    uint32  blocksize: 12;
    uint32  datalen  : 12;
    uint32  unused   :  5;
    uint32  sub_sof  :  1;
    uint32  eof      :  1;
    uint32  owner    :  1;

    uint32  buf_ptr;
    uint32  next_link_ptr;
};


//Pointer to the I2S DMA buffer data
static unsigned int *i2sBuf[I2SDMABUFCNT];
//I2S DMA buffer descriptors
static struct sdio_queue i2sBufDesc[I2SDMABUFCNT];
//Queue which contains empty DMA buffers
static xQueueHandle dmaQueue;
//DMA underrun counter
static long underrunCnt;

//This routine is called as soon as the DMA routine has something to tell us. All we
//handle here is the RX_EOF_INT status, which indicate the DMA has sent a buffer whose
//descriptor has the 'EOF' field set to 1.
LOCAL void slc_isr(void) {
	portBASE_TYPE HPTaskAwoken=0;
	struct sdio_queue *finishedDesc;
	uint32 slc_intr_status;
	int dummy;

	slc_intr_status = READ_PERI_REG(I2SINT_ST);
	if (slc_intr_status == 0) {
		//No interested interrupts pending
		return;
	}
	//clear all intrs
	WRITE_PERI_REG(I2SINT_CLR, 0xffffffff);
	if (slc_intr_status & I2S_I2S_TO_EOF_INT_ST) {
		//The DMA subsystem is done with this block: Push it on the queue so it can be re-used.
		finishedDesc=(struct sdio_queue*)READ_PERI_REG(I2S_TO_EOF_DES_ADDR);
		if (xQueueIsQueueFullFromISR(dmaQueue)) {
			//All buffers are empty. This means we have an underflow on our hands.
			underrunCnt++;
			//Pop the top off the queue; it's invalid now anyway.
			xQueueReceiveFromISR(dmaQueue, &dummy, &HPTaskAwoken);
		}
		//Dump the buffer on the queue so the rest of the software can fill it.
		xQueueSendFromISR(dmaQueue, (void*)(&finishedDesc->buf_ptr), &HPTaskAwoken);
	}
	//We're done.
	portEND_SWITCHING_ISR(HPTaskAwoken);
}


//Initialize I2S subsystem for DMA circular buffer use
void ICACHE_FLASH_ATTR i2sInit() {
	int x, y;
	
	underrunCnt=0;
	
	
	//Take care of the DMA buffers.
	for (y=0; y<I2SDMABUFCNT; y++) {
		//Allocate memory for this DMA sample buffer.
		i2sBuf[y]=malloc(I2SDMABUFLEN*4);
		//Clear sample buffer. We don't want noise.
		for (x=0; x<I2SDMABUFLEN; x++) {
			i2sBuf[y][x]=0;
		}
	}

	//Reset DMA
	SET_PERI_REG_MASK(I2S_LC_CONF, I2S_I2S_RX_RST | I2S_I2S_TX_RST_S);
	CLEAR_PERI_REG_MASK(I2S_LC_CONF, I2S_I2S_RX_RST | I2S_I2S_TX_RST_S);

	//Enable and configure DMA
	SET_PERI_REG_MASK(I2S_LC_CONF, I2S_I2S_CHECK_OWNER | I2S_I2S_TO_EOF_MODE);

	//Configure interrupt
	SET_PERI_REG_BITS(INTR_MAP_REG_B, PRODPORT_INTR_MAP_9, 1, PRODPORT_INTR_MAP_9_S);
	intr_matrix_set(ETS_SLC_SOURCE, ETS_SLC_INUM);
	xt_set_interrupt_handler(ETS_SLC_INUM, slc_isr, NULL);
	WRITE_PERI_REG(I2SINT_ENA, I2S_I2S_TO_EOF_INT_ENA | I2S_I2S_FROM_SUC_EOF_INT_ENA);
	WRITE_PERI_REG(I2SINT_CLR, 0xffffffff);
	xt_ints_on(1 << ETS_SLC_INUM);

	//Initialize DMA buffer descriptors in such a way that they will form a circular
	//buffer.
	for (x=0; x<I2SDMABUFCNT; x++) {
		i2sBufDesc[x].owner=1;
		i2sBufDesc[x].eof=1;
		i2sBufDesc[x].sub_sof=0;
		i2sBufDesc[x].datalen=I2SDMABUFLEN*4;
		i2sBufDesc[x].blocksize=I2SDMABUFLEN*4;
		i2sBufDesc[x].buf_ptr=(uint32_t)&i2sBuf[x][0];
		i2sBufDesc[x].unused=0;
		i2sBufDesc[x].next_link_ptr=(int)((x<(I2SDMABUFCNT-1))?(&i2sBufDesc[x+1]):(&i2sBufDesc[0]));
	}
	
	//Feed dma the 1st buffer desc addr
	//To send data to the I2S subsystem, counter-intuitively we use the RXLINK part, not the TXLINK as you might
	//expect. The TXLINK part still needs a valid DMA descriptor, even if it's unused: the DMA engine will throw
	//an error at us otherwise. Just feed it any random descriptor.
	CLEAR_PERI_REG_MASK(I2STX_LINK, I2S_I2S_TXLINK_ADDR);
	SET_PERI_REG_MASK(I2STX_LINK, ((uint32)&i2sBufDesc[1]) & I2S_I2S_TXLINK_ADDR); //any random desc is OK, we don't use TX but it needs something valid
	CLEAR_PERI_REG_MASK(I2SRX_LINK, I2S_I2S_RXLINK_ADDR);
	SET_PERI_REG_MASK(I2SRX_LINK, ((uint32)&i2sBufDesc[0]) & I2S_I2S_RXLINK_ADDR);


	//We use a queue to keep track of the DMA buffers that are empty. The ISR will push buffers to the back of the queue,
	//the mp3 decode will pull them from the front and fill them. For ease, the queue will contain *pointers* to the DMA
	//buffers, not the data itself. The queue depth is one smaller than the amount of buffers we have, because there's
	//always a buffer that is being used by the DMA subsystem *right now* and we don't want to be able to write to that
	//simultaneously.
	dmaQueue=xQueueCreate(I2SDMABUFCNT-1, sizeof(int*));

	//Start transmission
	SET_PERI_REG_MASK(I2STX_LINK, I2S_I2S_TXLINK_START);
	SET_PERI_REG_MASK(I2SRX_LINK, I2S_I2S_RXLINK_START);

//----

	//Init pins to i2s functions
	//Use GPIO 16/17/18 as I2S port
	SET_PERI_REG_MASK(GPIO_ENABLE,(1<<27));//ENABLE GPIO oe_enable
	SET_PERI_REG_MASK(GPIO_ENABLE1,(1<<0)|(1<<1));//ENABLE GPIO oe_enable
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL8,GPIO_GPIO_FUNC32_OUT_SEL, I2SO_WS_OUT_IDX, GPIO_GPIO_FUNC32_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO32_U, MCU_SEL, 0, MCU_SEL_S);
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL8,GPIO_GPIO_FUNC33_OUT_SEL, I2SO_DATA_OUT_IDX, GPIO_GPIO_FUNC33_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO33_U, MCU_SEL, 0, MCU_SEL_S);
	SET_PERI_REG_BITS(GPIO_FUNC_OUT_SEL6,GPIO_GPIO_FUNC27_OUT_SEL, I2SO_BCK_OUT_IDX, GPIO_GPIO_FUNC27_OUT_SEL_S);
	SET_PERI_REG_BITS(PERIPHS_IO_MUX_GPIO27_U, MCU_SEL, 0, MCU_SEL_S);

	//Reset I2S subsystem
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);
	CLEAR_PERI_REG_MASK(I2SCONF,I2S_I2S_RESET_MASK);

	//Select 16bits per channel (FIFO_MOD=0), no DMA access (FIFO only)
	CLEAR_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN|(I2S_I2S_RX_FIFO_MOD<<I2S_I2S_RX_FIFO_MOD_S)|(I2S_I2S_TX_FIFO_MOD<<I2S_I2S_TX_FIFO_MOD_S));
	//Enable DMA in i2s subsystem
	SET_PERI_REG_MASK(I2S_FIFO_CONF, I2S_I2S_DSCR_EN);

	//tx/rx binaureal
	CLEAR_PERI_REG_MASK(I2SCONF_CHAN, (I2S_TX_CHAN_MOD<<I2S_TX_CHAN_MOD_S)|(I2S_RX_CHAN_MOD<<I2S_RX_CHAN_MOD_S));

	//Clear int
	SET_PERI_REG_MASK(I2SINT_CLR,   I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);
	CLEAR_PERI_REG_MASK(I2SINT_CLR, I2S_I2S_TX_REMPTY_INT_CLR|I2S_I2S_TX_WFULL_INT_CLR|
			I2S_I2S_RX_WFULL_INT_CLR|I2S_I2S_PUT_DATA_INT_CLR|I2S_I2S_TAKE_DATA_INT_CLR);

	//trans master&rece slave,MSB shift,right_first,msb right
	CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
						(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_SLAVE_MOD|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((16&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						((7&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));


	//No idea if ints are needed...
	WRITE_PERI_REG(I2SINT_CLR, 0xffffffff);
	//Start transmission
	SET_PERI_REG_MASK(I2SCONF,I2S_I2S_TX_START);
}


#define BASEFREQ (160000000L)
#define ABS(x) (((x)>0)?(x):(-(x)))

//Set the I2S sample rate, in HZ
void i2sSetRate(int rate, int enaWordlenFuzzing) {
	//Find closest divider 
	int bestclkmdiv, bestbckdiv, bestbits, bestfreq=-10000;
	int tstfreq;
	int bckdiv, clkmdiv, bits=16;
	/*
		CLK_I2S = 160MHz / I2S_CLKM_DIV_NUM
		BCLK = CLK_I2S / I2S_BCK_DIV_NUM
		WS = BCLK/ 2 / (16 + I2S_BITS_MOD)
		Note that I2S_CLKM_DIV_NUM must be >5 for I2S data
		I2S_CLKM_DIV_NUM - 5-127
		I2S_BCK_DIV_NUM - 2-127
		
		We also have the option to send out more than 2x16 bit per sample. Most I2S codecs will
		ignore the extra bits and in the case of the 'fake' PWM/delta-sigma outputs, they will just lower the output
		voltage a bit, so we add them when it makes sense.
	*/
	for (bckdiv=2; bckdiv<64; bckdiv++) {
		for (clkmdiv=5; clkmdiv<64; clkmdiv++) {
			for (bits=16; bits<(enaWordlenFuzzing?20:17); bits++) {
				tstfreq=BASEFREQ/(bckdiv*clkmdiv*bits*2);
				if (ABS(rate-tstfreq)<ABS(rate-bestfreq)) {
					bestfreq=tstfreq;
					bestclkmdiv=clkmdiv;
					bestbckdiv=bckdiv;
					bestbits=bits;
				}
			}
		}
	}

	printf("ReqRate %d MDiv %d BckDiv %d Bits %d  Frq %d\n", 
		rate, bestclkmdiv, bestbckdiv, bestbits, (int)(BASEFREQ/(bestbckdiv*bestclkmdiv*bestbits*2)));


	CLEAR_PERI_REG_MASK(I2SCONF, I2S_TRANS_SLAVE_MOD|
						(I2S_BITS_MOD<<I2S_BITS_MOD_S)|
						(I2S_BCK_DIV_NUM <<I2S_BCK_DIV_NUM_S)|
						(I2S_CLKM_DIV_NUM<<I2S_CLKM_DIV_NUM_S));
	SET_PERI_REG_MASK(I2SCONF, I2S_RIGHT_FIRST|I2S_MSB_RIGHT|I2S_RECE_SLAVE_MOD|
						I2S_RECE_MSB_SHIFT|I2S_TRANS_MSB_SHIFT|
						((bestbits-16)<<I2S_BITS_MOD_S)|
						(((bestbckdiv)&I2S_BCK_DIV_NUM )<<I2S_BCK_DIV_NUM_S)|
						(((bestclkmdiv)&I2S_CLKM_DIV_NUM)<<I2S_CLKM_DIV_NUM_S));
}

//Current DMA buffer we're writing to
static unsigned int *currDMABuff=NULL;
//Current position in that DMA buffer
static int currDMABuffPos=0;


//This routine pushes a single, 32-bit sample to the I2S buffers. Call this at (on average) 
//at least the current sample rate. You can also call it quicker: it will suspend the calling
//thread if the buffer is full and resume when there's room again.
void i2sPushSample(unsigned int sample) {
	//Check if current DMA buffer is full.
	if (currDMABuffPos==I2SDMABUFLEN || currDMABuff==NULL) {
		//We need a new buffer. Pop one from the queue.
		xQueueReceive(dmaQueue, &currDMABuff, portMAX_DELAY);
		currDMABuffPos=0;
	}
	currDMABuff[currDMABuffPos++]=sample;
}


long ICACHE_FLASH_ATTR i2sGetUnderrunCnt() {
	return underrunCnt;
}
