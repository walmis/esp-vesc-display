#pragma GCC optimize("O3")
#include "FreeRTOS.h"
#include "task.h"
#include "semphr.h"
#include <string.h>


#include "spi.h"
#include "driver/spi.h"
#include "sdkconfig.h"
#include <driver/gpio.h>
#include "esp_log.h"


static TaskHandle_t spi_task_notify_handle;
static volatile bool spi_command_mode = 0;
static IRAM_ATTR uint32_t spi_buffer[16];
static IRAM_ATTR int spi_bufpos;
static IRAM_ATTR void spi_event_callback(int event, void *arg);

static int8_t _cs_pin = -1;
static xSemaphoreHandle _spi_lock;

IRAM_ATTR
void spi_addbuffer16(uint16_t d) {
	uint32_t pos32 = spi_bufpos/4;
	d = __builtin_bswap16(d);
	//ESP_LOGI(__func__, "%d", pos32);
	if((spi_bufpos & 3) == 0) {
		spi_buffer[pos32] = 0;
		spi_buffer[pos32] |= d;
	} else {
		spi_buffer[pos32] |= ((uint32_t)d)<<16;
	}
	spi_bufpos+=2;
	//((uint8_t*)spi_buffer)[spi_bufpos++] = d >> 8;
	//((uint8_t*)spi_buffer)[spi_bufpos++] = d & 0xFF;
}

IRAM_ATTR
void spi_addbuffer8(uint8_t d) {
	uint8_t* buf = (uint8_t*)spi_buffer;
	buf[spi_bufpos++] = d;
	//((uint8_t*)spi_buffer)[spi_bufpos++] = d >> 8;
	//((uint8_t*)spi_buffer)[spi_bufpos++] = d & 0xFF;
}


IRAM_ATTR
void spi_sendbuffer() {
	if(spi_task_notify_handle){
		ESP_LOGE(__func__, "SPI handle taken");
		return;
	}

	spi_task_notify_handle = xTaskGetCurrentTaskHandle();

	spi_trans_t trans = {};
	trans.mosi = spi_buffer;
	trans.bits.mosi = 8 * spi_bufpos;
	spi_trans(HSPI_HOST, &trans);
	spi_bufpos = 0;

	if(!ulTaskNotifyTake( pdTRUE, 100 )) {
		ESP_LOGE(__func__, "SPI xfer timeout");
	}
    spi_task_notify_handle = 0;
}

IRAM_ATTR
void spi_writeCommand16(uint16_t command) {
    spi_flushdata();

	gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 0);
    vTaskDelay(1);
	spi_command_mode = 1;
	spi_addbuffer16(command);
	spi_sendbuffer();
    vTaskDelay(1);
    gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 1);

}

IRAM_ATTR
void spi_writeCommand8(uint8_t command) {
    spi_flushdata();

	gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 0);
        vTaskDelay(1);

	spi_addbuffer8(command);
	spi_sendbuffer();
        vTaskDelay(1);

    gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 1);
}

IRAM_ATTR
void spi_writeData16(uint16_t data) {
	spi_addbuffer16(data);

	if(spi_bufpos >= 64) {
		spi_command_mode = 0;
		spi_sendbuffer();
	}
}


IRAM_ATTR
void spi_flushdata() {
	if(spi_bufpos) {
		spi_command_mode = 0;
		spi_sendbuffer();
	}
}

void spi_init() {
    if(!_spi_lock) {
        _spi_lock = xSemaphoreCreateMutex();
    }

	spi_config_t spi_config;
	// Load default interface parameters
	// CS_EN:1, MISO_EN:1, MOSI_EN:1, BYTE_TX_ORDER:1, BYTE_TX_ORDER:1, BIT_RX_ORDER:0, BIT_TX_ORDER:0, CPHA:0, CPOL:0
	spi_config.interface.val = SPI_DEFAULT_INTERFACE;
	// Load default interrupt enable
	// TRANS_DONE: true, WRITE_STATUS: false, READ_STATUS: false, WRITE_BUFFER: false, READ_BUFFER: false
	spi_config.intr_enable.val = SPI_MASTER_DEFAULT_INTR_ENABLE;
	// Cancel hardware cs
	spi_config.interface.cs_en = 0;
	spi_config.interface.miso_en = 1;
    spi_config.interface.mosi_en = 1;
	spi_config.interface.cpol = 1;
	spi_config.interface.cpha = 1;
	spi_config.interface.byte_tx_order = SPI_BYTE_ORDER_LSB_FIRST;
	spi_config.interface.bit_tx_order = SPI_BIT_ORDER_LSB_FIRST;
	// Set SPI to master mode
	// 8266 Only support half-duplex
	spi_config.mode = SPI_MASTER_MODE;
	// Set the SPI clock frequency division factor
	spi_config.clk_div = SPI_20MHz_DIV;
	// Register SPI event callback function
	spi_config.event_cb = spi_event_callback;
	spi_init(HSPI_HOST, &spi_config);
}

void spi_beginTransaction(uint8_t cspin) {
    xSemaphoreTake(_spi_lock, portMAX_DELAY );
    _cs_pin = cspin;            

    if(_cs_pin == CONFIG_GPIO_TOUCH_CS) {
        spi_clk_div_t div = SPI_2MHz_DIV;
        spi_set_clk_div(HSPI_HOST, &div);
    } else {
        spi_clk_div_t div = SPI_20MHz_DIV;
        spi_set_clk_div(HSPI_HOST, &div);
    }

    gpio_set_level((gpio_num_t)_cs_pin, 0);

}

void spi_endTransaction() {
    //write remaining data if any
    spi_flushdata();
    
    gpio_set_level((gpio_num_t)_cs_pin, 1);

    _cs_pin = -1;
    xSemaphoreGive(_spi_lock);
}

void spi_xfer(uint8_t* in, uint8_t* out, uint8_t len) {
    uint32_t inbuf[(len+4)/4] = {};
    uint32_t outbuf[(len+4)/4];
    for(int i = 0; i < len; i++) {
        ((uint8_t*)(outbuf))[i] = out[i];
    }
    
    spi_flushdata();

    spi_task_notify_handle = xTaskGetCurrentTaskHandle();

	spi_trans_t trans = {};
	trans.miso = inbuf;
	trans.bits.miso = 8 * len;
    trans.mosi = outbuf;
	trans.bits.mosi = 8 * len;
	spi_trans(HSPI_HOST, &trans);

	if(!ulTaskNotifyTake( pdTRUE, 100 )) {
		ESP_LOGE(__func__, "SPI xfer timeout");
	}
    for(int i = 0; i < len; i++) {
        in[i] = ((uint8_t*)(inbuf))[i];
    }
    //printf("snd %x rd %x\n", outbuf[0], ((uint8_t*)(inbuf))[1]);

}

static IRAM_ATTR void spi_event_callback(int event, void *arg)
{
	switch (event) {
		case SPI_INIT_EVENT: {
		}
		break;

		case SPI_TRANS_START_EVENT: {
            //assert(_cs_pin != -1);
			// if(spi_command_mode && _cs_pin == CONFIG_GPIO_TFT_CS) {
			// 	gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 0);
			// }
		}
		break;

		case SPI_TRANS_DONE_EVENT: {
            // assert(_cs_pin != -1);
            // if(_cs_pin == CONFIG_GPIO_TFT_CS) {
			//     gpio_set_level((gpio_num_t)CONFIG_GPIO_TFT_DC, 1);
            // }

			BaseType_t xHigherPriorityTaskWoken = 0;
			vTaskNotifyGiveFromISR( spi_task_notify_handle, &xHigherPriorityTaskWoken);
			spi_task_notify_handle = 0;
			if (xHigherPriorityTaskWoken) {
				taskYIELD();
			}
		}
		break;

		case SPI_DEINIT_EVENT: {
		}
		break;
	}
}