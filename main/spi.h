#pragma once
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

void spidrv_init();

void spi_beginTransaction(uint8_t cspin);
void spi_endTransaction();

void spi_flushdata();
void spi_writeData16(uint16_t data);
void spi_writeData8(uint8_t data);

void spi_writeCommand8(uint8_t command);
void spi_writeCommand16(uint16_t command);

void spi_sendbuffer();
void spi_addbuffer16(uint16_t d);
void spi_addbuffer8(uint8_t d);

void spi_send_aligned(void* buffer, int size);

void spi_xfer(uint8_t* in, uint8_t* out, uint8_t len);

#ifdef __cplusplus
}
#endif