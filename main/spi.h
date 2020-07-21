#pragma once
#include <stdint.h>

void spi_init();
void spi_flushdata();
void spi_writeData16(uint16_t data);
void spi_writeData8(uint8_t data);

void spi_writeCommand8(uint8_t command);
void spi_writeCommand16(uint16_t command);
void spi_writeCommand(uint8_t* buf, uint32_t size);

void spi_sendbuffer();
void spi_addbuffer16(uint16_t d);
void spi_addbuffer8(uint8_t d);
