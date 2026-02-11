#ifndef INC_MB85RC1M_H_
#define INC_MB85RC1M_H_

#include "main.h"



void MB85RC1M_Write(uint32_t addr, uint8_t * data, uint32_t len);
void MB85RC1M_Read(uint32_t addr, uint8_t * data, uint32_t len);

#endif
