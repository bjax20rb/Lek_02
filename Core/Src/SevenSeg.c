/*
 * SevenSeg.c
 *
 */
#include "main.h"
#include "SevenSeg.h"

uint8_t seg_array[] = {0x3f,0x06,0x5b,0x4f,0x66,0x6d,0x7d,0x07,0x7f,0x6f};

void TM1637_Delay()
{
	for (int i = 0; i < 100; i++)
	{
		__NOP();
	}
}

void TM1637_Start()
{
	HAL_GPIO_WritePin(SEG_DAT_GPIO_Port, SEG_DAT_Pin, GPIO_PIN_SET);
	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);
	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_DAT_GPIO_Port, SEG_DAT_Pin, GPIO_PIN_RESET);
	TM1637_Delay();
}

void TM1637_Stop()
{
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_RESET);
	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);
	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_RESET);
	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_DAT_GPIO_Port, SEG_DAT_Pin, GPIO_PIN_RESET);
	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);
	TM1637_Delay();
	HAL_GPIO_WritePin(SEG_DAT_GPIO_Port, SEG_DAT_Pin, GPIO_PIN_SET);
	TM1637_Delay();
}

void TM1637_WriteByte(uint8_t byte)
{
	for (uint8_t c = 0; c < 8; c++) {
		HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_RESET);
		if ((byte & 0x01) == 0) {
			HAL_GPIO_WritePin(SEG_DAT_GPIO_Port, SEG_DAT_Pin, GPIO_PIN_RESET);
		} else {
			HAL_GPIO_WritePin(SEG_DAT_GPIO_Port, SEG_DAT_Pin, GPIO_PIN_SET);
		}
		byte = byte >> 1;
		TM1637_Delay();
		HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);
		TM1637_Delay();
	}

	HAL_GPIO_WritePin(SEG_DAT_GPIO_Port, SEG_DAT_Pin, GPIO_PIN_SET);
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_RESET);
	TM1637_Delay();
	while(HAL_GPIO_ReadPin(SEG_DAT_GPIO_Port, SEG_DAT_Pin) == GPIO_PIN_SET)
		;
	HAL_GPIO_WritePin(SEG_DAT_GPIO_Port, SEG_DAT_Pin, GPIO_PIN_RESET);
	HAL_GPIO_WritePin(SEG_CLK_GPIO_Port, SEG_CLK_Pin, GPIO_PIN_SET);
	TM1637_Delay();
}


void TM1637_WriteCommand(uint8_t cmd)
{
	TM1637_Start();
	TM1637_WriteByte(cmd);
	TM1637_Stop();
}

void TM1637_WriteData(uint8_t addr,uint8_t data)
{
	TM1637_Start();
	TM1637_WriteByte(addr);
	TM1637_WriteByte(data);
	TM1637_Stop();
}

void DisplayDigits(uint8_t d1, uint8_t d2, uint8_t d3, uint8_t d4, uint8_t colon)
{
	TM1637_WriteCommand(0x44);
    TM1637_WriteData(0xc0,seg_array[d1]);
   	if(colon)
   		  TM1637_WriteData(0xc1,seg_array[d2] | 0x80);
   	else
   		  TM1637_WriteData(0xc1,seg_array[d2]);
    TM1637_WriteData(0xc2, seg_array[d3]);
  	TM1637_WriteData(0xc3, seg_array[d4]);
    TM1637_WriteCommand(0x8a);

}

