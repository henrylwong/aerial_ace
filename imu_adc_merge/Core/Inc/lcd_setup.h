#ifndef LCD_SETUP_H
#define LCD_SETUP_H

#include "stm32f3xx_hal.h"
#include "lcd.h"

typedef struct
{
	// Mirrors mode
	//   1: init
	//   2: cal_unflexed
	//   3: cal_flexed
	//   4: Advanced
	//   5: Standard
	int state;

	// For use in Advanced mode
	char yaw_mode[30]; //0 = rest, 1 = right, 2 = left
	char roll_mode[30]; //0 = rest, 1 = right, 2 = left
	char throttle_mode[30]; //0 = rest, 1 = up, 2 = down
	char pitch_mode[30]; // 0 = rest, 1 = up, 2 = down

	float yaw_num;
	float roll_num;
	float throttle_num;
	float pitch_num;

	char title[30];

	char command_ln1[200];
	char command_ln2[200];

	int update_stat_title;
	int update_stat_command;
} DispState;

extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

void SPI1_Setup();
void print_labels();
void print_title(DispState currDisp);
void print_command(DispState currDisp);
void print_stats(DispState currDisp);
void LCD_Update(DispState currDisp, float roll, float pitch, float throttle, float yaw, int state);
void small_delay();

#endif
