#ifndef LCD_SETUP_H
#define LCD_SETUP_H

#include "stm32f3xx_hal.h"
#include "lcd.h"
#include "utils.h"

#define CAL_CIRCLE_X 120
#define CAL_CIRCLE_Y 240
#define CAL_CIRCLE_RADIUS 60

#define LCD_UPDATE_PERIOD 5

typedef struct
{
	// Mirrors mode
	//   1: init
	//   2: cal_unflexed
	//   3: cal_flexed
	//   4: Advanced
	//   5: Standard
	states state;

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

extern DispState currDisp;
extern SPI_HandleTypeDef hspi1;
extern DMA_HandleTypeDef hdma_spi1_tx;

void LCD_print_labels();
void LCD_print_title(DispState currDisp);
void LCD_print_command(DispState currDisp);
void LCD_print_stats(DispState currDisp);
void LCD_print_progress(int time_secs, int curTim);
void LCD_print_circle(int time_secs, int CX, int CY, int radius, u16 color);
void LCD_generate_sectors(int time_secs, int CX, int CY, int radius);
void LCD_update(float roll, float pitch, float throttle, float yaw, int state, int total_time_sec, int cnt_sec);
void LCD_small_delay();

#endif
