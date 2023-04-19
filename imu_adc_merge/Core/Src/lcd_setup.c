/* Includes ------------------------------------------------------------------*/
#include "lcd_setup.h"

void small_delay()
{
    nano_wait(10000);
    return;
}

void print_labels()
{
	LCD_DrawString(5,10,  WHITE, BLACK, "Aerial Ace Status Window", 16, 0);
	LCD_DrawFillRectangle(2,35,300,110, LGRAYBLUE);
	LCD_DrawString(5,45,  WHITE, LGRAYBLUE, "Current Mode", 16, 0);
	LCD_DrawLine(2,30,300, 30, WHITE);
	LCD_DrawLine(2,185,300,185, WHITE);
}

void print_title(DispState currDisp)
{
	LCD_DrawString(50,70,  WHITE, LGRAYBLUE, currDisp.title, 16, 0);
	return;
}

void print_command(DispState currDisp)
{
	LCD_DrawString(0,120,  WHITE, BLACK, currDisp.command_ln1, 12, 0);
	if(currDisp.state == 2 || currDisp.state == 3)
	{
		LCD_DrawString(0,130,  WHITE, BLACK, currDisp.command_ln2, 12, 0);
	}
	for(int i = 0; i < 1000; i++);
	return;
}

void print_stats(DispState currDisp)
{
	char yaw[30];
	char roll[30];
	char pitch[30];
	char throttle[30];

	//LCD_Clear(BLACK);

//	sprintf(yaw, "%.3f", currDisp.yaw_num);
//	sprintf(roll, "%.3f", currDisp.roll_num);
//	sprintf(pitch, "%.3f", currDisp.pitch_num);
//	sprintf(throttle, "%.3f", currDisp.throttle_num);

	LCD_DrawLine(90,185,90,330, WHITE);
	LCD_DrawLine(160,185,160,330, WHITE);

	LCD_DrawString(10, 195, WHITE, BLACK, "PITCH", 16, 0); //90
	LCD_DrawString(10, 230, WHITE, BLACK, "YAW", 16, 0); //150
	LCD_DrawString(10, 265, WHITE, BLACK, "ROLL", 16, 0); //200
	LCD_DrawString(10, 300, WHITE, BLACK, "THROTTLE", 16, 0); //250

	LCD_DrawString(100, 195, WHITE, BLACK, pitch, 16, 0);
	LCD_DrawString(100, 230, WHITE, BLACK, yaw, 16, 0);
	LCD_DrawString(100, 265, WHITE, BLACK, roll, 16, 0);
	LCD_DrawString(100, 300, WHITE, BLACK, throttle, 16, 0);

	LCD_DrawString(170, 195, WHITE, BLACK, currDisp.pitch_mode, 16, 0);
	LCD_DrawString(170, 230, WHITE, BLACK, currDisp.yaw_mode, 16, 0);
	LCD_DrawString(170, 265, WHITE, BLACK, currDisp.roll_mode, 16, 0);
	LCD_DrawString(170, 300, WHITE, BLACK, currDisp.throttle_mode, 16, 0);
	return;
}

/**
 * Setup SPI1
 * PA5: SPI1 sck
 * PA7: SPI1 mosi
 * PA4: NSS, CS
 */
void SPI1_Setup()
{
  GPIOA->MODER &= ~0xcc00;
  GPIOA->MODER |= 0x8800; // AFR for pa5 and pa7

  GPIOB->MODER &= ~0xC00000;
  GPIOA->MODER &= ~0x300;
  GPIOB->MODER |= 0x400000;
  GPIOA->MODER |= 0x100;

  GPIOA->MODER &= ~0xc0;
  GPIOA->MODER |= 0x40;
  GPIOB->ODR |= 0x800;
  GPIOA->ODR |= 0x18;

  RCC->APB2ENR |= RCC_APB2ENR_SPI1EN;
  SPI1->CR2 &= ~SPI_CR2_DS;
  SPI1->CR1 &= ~(SPI_CR1_BR);
  SPI1->CR1 |= SPI_CR1_MSTR;
  SPI1->CR1 |= SPI_CR1_SSM | SPI_CR1_SSI;
  SPI1->CR1 |= SPI_CR1_SPE;
  return;
}


void LCD_Update(DispState currDisp, float roll, float pitch, float throttle, float yaw, int state)
{
	int zu, gu;

	if(roll >= 0)
	{
		strncpy(currDisp.roll_mode, "RIGHT", 29);
	}
	else
	{
		strncpy(currDisp.roll_mode, "LEFT", 29);
	}

	if(yaw >= 0)
	{
		strncpy(currDisp.yaw_mode, "RIGHT", 29);
	}
	else
	{
		strncpy(currDisp.yaw_mode, "LEFT", 29);
	}

	if(pitch >= 0)
	{
		strncpy(currDisp.pitch_mode, "UP", 29);
	}
	else
	{
		strncpy(currDisp.pitch_mode, "DOWN", 29);
	}

	if(throttle >= 0)
	{
		strncpy(currDisp.throttle_mode, "UP", 29);
	}
	else
	{
		strncpy(currDisp.throttle_mode, "DOWN", 29);
	}


	currDisp.state = state;
	if(currDisp.state == 1)
	{
		strncpy(currDisp.title, "INITIALISING", 29);
		strncpy(currDisp.command_ln1, "...Loading...", 199);
	}
	else if(currDisp.state == 2)
	{
		strncpy(currDisp.title, "CALIBRATION", 29);
		strncpy(currDisp.command_ln1, "Please Unflex your fingers until finger", 199);
		strncpy(currDisp.command_ln2, "angles are 0 degrees.", 199);
	}
	else if(currDisp.state == 3)
	{
		strncpy(currDisp.title, "CALIBRATION", 29);
		strncpy(currDisp.command_ln1, "Please Flex your fingers until finger", 199);
		strncpy(currDisp.command_ln2, "angles are 90 degrees.", 199);
	}
	else if(currDisp.state == 4)
	{
		strncpy(currDisp.title, "ADVANCED", 29);
		strncpy(currDisp.command_ln1, "Toggle switch to change mode to standard!", 199);
	}
	else if(currDisp.state == 5)
	{
		strncpy(currDisp.title, "STANDARD", 29);
		strncpy(currDisp.command_ln1, "Toggle switch to change mode to advanced!", 199);
	}

	currDisp.pitch_num = pitch;
	currDisp.yaw_num = yaw;
	currDisp.roll_num = roll;
	currDisp.throttle_num = throttle;

	print_labels();

	print_title(currDisp);
	if(currDisp.state == 4)
	{
		print_stats(currDisp);
	}
	print_command(currDisp);
}
