/* Includes ------------------------------------------------------------------*/
#include "lcd_setup.h"

double a[100];
double b[100];
double X_arr[50];
double Y_arr[50];
int sectors[CAL_TIME_SEC][6];

int CAL_CIRCLE_RADIUS_INNER = round(CAL_CIRCLE_RADIUS - CAL_CIRCLE_RADIUS / 7);

void small_delay() {
    nano_wait(10000);
    return;
}

void print_labels() {
	LCD_DrawString(5, 10, WHITE, BLACK, "Aerial Ace Status Window", 16, 0);
	LCD_DrawFillRectangle(2, 35, 300, 110, LGRAYBLUE);
	LCD_DrawString(5, 45, WHITE, LGRAYBLUE, "Current Mode", 16, 0);
	LCD_DrawLine(2, 30, 300, 30, WHITE);
	LCD_DrawLine(2, 160, 300, 160, WHITE);
}

void print_title(DispState currDisp) {
	LCD_DrawString(50, 70, WHITE, LGRAYBLUE, currDisp.title, 16, 0);
	return;
}

void print_command(DispState currDisp) {
	LCD_DrawString(0, 120,  WHITE, BLACK, currDisp.command_ln1, 12, 0);
	if(currDisp.state == CAL_UNFLEXED || currDisp.state == CAL_FLEXED)
	{
		LCD_DrawString(0, 130,  WHITE, BLACK, currDisp.command_ln2, 12, 0);
	}
	for(int i = 0; i < 1000; i++);
	return;
}

void print_stats(DispState currDisp) {
  char yaw[30];
	char roll[30];
	char pitch[30];
	char throttle[30];

	sprintf(yaw, "%.3f", currDisp.yaw_num);
	sprintf(roll, "%.3f", currDisp.roll_num);
	sprintf(pitch, "%.3f", currDisp.pitch_num);
	sprintf(throttle, "%.3f", currDisp.throttle_num);

	LCD_DrawLine(90, 165, 90, 330, WHITE);
	LCD_DrawLine(160, 165, 160, 330, WHITE);

	LCD_DrawString(10, 175, WHITE, BLACK, "PITCH", 16, 0); //90
	LCD_DrawString(10, 210, WHITE, BLACK, "YAW", 16, 0); //150
	LCD_DrawString(10, 245, WHITE, BLACK, "ROLL", 16, 0); //200
	LCD_DrawString(10, 280, WHITE, BLACK, "THROTTLE", 16, 0); //250

	LCD_DrawString(100, 175, WHITE, BLACK, pitch , 16, 0);
	LCD_DrawString(100, 210, WHITE, BLACK, yaw, 16, 0);
	LCD_DrawString(100, 245, WHITE, BLACK, roll, 16, 0);
	LCD_DrawString(100, 280, WHITE, BLACK, throttle , 16, 0);

	LCD_DrawString(170, 175, WHITE, BLACK, currDisp.pitch_mode, 16, 0);
	LCD_DrawString(170, 210, WHITE, BLACK, currDisp.yaw_mode, 16, 0);
	LCD_DrawString(170, 245, WHITE, BLACK, currDisp.roll_mode, 16, 0);
	LCD_DrawString(170, 280, WHITE, BLACK, currDisp.throttle_mode, 16, 0);
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

// void print_progress(int time_secs, int CX, int CY, int radius, int curTim) {
// 	int i, j;
// 	int X0;
// 	int X1;
// 	int X2;
// 	int Y0;
// 	int Y1;
// 	int Y2;
// 	int radius2 = round(radius - radius / 7);

// 	int sidesAB = radius;
// 	double sideC;
// 	double Cy;
// 	double Cx;
// 	int CyR;
// 	int CxR;

// 	int startX = CX;
// 	int startY = CY - radius;

// 	double central_angleD = 360 / time_secs;
// 	double other_angleD = (180 - central_angleD) / 2;

// 	double central_angleR = central_angleD * M_PI / 180;
// 	double other_angleR = other_angleD * M_PI / 180;

// 	char strs[3];

// 	if (curTim == time_secs - 1) {
// 		X0 = CX;
// 		Y0 = CY - radius2;
// 	} else {
// 		X0 = X_arr[time_secs - 2 - curTim];
// 		Y0 = Y_arr[time_secs - 2 - curTim];
// 	}

// 	X1 = CX;
// 	Y1 = CY;
// 	sidesAB = radius2;

// 	sideC = sqrt(2 * (sidesAB * sidesAB) - 2 * (sidesAB * sidesAB) * cos(central_angleR));

// 	Cy = sin(b[time_secs - 1 - curTim]) * sideC;
// 	Cx = cos(b[time_secs - 1 - curTim]) * sideC;

// 	CyR = round(Cy);
// 	CxR = round(Cx);

// 	X2 = X0 + CxR;
// 	Y2 = Y0 + CyR;

// 	X_arr[time_secs - 1 - curTim] = X2;
// 	Y_arr[time_secs - 1 - curTim] = Y2;

//   sprintf(strs, "%d", time_secs - curTim);
// 	LCD_DrawFillRectangle(CX - 8, CY - 8, CX + 8, CY + 8, BLACK);
// 	LCD_DrawFillTriangle(X0, Y0, X1, Y1, X2, Y2, BLACK);
// 	LCD_DrawString(CX - 8, CY - 8, WHITE, BLACK, strs, 16, 1);
// }

void print_progress(int time_secs, int curTim) {
	if (curTim >= time_secs) {return;}

	char strs[3];
	sprintf(strs, "%d", curTim);

	LCD_DrawFillTriangle(sectors[curTim][0], sectors[curTim][1], sectors[curTim][2], sectors[curTim][3], sectors[curTim][4], sectors[curTim][5], BLACK);
	LCD_DrawFillRectangle(CAL_CIRCLE_X - 8, CAL_CIRCLE_Y - 8, CAL_CIRCLE_X + 8, CAL_CIRCLE_Y + 8, BLACK);
	LCD_DrawString(CAL_CIRCLE_X - 8, CAL_CIRCLE_Y - 8, WHITE, BLACK, strs, 16, 1);
}

// void makeCircle(int time_secs, int CX, int CY, int radius, u16 color) {
// 	int i, j;
// 	int X0;
// 	int X1;
// 	int X2;
// 	int Y0;
// 	int Y1;
// 	int Y2;

// 	int sidesAB = radius;
// 	double sideC;
// 	double Cy;
// 	double Cx;
// 	int CyR;
// 	int CxR;

// 	int startX = CX;
// 	int startY = CY - radius;

// 	double central_angleD = 360 / time_secs;
// 	double other_angleD = (180 - central_angleD) / 2;

// 	double central_angleR = central_angleD * M_PI / 180;
// 	double other_angleR = other_angleD * M_PI / 180;

// 	char strs[3];

// 	//For progress indicator
// 	for(i = 0; i < time_secs; i++) {
// 		if (i == 0) {
// 			a[i] = (90 - other_angleD);
// 			b[i] = a[i] * M_PI / 180;
// 		} else {
// 			a[i] = (360 - ((180 - a[i - 1]) + 2 * other_angleD));
// 			b[i] = a[i] * M_PI / 180;
// 		}
// 	}

// 	X0 = startX;
// 	Y0 = startY;
// 	X1 = CX;
// 	Y1 = CY;

// 	for(i = 0; i < time_secs; i++) {
// 		sideC = sqrt(2 * (sidesAB * sidesAB) - 2 * (sidesAB * sidesAB) * cos(central_angleR));

// 		Cy = sin(b[i]) * sideC;
// 		Cx = cos(b[i]) * sideC;

// 		CyR = round(Cy);
// 		CxR = round(Cx);

// 		X2 = X0 + CxR;
// 		Y2 = Y0 + CyR;
// 		LCD_DrawFillTriangle(X0, Y0, X1, Y1, X2, Y2, color);
// 		X0 = X2;
// 		Y0 = Y2;
// 	}

// 	LCD_DrawFillRectangle(startX - 2, startY, CX , CY, color);
// 	return;
// }

void makeCircle(int time_secs, int CX, int CY, int radius, u16 color) {
	calculateCircle(time_secs, CX, CY, radius);
	for(int i = 0; i < time_secs; i++) {
		LCD_DrawFillTriangle(sectors[i][0], sectors[i][1], sectors[i][2], sectors[i][3], sectors[i][4], sectors[i][5], color);
	}
}

void calculateCircle(int time_secs, int CX, int CY, int radius) {
	int X0;
	int X1;
	int X2;
	int Y0;
	int Y1;
	int Y2;

	int sidesAB = radius;
	double sideC;
	double Cy;
	double Cx;
	int CyR;
	int CxR;

	int startX = CX;
	int startY = CY - radius;

	double central_angleD = 360 / time_secs;
	double other_angleD = (180 - central_angleD) / 2;

	double central_angleR = central_angleD * M_PI / 180;
	double other_angleR = other_angleD * M_PI / 180;

	//For progress indicator
	for(int i = 0; i < time_secs; i++) {
		if (i == 0) {
			a[i] = (90 - other_angleD);
			b[i] = a[i] * M_PI / 180;
		} else {
			a[i] = (360 - ((180 - a[i - 1]) + 2 * other_angleD));
			b[i] = a[i] * M_PI / 180;
		}
	}

	X0 = startX;
	Y0 = startY;
	X1 = CX;
	Y1 = CY;

	for(int i = 0; i < time_secs; i++) {
		sideC = sqrt(2 * (sidesAB * sidesAB) - 2 * (sidesAB * sidesAB) * cos(central_angleR));

		Cy = sin(b[i]) * sideC;
		Cx = cos(b[i]) * sideC;

		CyR = round(Cy);
		CxR = round(Cx);

		X2 = X0 + CxR;
		Y2 = Y0 + CyR;

		sectors[i][0] = X0;
		sectors[i][1] = Y0;
		sectors[i][2] = X1;
		sectors[i][3] = Y1;
		sectors[i][4] = X2;
		sectors[i][5] = Y2;
		X0 = X2;
		Y0 = Y2;
	}

	sectors[time_secs - 1][5] = startX;
	sectors[time_secs - 1][5] = startY;

	return;
}

void LCD_Update(float roll, float pitch, float throttle, float yaw, int state, int total_time_sec, int cnt_sec) {
	print_labels(); // @henry: can be done in init?

	currDisp.state = state;
	if (currDisp.state == INIT) {
		strncpy(currDisp.title, "INITIALISING", 29);
		strncpy(currDisp.command_ln1, "...Loading...", 199);
	} else if(currDisp.state == CAL_UNFLEXED) {
		strncpy(currDisp.title, "CALIBRATION", 29);
		strncpy(currDisp.command_ln1, "Please unflex your fingers until finger", 199);
		strncpy(currDisp.command_ln2, "angles are 0 degrees.", 199);
	} else if(currDisp.state == CAL_FLEXED) {
		strncpy(currDisp.title, "CALIBRATION", 29);
		strncpy(currDisp.command_ln1, "Please flex your fingers until finger", 199);
		strncpy(currDisp.command_ln2, "angles are 90 degrees.", 199);
	} else if(currDisp.state == MODE_STANDARD) {
		strncpy(currDisp.title, "STANDARD", 29);
		strncpy(currDisp.command_ln1, "Toggle switch to change mode to advanced!", 199);
	} else if(currDisp.state == MODE_ADVANCED) {
		strncpy(currDisp.title, "ADVANCED", 29);
		strncpy(currDisp.command_ln1, "Toggle switch to change mode to standard!", 199);
	}

	print_title(currDisp);

	if (currDisp.state == CAL_FLEXED || currDisp.state == CAL_UNFLEXED) {
		if (cnt_sec == CAL_TIME_SEC) {
			makeCircle(CAL_TIME_SEC, CAL_CIRCLE_X, CAL_CIRCLE_Y, CAL_CIRCLE_RADIUS, BRED);
			makeCircle(CAL_TIME_SEC, CAL_CIRCLE_X, CAL_CIRCLE_Y, CAL_CIRCLE_RADIUS_INNER, BLACK);
			calculateCircle(CAL_TIME_SEC, CAL_CIRCLE_X, CAL_CIRCLE_Y, CAL_CIRCLE_RADIUS);
		}
		// print_progress(CAL_TIME_SEC, CAL_CIRCLE_X, CAL_CIRCLE_Y, CAL_CIRCLE_RADIUS, cnt_sec);
		print_progress(CAL_TIME_SEC, cnt_sec);
		// LCD_DrawFillRectangle(CAL_CIRCLE_X - 10, CAL_CIRCLE_Y - 10, CAL_CIRCLE_X + 10 , CAL_CIRCLE_Y + 10, BLACK);
	  // LCD_DrawFillRectangle(CAL_CIRCLE_X - 5, CAL_CIRCLE_Y - CAL_CIRCLE_RADIUS_INNER, CAL_CIRCLE_X + 2 , CAL_CIRCLE_Y, BLACK);
	} else if (currDisp.state == MODE_ADVANCED) {
		currDisp.pitch_num = pitch;
		currDisp.yaw_num = yaw;
		currDisp.roll_num = roll;
		currDisp.throttle_num = throttle;
		if (roll >= 0.5 + GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.roll_mode, "RIGHT", 29);
		} else if (roll <= 0.5 - GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.roll_mode, "LEFT", 29);
		} else {
			strncpy(currDisp.roll_mode, "-----", 29);
		}

		if (pitch >= 0.5 + GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.pitch_mode, "UP", 29);
		} else if (pitch <= 0.5 - GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.pitch_mode, "DOWN", 29);
		} else {
			strncpy(currDisp.pitch_mode, "-----", 29);	
		}

		if (yaw >= 0.5 + GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.yaw_mode, "RIGHT", 29);
		} else if (yaw <= 0.5 - GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.yaw_mode, "LEFT", 29);
		} else {
			strncpy(currDisp.yaw_mode, "-----", 29);
		}

		if (throttle >= 0.5) {
			strncpy(currDisp.throttle_mode, "UP", 29);
		} else if (throttle < 0 + GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.throttle_mode, "-----", 29);	
		} else {
			strncpy(currDisp.throttle_mode, "DOWN", 29);	
		}

		print_stats(currDisp);
	}

	print_command(currDisp);
}
