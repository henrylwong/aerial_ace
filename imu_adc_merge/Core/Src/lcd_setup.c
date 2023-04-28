/* Includes ------------------------------------------------------------------*/
#include "lcd_setup.h"

double a[100];
double b[100];
double X_arr[50];
double Y_arr[50];
int sectors[CAL_TIME_SEC][6];

int CAL_CIRCLE_RADIUS_INNER = round(CAL_CIRCLE_RADIUS - CAL_CIRCLE_RADIUS / 7);

void LCD_small_delay() {
    nano_wait(10000);
    return;
}

void LCD_print_labels() {
	LCD_DrawString(5, 10, WHITE, BLACK, "Aerial Ace Status Window", 16, 0);
	LCD_DrawFillRectangle(2, 35, 300, 110, currDisp.color);
	LCD_DrawString(5, 45, BLACK, currDisp.color, "Current Mode", 16, 0);
	LCD_DrawLine(2, 30, 300, 30, WHITE);
	LCD_DrawLine(2, 160, 300, 160, WHITE);
}

void LCD_print_title(DispState currDisp) {
	LCD_DrawString(50, 70, BLACK, currDisp.color, currDisp.title, 16, 0);
	return;
}

void LCD_print_command(DispState currDisp) {
	
	LCD_DrawString(0, 120,  WHITE, BLACK, currDisp.command_ln1, 12, 0);
	LCD_DrawString(0, 130,  WHITE, BLACK, currDisp.command_ln2, 12, 0);
	for(int i = 0; i < 1000; i++);
	return;
}

void LCD_print_stats(DispState currDisp) {
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
	LCD_DrawString(10, 210, WHITE, BLACK, "ROLL", 16, 0); //150
	LCD_DrawString(10, 245, WHITE, BLACK, "THROTTLE", 16, 0); //200
	LCD_DrawString(10, 280, WHITE, BLACK, "YAW", 16, 0); //250

	LCD_DrawString(100, 175, WHITE, BLACK, pitch , 16, 0);
	LCD_DrawString(100, 210, WHITE, BLACK, roll, 16, 0);
	LCD_DrawString(100, 245, WHITE, BLACK, throttle, 16, 0);
	LCD_DrawString(100, 280, WHITE, BLACK, yaw , 16, 0);

	LCD_DrawString(170, 175, WHITE, BLACK, currDisp.pitch_mode, 16, 0);
	LCD_DrawString(170, 210, WHITE, BLACK, currDisp.roll_mode, 16, 0);
	LCD_DrawString(170, 245, WHITE, BLACK, currDisp.throttle_mode, 16, 0);
	LCD_DrawString(170, 280, WHITE, BLACK, currDisp.yaw_mode, 16, 0);
	return;
}

void LCD_print_progress(int time_secs, int curTim) {
	char strs[3];
	sprintf(strs, "%d", curTim);

	if (curTim < time_secs) {
		LCD_DrawFillTriangle(sectors[curTim][0], sectors[curTim][1], sectors[curTim][2], sectors[curTim][3], sectors[curTim][4], sectors[curTim][5], BLACK);
	}
	LCD_DrawFillRectangle(CAL_CIRCLE_X - 8, CAL_CIRCLE_Y - 8, CAL_CIRCLE_X + 8, CAL_CIRCLE_Y + 8, BLACK);
	LCD_DrawString(CAL_CIRCLE_X - 8, CAL_CIRCLE_Y - 8, WHITE, BLACK, strs, 16, 1);
}

void LCD_print_circle(int time_secs, int CX, int CY, int radius, u16 color) {
	LCD_generate_sectors(time_secs, CX, CY, radius);
	for(int i = 0; i < time_secs; i++) {
		LCD_DrawFillTriangle(sectors[i][0], sectors[i][1], sectors[i][2], sectors[i][3], sectors[i][4], sectors[i][5], color);
	}
}

void LCD_generate_sectors(int time_secs, int CX, int CY, int radius) {
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

void LCD_update(float roll, float pitch, float throttle, float yaw, int state, int total_time_sec, int cnt_sec) {
	currDisp.state = state;
	if (currDisp.state == INIT) {
		strncpy(currDisp.title, "INITIALISING        ", 29);
		strncpy(currDisp.command_ln1, " .            ...loading...            ", 199);
		strncpy(currDisp.command_ln2, "                                       ", 199);
		currDisp.color = LGRAY;
	} else if(currDisp.state == CAL_UNFLEXED) {
		strncpy(currDisp.title, "CALIBRATION - UNFLEX", 29);
		strncpy(currDisp.command_ln1, "Please unflex your fingers until finger", 199);
		strncpy(currDisp.command_ln2, "angles are 0 degrees.                  ", 199);
		currDisp.color = GRED;
	} else if(currDisp.state == CAL_FLEXED) {
		strncpy(currDisp.title, "CALIBRATION - FLEX  ", 29);
		strncpy(currDisp.command_ln1, "Please flex your fingers until finger  ", 199);
		strncpy(currDisp.command_ln2, "angles are 90 degrees.                 ", 199);
		currDisp.color = GBLUE;
	} else if(currDisp.state == MODE_STANDARD) {
		strncpy(currDisp.title, "STANDARD            ", 29);
		strncpy(currDisp.command_ln1, "Toggle switch for advanced mode!       ", 199);
		strncpy(currDisp.command_ln2, "                                       ", 199);
		currDisp.color = BROWN;
	} else if(currDisp.state == MODE_ADVANCED) {
		strncpy(currDisp.title, "ADVANCED            ", 29);
		strncpy(currDisp.command_ln1, "Toggle switch for standard mode!       ", 199);
		strncpy(currDisp.command_ln2, "                                       ", 199);
		currDisp.color = CYAN;
	}

	LCD_print_labels(); // @henry: can be done in init?
	LCD_print_title(currDisp);

	if (currDisp.state == CAL_FLEXED || currDisp.state == CAL_UNFLEXED) {
		if (cnt_sec == CAL_TIME_SEC) {
			LCD_print_circle(CAL_TIME_SEC, CAL_CIRCLE_X, CAL_CIRCLE_Y, CAL_CIRCLE_RADIUS, currDisp.color);
			LCD_print_circle(CAL_TIME_SEC, CAL_CIRCLE_X, CAL_CIRCLE_Y, CAL_CIRCLE_RADIUS_INNER, BLACK);
			LCD_generate_sectors(CAL_TIME_SEC, CAL_CIRCLE_X, CAL_CIRCLE_Y, CAL_CIRCLE_RADIUS);
		}
		LCD_print_progress(CAL_TIME_SEC, cnt_sec);

	} else if (currDisp.state == MODE_ADVANCED) {
		currDisp.pitch_num = pitch;
		currDisp.yaw_num = yaw;
		currDisp.roll_num = roll;
		currDisp.throttle_num = throttle;
		if (roll >= 0.5 + GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.roll_mode, "RIGHT", 29);
		} else if (roll <= 0.5 - GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.roll_mode, "LEFT ", 29);
		} else {
			strncpy(currDisp.roll_mode, "-----", 29);
		}

		if (pitch >= 0.5 + GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.pitch_mode, "UP   ", 29);
		} else if (pitch <= 0.5 - GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.pitch_mode, "DOWN ", 29);
		} else {
			strncpy(currDisp.pitch_mode, "-----", 29);	
		}

		if (yaw >= 0.5 + GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.yaw_mode, "LEFT", 29);
		} else if (yaw <= 0.5 - GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.yaw_mode, "RIGHT ", 29);
		} else {
			strncpy(currDisp.yaw_mode, "-----", 29);
		}

		if (throttle >= 0.5) {
			strncpy(currDisp.throttle_mode, "UP   ", 29);
		} else if (throttle < 0 + GIMBAL_IDLE_THRESH) {
			strncpy(currDisp.throttle_mode, "-----", 29);	
		} else {
			strncpy(currDisp.throttle_mode, "DOWN ", 29);	
		}

		LCD_print_stats(currDisp);
	} else if (currDisp.state == MODE_STANDARD) {
		LCD_DrawFillRectangle(10, 165, 230, 330, BLACK);
	}

	LCD_print_command(currDisp);
}
