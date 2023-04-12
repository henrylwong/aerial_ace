/* Includes ------------------------------------------------------------------*/
#include "stm32f3xx_hal.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

extern int ADC_vals[4];
//extern int ADC1_val;
//extern int ADC2_val;
//extern int ADC3_val;
//extern int ADC4_val;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
static void ADC_ADVREGEN(ADC_TypeDef* adc);
static void ADC_Calibrate(ADC_TypeDef*);
static int ADC_GetVal(ADC_TypeDef*);
static void ADC_Read(void);

/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */

static void ADC_ADVREGEN(ADC_TypeDef* adc) {
	adc->CR &= ~ADC_CR_ADVREGEN;
	adc->CR |= 0x1UL << ADC_CR_ADVREGEN_Pos;
}

static void ADC_Read(void) {
//	ADC1_val = ADC_GetVal(ADC1);
//	ADC2_val = ADC_GetVal(ADC2);
//	ADC3_val = ADC_GetVal(ADC3);
//	ADC4_val = ADC_GetVal(ADC4);
	ADC_vals[0] = ADC_GetVal(ADC1);
	ADC_vals[1] = ADC_GetVal(ADC2);
	ADC_vals[2] = ADC_GetVal(ADC3);
	ADC_vals[3] = ADC_GetVal(ADC4);
}

static void ADC_Calibrate(ADC_TypeDef* adc) {
  if (adc != ADC1 && adc != ADC2 && adc != ADC3 && adc != ADC4) {
	  return;
  }

  adc->CR &= ~ADC_CR_ADEN; // Disable ADC
  adc->CR |= ADC_CR_ADCALDIF; // Calibration for Single-ended input mode
  adc->CR |= ADC_CR_ADCAL; // Start ADC calibration
  while (adc->CR & ADC_CR_ADCAL);
}

static int ADC_GetVal(ADC_TypeDef* adc) {
	if (adc != ADC1 && adc != ADC2 && adc != ADC3 && adc != ADC4) {
		return 0;
	}
	return adc->DR;
}
