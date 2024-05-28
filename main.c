/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file           : main.c
 * @brief          : Main program body
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2023 STMicroelectronics.
 * All rights reserved.
 *
 * This software is licensed under terms that can be found in the LICENSE file
 * in the root directory of this software component.
 * If no LICENSE file comes with this software, it is provided AS-IS.
 *
 ******************************************************************************
 */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <math.h>
#include <stdio.h>
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
ADC_HandleTypeDef hadc1;
ADC_HandleTypeDef hadc2;

I2C_HandleTypeDef hi2c1;

SPI_HandleTypeDef hspi1;

TIM_HandleTypeDef htim1;
TIM_HandleTypeDef htim2;
TIM_HandleTypeDef htim3;
TIM_HandleTypeDef htim4;

UART_HandleTypeDef huart1;

PCD_HandleTypeDef hpcd_USB_FS;

/* USER CODE BEGIN PV */

//uint8_t bufferRTC[100];
//uint32_t bufferindex = 0;
//int compare = 0;
//uint32_t hour;
//uint32_t minutes;
//uint32_t seconds;
//RTC_TimeTypeDef mytime = { .Hours = 1, .Minutes = 12, .Seconds = 2 };
/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_I2C1_Init(void);
static void MX_SPI1_Init(void);
static void MX_USB_PCD_Init(void);
static void MX_TIM3_Init(void);
static void MX_TIM4_Init(void);
static void MX_ADC1_Init(void);
static void MX_TIM1_Init(void);
static void MX_TIM2_Init(void);
static void MX_ADC2_Init(void);
static void MX_USART1_UART_Init(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
int key_time = 0;
int state_prim = 4;
TIM_HandleTypeDef *pwm_timer = &htim3;
int sin_counter = 400;
int tr_counter = 10;
int maxBuzzer = 500;
int minBuzzer = 0;
int flag = 0;
int timer = 0;
int vol, ldrVal;
int dimstep = 0;
int maxVol = 4095;
int minVol = 100;
int range = 4095 - 100;
int counterTimerAdc = 0;
int treshold = 10000, treshold1 = 0;
int first_light = 0;
int first_cnt = 0;
int countervol = 0;
int sum = 0, avg = 0, state_tr = 0, first_tap = 0;
int warnCount = 0;
int timerState2 = 0;
int alreadyWarned = 1;
int leds[8] = { GPIO_PIN_8, GPIO_PIN_9, GPIO_PIN_10, GPIO_PIN_11, GPIO_PIN_12,
GPIO_PIN_13, GPIO_PIN_14, GPIO_PIN_15 };
char input_uart[50];
int light = 0;
int warNum = 0;
char rx_byte[1];
char buffer_uart[50];
int buffer_index = 0;
uint32_t state = 0;
uint32_t buffer[100];
uint8_t buffer2transmist[100];

typedef struct {
	GPIO_TypeDef *port;
	uint16_t pin;
} pin_type;

typedef struct {
	pin_type digit_activators[4];
	pin_type BCD_input[4];
	uint32_t digits[4];
	uint32_t number;
} seven_segment_type;

void uart_send(uint8_t *buffer2transmist, int len) {
	HAL_UART_Transmit(&huart1, buffer2transmist, len, 100);
}

void ledOff_board() {
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(GPIOE, leds[i], 0);
	}
}
void ledOn_board() {
	for (int i = 0; i < 8; i++) {
		HAL_GPIO_WritePin(GPIOE, leds[i], 1);
	}
}

seven_segment_type seven_segment = { .digit_activators = { { .port = GPIOD,
		.pin = GPIO_PIN_8 }, { .port = GPIOD, .pin = GPIO_PIN_9 }, { .port =
GPIOD, .pin = GPIO_PIN_11 }, { .port = GPIOD, .pin = GPIO_PIN_10 } },
		.BCD_input = { { .port = GPIOA, .pin = GPIO_PIN_8 }, { .port = GPIOA,
				.pin = GPIO_PIN_10 }, { .port = GPIOA, .pin = GPIO_PIN_9 }, {
				.port = GPIOC, .pin = GPIO_PIN_9 } }, .digits = { 0, 0, 0, 0 },
		.number = 0 };

void seven_segment_display_decimal(uint32_t n) {
	if (n < 10) {
		HAL_GPIO_WritePin(seven_segment.BCD_input[0].port,
				seven_segment.BCD_input[0].pin,
				(n & 1) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(seven_segment.BCD_input[1].port,
				seven_segment.BCD_input[1].pin,
				(n & 2) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(seven_segment.BCD_input[2].port,
				seven_segment.BCD_input[2].pin,
				(n & 4) ? GPIO_PIN_SET : GPIO_PIN_RESET);
		HAL_GPIO_WritePin(seven_segment.BCD_input[3].port,
				seven_segment.BCD_input[3].pin,
				(n & 8) ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}
}

void seven_segment_deactivate_digits(void) {
	for (int i = 0; i < 4; ++i) {
		HAL_GPIO_WritePin(seven_segment.digit_activators[i].port,
				seven_segment.digit_activators[i].pin, GPIO_PIN_SET);
	}
}

void seven_segment_activate_digit(uint32_t d) {
	if (d < 4) {
		HAL_GPIO_WritePin(seven_segment.digit_activators[d].port,
				seven_segment.digit_activators[d].pin, GPIO_PIN_RESET);
	}
}

void seven_segment_set_num(uint32_t n) {
	if (n < 10000) {
		seven_segment.number = n;
		for (uint32_t i = 0; i < 4; ++i) {
			seven_segment.digits[3 - i] = n % 10;
			n /= 10;
		}
	}
}

void seven_segment_refresh(void) {

	static uint32_t last_time = 0;
	if (HAL_GetTick() - last_time > 5) {
		seven_segment_deactivate_digits();
		seven_segment_activate_digit(state);
		seven_segment_display_decimal(seven_segment.digits[state]);
		state = (state + 1) % 4;
		last_time = HAL_GetTick();
		if (state == state_prim && state_prim != 0) {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 1);
		} else {
			HAL_GPIO_WritePin(GPIOC, GPIO_PIN_8, 0);
		}
	}
}

void programInit() {
	seven_segment_set_num(0000);

}

void programLoop() {
	seven_segment.digits[3]=warnCount;
	seven_segment_refresh();

}

void ledOff() {
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);
}
void ledOn() {

	if (seven_segment.digits[1] == 0) {
		ledOff();
	} else if (seven_segment.digits[1] == 1) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

	} else if (seven_segment.digits[1] == 2) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 0);
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

	} else if (seven_segment.digits[1] == 3) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 0);

	} else if (seven_segment.digits[1] == 4) {
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3,
				(seven_segment.digits[0] * 100) + (vol * 50));
		__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4,
				(seven_segment.digits[0] * 100) + (vol * 50));
	}
}

void buzzer_Change_Tone(uint16_t pwm_freq, uint16_t volume) // pwm_freq (1 - 20000), volume (0 - 1000)
{
	if (pwm_freq == 0 || pwm_freq > 20000) {
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, 0);
	} else {
		const uint32_t internal_clock_freq = HAL_RCC_GetSysClockFreq();
		const uint16_t prescaler = 1 + internal_clock_freq / pwm_freq / 60000;
		const uint32_t timer_clock = internal_clock_freq / prescaler;
		const uint32_t period_cycles = timer_clock / pwm_freq;
		const uint32_t pulse_width = volume * period_cycles / 1000 / 2;

		pwm_timer->Instance->PSC = prescaler - 1;
		pwm_timer->Instance->ARR = period_cycles - 1;
		pwm_timer->Instance->EGR = TIM_EGR_UG;
		__HAL_TIM_SET_COMPARE(&htim4, TIM_CHANNEL_1, pulse_width); // pwm_timer->Instance->CCR2 = pulse_width;
	}
}

void warning() { //buzzer

	if (seven_segment.digits[2] == 1) { //sin
		static uint32_t last_millis = 0;
		buzzer_Change_Tone(
				(((maxBuzzer - minBuzzer) / 2)
						* sin(2 * 3.14 * sin_counter / 1000)
						+ ((maxBuzzer + minBuzzer) / 2)), 5);
		if (HAL_GetTick() - last_millis > 500) {
			uint8_t arr2[] = "[INFO] Digit changed\n";
			int len = sprintf(buffer2transmist,
					"[Info] Wave changed to {sin}\n");
			uart_send(arr2, sizeof(arr2));
			uart_send(buffer2transmist, len);
			last_millis = HAL_GetTick();
		}
	} else if (seven_segment.digits[2] == 2) { //square
		if (flag == 1) {
			buzzer_Change_Tone(1000, 50);
		} else {
			buzzer_Change_Tone(0, 50);
		}
		static uint32_t last_millis = 0;
		if (HAL_GetTick() - last_millis > 500) {
			uint8_t arr2[] = "[INFO] Digit changed\n";
			int len = sprintf(buffer2transmist,
					"[INFO] Wave changed to {Square}\n");
			uart_send(arr2, sizeof(arr2));
			uart_send(buffer2transmist, len);
			last_millis = HAL_GetTick();
		}
	} else if (seven_segment.digits[2] == 3) { //triangular
		static uint32_t last_millis = 0;
		buzzer_Change_Tone(tr_counter, 100);
		if (tr_counter > 2000)
			tr_counter = 0;
		if (HAL_GetTick() - last_millis > 500) {
			uint8_t arr2[] = "[INFO] Digit changed\n";
			int len = sprintf(buffer2transmist,
					"[Info] Wave changed to {triangular}\n");
			uart_send(arr2, sizeof(arr2));
			uart_send(buffer2transmist, len);
			last_millis = HAL_GetTick();
		}
	}
}

void increment_selected_digit() {
	if (state_prim == 1) { // 0 ta 9
		seven_segment.digits[0] = (seven_segment.digits[0] + 1) % 10;
	} else if (state_prim == 2) { // 0 ta 4
		seven_segment.digits[1] = (seven_segment.digits[1] + 1) % 5;

	} else if (state_prim == 3) { // 1 ta 3
		seven_segment.digits[2] = (seven_segment.digits[2] + 1) % 4;
		if (seven_segment.digits[2] == 0) {
			seven_segment.digits[2] = 1;
		}

	}
}
void decrese_selected_digit() {
	if (state_prim == 1) {
		if (seven_segment.digits[0] == 0) {
			seven_segment.digits[0] = 9;
		} else {
			seven_segment.digits[0] = (seven_segment.digits[0] - 1) % 9;
		}
	} else if (state_prim == 2) {
		if (seven_segment.digits[1] == 0) {
			seven_segment.digits[1] = 4;
		} else {
			seven_segment.digits[1] = (seven_segment.digits[1] - 1) % 5;
		}
	} else if (state_prim == 3) {
		if (seven_segment.digits[2] == 1) {
			seven_segment.digits[2] = 4;
		}
		seven_segment.digits[2] = (seven_segment.digits[2] - 1) % 4;
	}
}

void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin) {

	if (HAL_GetTick() - key_time < 200)
		return;
	key_time = HAL_GetTick();

	if (GPIO_Pin == GPIO_PIN_0) { //button
		state_prim++;
		if (state_prim > 3) {
			state_prim = 1;
		}
		if (first_tap == 0) {
			treshold = treshold1;
			first_tap = 1;
			unsigned char data[100] = "SALAM";
			int n = sprintf(data, "treshhold:  %d\n", treshold);
			HAL_UART_Transmit(&huart1, data, n, 1000);
			seven_segment_set_num(0000);
			state_tr = 1;
		}
	} else if (GPIO_Pin == GPIO_PIN_1) {
		increment_selected_digit();
		//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);
		ledOn();
		static uint32_t last_millis = 0;
		if (HAL_GetTick() - last_millis > 500) {
			uint8_t arr2[] = "[INFO] Digit changed\n";
			uart_send(arr2, sizeof(arr2));
			int len = snprintf(buffer2transmist, sizeof buffer2transmist,
					"[Info] Digit {%d} Increased\n", state_prim);
			uart_send(buffer2transmist, len);
			last_millis = HAL_GetTick();
		}
	} else if (GPIO_Pin == GPIO_PIN_2) {
		decrese_selected_digit();
		static uint32_t last_millis = 0;
		//HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_11);
		if (HAL_GetTick() - last_millis > 500) {
			uint8_t arr2[] = "[INFO] Digit changed\n";
			int len = sprintf(buffer2transmist, "[Info] Digit {%d} Dcreased\n",
					state_prim);
			uart_send(arr2, sizeof(arr2));
			uart_send(buffer2transmist, len);
			last_millis = HAL_GetTick();
		}
	}

}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim) {
	if (htim->Instance == TIM2) {
		timerState2++;
		counterTimerAdc++;
		sin_counter++;
		tr_counter++;
		ledOn_board();
		if (ldrVal > treshold) {
			ledOff_board();
//			ledOff();
			warning();
			if (alreadyWarned == 1) {
				static uint32_t last_millis = 0;
				if (HAL_GetTick() - last_millis > 500) {
					uint8_t arr2[] = "[INFO] Digit changed\n";
					uart_send(arr2, sizeof(arr2));
					int len = sprintf(buffer2transmist,
							"[WARN] Critical Situation. Light value:{%d}",
							ldrVal);
					uart_send(buffer2transmist, len);
					last_millis = HAL_GetTick();
				}
				state_tr = 2;
				ledOff();
				if (timerState2 < 500) {
					seven_segment_activate_digit(state);
				} else if (timerState2 < 1000) {
					seven_segment_deactivate_digits();
				} else {
					timerState2 = 0;
				}
			} else {
				warnCount++;
				if(warnCount>9)
					warnCount=0;
				alreadyWarned = 1;
			}
		} else {
			alreadyWarned = 0;
			buzzer_Change_Tone(0, 0);
		}
		HAL_ADC_Start_IT(&hadc1);
		if (counterTimerAdc > 100) {
			HAL_ADC_Start_IT(&hadc2);

			counterTimerAdc = 0;
		}
		timer++;
		if (timer == 500) {
			if (flag == 0) {
				flag = 1;
			} else {
				flag = 0;
			}
			timer = 0;
		}
		ledOn();
	}

}

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc)

{
	if (hadc->Instance == ADC1) {
		int y = HAL_ADC_GetValue(&hadc1);
		static uint32_t last = 0;
		vol = ((int) y * 20 / 4095);
		unsigned char data[100] = "SALAM";
		countervol++;
		vol -= 10;
		sum += vol;
		if (countervol == 100 && state_tr == 0) {
			avg = sum / 100;
			seven_segment_set_num(((avg + first_light) * 20) - 160); //scale for 20
			treshold1 = ((avg + first_light) * 20) - 160;
			if (HAL_GetTick() - last > 1000) {
				int n = sprintf(buffer2transmist, "tr1  %d vol  %d\n",
						treshold1, vol);
				uart_send(buffer2transmist, n);
//			HAL_UART_Transmit(&huart1, data, n, 1000);
				last = HAL_GetTick();
			}
			avg = 0;
			sum = 0;
			countervol = 0;
		}
	}

	if (hadc->Instance == ADC2) {
		int x = HAL_ADC_GetValue(&hadc2);
		ldrVal = ((int) x * 80 / 4095); //scale
		if (first_cnt == 0) {
			first_light = ((int) x * 30 / 4095);
			first_cnt = 1;
		}
	}
}

void check_uart() {
	dimstep = seven_segment.digits[0];
	light = seven_segment.digits[1];
	warNum = seven_segment.digits[2];
	uint8_t temp[100];
	uint8_t len;

	if (input_uart[0] == 'd' && input_uart[1] - '0' >= 0
			&& input_uart[1] - '0' <= 9 && input_uart[2] == 0xa) {
		if (dimstep > input_uart[1] - '0') {
			len = sprintf(temp, "Dimstep Decreased\n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		} else if (dimstep < input_uart[1] - '0') {
			len = sprintf(temp, "Dimstep increased \n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		}
		dimstep = input_uart[1] - '0';
		seven_segment.digits[0]=dimstep;

	} else if (input_uart[0] == 'l' && input_uart[1] - '0' >= 0
			&& input_uart[1] - '0' <= 4 && input_uart[2] == 0xa) {
		if (light > input_uart[1] - '0') {
			len = sprintf(temp, "Digit light decreased \n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		} else if (light < input_uart[1] - '0') {
			len = sprintf(temp, "Digit light increased \n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		}
		light = input_uart[1] - '0';
		seven_segment.digits[1]=light;


	} else if (input_uart[0] == 'w' && input_uart[1] - '0' >= 1
			&& input_uart[1] - '0' <= 3 && input_uart[2] == 0xa) {
		if (warNum > input_uart[1] - '0') {
			len = sprintf(temp, "Digit warNum Decreased\n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		} else if (warNum < input_uart[1] - '0') {
			len = sprintf(temp, "Digit warNum Increased\n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		}
		warNum = input_uart[1] - '0';
		seven_segment.digits[2]=warNum;

		if (warNum == 1) {
			len = sprintf(temp, "Wave changed to sin\n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		} else if (warNum == 2) {
			len = sprintf(temp, "Wave changed to square\n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		} else if (warNum == 3) {
			len = sprintf(temp, "Wave changed to 3\n");
			HAL_UART_Transmit(&huart1, temp, len, HAL_MAX_DELAY);
		}
	} else {
		HAL_UART_Transmit(&huart1, "INVALID PACKET", 14, HAL_MAX_DELAY);
	}

}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart) {
	if (huart->Instance == USART1) {
		input_uart[buffer_index++] = rx_byte[0];
		HAL_UART_Transmit(&huart1, rx_byte, 1, 1000);
		if (rx_byte[0] == '\n') {
			HAL_GPIO_TogglePin(GPIOE, GPIO_PIN_10);

			buffer_index = 0;
			HAL_UART_Transmit(&huart1, buffer_uart, buffer_index,
			HAL_MAX_DELAY);
			check_uart();
		}

		HAL_UART_Receive_IT(&huart1, rx_byte, 1);

	}
}

/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{
  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C1_Init();
  MX_SPI1_Init();
  MX_USB_PCD_Init();
  MX_TIM3_Init();
  MX_TIM4_Init();
  MX_ADC1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_ADC2_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
	programInit();
	HAL_TIM_Base_Start_IT(&htim2);
	HAL_TIM_Base_Start_IT(&htim3);
	HAL_TIM_Base_Start_IT(&htim4);

	HAL_TIM_PWM_Start(pwm_timer, TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(pwm_timer, TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(pwm_timer, TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(pwm_timer, TIM_CHANNEL_4);
	HAL_TIM_PWM_Start(&htim4, TIM_CHANNEL_1);
	HAL_ADC_Start_IT(&hadc2);
	HAL_ADC_Start_IT(&hadc1);
	HAL_UART_Receive_IT(&huart1, rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
	while (1) {
		programLoop();
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
	}
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};
  RCC_PeriphCLKInitTypeDef PeriphClkInit = {0};

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI|RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_BYPASS;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK)
  {
    Error_Handler();
  }
  PeriphClkInit.PeriphClockSelection = RCC_PERIPHCLK_USB|RCC_PERIPHCLK_USART1
                              |RCC_PERIPHCLK_I2C1|RCC_PERIPHCLK_TIM1
                              |RCC_PERIPHCLK_ADC12;
  PeriphClkInit.Usart1ClockSelection = RCC_USART1CLKSOURCE_PCLK2;
  PeriphClkInit.Adc12ClockSelection = RCC_ADC12PLLCLK_DIV1;
  PeriphClkInit.I2c1ClockSelection = RCC_I2C1CLKSOURCE_HSI;
  PeriphClkInit.USBClockSelection = RCC_USBCLKSOURCE_PLL;
  PeriphClkInit.Tim1ClockSelection = RCC_TIM1CLK_HCLK;
  if (HAL_RCCEx_PeriphCLKConfig(&PeriphClkInit) != HAL_OK)
  {
    Error_Handler();
  }
}

/**
  * @brief ADC1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC1_Init(void)
{

  /* USER CODE BEGIN ADC1_Init 0 */

  /* USER CODE END ADC1_Init 0 */

  ADC_MultiModeTypeDef multimode = {0};
  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC1_Init 1 */

  /* USER CODE END ADC1_Init 1 */

  /** Common config
  */
  hadc1.Instance = ADC1;
  hadc1.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc1.Init.Resolution = ADC_RESOLUTION_12B;
  hadc1.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc1.Init.ContinuousConvMode = DISABLE;
  hadc1.Init.DiscontinuousConvMode = DISABLE;
  hadc1.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc1.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc1.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc1.Init.NbrOfConversion = 1;
  hadc1.Init.DMAContinuousRequests = DISABLE;
  hadc1.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc1.Init.LowPowerAutoWait = DISABLE;
  hadc1.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure the ADC multi-mode
  */
  multimode.Mode = ADC_MODE_INDEPENDENT;
  if (HAL_ADCEx_MultiModeConfigChannel(&hadc1, &multimode) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_2;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc1, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC1_Init 2 */

  /* USER CODE END ADC1_Init 2 */

}

/**
  * @brief ADC2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_ADC2_Init(void)
{

  /* USER CODE BEGIN ADC2_Init 0 */

  /* USER CODE END ADC2_Init 0 */

  ADC_ChannelConfTypeDef sConfig = {0};

  /* USER CODE BEGIN ADC2_Init 1 */

  /* USER CODE END ADC2_Init 1 */

  /** Common config
  */
  hadc2.Instance = ADC2;
  hadc2.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
  hadc2.Init.Resolution = ADC_RESOLUTION_12B;
  hadc2.Init.ScanConvMode = ADC_SCAN_DISABLE;
  hadc2.Init.ContinuousConvMode = DISABLE;
  hadc2.Init.DiscontinuousConvMode = DISABLE;
  hadc2.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
  hadc2.Init.ExternalTrigConv = ADC_SOFTWARE_START;
  hadc2.Init.DataAlign = ADC_DATAALIGN_RIGHT;
  hadc2.Init.NbrOfConversion = 1;
  hadc2.Init.DMAContinuousRequests = DISABLE;
  hadc2.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
  hadc2.Init.LowPowerAutoWait = DISABLE;
  hadc2.Init.Overrun = ADC_OVR_DATA_OVERWRITTEN;
  if (HAL_ADC_Init(&hadc2) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Regular Channel
  */
  sConfig.Channel = ADC_CHANNEL_1;
  sConfig.Rank = ADC_REGULAR_RANK_1;
  sConfig.SingleDiff = ADC_SINGLE_ENDED;
  sConfig.SamplingTime = ADC_SAMPLETIME_601CYCLES_5;
  sConfig.OffsetNumber = ADC_OFFSET_NONE;
  sConfig.Offset = 0;
  if (HAL_ADC_ConfigChannel(&hadc2, &sConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN ADC2_Init 2 */

  /* USER CODE END ADC2_Init 2 */

}

/**
  * @brief I2C1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_I2C1_Init(void)
{

  /* USER CODE BEGIN I2C1_Init 0 */

  /* USER CODE END I2C1_Init 0 */

  /* USER CODE BEGIN I2C1_Init 1 */

  /* USER CODE END I2C1_Init 1 */
  hi2c1.Instance = I2C1;
  hi2c1.Init.Timing = 0x2000090E;
  hi2c1.Init.OwnAddress1 = 0;
  hi2c1.Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
  hi2c1.Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
  hi2c1.Init.OwnAddress2 = 0;
  hi2c1.Init.OwnAddress2Masks = I2C_OA2_NOMASK;
  hi2c1.Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
  hi2c1.Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;
  if (HAL_I2C_Init(&hi2c1) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Analogue filter
  */
  if (HAL_I2CEx_ConfigAnalogFilter(&hi2c1, I2C_ANALOGFILTER_ENABLE) != HAL_OK)
  {
    Error_Handler();
  }

  /** Configure Digital filter
  */
  if (HAL_I2CEx_ConfigDigitalFilter(&hi2c1, 0) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN I2C1_Init 2 */

  /* USER CODE END I2C1_Init 2 */

}

/**
  * @brief SPI1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_SPI1_Init(void)
{

  /* USER CODE BEGIN SPI1_Init 0 */

  /* USER CODE END SPI1_Init 0 */

  /* USER CODE BEGIN SPI1_Init 1 */

  /* USER CODE END SPI1_Init 1 */
  /* SPI1 parameter configuration*/
  hspi1.Instance = SPI1;
  hspi1.Init.Mode = SPI_MODE_MASTER;
  hspi1.Init.Direction = SPI_DIRECTION_2LINES;
  hspi1.Init.DataSize = SPI_DATASIZE_4BIT;
  hspi1.Init.CLKPolarity = SPI_POLARITY_LOW;
  hspi1.Init.CLKPhase = SPI_PHASE_1EDGE;
  hspi1.Init.NSS = SPI_NSS_SOFT;
  hspi1.Init.BaudRatePrescaler = SPI_BAUDRATEPRESCALER_4;
  hspi1.Init.FirstBit = SPI_FIRSTBIT_MSB;
  hspi1.Init.TIMode = SPI_TIMODE_DISABLE;
  hspi1.Init.CRCCalculation = SPI_CRCCALCULATION_DISABLE;
  hspi1.Init.CRCPolynomial = 7;
  hspi1.Init.CRCLength = SPI_CRC_LENGTH_DATASIZE;
  hspi1.Init.NSSPMode = SPI_NSS_PULSE_ENABLE;
  if (HAL_SPI_Init(&hspi1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN SPI1_Init 2 */

  /* USER CODE END SPI1_Init 2 */

}

/**
  * @brief TIM1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM1_Init(void)
{

  /* USER CODE BEGIN TIM1_Init 0 */

  /* USER CODE END TIM1_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM1_Init 1 */

  /* USER CODE END TIM1_Init 1 */
  htim1.Instance = TIM1;
  htim1.Init.Prescaler = 47;
  htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim1.Init.Period = 999;
  htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim1.Init.RepetitionCounter = 0;
  htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim1) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterOutputTrigger2 = TIM_TRGO2_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM1_Init 2 */

  /* USER CODE END TIM1_Init 2 */

}

/**
  * @brief TIM2 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM2_Init(void)
{

  /* USER CODE BEGIN TIM2_Init 0 */

  /* USER CODE END TIM2_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};

  /* USER CODE BEGIN TIM2_Init 1 */

  /* USER CODE END TIM2_Init 1 */
  htim2.Instance = TIM2;
  htim2.Init.Prescaler = 47;
  htim2.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim2.Init.Period = 999;
  htim2.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim2.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim2) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim2, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim2, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM2_Init 2 */

  /* USER CODE END TIM2_Init 2 */

}

/**
  * @brief TIM3 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM3_Init(void)
{

  /* USER CODE BEGIN TIM3_Init 0 */

  /* USER CODE END TIM3_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM3_Init 1 */

  /* USER CODE END TIM3_Init 1 */
  htim3.Instance = TIM3;
  htim3.Init.Prescaler = 47;
  htim3.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim3.Init.Period = 999;
  htim3.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim3.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim3, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim3) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim3, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_2) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_3) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_ConfigChannel(&htim3, &sConfigOC, TIM_CHANNEL_4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM3_Init 2 */

  /* USER CODE END TIM3_Init 2 */
  HAL_TIM_MspPostInit(&htim3);

}

/**
  * @brief TIM4 Initialization Function
  * @param None
  * @retval None
  */
static void MX_TIM4_Init(void)
{

  /* USER CODE BEGIN TIM4_Init 0 */

  /* USER CODE END TIM4_Init 0 */

  TIM_ClockConfigTypeDef sClockSourceConfig = {0};
  TIM_MasterConfigTypeDef sMasterConfig = {0};
  TIM_OC_InitTypeDef sConfigOC = {0};

  /* USER CODE BEGIN TIM4_Init 1 */

  /* USER CODE END TIM4_Init 1 */
  htim4.Instance = TIM4;
  htim4.Init.Prescaler = 0;
  htim4.Init.CounterMode = TIM_COUNTERMODE_UP;
  htim4.Init.Period = 65535;
  htim4.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
  htim4.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
  if (HAL_TIM_Base_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
  if (HAL_TIM_ConfigClockSource(&htim4, &sClockSourceConfig) != HAL_OK)
  {
    Error_Handler();
  }
  if (HAL_TIM_PWM_Init(&htim4) != HAL_OK)
  {
    Error_Handler();
  }
  sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
  sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
  if (HAL_TIMEx_MasterConfigSynchronization(&htim4, &sMasterConfig) != HAL_OK)
  {
    Error_Handler();
  }
  sConfigOC.OCMode = TIM_OCMODE_PWM1;
  sConfigOC.Pulse = 0;
  sConfigOC.OCPolarity = TIM_OCPOLARITY_HIGH;
  sConfigOC.OCFastMode = TIM_OCFAST_DISABLE;
  if (HAL_TIM_PWM_ConfigChannel(&htim4, &sConfigOC, TIM_CHANNEL_1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN TIM4_Init 2 */

  /* USER CODE END TIM4_Init 2 */
  HAL_TIM_MspPostInit(&htim4);

}

/**
  * @brief USART1 Initialization Function
  * @param None
  * @retval None
  */
static void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 38400;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  huart1.Init.OneBitSampling = UART_ONE_BIT_SAMPLE_DISABLE;
  huart1.AdvancedInit.AdvFeatureInit = UART_ADVFEATURE_NO_INIT;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

/**
  * @brief USB Initialization Function
  * @param None
  * @retval None
  */
static void MX_USB_PCD_Init(void)
{

  /* USER CODE BEGIN USB_Init 0 */

  /* USER CODE END USB_Init 0 */

  /* USER CODE BEGIN USB_Init 1 */

  /* USER CODE END USB_Init 1 */
  hpcd_USB_FS.Instance = USB;
  hpcd_USB_FS.Init.dev_endpoints = 8;
  hpcd_USB_FS.Init.speed = PCD_SPEED_FULL;
  hpcd_USB_FS.Init.phy_itface = PCD_PHY_EMBEDDED;
  hpcd_USB_FS.Init.low_power_enable = DISABLE;
  hpcd_USB_FS.Init.battery_charging_enable = DISABLE;
  if (HAL_PCD_Init(&hpcd_USB_FS) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USB_Init 2 */

  /* USER CODE END USB_Init 2 */

}

/**
  * @brief GPIO Initialization Function
  * @param None
  * @retval None
  */
static void MX_GPIO_Init(void)
{
  GPIO_InitTypeDef GPIO_InitStruct = {0};

  /* GPIO Ports Clock Enable */
  __HAL_RCC_GPIOE_CLK_ENABLE();
  __HAL_RCC_GPIOC_CLK_ENABLE();
  __HAL_RCC_GPIOF_CLK_ENABLE();
  __HAL_RCC_GPIOA_CLK_ENABLE();
  __HAL_RCC_GPIOB_CLK_ENABLE();
  __HAL_RCC_GPIOD_CLK_ENABLE();

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOE, CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|GPIO_PIN_10
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOC, GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOD, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11, GPIO_PIN_RESET);

  /*Configure GPIO pin Output Level */
  HAL_GPIO_WritePin(GPIOA, GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10, GPIO_PIN_RESET);

  /*Configure GPIO pins : CS_I2C_SPI_Pin LD4_Pin LD3_Pin PE10
                           LD7_Pin LD9_Pin LD10_Pin LD8_Pin
                           LD6_Pin */
  GPIO_InitStruct.Pin = CS_I2C_SPI_Pin|LD4_Pin|LD3_Pin|GPIO_PIN_10
                          |LD7_Pin|LD9_Pin|LD10_Pin|LD8_Pin
                          |LD6_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : MEMS_INT3_Pin MEMS_INT4_Pin */
  GPIO_InitStruct.Pin = MEMS_INT3_Pin|MEMS_INT4_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_EVT_RISING;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(GPIOE, &GPIO_InitStruct);

  /*Configure GPIO pins : PC0 PC1 PC2 */
  GPIO_InitStruct.Pin = GPIO_PIN_0|GPIO_PIN_1|GPIO_PIN_2;
  GPIO_InitStruct.Mode = GPIO_MODE_IT_RISING;
  GPIO_InitStruct.Pull = GPIO_PULLUP;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pins : PC3 PC6 PC7 PC8
                           PC9 */
  GPIO_InitStruct.Pin = GPIO_PIN_3|GPIO_PIN_6|GPIO_PIN_7|GPIO_PIN_8
                          |GPIO_PIN_9;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

  /*Configure GPIO pin : B1_Pin */
  GPIO_InitStruct.Pin = B1_Pin;
  GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  HAL_GPIO_Init(B1_GPIO_Port, &GPIO_InitStruct);

  /*Configure GPIO pins : PD8 PD9 PD10 PD11 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10|GPIO_PIN_11;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

  /*Configure GPIO pins : PA8 PA9 PA10 */
  GPIO_InitStruct.Pin = GPIO_PIN_8|GPIO_PIN_9|GPIO_PIN_10;
  GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
  GPIO_InitStruct.Pull = GPIO_NOPULL;
  GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
  HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

  /* EXTI interrupt init*/
  HAL_NVIC_SetPriority(EXTI0_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI0_IRQn);

  HAL_NVIC_SetPriority(EXTI1_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI1_IRQn);

  HAL_NVIC_SetPriority(EXTI2_TSC_IRQn, 0, 0);
  HAL_NVIC_EnableIRQ(EXTI2_TSC_IRQn);

}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
	/* User can add his own implementation to report the HAL error return state */
	__disable_irq();
	while (1) {
	}
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
