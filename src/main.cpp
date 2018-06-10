#include "main.h"
#include "stm32f0xx_hal.h"
#include "led.h"
#include <vector>

ADC_HandleTypeDef hadc;
TIM_HandleTypeDef htim1, htim6, htim7;

void SystemClock_Config(void);
static void MX_GPIO_Init(void);
static void MX_TIM1_Init(void);
static void MX_ADC_Init(void);
static void User_Btn_Timer_Init();
static void AnimationStepTimerInit();

#define BTN_PRESSED_THRESHOLD_PERIODS 15  // 1.5 seconds
#define ASPECT_ROTATE_PERIODS 25  // 2.5 seconds

volatile int BTN_PRESSED_NUM_PERIODS = 0;
volatile int NUM_PERIODS_SINCE_LAST_ROTATE = 0;
static std::vector<AnimatedLED>* LEDS = new std::vector<AnimatedLED>{
		AnimatedLED(GPIOC, GPIO_PIN_3),
		AnimatedLED(GPIOC, GPIO_PIN_1),
		AnimatedLED(GPIOC, GPIO_PIN_0)
};

static std::vector<SignalHead>* HEADS = new std::vector<SignalHead>{

};

volatile LampStyle CURRENT_STYLE = LampStyle::SEARCHLIGHT;

// NOTE: Any function that overrides a "weak" HAL fn
// must be 'extern "C"' to ensure name lookup works correctly.
// https://electronics.stackexchange.com/questions/279524/stm32-interrupts-and-c-dont-go-well-together
extern "C" void TIM6_IRQHandler() {
	HAL_TIM_IRQHandler(&htim6);
}

extern "C" void TIM7_IRQHandler() {
	HAL_TIM_IRQHandler(&htim7);
}

static inline void LD2_Set(short state) {
	HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin,
			state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

void RotateLampStyleMode() {
	switch (CURRENT_STYLE) {
	case LampStyle::LED:
		CURRENT_STYLE = LampStyle::INCANDESCENT;
		break;
	case LampStyle::INCANDESCENT:
		CURRENT_STYLE = LampStyle::SEARCHLIGHT;
		break;
	default:
		CURRENT_STYLE = LampStyle::LED;
		break;
	}

	for (AnimatedLED& led : *LEDS) {
		led.setStyle(CURRENT_STYLE);
	}
}

void RotateAspects() {

}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM6) {
		if (HAL_GPIO_ReadPin(user_btn_GPIO_Port, user_btn_Pin)
				== GPIO_PIN_RESET) {
			// button is pressed
			BTN_PRESSED_NUM_PERIODS++;

			if (BTN_PRESSED_NUM_PERIODS > BTN_PRESSED_THRESHOLD_PERIODS) {
				RotateLampStyleMode();
				// restart count (and give buffer time) to avoid one rotation per
				// timer period while button is held over threshold.
				BTN_PRESSED_NUM_PERIODS = -BTN_PRESSED_THRESHOLD_PERIODS;
			} else if (BTN_PRESSED_NUM_PERIODS < 0) {
				HAL_GPIO_TogglePin(ld2_GPIO_Port, ld2_Pin);
			}
		} else {
			// button is released
			BTN_PRESSED_NUM_PERIODS = 0;
			LD2_Set(0);
		}

		if (NUM_PERIODS_SINCE_LAST_ROTATE++ > ASPECT_ROTATE_PERIODS) {
			NUM_PERIODS_SINCE_LAST_ROTATE = 0;
			RotateAspects();
		}
	} else if (htim->Instance == TIM7) {
		// Called at 100kHz
		static uint32_t counter = 0;
		++counter;

		for (AnimatedLED& led : *LEDS) {
			led.compute_pwm_state();
			led.update_state();
		}

		if (counter >= 100) {
			// every 1000 Hz
			for (AnimatedLED& led : *LEDS) {
				led.compute_animation_state();
			}
			counter = 0;
		}
	}
}

int main(void) {
	/* Reset of all peripherals, Initializes the Flash interface and the Systick. */
	HAL_Init();
	/* Configure the system clock */
	SystemClock_Config();
	/* Initialize all configured peripherals */
	MX_GPIO_Init();
	MX_TIM1_Init();
	MX_ADC_Init();
	User_Btn_Timer_Init();
	AnimationStepTimerInit();


	for (LED& led : *LEDS) {
		led.init();
	}

	SignalHead head((*LEDS)[2], (*LEDS)[1], (*LEDS)[0]);

	// Show current mode on LD2 via blink pattern
	bool toggle = true;
	while (1) {
		head.rotate_aspect();

		for (int i = 0; i < (unsigned short) CURRENT_STYLE; i++) {
			LD2_Set(1);
			HAL_Delay(100);
			LD2_Set(0);
			HAL_Delay(100);
		}
		HAL_Delay(2000);
	}
}

void SystemClock_Config(void) {

	RCC_OscInitTypeDef RCC_OscInitStruct;
	RCC_ClkInitTypeDef RCC_ClkInitStruct;

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI14
			| RCC_OSCILLATORTYPE_HSE;
	RCC_OscInitStruct.HSEState = RCC_HSE_ON;
	RCC_OscInitStruct.HSI14State = RCC_HSI14_ON;
	RCC_OscInitStruct.HSI14CalibrationValue = 16;
	RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
	RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
	RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL6;
	RCC_OscInitStruct.PLL.PREDIV = RCC_PREDIV_DIV1;
	if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Initializes the CPU, AHB and APB busses clocks
	 */
	RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK | RCC_CLOCKTYPE_SYSCLK
			| RCC_CLOCKTYPE_PCLK1;
	RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
	RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
	RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV1;

	if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure the Systick interrupt time
	 */
	HAL_SYSTICK_Config(HAL_RCC_GetHCLKFreq() / 1000);

	/**Configure the Systick
	 */
	HAL_SYSTICK_CLKSourceConfig(SYSTICK_CLKSOURCE_HCLK);

	/* SysTick_IRQn interrupt configuration */
	HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

/* ADC init function */
static void MX_ADC_Init(void) {

	ADC_ChannelConfTypeDef sConfig;

	/**Configure the global features of the ADC (Clock, Resolution, Data Alignment and number of conversion)
	 */
	hadc.Instance = ADC1;
	hadc.Init.ClockPrescaler = ADC_CLOCK_ASYNC_DIV1;
	hadc.Init.Resolution = ADC_RESOLUTION_12B;
	hadc.Init.DataAlign = ADC_DATAALIGN_RIGHT;
	hadc.Init.ScanConvMode = ADC_SCAN_DIRECTION_FORWARD;
	hadc.Init.EOCSelection = ADC_EOC_SINGLE_CONV;
	hadc.Init.LowPowerAutoWait = DISABLE;
	hadc.Init.LowPowerAutoPowerOff = DISABLE;
	hadc.Init.ContinuousConvMode = DISABLE;
	hadc.Init.DiscontinuousConvMode = DISABLE;
	hadc.Init.ExternalTrigConv = ADC_SOFTWARE_START;
	hadc.Init.ExternalTrigConvEdge = ADC_EXTERNALTRIGCONVEDGE_NONE;
	hadc.Init.DMAContinuousRequests = DISABLE;
	hadc.Init.Overrun = ADC_OVR_DATA_PRESERVED;
	if (HAL_ADC_Init(&hadc) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	/**Configure for the selected ADC regular channel to be converted.
	 */
	sConfig.Channel = ADC_CHANNEL_0;
	sConfig.Rank = ADC_RANK_CHANNEL_NUMBER;
	sConfig.SamplingTime = ADC_SAMPLETIME_1CYCLE_5;
	if (HAL_ADC_ConfigChannel(&hadc, &sConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

/* TIM1 init function */
static void MX_TIM1_Init(void) {

	TIM_ClockConfigTypeDef sClockSourceConfig;
	TIM_MasterConfigTypeDef sMasterConfig;

	htim1.Instance = TIM1;
	htim1.Init.Prescaler = 0;
	htim1.Init.CounterMode = TIM_COUNTERMODE_UP;
	htim1.Init.Period = 0;
	htim1.Init.ClockDivision = TIM_CLOCKDIVISION_DIV1;
	htim1.Init.RepetitionCounter = 0;
	htim1.Init.AutoReloadPreload = TIM_AUTORELOAD_PRELOAD_DISABLE;
	if (HAL_TIM_Base_Init(&htim1) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sClockSourceConfig.ClockSource = TIM_CLOCKSOURCE_INTERNAL;
	if (HAL_TIM_ConfigClockSource(&htim1, &sClockSourceConfig) != HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

	sMasterConfig.MasterOutputTrigger = TIM_TRGO_RESET;
	sMasterConfig.MasterSlaveMode = TIM_MASTERSLAVEMODE_DISABLE;
	if (HAL_TIMEx_MasterConfigSynchronization(&htim1, &sMasterConfig)
			!= HAL_OK) {
		_Error_Handler(__FILE__, __LINE__);
	}

}

// Init basic timer TIM6 for user btn (long?) press detection.
static void User_Btn_Timer_Init() {
	htim6.Instance = TIM6;
	htim6.Init.Prescaler = 47999; // 48MHz / 48000 = 1000Hz
	htim6.Init.Period = 99;  // 1000Hz / 100 = 10Hz = 0.1s
	__HAL_RCC_TIM6_CLK_ENABLE();
	HAL_NVIC_SetPriority(TIM6_IRQn, 2, 0);
	HAL_NVIC_EnableIRQ(TIM6_IRQn);
	HAL_TIM_Base_Init(&htim6);
	HAL_TIM_Base_Start_IT(&htim6);
}

static void AnimationStepTimerInit() {
	htim7.Instance = TIM7;
	htim7.Init.Prescaler = 479; // 48MHz / 4800 = 100kHz
	htim7.Init.Period = 1;  // as above
	__HAL_RCC_TIM7_CLK_ENABLE();
	HAL_NVIC_SetPriority(TIM7_IRQn, 1, 0);
	HAL_NVIC_EnableIRQ(TIM7_IRQn);
	HAL_TIM_Base_Init(&htim7);
	HAL_TIM_Base_Start_IT(&htim7);
}

/** Configure pins as 
 * Analog
 * Input
 * Output
 * EVENT_OUT
 * EXTI
 */
static void MX_GPIO_Init(void) {

	GPIO_InitTypeDef GPIO_InitStruct;

	/* GPIO Ports Clock Enable */
	__HAL_RCC_GPIOC_CLK_ENABLE()
	;
	__HAL_RCC_GPIOF_CLK_ENABLE()
	;
	__HAL_RCC_GPIOA_CLK_ENABLE()
	;

	/*Configure GPIO pin Output Level */
	HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET);

	/*Configure GPIO pin : user_btn_Pin */
	GPIO_InitStruct.Pin = user_btn_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	HAL_GPIO_Init(user_btn_GPIO_Port, &GPIO_InitStruct);

	/*Configure GPIO pin : ld2_Pin */
	GPIO_InitStruct.Pin = ld2_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Pull = GPIO_NOPULL;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(ld2_GPIO_Port, &GPIO_InitStruct);
}

extern "C" {

/**
 * @brief  This function is executed in case of error occurrence.
 * @param  file: The file name as string.
 * @param  line: The line in file as a number.
 * @retval None
 */
void _Error_Handler(char *file, int line) {
	while (1) {
		HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_SET);
		HAL_Delay(150);
		HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin, GPIO_PIN_RESET);
		HAL_Delay(150);
	}
}

#ifdef  USE_FULL_ASSERT
/**
 * @brief  Reports the name of the source file and the source line number
 *         where the assert_param error has occurred.
 * @param  file: pointer to the source file name
 * @param  line: assert_param error line source number
 * @retval None
 */
void assert_failed(uint8_t* file, uint32_t line) {
	/* USER CODE BEGIN 6 */
	/* User can add his own implementation to report the file name and line number,
	 tex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
	/* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */

}  // extern "C"

