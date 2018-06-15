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

#define ASPECT_ROTATE_PERIODS 50  // 5 seconds
volatile int NUM_PERIODS_SINCE_LAST_ASPECT_ROTATE = 0;

// Leftmost column on proto-board
static AnimatedLED LED_C0(GPIOC, GPIO_PIN_0);
static AnimatedLED LED_C1(GPIOC, GPIO_PIN_1);
static AnimatedLED LED_C3(GPIOC, GPIO_PIN_3);

static AnimatedLED LED_A1(GPIOA, GPIO_PIN_1);
static AnimatedLED LED_B0(GPIOB, GPIO_PIN_0);
static AnimatedLED LED_C2(GPIOC, GPIO_PIN_2);

static AnimatedLED LED_C10(GPIOC, GPIO_PIN_10);
static AnimatedLED LED_C11(GPIOC, GPIO_PIN_11);
static AnimatedLED LED_C12(GPIOC, GPIO_PIN_12);

// Center column
static AnimatedLED LED_B13(GPIOB, GPIO_PIN_13);
static AnimatedLED LED_B14(GPIOB, GPIO_PIN_14);
static AnimatedLED LED_B15(GPIOB, GPIO_PIN_15);

static AnimatedLED LED_B3(GPIOB, GPIO_PIN_3);
static AnimatedLED LED_B4(GPIOB, GPIO_PIN_4);
static AnimatedLED LED_B5(GPIOB, GPIO_PIN_5);

static AnimatedLED LED_B11(GPIOB, GPIO_PIN_11);
static AnimatedLED LED_B12(GPIOB, GPIO_PIN_12);
static AnimatedLED LED_A11(GPIOA, GPIO_PIN_11);

// Rightmost column
static AnimatedLED LED_C5(GPIOC, GPIO_PIN_5);
static AnimatedLED LED_C6(GPIOC, GPIO_PIN_6);
static AnimatedLED LED_C8(GPIOC, GPIO_PIN_8);

static AnimatedLED LED_A9(GPIOA, GPIO_PIN_9);
static AnimatedLED LED_A8(GPIOA, GPIO_PIN_8);
static AnimatedLED LED_B10(GPIOB, GPIO_PIN_10);


static std::vector<SignalHead>* COL_A_HEADS = new std::vector<SignalHead>{
	SignalHead(&LED_C3, &LED_C1, &LED_C0, LampStyle::INCANDESCENT),
	SignalHead(&LED_A1, &LED_B0, &LED_C2, LampStyle::LED),
	SignalHead(&LED_C10, &LED_C11, &LED_C12, LampStyle::SEARCHLIGHT),
};

static std::vector<SignalHead>* COL_B_HEADS = new std::vector<SignalHead>{
	SignalHead(&LED_C5, &LED_C6, &LED_C8, LampStyle::SEARCHLIGHT),
	SignalHead(&LED_B11, &LED_B12, &LED_A11, LampStyle::SEARCHLIGHT),  // upper
	SignalHead(&LED_B3, &LED_B5, &LED_B4, LampStyle::SEARCHLIGHT),  // lower
};

static std::vector<SignalHead>* COL_C_HEADS = new std::vector<SignalHead>{
	SignalHead(&LED_A9, &LED_A8, &LED_B10, LampStyle::INCANDESCENT),
	SignalHead(&LED_B13, &LED_B14, &LED_B15, LampStyle::INCANDESCENT),
};

static std::vector<SignalHead*>* ALL_HEADS = new std::vector<SignalHead*>();

// NOTE: Any function that overrides a "weak" HAL fn
// must be 'extern "C"' to ensure name lookup works correctly.
// https://electronics.stackexchange.com/questions/279524/stm32-interrupts-and-c-dont-go-well-together
extern "C" void TIM6_IRQHandler() {
	HAL_TIM_IRQHandler(&htim6);
}

extern "C" void TIM7_IRQHandler() {
	HAL_TIM_IRQHandler(&htim7);
}

static inline void LD2_Set(bool state) {
	HAL_GPIO_WritePin(ld2_GPIO_Port, ld2_Pin,
			state ? GPIO_PIN_SET : GPIO_PIN_RESET);
}

static std::vector<SignalHead*> AllSignalHeads() {
	std::vector<SignalHead*> heads;
	for (std::vector<SignalHead>* vec : {COL_A_HEADS, COL_B_HEADS, COL_C_HEADS}) {
		for (SignalHead& head : *vec) {
			heads.push_back(&head);
		}
	}
	return heads;
}

static std::vector<std::pair<SignalHead_Aspect, SignalHead_Aspect>> SEARCHLIGHT_ASPECTS {
	std::make_pair(SignalHead_Aspect::Green, SignalHead_Aspect::Red),
	std::make_pair(SignalHead_Aspect::Red, SignalHead_Aspect::Red),
	std::make_pair(SignalHead_Aspect::Amber, SignalHead_Aspect::Red),
	std::make_pair(SignalHead_Aspect::Amber, SignalHead_Aspect::Amber),
	std::make_pair(SignalHead_Aspect::Red, SignalHead_Aspect::Green),
	std::make_pair(SignalHead_Aspect::Red, SignalHead_Aspect::Amber),
};

static std::vector<std::pair<SignalHead_Aspect, SignalHead_Aspect>> CPL_ASPECTS {
	std::make_pair(SignalHead_Aspect::Red, SignalHead_Aspect::None),
	std::make_pair(SignalHead_Aspect::None, SignalHead_Aspect::Lunar),
	std::make_pair(SignalHead_Aspect::None, SignalHead_Aspect::Lunar_Upper),
	std::make_pair(SignalHead_Aspect::Amber, SignalHead_Aspect::None),
	std::make_pair(SignalHead_Aspect::Amber, SignalHead_Aspect::Lower),
	std::make_pair(SignalHead_Aspect::Amber, SignalHead_Aspect::Upper),
	std::make_pair(SignalHead_Aspect::Green, SignalHead_Aspect::None),
	std::make_pair(SignalHead_Aspect::Green, SignalHead_Aspect::Lower),
	std::make_pair(SignalHead_Aspect::Green, SignalHead_Aspect::Upper),
};

static std::vector<SignalMast*> SIGNAL_MASTS {
	new SingleOutputMast(&(*COL_A_HEADS)[0]),
	new SingleOutputMast(&(*COL_A_HEADS)[1]),
	new SingleOutputMast(&(*COL_A_HEADS)[2]),
	new SingleOutputMast(&(*COL_B_HEADS)[0]),
	new DoubleOutputMast(&(*COL_B_HEADS)[1], &(*COL_B_HEADS)[2], &SEARCHLIGHT_ASPECTS),
	new DoubleOutputMast(&(*COL_C_HEADS)[0], &(*COL_C_HEADS)[1], &CPL_ASPECTS),
};

void RotateAspects() {
	for (SignalMast* mast : SIGNAL_MASTS) {
		mast->rotate_aspect();
	}
}

extern "C" void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef* htim) {
	if (htim->Instance == TIM6) {
		if (NUM_PERIODS_SINCE_LAST_ASPECT_ROTATE++ > ASPECT_ROTATE_PERIODS) {
			NUM_PERIODS_SINCE_LAST_ASPECT_ROTATE = 0;
			RotateAspects();
		}
	} else if (htim->Instance == TIM7) {
		// Called at 1kHz
		for (SignalHead* head : *ALL_HEADS) {
			head->compute_animation_state();
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

	for (SignalHead* head : AllSignalHeads()) {
		head->init();
		ALL_HEADS->push_back(head);
	}

	User_Btn_Timer_Init();
	AnimationStepTimerInit();

	while (1) {
		for (SignalHead* head : *ALL_HEADS) {
			head->compute_and_update_pwm_state();
		}
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
	htim7.Init.Prescaler = 47999; // 48MHz / 48000 = 1kHz
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
	__HAL_RCC_GPIOA_CLK_ENABLE();
	__HAL_RCC_GPIOB_CLK_ENABLE();
	__HAL_RCC_GPIOC_CLK_ENABLE();
	__HAL_RCC_GPIOF_CLK_ENABLE();


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

