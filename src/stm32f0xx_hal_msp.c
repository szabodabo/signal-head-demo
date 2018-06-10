#include "stm32f0xx_hal.h"

extern void _Error_Handler(char *, int);

/**
  * Initializes the Global MSP.
  */
void HAL_MspInit(void) {
  __HAL_RCC_SYSCFG_CLK_ENABLE();

  /* System interrupt init*/
  /* SVC_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SVC_IRQn, 0, 0);
  /* PendSV_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(PendSV_IRQn, 0, 0);
  /* SysTick_IRQn interrupt configuration */
  HAL_NVIC_SetPriority(SysTick_IRQn, 0, 0);
}

void HAL_ADC_MspInit(ADC_HandleTypeDef* hadc) {
  GPIO_InitTypeDef GPIO_InitStruct;
  if(hadc->Instance==ADC1)
  {
    /* Peripheral clock enable */
    __HAL_RCC_ADC1_CLK_ENABLE();
  
    /**ADC GPIO Configuration    
    PA0     ------> ADC_IN0 
    */
    GPIO_InitStruct.Pin = GPIO_PIN_0;
    GPIO_InitStruct.Mode = GPIO_MODE_ANALOG;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);
  }
}

void HAL_ADC_MspDeInit(ADC_HandleTypeDef* hadc) {
  if (hadc->Instance==ADC1) {
    /* Peripheral clock disable */
    __HAL_RCC_ADC1_CLK_DISABLE();
  
    /**ADC GPIO Configuration    
    PA0     ------> ADC_IN0 
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_0);
  }
}

void HAL_TIM_Base_MspInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance==TIM1) {
    /* Peripheral clock enable */
    __HAL_RCC_TIM1_CLK_ENABLE();
  }

}

void HAL_TIM_Base_MspDeInit(TIM_HandleTypeDef* htim_base) {
  if (htim_base->Instance==TIM1) {
    /* Peripheral clock disable */
    __HAL_RCC_TIM1_CLK_DISABLE();
  }
}

