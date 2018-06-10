/*
 * led.h
 *
 *  Created on: Jun 10, 2018
 *      Author: dakota
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f0xx_hal.h"

class LED {
public:
	LED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) :
			gpio_port_(gpio_port), gpio_pin_(gpio_pin) {}

	inline void init() {
		GPIO_InitTypeDef GPIO_InitStructure;

		// Configure pin in output push/pull mode
		GPIO_InitStructure.Pin = gpio_pin_;
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_PP;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_LOW;
		HAL_GPIO_Init(gpio_port_, &GPIO_InitStructure);

		// Start with led turned off
		turnOff();
	}

	inline void __attribute__((always_inline)) turnOn() {
		HAL_GPIO_WritePin(gpio_port_, gpio_pin_, GPIO_PIN_SET);
	}

	inline void __attribute__((always_inline)) turnOff() {
		HAL_GPIO_WritePin(gpio_port_, gpio_pin_, GPIO_PIN_RESET);

	}
private:
	/*global*/ GPIO_TypeDef* gpio_port_;
	const uint16_t gpio_pin_;
};


#endif /* LED_H_ */
