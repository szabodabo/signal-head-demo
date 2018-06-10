/*
 * led.h
 *
 *  Created on: Jun 10, 2018
 *      Author: dakota
 */

#ifndef LED_H_
#define LED_H_

#include "stm32f0xx_hal.h"
#include <vector>
#include <map>
#include <algorithm>

enum class LampStyle
	: unsigned short {
		LED = 1, INCANDESCENT = 2, SEARCHLIGHT = 3,
};

class LED {
public:
	LED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) :
			gpio_port_(gpio_port), gpio_pin_(gpio_pin) {
	}

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

	void turnOn() {
		on_ = true;
	}

	void turnOff() {
		on_ = false;
	}

	void update_state() {
		HAL_GPIO_WritePin(gpio_port_, gpio_pin_,
				on_ ? GPIO_PIN_SET : GPIO_PIN_RESET);
	}

protected:
	bool is_on() { return on_; }

private:
	/*global*/
	GPIO_TypeDef* gpio_port_;
	const uint16_t gpio_pin_;
	bool on_ = false;
};

class AnimationFunction {
public:
	virtual ~AnimationFunction() = default;
	virtual float get_value(float msec) const = 0;
};

class SquareFunction: public AnimationFunction {
public:
	SquareFunction(bool is_onward) :
			is_onward_(is_onward) {
	}

	float get_value(float msec) const override {
		return is_onward_ ? 1.0 : 0.0;
	}

private:
	bool is_onward_ = true;
};

class FadeFn: public AnimationFunction {
public:
	FadeFn(bool onward) :
			onward_(onward) {
	}
	float get_value(float msec) const override {
		// .5 second fade
		float frac = std::min(msec / 500, 1.0f);
		if (onward_) {
			return frac;
		} else {
			return 1.0 - frac;
		}
	}
private:
	bool onward_ = true;
};

static SquareFunction* LED_ON_ANIMATION = new SquareFunction(
/*is_onward=*/true);
static SquareFunction* LED_OFF_ANIMATION = new SquareFunction(
/*is_onward=*/false);

static FadeFn* FADE_ON_ANIMATION = new FadeFn(true);
static FadeFn* FADE_OFF_ANIMATION = new FadeFn(false);

class DimmableLED: public LED {
public:
	DimmableLED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) :
			LED(gpio_port, gpio_pin) {
	}

	// 0 to 255.
	void set_brightness_level(unsigned short brightness) {
		brightness_ = brightness;
	}

	// Call this at PWM frequency.
	void compute_pwm_state() {
		if (++counter_ >= 255) {
			counter_ = 0;
		}
		if (counter_ > brightness_) {
			LED::turnOff();
		} else {
			LED::turnOn();
		}
	}

private:
	unsigned short brightness_ = 255;
	unsigned short counter_ = 0;
};

class AnimatedLED: public DimmableLED {
public:
	AnimatedLED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, LampStyle style =
			LampStyle::INCANDESCENT) :
			DimmableLED(gpio_port, gpio_pin) {
		setStyle(style);
	}

	void animate_on() {
		if (is_on()) {
			return;
		}
		current_fn_ = on_fn_;
		animation_start_ = HAL_GetTick();
	}

	void animate_off() {
		if (!is_on()) {
			return;
		}
		current_fn_ = off_fn_;
		animation_start_ = HAL_GetTick();
	}

	void setStyle(LampStyle style) {
		switch (style) {
		case LampStyle::INCANDESCENT:
			on_fn_ = FADE_ON_ANIMATION;
			off_fn_ = FADE_OFF_ANIMATION;
			break;
		case LampStyle::LED:
		default:
			on_fn_ = LED_ON_ANIMATION;
			off_fn_ = LED_OFF_ANIMATION;
			break;
		}
	}

	void compute_animation_state() {
		if (!current_fn_ || animation_start_ == 0) {
			return;
		}
		uint32_t msec_elapsed = HAL_GetTick() - animation_start_;
		if (msec_elapsed > 1100) {
			current_fn_ = nullptr;
			return;
		}
		set_brightness_level(current_fn_->get_value(msec_elapsed) * 255.0);
	}

private:
	AnimationFunction *on_fn_, *off_fn_;
	AnimationFunction* current_fn_ = nullptr;
	uint32_t animation_start_ = 0;
};

enum class SignalHead_Aspect {
	Red, Amber, Green,
};

class SignalHead {
public:
	SignalHead(AnimatedLED& green, AnimatedLED& amber, AnimatedLED& red,
			SignalHead_Aspect aspect = SignalHead_Aspect::Red) :
			green_(green), amber_(amber), red_(red) {
		set_aspect(aspect);
	}

	void set_aspect(SignalHead_Aspect aspect) {
		aspect_ = aspect;

		switch (aspect_) {
		case SignalHead_Aspect::Green:
			green_.animate_on();
			red_.animate_off();
			amber_.animate_off();
			break;
		case SignalHead_Aspect::Amber:
			amber_.animate_on();
			red_.animate_off();
			green_.animate_off();
			break;
		case SignalHead_Aspect::Red:
		default:
			red_.animate_on();
			green_.animate_off();
			amber_.animate_off();
			break;
		}
	}

	void rotate_aspect() {
		static std::map<SignalHead_Aspect, SignalHead_Aspect> map = {
			std::make_pair(SignalHead_Aspect::Green, SignalHead_Aspect::Red),
			std::make_pair(SignalHead_Aspect::Red, SignalHead_Aspect::Amber),
			std::make_pair(SignalHead_Aspect::Amber, SignalHead_Aspect::Green),
		};
		set_aspect(map.at(aspect_));
	}

private:
	AnimatedLED& green_;
	AnimatedLED& amber_;
	AnimatedLED& red_;
	SignalHead_Aspect aspect_;
};

#endif /* LED_H_ */
