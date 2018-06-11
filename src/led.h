#ifndef LED_H_
#define LED_H_

#include "stm32f0xx_hal.h"
#include <vector>
#include <map>
#include <algorithm>
#include <memory>
#include <functional>

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
		update_state();
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
	bool is_on() {
		return on_;
	}

private:
	/*global*/
	GPIO_TypeDef* gpio_port_;
	const uint16_t gpio_pin_;
	bool on_ = false;
};

class AnimationFunction {
public:
	virtual ~AnimationFunction() = default;
	virtual float get_value(float msec_elapsed,
			float current_brightness_frac) const = 0;
	virtual bool is_done(float elapsed_msec) const = 0;
	virtual float get_total_duration() const = 0;
};

class SquareFunction: public AnimationFunction {
public:
	SquareFunction(bool is_onward) :
			is_onward_(is_onward) {
	}

	float get_value(float msec_elapsed, float current_brightness_frac) const
			override {
		if (is_onward_ && msec_elapsed > on_delay_msec_) {
			return 1;
		} else {
			return 0;
		}
	}

	bool is_done(float elapsed_msec) const override {
		return is_onward_ ? (elapsed_msec > on_delay_msec_): true;
	}

	float get_total_duration() const override { return on_delay_msec_; }

private:
	bool is_onward_ = true;
	float on_delay_msec_ = 200;
};

class FadeFn: public AnimationFunction {
public:
	FadeFn(float from_frac, float to_frac, float additional_delay_msec = 0, float duration_msec = 300) :
			from_frac_(from_frac), to_frac_(to_frac),
			delay_msec_(additional_delay_msec),
			total_duration_msec_(duration_msec + additional_delay_msec) {
		onward_ = to_frac > from_frac;
		if (onward_ && additional_delay_msec == 0) {  // hack: pls fix
			float special_on_delay = total_duration_msec_ * (0.66f);
			delay_msec_ += special_on_delay;
			total_duration_msec_ += special_on_delay;
		}
	}

	float get_value(float msec_elapsed, float current_brightness_frac) const
			override {
		msec_elapsed = std::max(0.0f, msec_elapsed - delay_msec_);

		float frac = std::min(msec_elapsed / total_duration_msec_, 1.0f);
		if (onward_) {
			// Limit onward value to to_frac_
			if (frac > to_frac_) { frac = to_frac_; }
			// If already "more" on, use current brightness.
			return std::max(frac, current_brightness_frac);
		} else {
			frac = 1.0f - frac;
			// Limit offward value to to_frac
			if (frac < to_frac_) { frac = to_frac_; }
			// If already "more" off, use current brightness.
			return std::min(frac, current_brightness_frac);
		}
	}

	bool is_done(float elapsed_msec) const override {
		return elapsed_msec > total_duration_msec_;
	}

	float get_total_duration() const override { return total_duration_msec_; }

private:
	bool onward_ = true;
	float from_frac_, to_frac_;
	float delay_msec_ = 0;
	float total_duration_msec_ = 300;
};

SquareFunction* NewLEDOnAnimation() {
	return new SquareFunction(/*is_onward=*/true);
}
SquareFunction* NewLEDOffAnimation() {
	return new SquareFunction(/*is_onward=*/false);
}

FadeFn* NewFadeOffAnimation(float delay_ms = 0, float duration_ms = 300) {
	return new FadeFn(1.0, 0.0, delay_ms, duration_ms);
}
FadeFn* NewFadeOnAnimation(float delay_ms = 0, float duration_ms = 300) {
	return new FadeFn(0.0, 1.0, delay_ms, duration_ms);
}


class DimmableLED: public LED {
public:
	DimmableLED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) :
			LED(gpio_port, gpio_pin) {
	}

	// 0 to 255.
	void set_brightness_level(unsigned short brightness) {
		brightness_ = brightness;
	}

	unsigned short get_brightness_level() {
		return brightness_;
	}

	// Call this at PWM frequency.
	void compute_pwm_state() {
		if (++counter_ >= 255) {
			counter_ = 0;
		}
		if (counter_ < brightness_) {
			LED::turnOn();
		} else {
			LED::turnOff();
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
	}

	void do_animation(std::vector<AnimationFunction*> animations) {
		for (auto& fn : animations) {
			animation_queue_.push_back(std::unique_ptr<AnimationFunction>(fn));
		}
	}

	void compute_animation_state() {
		if (animation_queue_.empty()) {
			return;
		}
		if (current_animation_start_tick_ == 0) {
			// unstarted animation
			current_animation_start_tick_ = HAL_GetTick();
		}

		uint32_t msec_elapsed = HAL_GetTick() - current_animation_start_tick_;
		AnimationFunction* current_animation = animation_queue_[0].get();
		float old_brightness_frac = (float) get_brightness_level() / 255.0;
		float animation_fraction = current_animation->get_value(msec_elapsed,
				old_brightness_frac);
		set_brightness_level(animation_fraction * 255.0);

		if (current_animation->is_done(msec_elapsed)) {
			animation_queue_.erase(animation_queue_.begin());
			current_animation_start_tick_ = 0;
		}
	}

private:
	std::vector<std::unique_ptr<AnimationFunction>> animation_queue_;
	uint32_t current_animation_start_tick_ = 0;
};

enum class SignalHead_Aspect {
	Red, Amber, Green,
};

class SignalHead {
public:
	SignalHead(AnimatedLED* green, AnimatedLED* amber, AnimatedLED* red,
			SignalHead_Aspect aspect = SignalHead_Aspect::Red,
			LampStyle style = LampStyle::SEARCHLIGHT) :
			green_(green), amber_(amber), red_(red), style_(style) {
	}

	void rotate_aspect() {
		static std::map<SignalHead_Aspect, SignalHead_Aspect> map = {
				std::make_pair(SignalHead_Aspect::Green,
						SignalHead_Aspect::Red), std::make_pair(
						SignalHead_Aspect::Red, SignalHead_Aspect::Amber),
				std::make_pair(SignalHead_Aspect::Amber,
						SignalHead_Aspect::Green), };
		set_aspect(map.at(aspect_));
	}

	void set_style(LampStyle style) {
		style_ = style;
	}

	void compute_and_update_pwm_state() {
		for (AnimatedLED* led : { green_, amber_, red_ }) {
			led->compute_pwm_state();
			led->update_state();
		}
	}

	void compute_animation_state() {
		for (AnimatedLED* led : { green_, amber_, red_ }) {
			led->compute_animation_state();
		}
	}

	void init() {
		for (AnimatedLED* led : { green_, amber_, red_ }) {
			led->init();
		}
		set_aspect(SignalHead_Aspect::Red);
	}

private:
	void set_aspect(SignalHead_Aspect new_aspect) {
		std::function<AnimationFunction*()> simple_on, simple_off;
		if (style_ == LampStyle::LED) {
			simple_on = []() { return NewLEDOnAnimation(); };
			simple_off = []() { return NewLEDOffAnimation(); };
		} else {
			simple_on = [](){ return NewFadeOnAnimation(); };
			simple_off = [](){ return NewFadeOffAnimation(); };
		}

		if (aspect_ == new_aspect) { return; }

		if (style_ != LampStyle::SEARCHLIGHT) {
			if (new_aspect == SignalHead_Aspect::Green) {
				green_->do_animation({simple_on()});
				amber_->do_animation({simple_off()});
				red_->do_animation({simple_off()});
			} else if (new_aspect == SignalHead_Aspect::Amber) {
				green_->do_animation({simple_off()});
				amber_->do_animation({simple_on()});
				red_->do_animation({simple_off()});
			} else {
				green_->do_animation({simple_off()});
				amber_->do_animation({simple_off()});
				red_->do_animation({simple_on()});
			}
		} else {
			if (aspect_ != SignalHead_Aspect::Red
					&& new_aspect != SignalHead_Aspect::Red) {
				// we need to bounce via red if going from one non-red to another.
				AnimatedLED *to_turn_off, *to_turn_on;

				if (new_aspect == SignalHead_Aspect::Green) {
					to_turn_off = amber_;
					to_turn_on = green_;
				} else {
					to_turn_off = green_;
					to_turn_on = amber_;
				}
				to_turn_off->do_animation({simple_off()});

				// TODO: simple_on/off?
				auto* red_in_fn = new FadeFn(/*from*/0.0, /*to*/1.0, /*delay*/0, /*dur*/200);
				red_->do_animation({red_in_fn});
				auto* red_out_fn = new FadeFn(/*from*/1.0, /*to*/0.0, /*delay*/0, /*dur*/200);
				red_->do_animation({red_out_fn});

				auto* final_on_fn = new FadeFn(/*from*/0.0, /*to*/1.0, /*delay*/400, /*dur*/300);
				to_turn_on->do_animation({final_on_fn});
			} else if (new_aspect == SignalHead_Aspect::Red) {
				green_->do_animation({simple_off()});
				amber_->do_animation({simple_off()});
				AnimationFunction* initial_red_in = NewFadeOnAnimation();
				red_->do_animation({
					initial_red_in,
					NewFadeOffAnimation(/*delay_ms=*/0,
							            /*duration_ms=*/300),
					NewFadeOnAnimation(/*delay_ms=*/0,
										/*duration_ms=*/150)});
			} else {
				// Just do a normal incandescent transition.
				red_->do_animation({simple_off()});
				if (new_aspect == SignalHead_Aspect::Green) {
					green_->do_animation({simple_on()});
					amber_->do_animation({simple_off()});
				} else {
					green_->do_animation({simple_off()});
					amber_->do_animation({simple_on()});
				}
			}
		}

		aspect_ = new_aspect;
	}

	AnimatedLED* green_;
	AnimatedLED* amber_;
	AnimatedLED* red_;
	SignalHead_Aspect aspect_;
	LampStyle style_;
};

#endif /* LED_H_ */
