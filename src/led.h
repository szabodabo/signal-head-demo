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
		GPIO_InitStructure.Mode = GPIO_MODE_OUTPUT_OD;
		GPIO_InitStructure.Pull = GPIO_NOPULL;
		GPIO_InitStructure.Speed = GPIO_SPEED_FREQ_HIGH;
		update_state();  // init off
		HAL_GPIO_Init(gpio_port_, &GPIO_InitStructure);
	}

	void turnOn() {
		on_ = true;
	}

	void turnOff() {
		on_ = false;
	}

	inline void update_state() {
		HAL_GPIO_WritePin(gpio_port_, gpio_pin_,
				on_ ? GPIO_PIN_RESET : GPIO_PIN_SET);
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
	FadeFn(float from_frac, float to_frac, float delay_msec = 0, float duration_msec = 300) :
			from_frac_(from_frac), to_frac_(to_frac),
			delay_msec_(delay_msec),
			total_duration_msec_(duration_msec + delay_msec) {
		onward_ = to_frac > from_frac;
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

#define DEFAULT_FADE_IN_DELAY_MSEC 400
#define DEFAULT_FADE_DURATION_MSEC 300

FadeFn* NewFadeOffAnimation(float delay_ms = 0,
		 	 	 	 	 	float duration_ms = DEFAULT_FADE_DURATION_MSEC) {
	return new FadeFn(1.0, 0.0, delay_ms, duration_ms);
}
FadeFn* NewFadeOnAnimation(float delay_ms = DEFAULT_FADE_IN_DELAY_MSEC,
		                   float duration_ms = DEFAULT_FADE_DURATION_MSEC) {
	return new FadeFn(0.0, 1.0, delay_ms, duration_ms);
}


class DimmableLED: public LED {
public:
	DimmableLED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin) :
			LED(gpio_port, gpio_pin) {
	}

	// 0 to 1.
	void set_brightness_level(float brightness_frac) {
		brightness_ = brightness_frac * (float) counter_total_;
	}

	// 0 to 1.
	float get_brightness_level() {
		return brightness_ / (float) counter_total_;
	}

	// Call this at PWM frequency.
	inline void compute_pwm_state() {
		if (++counter_ >= counter_total_) {
			counter_ = 0;
		}
		if (counter_ < brightness_) {
			LED::turnOn();
		} else {
			LED::turnOff();
		}
	}

private:
	unsigned short brightness_ = 0;
	unsigned short counter_ = 0;
	static const unsigned short counter_total_ = 300;
};

class AnimatedLED: public DimmableLED {
public:
	AnimatedLED(GPIO_TypeDef* gpio_port, uint16_t gpio_pin, LampStyle style =
			LampStyle::INCANDESCENT) :
			DimmableLED(gpio_port, gpio_pin) {
	}

	void do_animation(std::vector<AnimationFunction*> animations) {
		//__disable_irq();
		for (auto& fn : animations) {
			animation_queue_.push_back(std::unique_ptr<AnimationFunction>(fn));
		}
		//__enable_irq();
	}

	void compute_animation_state() {
		//__disable_irq();
		if (!animation_queue_.empty()) {
			if (current_animation_start_tick_ == 0) {
				// unstarted animation
				current_animation_start_tick_ = HAL_GetTick();
			}


			uint32_t msec_elapsed = HAL_GetTick() - current_animation_start_tick_;
			AnimationFunction* current_animation = animation_queue_[0].get();
			float animation_fraction = current_animation->get_value(msec_elapsed,
					get_brightness_level());
			set_brightness_level(animation_fraction);

			if (current_animation->is_done(msec_elapsed)) {
				animation_queue_.erase(animation_queue_.begin());
				current_animation_start_tick_ = 0;
			}
		}
		//__enable_irq();
	}

private:
	std::vector<std::unique_ptr<AnimationFunction>> animation_queue_;
	uint32_t current_animation_start_tick_ = 0;
};

enum class SignalHead_Aspect {
	Red, Amber, Green,
	None,
	Lunar, Upper, Lunar_Upper, Lower,
};

class SignalHead {
public:
	SignalHead(AnimatedLED* red, AnimatedLED* amber, AnimatedLED* green,
			LampStyle style = LampStyle::SEARCHLIGHT) :
			green_(green), amber_(amber), red_(red), style_(style) {
	}

	void set_style(LampStyle style) {
		style_ = style;
	}

	inline void compute_and_update_pwm_state() {
		for (AnimatedLED* led : { green_, amber_, red_ }) {
			led->compute_pwm_state();
			led->update_state();
		}
	}

	inline void compute_animation_state() {
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

	SignalHead_Aspect get_aspect() const { return aspect_; }

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

		if (new_aspect == SignalHead_Aspect::None) {
			for (auto* led : {green_, amber_, red_}) {
				led->do_animation({simple_off()});
			}
			return;
		}

		if (style_ != LampStyle::SEARCHLIGHT) {
			std::vector<AnimatedLED*> on_lamps = {red_};
			if (new_aspect == SignalHead_Aspect::Green) {
				on_lamps = {green_};
			} else if (new_aspect == SignalHead_Aspect::Amber) {
				on_lamps = {amber_};
			} else if (new_aspect == SignalHead_Aspect::None) {
				on_lamps.clear();
			} else if (new_aspect == SignalHead_Aspect::Lunar) {
				on_lamps = {amber_};
			} else if (new_aspect == SignalHead_Aspect::Lunar_Upper) {
				on_lamps = {amber_, red_};
			} else if (new_aspect == SignalHead_Aspect::Lower) {
				on_lamps = {green_};
			} else if (new_aspect == SignalHead_Aspect::Upper) {
				on_lamps = {red_};
			}
			for (auto* led : {green_, amber_, red_}) {
				if (std::find(on_lamps.begin(), on_lamps.end(), led) != on_lamps.end()) {
					led->do_animation({simple_on()});
				} else {
					led->do_animation({simple_off()});
				}
			}
		} else {
			if (aspect_ != SignalHead_Aspect::Red
					&& new_aspect != SignalHead_Aspect::Red) {
				// we need to bounce via red if going from one non-red to another.
				AnimatedLED *to_turn_off, *to_turn_on;

				static float bounce_duration_msec = 100;

				if (new_aspect == SignalHead_Aspect::Green) {
					to_turn_off = amber_;
					to_turn_on = green_;
				} else {
					to_turn_off = green_;
					to_turn_on = amber_;
				}
				to_turn_off->do_animation({simple_off()});

				auto* red_in_fn = NewFadeOnAnimation(
						/*delay_ms*/ DEFAULT_FADE_IN_DELAY_MSEC,
						/*duration*/bounce_duration_msec);
				red_->do_animation({red_in_fn});
				auto* red_out_fn = NewFadeOffAnimation(
						/*delay_ms*/0,
						/*duration*/bounce_duration_msec);
				red_->do_animation({red_out_fn});

				float final_delay_msec =
						red_in_fn->get_total_duration()
						+ red_out_fn->get_total_duration()
						+ bounce_duration_msec + 75;

				auto* final_on_fn = new FadeFn(/*from*/0.0, /*to*/1.0,
											   final_delay_msec, /*dur*/DEFAULT_FADE_DURATION_MSEC);
				to_turn_on->do_animation({final_on_fn});
			} else if (new_aspect == SignalHead_Aspect::Red) {
				green_->do_animation({simple_off()});
				amber_->do_animation({simple_off()});
				AnimationFunction* initial_red_in = new FadeFn(
						/*from*/0, /*to*/0.8,
						/*delay*/DEFAULT_FADE_IN_DELAY_MSEC,
						/*dur*/400 * 0.65);
				static float first_dip_to_frac = 0.1;
				static float first_dip_dur_ms = 400*0.55;
				static float second_dip_to_frac = 0.4;
				static float second_dip_dur_ms = first_dip_dur_ms * 0.7;
				red_->do_animation({
					initial_red_in,
					new FadeFn(/*from*/0.8, first_dip_to_frac, /*delay*/0, first_dip_dur_ms*.5),
					new FadeFn(first_dip_to_frac, /*to*/0.9, /*delay*/0, first_dip_dur_ms),
					new FadeFn(/*from*/0.9, second_dip_to_frac, /*delay*/0, second_dip_dur_ms*.5),
					new FadeFn(second_dip_to_frac, /*to*/1.0, /*delay*/0, second_dip_dur_ms),
				});
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

private:
	AnimatedLED* green_;
	AnimatedLED* amber_;
	AnimatedLED* red_;
	SignalHead_Aspect aspect_;
	LampStyle style_;
};

class SignalMast {
public:
	virtual void rotate_aspect() = 0;
};

class SingleOutputMast : public SignalMast {
public:
	SingleOutputMast(SignalHead* head) : head_(head) {}

	void rotate_aspect() override {
		static std::map<SignalHead_Aspect, SignalHead_Aspect> map = {
			std::make_pair(SignalHead_Aspect::Green, SignalHead_Aspect::Red),
			std::make_pair(SignalHead_Aspect::Red, SignalHead_Aspect::Amber),
			std::make_pair(SignalHead_Aspect::Amber,SignalHead_Aspect::Green),
		};

		head_->set_aspect(map.at(head_->get_aspect()));
	}
private:
	SignalHead* head_;
};

class DoubleOutputMast : public SignalMast {
public:
	DoubleOutputMast(SignalHead* upper, SignalHead* lower,
		std::vector<std::pair<SignalHead_Aspect, SignalHead_Aspect>>* aspects)
 	 	 	 : upper_(upper), lower_(lower), aspects_(aspects) {
	}

	void rotate_aspect() override {
		++current_aspect_;
		if (current_aspect_ >= aspects_->size()) {
			current_aspect_ = 0;
		}
		set_current_aspect();
	}

private:
	void set_current_aspect() {
		auto& aspect_pair = (*aspects_)[current_aspect_];
		upper_->set_aspect(aspect_pair.first);
		lower_->set_aspect(aspect_pair.second);
	}

	SignalHead* upper_;
	SignalHead* lower_;
	uint8_t current_aspect_ = 0;
	std::vector<std::pair<SignalHead_Aspect, SignalHead_Aspect>>* aspects_;
};

#endif /* LED_H_ */
