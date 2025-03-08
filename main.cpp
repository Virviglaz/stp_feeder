#include "stm8s_adc.h"
#include "stm8s_gpio.h"
#include "stm8s_tim2.h"
#include "stm8s_delay.h"

#define STP_CLK			GPIOD, PIN_3 /* TIM2-CH2 */
#define STP_DIR			GPIOD, PIN_2
#define STP_ENA			GPIOC, PIN_5
#define SW_LEFT			GPIOC, PIN_6
#define SW_RIGHT		GPIOC, PIN_7
#define STP_CLK_POL_INVERT	true
#define STP_ACC			1
#define TIM_BASE		12000
#define SPEED_THRES		4

#define MAX(x, y)		(x) > (y) ? (x) : (y)
#define MIN(x, y)		(x) < (y) ? (x) : (y)

enum dir_e { NEUTRAL, SWITCH_LEFT, SWITCH_RIGHT };
enum state_e {
	STOP,
	ACC_LEFT,
	ACC_RIGHT,
	RUN_LEFT,
	RUN_RIGHT,
};

static inline void init()
{
	gpio_init(STP_CLK, OUTPUT);
	gpio_init(STP_DIR, OUTPUT);
	gpio_init(STP_ENA, OUTPUT);
	gpio_pin_switch(STP_ENA, STP_CLK_POL_INVERT);
	gpio_init(SW_LEFT, INPUT);
	gpio_init(SW_RIGHT, INPUT);
	gpio_pullup(SW_LEFT, true);
	gpio_pullup(SW_RIGHT, true);
	adc_init(ADC1_CHANNEL_5, ADC1_PRESSEL_FCPU_D8);
	tim2_init(TIM2_HSI_DIV_32, TIM_BASE);
	tim2_pwm_init(TIM2_PWM2, 10);
	delays_init();
}

static void stepper_enable(bool state)
{
	gpio_pin_switch(STP_ENA, state != STP_CLK_POL_INVERT);
}

static enum dir_e get_switch()
{
	if (!gpio_read(SW_LEFT))
		return SWITCH_LEFT;
	if (!gpio_read(SW_RIGHT))
		return SWITCH_RIGHT;
	return NEUTRAL;
}

static uint16_t get_speed()
{
	if (get_switch() != NEUTRAL)
		return adc_read(ADC1_CHANNEL_5);
	return 0;
}

static inline void set_speed(uint16_t speed)
{
	tim2_disable();

	if (!speed)
		return;

	if (speed > TIM_BASE)
		speed = TIM_BASE;

	tim2_set_period(TIM_BASE / speed);
	tim2_enable();
}

static inline void set_dir(enum dir_e dir)
{
	gpio_pin_switch(STP_DIR, dir == SWITCH_RIGHT);
}

static void state_machine(enum state_e& state, uint16_t& speed, bool& change)
{
	enum dir_e switch_pos = get_switch();
	uint16_t des_speed = get_speed();

	switch(state) {
	case STOP:
		if (change) {
			change = false;
			speed = 0;
			stepper_enable(false);
		}

		if (switch_pos == SWITCH_LEFT) {
			change = true;
			state = ACC_LEFT;
		}

		if (switch_pos == SWITCH_RIGHT) {
			change = true;
			state = ACC_RIGHT;
		}
		break;
	case RUN_LEFT:
		if (change) {
			change = false;
		}

		if (switch_pos == NEUTRAL || switch_pos == SWITCH_RIGHT) {
			change = true;
			state = STOP;
		}

		speed = des_speed;
		break;
	case RUN_RIGHT:
		if (change) {
			change = false;
		}

		if (switch_pos == NEUTRAL || switch_pos == SWITCH_LEFT) {
			change = true;
			state = STOP;
		}

		speed = des_speed;
		break;
	case ACC_LEFT:
		if (change) {
			change = false;
			stepper_enable(true);
			set_dir(SWITCH_LEFT);
			speed = 0;
		}

		speed += STP_ACC;
		if (speed >= des_speed) {
			change = true;
			state = RUN_LEFT;
		}
		break;
	case ACC_RIGHT:
		if (change) {
			change = false;
			stepper_enable(true);
			set_dir(SWITCH_RIGHT);
			speed = 0;
		}

		speed += STP_ACC;
		if (speed >= des_speed) {
			change = true;
			state = RUN_RIGHT;
		}
		break;
	}
}

int main()
{
	init();
	enum state_e state = STOP;
	uint16_t speed = 0;
	uint16_t prev_speed = 0;
	bool change = false;

	while (1) {
		state_machine(state, speed, change);
		if (prev_speed != speed) {
			set_speed(speed);
			prev_speed = speed;
		}
		delay_ms(1);
	}
}
