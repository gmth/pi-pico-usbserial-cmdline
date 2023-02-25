#include "alpha_power_modes.h"

#include "pico/stdlib.h"

#define TIME_MS_RESET_PIN_LOW       1000
#define TIME_MS_RECOVERY_PIN_LOW    ((TIME_MS_RESET_PIN_LOW) + 500)

static int g_pin_reset = -1;
static int g_pin_recovery = -1;

static int64_t on_cb_reset(long alarm_id, void* context)
{
    // pin back high
    gpio_set_mask(1ul << g_pin_reset);
    return 0;
}

static int64_t on_cb_recovery(long alarm_id, void* context)
{
    // pin back high
    gpio_set_mask(1ul << g_pin_recovery);
    return 0;
}

int apm_init(int pin_reset, int pin_recovery)
{
	gpio_init(pin_reset);
	gpio_init(pin_recovery);
	gpio_set_dir(pin_reset, GPIO_OUT);
	gpio_set_dir(pin_recovery, GPIO_OUT);

    g_pin_reset = pin_reset;
    g_pin_recovery = pin_recovery;
    
    gpio_set_mask(1ul << g_pin_recovery);
    gpio_set_mask(1ul << g_pin_reset);

    return 0;
}

int apm_do_reset(void)
{
    if (g_pin_reset < 0) {
        // invalid state, return early
        return -1;
    }

    gpio_clr_mask(1ul << g_pin_reset);
    add_alarm_in_ms(TIME_MS_RESET_PIN_LOW, on_cb_reset, NULL, false);

    return 0;
}

int apm_do_recovery(void)
{
    if ((g_pin_reset < 0) || (g_pin_recovery < 0)) {
        return -1;
    }

    // set recovery pin
    gpio_clr_mask(1ul << g_pin_recovery);

    apm_do_reset();
    add_alarm_in_ms(TIME_MS_RECOVERY_PIN_LOW, on_cb_recovery, NULL, false);

    return 0;
}