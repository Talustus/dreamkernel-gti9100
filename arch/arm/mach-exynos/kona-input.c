/*
 *  arch/arm/mach-exynos/p4-input.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/gpio.h>
#include <linux/delay.h>
#include <linux/platform_device.h>
#include <plat/gpio-cfg.h>
#include <plat/iic.h>
#include <linux/regulator/consumer.h>

#if defined(CONFIG_TOUCHSCREEN_SYNAPTICS_S7301)
#include <linux/synaptics_s7301.h>
static bool have_tsp_ldo;
static struct charger_callbacks *charger_callbacks;

void synaptics_ts_charger_infom(bool en)
{
	if (charger_callbacks && charger_callbacks->inform_charger)
		charger_callbacks->inform_charger(charger_callbacks, en);
}

static void synaptics_ts_register_callback(struct charger_callbacks *cb)
{
	charger_callbacks = cb;
	printk(KERN_DEBUG "[TSP] %s\n", __func__);
}

static int synaptics_ts_set_power(bool en)
{
	struct regulator *regulator;

	if (!have_tsp_ldo)
		return -1;
	printk(KERN_DEBUG "[TSP] %s(%d)\n", __func__, en);

	regulator = regulator_get(NULL, "tsp_3.3v");
	if (IS_ERR(regulator))
		return PTR_ERR(regulator);

	if (en) {
		s3c_gpio_cfgpin(GPIO_TSP_SDA_18V, S3C_GPIO_SFN(0x3));
		s3c_gpio_setpull(GPIO_TSP_SDA_18V, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(GPIO_TSP_SCL_18V, S3C_GPIO_SFN(0x3));
		s3c_gpio_setpull(GPIO_TSP_SCL_18V, S3C_GPIO_PULL_UP);
		s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_TSP_LDO_ON, 1);

		regulator_enable(regulator);

		s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_NONE);
		s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_SFN(0xf));
	} else {
		s3c_gpio_cfgpin(GPIO_TSP_SDA_18V, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TSP_SDA_18V, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_TSP_SDA_18V, 0);
		s3c_gpio_cfgpin(GPIO_TSP_SCL_18V, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TSP_SCL_18V, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_TSP_SCL_18V, 0);
		s3c_gpio_cfgpin(GPIO_TSP_INT, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TSP_INT, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_TSP_INT, 0);
		s3c_gpio_cfgpin(GPIO_TSP_LDO_ON, S3C_GPIO_OUTPUT);
		s3c_gpio_setpull(GPIO_TSP_LDO_ON, S3C_GPIO_PULL_NONE);
		gpio_set_value(GPIO_TSP_LDO_ON, 0);

		if (regulator_is_enabled(regulator))
			regulator_disable(regulator);
	}

	regulator_put(regulator);
	return 0;
}

static void synaptics_ts_reset(void)
{
	printk(KERN_DEBUG "[TSP] %s\n", __func__);
	synaptics_ts_set_power(false);
	msleep(100);
	synaptics_ts_set_power(true);
}

static struct synaptics_platform_data synaptics_ts_pdata = {
	.gpio_attn = GPIO_TSP_INT,
	.max_x = 1279,
	.max_y = 799,
	.max_pressure = 255,
	.max_width = 100,
	.x_line = 26,
	.y_line = 41,
	.swap_xy = true,
	.invert_x = false,
	.invert_y = true,
#if defined(CONFIG_SEC_TOUCHSCREEN_SURFACE_TOUCH)
	.palm_threshold = 28,
#endif
	.set_power = synaptics_ts_set_power,
	.hw_reset = synaptics_ts_reset,
	.register_cb = synaptics_ts_register_callback,
};

static struct i2c_board_info i2c_synaptics[] __initdata = {
	{
		I2C_BOARD_INFO(SYNAPTICS_TS_NAME,
			SYNAPTICS_TS_ADDR),
		.platform_data = &synaptics_ts_pdata,
	},
};
#endif	/* CONFIG_TOUCHSCREEN_SYNAPTICS_S7301 */
static u32 hw_rev;
void __init kona_tsp_init(u32 system_rev)
{
	int gpio = 0, irq = 0;
	hw_rev = system_rev;

	printk(KERN_DEBUG "[TSP] %s rev : %u\n",
		__func__, hw_rev);

	gpio = GPIO_TSP_LDO_ON;
	gpio_request(gpio, "TSP_LDO_ON");
	gpio_direction_output(gpio, 1);
	gpio_export(gpio, 0);

	have_tsp_ldo = true;

	gpio = GPIO_TSP_INT;
	gpio_request(gpio, "TSP_INT");
	s3c_gpio_cfgpin(gpio, S3C_GPIO_SFN(0xf));
	s3c_gpio_setpull(gpio, S3C_GPIO_PULL_UP);
	s5p_register_gpio_interrupt(gpio);
	irq = gpio_to_irq(gpio);

#ifdef CONFIG_S3C_DEV_I2C3
	s3c_i2c3_set_platdata(NULL);
	i2c_synaptics[0].irq = irq;
	i2c_register_board_info(3, i2c_synaptics,
		ARRAY_SIZE(i2c_synaptics));
#endif	/* CONFIG_S3C_DEV_I2C3 */

}

#if defined(CONFIG_KEYBOARD_GPIO)
#include <mach/sec_debug.h>
#include <linux/gpio_keys.h>
#define GPIO_KEYS(_code, _gpio, _active_low, _iswake, _hook)	\
{							\
	.code = _code,					\
	.gpio = _gpio,					\
	.active_low = _active_low,			\
	.type = EV_KEY,					\
	.wakeup = _iswake,				\
	.debounce_interval = 10,			\
	.isr_hook = _hook,				\
	.value = 1					\
}

struct gpio_keys_button kona_buttons[] = {
	GPIO_KEYS(KEY_VOLUMEUP, GPIO_VOL_UP,
		  1, 1, sec_debug_check_crash_key),
	GPIO_KEYS(KEY_VOLUMEDOWN, GPIO_VOL_DOWN,
		  1, 1, sec_debug_check_crash_key),
	GPIO_KEYS(KEY_POWER, GPIO_nPOWER,
		  1, 1, sec_debug_check_crash_key),
	GPIO_KEYS(KEY_HOMEPAGE, GPIO_OK_KEY_ANDROID,
		  1, 1, sec_debug_check_crash_key),
};

struct gpio_keys_platform_data kona_gpiokeys_platform_data = {
	kona_buttons,
	ARRAY_SIZE(kona_buttons),
};

static struct platform_device kona_keypad = {
	.name	= "gpio-keys",
	.dev	= {
		.platform_data = &kona_gpiokeys_platform_data,
	},
};
#endif
void __init kona_key_init(void)
{
#if defined(CONFIG_KEYBOARD_GPIO)
	platform_device_register(&kona_keypad);
#endif
}
