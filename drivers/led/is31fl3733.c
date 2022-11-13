/*
 * Copyright (c) 2022 Javier Alvarez (javier.alvarez@allthingsembedded.net)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT is31fl3733_leds

/**
 * @file
 * @brief IS31FL3733 Dots matrix led driver
 */

#include <drivers/led.h>
#include <drivers/gpio.h>
#include <drivers/i2c.h>
#include <device.h>
#include <zephyr.h>

#include <logging/log.h>

LOG_MODULE_REGISTER(is31fl3733_leds, CONFIG_LED_LOG_LEVEL);

#define COMMAND_REGISTER_REG 0xFD
#define COMMAND_REGISTER_WRITE_LOCK_REG 0xFE
#define COMMAND_REGISTER_LOCK_PASS 0xC5

#define PAGE_CONTROL 0x00
#define PAGE_PWM 0x01
#define PAGE_FUNCTION 0x03

#define RESET_REG 0x11

struct is31fl3733_child_config {
	int channel;
};

/** IS31FL3733 configuration (DT). */
struct is31fl3733_config {
	/** I2C controller device name. */
	const char *const i2c_name;
	/** I2C chip address. */
	const uint8_t i2c_address;
	const struct gpio_dt_spec gpio_reset_spec;

	const struct is31fl3733_child_config *const leds;
	const int num_leds;
};

/** IS31FL3733 data . */
struct is31fl3733_data {
	/** I2C controller device. */
	const struct device *const i2c_dev;
};

static int is31fl3733_write_reg(const struct device *dev, uint8_t reg_addr, uint8_t value)
{
	const struct is31fl3733_config *const config = dev->config;
	const struct is31fl3733_data *const data = dev->data;
	return i2c_reg_write_byte(data->i2c_dev, config->i2c_address, reg_addr, value);
}

static int is31fl3733_read_reg(const struct device *dev, uint8_t reg_addr, uint8_t *value)
{
	const struct is31fl3733_config *const config = dev->config;
	const struct is31fl3733_data *const data = dev->data;
	return i2c_reg_read_byte(data->i2c_dev, config->i2c_address, reg_addr, value);
}

static int is31fl3733_set_page(const struct device *dev, uint32_t page)
{
	int retval = is31fl3733_write_reg(dev, COMMAND_REGISTER_WRITE_LOCK_REG,
					  COMMAND_REGISTER_LOCK_PASS);
	if (retval < 0) {
		LOG_ERR("Error setting register COMMAND_REGISTER_WRITE_LOCK_REG");
		return retval;
	}

	is31fl3733_write_reg(dev, COMMAND_REGISTER_REG, page);
	if (retval < 0) {
		LOG_ERR("Error setting register COMMAND_REGISTER_REG");
		return retval;
	}

	return 0;
}

static int is31fl3733_set_channel(const struct device *const dev, const uint32_t channel,
				  const bool on)
{
	int retval = is31fl3733_set_page(dev, PAGE_PWM);
	if (retval < 0) {
		LOG_ERR("Error setting page");
		return retval;
	}

	retval = is31fl3733_write_reg(dev, channel, on ? 0xff : 0);
	if (retval < 0) {
		LOG_ERR("Error reading channel %d", channel);
		return retval;
	}

	return 0;
}

static int is31fl3733_reset(const struct device *const dev)
{
	uint8_t val;
	int retval = is31fl3733_set_page(dev, PAGE_FUNCTION);
	if (retval < 0) {
		LOG_ERR("Error setting page for reset");
		return retval;
	}

	retval = is31fl3733_read_reg(dev, RESET_REG, &val);
	if (retval < 0) {
		LOG_ERR("Error reading reset register");
		return retval;
	}
	return 0;
}

static int is31fl3733_configure(const struct device *const dev)
{
	int retval = is31fl3733_set_page(dev, 3);
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 2, (2 << 5) | (0 << 1));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 2, (2 << 5));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 3, (2 << 5) | (3 << 1));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 4, (0 << 4));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 6, (2 << 5) | (0 << 1));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 7, (2 << 5) | (2 << 1));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 8, (0 << 4));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 0xA, (1 << 5) | (0 << 1));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 0xB, (1 << 5) | (1 << 1));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 0xC, (0 << 4));
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 0, 1);
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 0, 3);
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 0xE, 0);
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_write_reg(dev, 1, 128);
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_set_page(dev, 0);
	if (retval < 0) {
		return retval;
	}

	for (uint32_t i = 0; i < 16 * 12; i++) {
		retval = is31fl3733_write_reg(dev, i, 0xff);
		if (retval < 0) {
			return retval;
		}
	}
	return 0;
}

static int is31fl3733_set_brightness(const struct device *dev, uint32_t led, uint8_t value)
{
	return is31fl3733_set_channel(dev, led, value > 0);
}

static int is31fl3733_led_on(const struct device *dev, uint32_t led)
{
	return 0;
}

static int is31fl3733_led_off(const struct device *dev, uint32_t led)
{
	return 0;
}

static int is31fl3733_leds_init(const struct device *dev)
{
	const struct is31fl3733_config *config = dev->config;

	int retval = gpio_pin_configure_dt(&config->gpio_reset_spec, GPIO_OUTPUT_HIGH);
	if (retval < 0) {
		LOG_ERR("Init error: could not claim gpio");
		return retval;
	}

	retval = is31fl3733_reset(dev);
	if (retval < 0) {
		LOG_ERR("Init error: unable to reset device");
		return retval;
	}

	retval = is31fl3733_configure(dev);
	if (retval < 0) {
		LOG_ERR("Init error: unable to configure device");
		return retval;
	}

	return 0;
}

static const struct led_driver_api is31fl3733_leds_api = {
	.on = is31fl3733_led_on,
	.off = is31fl3733_led_off,
	.set_brightness = is31fl3733_set_brightness,
};

#define IS31FL3733_DEFINE_CHILD_LED(node_id)                                                       \
	{                                                                                          \
		.channel = DT_PROP(node_id, channel),                                              \
	},

#define IS31FL3733_DEFINE_CONFIG(index)                                                            \
	static const struct is31fl3733_child_config is31fl3733_child_config_##index[] = {          \
		DT_INST_FOREACH_CHILD(index, IS31FL3733_DEFINE_CHILD_LED)                          \
	};                                                                                         \
                                                                                                   \
	static const struct is31fl3733_config is31fl3733_config_##index = {                        \
		.i2c_name = DT_INST_BUS_LABEL(index),                                              \
		.i2c_address = DT_INST_REG_ADDR(index),                                            \
		.gpio_reset_spec = GPIO_DT_SPEC_INST_GET(index, power_gpios),                      \
		.leds = is31fl3733_child_config_##index,                                           \
		.num_leds = ARRAY_SIZE(is31fl3733_child_config_##index),                           \
	}

#define IS31FL3733_DEFINE_DATA(index)                                                              \
	static struct is31fl3733_data is31fl3733_data_##index = {                                  \
		.i2c_dev = DEVICE_DT_GET(DT_INST_BUS(index)),                                      \
	}

#define IS31FL3733_DEVICE(i)                                                                       \
	IS31FL3733_DEFINE_CONFIG(i);                                                               \
	IS31FL3733_DEFINE_DATA(i);                                                                 \
	DEVICE_DT_INST_DEFINE(i, &is31fl3733_leds_init, NULL, &is31fl3733_data_##i,                \
			      &is31fl3733_config_##i, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,       \
			      &is31fl3733_leds_api);

DT_INST_FOREACH_STATUS_OKAY(IS31FL3733_DEVICE)
