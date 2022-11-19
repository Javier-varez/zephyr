/*
 * Copyright (c) 2022 Javier Alvarez (javier.alvarez@allthingsembedded.net)
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT lumissil_is31fl3733

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

enum Mode {
	PWM = 0,
	AUTO_BREATH_1 = 1,
	AUTO_BREATH_2 = 2,
	AUTO_BREATH_3 = 3,
};

struct is31fl3733_child_config {
	const char *type;
	uint32_t red_channel;
	uint32_t green_channel;
	uint32_t blue_channel;
	uint32_t default_color[3];
	enum Mode mode;
};

/** IS31FL3733 configuration (DT). */
struct is31fl3733_config {
	/** I2C controller device name. */
	const char *const i2c_name;
	/** I2C chip address. */
	const uint8_t i2c_address;
	const struct gpio_dt_spec *gpio_reset_spec;

	const struct is31fl3733_child_config *const leds;
	const int num_leds;
};

/** IS31FL3733 data . */
struct is31fl3733_data {
	/** I2C controller device. */
	const struct device *const i2c_dev;
	/** Currently selected page. */
	int page;
	/** Cached state bitmap  */
	uint8_t state[12 * 16 / 8];
};

struct is31fl3733_reg {
	int page;
	uint8_t addr;
};

struct is31fl3733_setting {
	struct is31fl3733_reg reg;
	uint8_t value;
};

#define IS31FL3733_REG(page_num, reg_addr)                                                         \
	{                                                                                          \
		.page = page_num, .addr = reg_addr                                                 \
	}

#define PAGE_CONTROL 0x00
#define PAGE_PWM 0x01
#define PAGE_AUTO_BREATH_MODE 0x02
#define PAGE_FUNCTION 0x03

#define CONFIG_REG IS31FL3733_REG(PAGE_FUNCTION, 0)
#define GCC_REG IS31FL3733_REG(PAGE_FUNCTION, 1)
#define ABM_1_REG_1 IS31FL3733_REG(PAGE_FUNCTION, 2)
#define ABM_1_REG_2 IS31FL3733_REG(PAGE_FUNCTION, 3)
#define ABM_1_REG_3 IS31FL3733_REG(PAGE_FUNCTION, 4)
#define ABM_1_REG_4 IS31FL3733_REG(PAGE_FUNCTION, 5)
#define ABM_2_REG_1 IS31FL3733_REG(PAGE_FUNCTION, 6)
#define ABM_2_REG_2 IS31FL3733_REG(PAGE_FUNCTION, 7)
#define ABM_2_REG_3 IS31FL3733_REG(PAGE_FUNCTION, 8)
#define ABM_2_REG_4 IS31FL3733_REG(PAGE_FUNCTION, 9)
#define ABM_3_REG_1 IS31FL3733_REG(PAGE_FUNCTION, 0xa)
#define ABM_3_REG_2 IS31FL3733_REG(PAGE_FUNCTION, 0xb)
#define ABM_3_REG_3 IS31FL3733_REG(PAGE_FUNCTION, 0xc)
#define ABM_3_REG_4 IS31FL3733_REG(PAGE_FUNCTION, 0xd)
#define TIME_UPDATE_REG IS31FL3733_REG(PAGE_FUNCTION, 0xe)
#define RESET_REG IS31FL3733_REG(PAGE_FUNCTION, 0x11)

// These 2 registers are page independent since they are used for page switching
#define COMMAND_REGISTER_REG 0xFD
#define COMMAND_REGISTER_WRITE_LOCK_REG 0xFE
#define COMMAND_REGISTER_LOCK_PASS 0xC5

static const struct is31fl3733_setting init_settings[] = {
	{ .reg = ABM_1_REG_1, .value = (2 << 5) | (0 << 1) },
	{ .reg = ABM_1_REG_2, .value = (2 << 5) | (3 << 1) },
	{ .reg = ABM_1_REG_3, .value = (0 << 4) },
	{ .reg = ABM_2_REG_1, .value = (2 << 5) | (0 << 1) },
	{ .reg = ABM_2_REG_2, .value = (2 << 5) | (2 << 1) },
	{ .reg = ABM_2_REG_3, .value = (0 << 4) },
	{ .reg = ABM_3_REG_1, .value = (1 << 5) | (0 << 1) },
	{ .reg = ABM_3_REG_2, .value = (1 << 5) | (1 << 1) },
	{ .reg = ABM_3_REG_3, .value = (0 << 4) },
	{ .reg = CONFIG_REG, .value = 1 },
	{ .reg = CONFIG_REG, .value = 3 },
	{ .reg = TIME_UPDATE_REG, .value = 0 },
	{ .reg = GCC_REG, .value = 128 },
};

static int is31fl3733_set_page(const struct device *dev, uint32_t page)
{
	struct is31fl3733_data *const data = dev->data;
	const struct is31fl3733_config *const config = dev->config;

	if (data->page == page) {
		// Already selected
		return 0;
	}

	int retval =
		i2c_reg_write_byte(data->i2c_dev, config->i2c_address,
				   COMMAND_REGISTER_WRITE_LOCK_REG, COMMAND_REGISTER_LOCK_PASS);
	if (retval < 0) {
		LOG_ERR("Error setting register COMMAND_REGISTER_WRITE_LOCK_REG");
		return retval;
	}

	retval = i2c_reg_write_byte(data->i2c_dev, config->i2c_address, COMMAND_REGISTER_REG, page);
	if (retval < 0) {
		LOG_ERR("Error setting register COMMAND_REGISTER_REG");
		return retval;
	}

	data->page = page;
	return 0;
}

static int is31fl3733_write_reg(const struct device *const dev, const struct is31fl3733_reg reg,
				const uint8_t value)
{
	int retval = is31fl3733_set_page(dev, reg.page);
	if (retval < 0) {
		return retval;
	}

	const struct is31fl3733_config *const config = dev->config;
	const struct is31fl3733_data *const data = dev->data;
	return i2c_reg_write_byte(data->i2c_dev, config->i2c_address, reg.addr, value);
}

static int is31fl3733_read_reg(const struct device *const dev, const struct is31fl3733_reg reg,
			       uint8_t *const value)
{
	int retval = is31fl3733_set_page(dev, reg.page);
	if (retval < 0) {
		return retval;
	}

	const struct is31fl3733_config *const config = dev->config;
	const struct is31fl3733_data *const data = dev->data;
	return i2c_reg_read_byte(data->i2c_dev, config->i2c_address, reg.addr, value);
}

static int is31fl3733_set_pixel_mode(const struct device *dev, uint32_t led, enum Mode mode)
{
	const struct is31fl3733_config *const config = dev->config;
	const struct is31fl3733_data *const data = dev->data;

	if (led >= config->num_leds) {
		return -ENOENT;
	}

	const struct is31fl3733_child_config *const led_config = &config->leds[led];

	int retval = is31fl3733_set_page(dev, PAGE_AUTO_BREATH_MODE);
	if (retval < 0) {
		return retval;
	}

	retval = i2c_reg_write_byte(data->i2c_dev, config->i2c_address, led_config->red_channel,
				    mode);
	if (retval < 0) {
		return retval;
	}

	retval = i2c_reg_write_byte(data->i2c_dev, config->i2c_address, led_config->green_channel,
				    mode);
	if (retval < 0) {
		return retval;
	}

	return i2c_reg_write_byte(data->i2c_dev, config->i2c_address, led_config->blue_channel,
				  mode);
}

static int is31fl3733_apply_settings(const struct device *dev,
				     const struct is31fl3733_setting *settings,
				     uint32_t settings_length)
{
	if (settings_length == 0xFFFFFFFF) {
		return -EINVAL;
	}

	for (uint32_t i = 0; i < settings_length; i++) {
		int retval = is31fl3733_write_reg(dev, settings[i].reg, settings[i].value);
		if (retval < 0) {
			return retval;
		}
	}
	return 0;
}

static int is31fl3733_set_brightness(const struct device *const dev, const uint32_t led,
				     const uint8_t brightness)
{
	const struct is31fl3733_config *const config = dev->config;
	const struct is31fl3733_data *const data = dev->data;

	if (led >= config->num_leds) {
		return -ENOENT;
	}

	const struct is31fl3733_child_config *const led_config = &config->leds[led];

	int retval = is31fl3733_set_page(dev, PAGE_PWM);
	if (retval < 0) {
		return retval;
	}

	retval = i2c_reg_write_byte(data->i2c_dev, config->i2c_address, led_config->red_channel,
				    ((uint32_t)brightness) * (led_config->default_color[0]) / 256);
	if (retval < 0) {
		return retval;
	}

	retval = i2c_reg_write_byte(data->i2c_dev, config->i2c_address, led_config->green_channel,
				    ((uint32_t)brightness) * (led_config->default_color[1]) / 256);
	if (retval < 0) {
		return retval;
	}

	return i2c_reg_write_byte(data->i2c_dev, config->i2c_address, led_config->blue_channel,
				  ((uint32_t)brightness) * (led_config->default_color[2]) / 256);
}

static int is31fl3733_reset(const struct device *const dev)
{
	uint8_t val;
	const struct is31fl3733_reg reg = RESET_REG;
	int retval = is31fl3733_read_reg(dev, reg, &val);
	if (retval < 0) {
		LOG_ERR("Error reading reset register");
	}
	return retval;
}

static int is31fl3733_channel_on_off(const struct device *const dev, const uint32_t channel,
				     const bool on)
{
	struct is31fl3733_data *const data = dev->data;
	const struct is31fl3733_reg reg = { .page = PAGE_CONTROL, .addr = channel / 8 };

	if (on) {
		data->state[channel / 8] |= 1 << (channel % 8);
	} else {
		data->state[channel / 8] &= ~(1 << (channel % 8));
	}

	return is31fl3733_write_reg(dev, reg, data->state[channel / 8]);
}

static int is31fl3733_led_on_off(const struct device *const dev, const uint32_t led, const bool on)
{
	const struct is31fl3733_config *const config = dev->config;

	if (led >= config->num_leds) {
		return -ENOENT;
	}

	const struct is31fl3733_child_config *const led_config = &config->leds[led];

	int retval = is31fl3733_channel_on_off(dev, led_config->red_channel, on);
	if (retval < 0) {
		return retval;
	}

	retval = is31fl3733_channel_on_off(dev, led_config->green_channel, on);
	if (retval < 0) {
		return retval;
	}

	return is31fl3733_channel_on_off(dev, led_config->blue_channel, on);
}

static int is31fl3733_configure(const struct device *const dev)
{
	int retval = is31fl3733_apply_settings(dev, init_settings, ARRAY_SIZE(init_settings));
	if (retval < 0) {
		return retval;
	}

	// Configure each led
	// - mode
	// - brightness
	// - initial state

	const struct is31fl3733_config *const config = dev->config;
	for (uint32_t led = 0; led < config->num_leds; led++) {
		int retval = is31fl3733_set_pixel_mode(dev, led, config->leds[led].mode);
		if (retval < 0) {
			return retval;
		}

		retval = is31fl3733_set_brightness(dev, led, 0xff);
		if (retval < 0) {
			return retval;
		}

		retval = is31fl3733_led_on_off(dev, led, true);
		if (retval < 0) {
			return retval;
		}
	}

	return 0;
}

static int is31fl3733_led_on(const struct device *dev, const uint32_t led)
{
	return is31fl3733_led_on_off(dev, led, true);
}

static int is31fl3733_led_off(const struct device *dev, const uint32_t led)
{
	return is31fl3733_led_on_off(dev, led, false);
}

static int is31fl3733_led_brightness(const struct device *const dev, const uint32_t led,
				     const uint8_t value)
{
	const int retval = is31fl3733_led_on_off(dev, led, value > 0);
	if (retval < 0) {
		return retval;
	}

	return is31fl3733_set_brightness(dev, led, value);
}

static int is31fl3733_leds_init(const struct device *dev)
{
	const struct is31fl3733_config *config = dev->config;

	int retval = 0;

	if (config->gpio_reset_spec != NULL) {
		retval = gpio_pin_configure_dt(config->gpio_reset_spec, GPIO_OUTPUT_HIGH);
		if (retval < 0) {
			LOG_ERR("Init error: could not claim gpio");
			return -EINVAL;
		}
	}

	retval = is31fl3733_reset(dev);
	if (retval < 0) {
		LOG_ERR("Init error: unable to reset device");
		return -ENODEV;
	}

	retval = is31fl3733_configure(dev);
	if (retval < 0) {
		LOG_ERR("Init error: unable to configure device");
		return -ENODEV;
	}

	return 0;
}

static const struct led_driver_api is31fl3733_leds_api = {
	.on = is31fl3733_led_on,
	.off = is31fl3733_led_off,
	.set_brightness = is31fl3733_led_brightness,
};

#define IS31FL3733_DEFINE_CHILD_LED(node_id)                                                       \
	{                                                                                          \
		.type = DT_PROP(node_id, type),                                                    \
		.red_channel = DT_PROP_OR(node_id, red_channel, 0),                                \
		.green_channel = DT_PROP_OR(node_id, green_channel, 0),                            \
		.blue_channel = DT_PROP_OR(node_id, blue_channel, 0),                              \
		.default_color = DT_PROP_OR(node_id, default_color, __DEBRACKET({ 0, 0, 0 })),     \
		.mode = DT_PROP_OR(node_id, mode, Mode::PWM),                                      \
	},

#define IS31FL3733_DEFINE_POWER_GPIO(index)                                                        \
	const struct gpio_dt_spec gpio_reset_spec_##index =                                        \
		GPIO_DT_SPEC_INST_GET(index, power_gpios)

#define IS31FL3733_DEFINE_CONFIG(index)                                                            \
	static const struct is31fl3733_child_config is31fl3733_child_config_##index[] = {          \
		DT_INST_FOREACH_CHILD(index, IS31FL3733_DEFINE_CHILD_LED)                          \
	};                                                                                         \
                                                                                                   \
	/* Condtitionally define the power gpio (if available) */                                  \
	COND_CODE_1(DT_INST_NODE_HAS_PROP(index, power_gpios),                                     \
		    (IS31FL3733_DEFINE_POWER_GPIO(index)), ());                                    \
                                                                                                   \
	static const struct is31fl3733_config is31fl3733_config_##index = {                        \
		.i2c_name = DT_INST_BUS_LABEL(index),                                              \
		.i2c_address = DT_INST_REG_ADDR(index),                                            \
		.gpio_reset_spec = COND_CODE_1(DT_INST_NODE_HAS_PROP(index, power_gpios),          \
					       (&gpio_reset_spec_##index), (NULL)),                \
		.leds = is31fl3733_child_config_##index,                                           \
		.num_leds = ARRAY_SIZE(is31fl3733_child_config_##index),                           \
	}

#define IS31FL3733_DEFINE_DATA(index)                                                              \
	static struct is31fl3733_data is31fl3733_data_##index = {                                  \
		.i2c_dev = DEVICE_DT_GET(DT_INST_BUS(index)),                                      \
		.page = -1,                                                                        \
	}

#define IS31FL3733_DEVICE(i)                                                                       \
	IS31FL3733_DEFINE_CONFIG(i);                                                               \
	IS31FL3733_DEFINE_DATA(i);                                                                 \
	DEVICE_DT_INST_DEFINE(i, &is31fl3733_leds_init, NULL, &is31fl3733_data_##i,                \
			      &is31fl3733_config_##i, POST_KERNEL, CONFIG_LED_INIT_PRIORITY,       \
			      &is31fl3733_leds_api);

DT_INST_FOREACH_STATUS_OKAY(IS31FL3733_DEVICE)
