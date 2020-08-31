/*
 * Copyright (c) 2019 Manivannan Sadhasivam
 * Copyright (c) 2020 Andreas Sandberg
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <drivers/gpio.h>
#include <drivers/lora.h>
#include <drivers/spi.h>
#include <zephyr.h>

#include <lr1110/lr1110.h>
#include <lr1110_radio.h>
#include <lr1110_system_types.h>
#include <lr1110_system.h>

#include "sx12xx_common.h"

#include <logging/log.h>
LOG_MODULE_REGISTER(lr1110, CONFIG_LORA_LOG_LEVEL);

#if DT_HAS_COMPAT_STATUS_OKAY(semtech_lr1110)
#define DT_DRV_COMPAT semtech_lr1110
#define LR1110_DEVICE_ID LR1110
#else
#error No LR1110 instance in device tree.
#endif

BUILD_ASSERT(DT_NUM_INST_STATUS_OKAY(semtech_lr1110) <= 1,
	     "Multiple LR1110 instances in DT");

#define HAVE_GPIO_CS DT_INST_SPI_DEV_HAS_CS_GPIOS(0)
#define GPIO_CS_LABEL DT_INST_SPI_DEV_CS_GPIOS_LABEL(0)
#define GPIO_CS_PIN DT_INST_SPI_DEV_CS_GPIOS_PIN(0)
#define GPIO_CS_FLAGS DT_INST_SPI_DEV_CS_GPIOS_FLAGS(0)

#define GPIO_RESET_PIN DT_INST_GPIO_PIN(0, reset_gpios)
#define GPIO_BUSY_PIN DT_INST_GPIO_PIN(0, busy_gpios)
#define GPIO_DIO1_PIN DT_INST_GPIO_PIN(0, dio1_gpios)

#define HAVE_TCXO DT_INST_NODE_HAS_PROP(0, tcxo)

#if DT_INST_NODE_HAS_PROP(0, tcxo_power_startup_delay_ms)
#define TCXO_POWER_STARTUP_DELAY_MS DT_INST_PROP(0, tcxo_power_startup_delay_ms)
#else
#define TCXO_POWER_STARTUP_DELAY_MS 0
#endif

#if (HAVE_TCXO == 1)
#define BOARD_TCXO_WAKEUP_TIME 5 // 5 milliseconds
#else
#define BOARD_TCXO_WAKEUP_TIME 0
#endif

struct lr1110_data {
	struct device *reset;
	struct device *busy;
	struct device *dio1;
	struct gpio_callback dio1_irq_callback;
	struct k_work dio1_irq_work;
	lr1110_dio_irq_handler radio_dio_irq;
	struct device *spi;
	struct spi_config spi_cfg;
#if HAVE_GPIO_CS
	struct spi_cs_control spi_cs;
#endif
} dev_data;

static lr1110_hal_status_t lr1110_hal_wait_on_busy(const void *context);

static int lr1110_spi_transceive(const uint8_t *req_tx, const uint8_t *req_rx,
				 const size_t req_len, const void *data_tx,
				 const void *data_rx, const size_t data_len)
{
	int ret;

	const struct spi_buf tx_buf[] = { {
						  .buf = (void *)req_tx,
						  .len = (size_t)req_len,
					  },
					  { .buf = (void *)data_tx,
					    .len = (size_t)data_len } };

	const struct spi_buf rx_buf[] = { {
						  .buf = (void *)req_rx,
						  .len = (size_t)req_len,
					  },
					  { .buf = (void *)data_rx,
					    .len = (size_t)data_len } };

	const struct spi_buf_set tx = {
		.buffers = tx_buf,
		.count = ARRAY_SIZE(tx_buf),
	};

	const struct spi_buf_set rx = { .buffers = rx_buf,
					.count = ARRAY_SIZE(rx_buf) };

	if (!req_rx && !data_rx) {
		ret = spi_write(dev_data.spi, &dev_data.spi_cfg, &tx);
	} else {
		ret = spi_transceive(dev_data.spi, &dev_data.spi_cfg, &tx, &rx);
	}

	if (ret < 0) {
		LOG_ERR("SPI transaction failed: %i", ret);
	}

	return ret;
}

lr1110_hal_status_t lr1110_hal_write(const void *context,
				     const uint8_t *command,
				     const uint16_t command_length,
				     const uint8_t *data,
				     const uint16_t data_length)

{
	LOG_DBG("Writing: command len %" PRIu16 " - date len %" PRIu16,
		command_length, data_length);

	if (lr1110_hal_wakeup(context) == LR1110_HAL_STATUS_OK) {
		if (lr1110_spi_transceive(command, NULL, command_length, data,
					  NULL, data_length) < 0) {
			LOG_ERR("SPI transceive failed");
			return LR1110_HAL_STATUS_ERROR;
		}

		// 0x011B - LR1110_SYSTEM_SET_SLEEP_OC
		if (((command[0] << 8) | command[1]) != 0x011B) {
			return lr1110_hal_wait_on_busy(context);
		} else {
			return LR1110_HAL_STATUS_OK;
		}
	}
	return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_read(const void *context, const uint8_t *command,
				    const uint16_t command_length,
				    uint8_t *data, const uint16_t data_length)
{
	if (lr1110_hal_wakeup(context) == LR1110_HAL_STATUS_OK) {
		lr1110_spi_transceive(command, NULL, command_length, NULL, NULL,
				      0);

		lr1110_hal_wait_on_busy(context);

		// Send dummy byte, then read data
		uint8_t dummy = 0;
		lr1110_spi_transceive(&dummy, NULL, 1, NULL, data, data_length);

		return lr1110_hal_wait_on_busy(context);
	}
	return LR1110_HAL_STATUS_ERROR;
}

lr1110_hal_status_t lr1110_hal_write_read(const void *context,
					  const uint8_t *command, uint8_t *data,
					  const uint16_t data_length)
{
	if (lr1110_hal_wakeup(context) == LR1110_HAL_STATUS_OK) {
		lr1110_spi_transceive(command, data, data_length, NULL, NULL,
				      0);

		// 0x011B - LR1110_SYSTEM_SET_SLEEP_OC
		if (((command[0] << 8) | command[1]) != 0x011B) {
			return lr1110_hal_wait_on_busy(context);
		} else {
			return LR1110_HAL_STATUS_OK;
		}
	}
	return LR1110_HAL_STATUS_ERROR;
}

uint32_t lr1110_board_get_tcxo_wakeup_time(const void *context)
{
	return TCXO_POWER_STARTUP_DELAY_MS;
}

static void lr1110_baord_init_tcxo_io(const void *context)
{
#if HAVE_TCXO
	LOG_DBG("TCXO on shield");
	lr1110_system_set_tcxo_mode(
		context, LR1110_SYSTEM_TCXO_SUPPLY_VOLTAGE_1_8V,
		(lr1110_board_get_tcxo_wakeup_time(context) * 1000) / 30.52);

	uint8_t calib_params = LR1110_SYSTEM_CALIBRATE_LF_RC_MASK |
			       LR1110_SYSTEM_CALIBRATE_HF_RC_MASK |
			       LR1110_SYSTEM_CALIBRATE_PLL_MASK |
			       LR1110_SYSTEM_CALIBRATE_ADC_MASK |
			       LR1110_SYSTEM_CALIBRATE_IMG_MASK |
			       LR1110_SYSTEM_CALIBRATE_PLL_TX_MASK;
	lr1110_system_calibrate(context, calib_params);
#else
	LOG_DBG("No TCXO configured");
#endif
}

void lr1110_hal_reset(const void *context)
{
	LOG_DBG("Resetting radio");
	gpio_pin_set(dev_data.reset, GPIO_RESET_PIN, 1);
	k_sleep(K_MSEC(1));
	gpio_pin_set(dev_data.reset, GPIO_RESET_PIN, 0);
}

void lr1110_board_set_rf_tx_power(const void *context, int8_t power)
{
	LOG_DBG("power: %" PRIi8, power);

	// TODO: Add PA Config check
	if (power > 0) {
		if (power > 22) {
			power = 22;
		}
	} else {
		if (power < -9) {
			power = -9;
		}
	}
	lr1110_radio_set_tx_params(context, power, LR1110_RADIO_RAMP_TIME_40U);
}

static lr1110_hal_status_t lr1110_hal_wait_on_busy(const void *context)
{
	while (gpio_pin_get(dev_data.busy, GPIO_BUSY_PIN)) {
		k_sleep(K_MSEC(1));
	}
	return LR1110_HAL_STATUS_OK;
}

lr1110_hal_status_t lr1110_hal_wakeup(const void *context)
{
	LOG_DBG("Getting operating mode");
	if ((lr1110_hal_get_operating_mode(context) ==
	     LR1110_HAL_OP_MODE_SLEEP) ||
	    (lr1110_hal_get_operating_mode(context) ==
	     LR1110_HAL_OP_MODE_RX_DC)) {
		// Wakeup radio
		gpio_pin_set(dev_data.spi_cs.gpio_dev, GPIO_CS_PIN, 0);
		gpio_pin_set(dev_data.spi_cs.gpio_dev, GPIO_CS_PIN, 1);

		// Radio is awake in STDBY_RC mode
		((lr1110_t *)context)->op_mode = LR1110_HAL_OP_MODE_STDBY_RC;
	}

	// Wait on busy pin for 100 ms
	LOG_DBG("Waiting for device...");
	return lr1110_hal_wait_on_busy(context);
	LOG_DBG("Device ready");
}

lr1110_hal_operating_mode_t lr1110_hal_get_operating_mode(const void *context)
{
	return ((lr1110_t *)context)->op_mode;
}

void lr1110_hal_set_operating_mode(const void *context,
				   lr1110_hal_operating_mode_t op_mode)
{
	((lr1110_t *)context)->op_mode = op_mode;
}

static void lr1110_dio1_irq_work_handler(struct k_work *work)
{
	LOG_DBG("Processing DIO1 interrupt");
	if (!dev_data.radio_dio_irq) {
		LOG_WRN("DIO1 interrupt without valid HAL IRQ callback.");
		return;
	}

	dev_data.radio_dio_irq(NULL);
	if (Radio.IrqProcess) {
		Radio.IrqProcess();
	}
}

static void lr1110_dio1_irq_callback(struct device *dev,
				     struct gpio_callback *cb, uint32_t pins)
{
	if (pins & BIT(GPIO_DIO1_PIN)) {
		k_work_submit(&dev_data.dio1_irq_work);
	}
}

void lr1110_board_init(const void *context, lr1110_dio_irq_handler dio_irq)
{
	lr1110_system_reset(context);
	lr1110_hal_set_operating_mode(context, LR1110_HAL_OP_MODE_STDBY_RC);

	// setup interrupt
	dev_data.radio_dio_irq = dio_irq;
	k_work_init(&dev_data.dio1_irq_work, lr1110_dio1_irq_work_handler);
	gpio_init_callback(&dev_data.dio1_irq_callback,
			   lr1110_dio1_irq_callback, BIT(GPIO_DIO1_PIN));
	if (gpio_add_callback(dev_data.dio1, &dev_data.dio1_irq_callback) < 0) {
		LOG_ERR("Could not set GPIO callback for DIO1 interrupt.");
		return;
	}
	gpio_pin_interrupt_configure(dev_data.dio1, GPIO_DIO1_PIN,
				     GPIO_INT_EDGE_TO_ACTIVE);

	lr1110_system_stat1_t stat1;
	lr1110_system_stat2_t stat2;
	uint32_t irq = 0;
	lr1110_system_get_status(context, &stat1, &stat2, &irq);
	lr1110_system_version_t version;
	lr1110_system_get_version(context, &version);
	lr1110_system_errors_t errors = { 0 };
	lr1110_system_get_errors(context, &errors);
	lr1110_system_clear_errors(context);

	// Initialize TCXO control
	lr1110_baord_init_tcxo_io(context);

	// Initialize RF switch control
	lr1110_system_rfswitch_config_t rf_switch_configuration;
	rf_switch_configuration.enable =
		LR1110_SYSTEM_RFSW0_HIGH | LR1110_SYSTEM_RFSW1_HIGH;
	rf_switch_configuration.standby = 0;
	rf_switch_configuration.rx = LR1110_SYSTEM_RFSW0_HIGH;
	rf_switch_configuration.tx =
		LR1110_SYSTEM_RFSW0_HIGH | LR1110_SYSTEM_RFSW1_HIGH;
	rf_switch_configuration.wifi = 0;
	rf_switch_configuration.gnss = 0;

	lr1110_system_set_dio_as_rf_switch(context, &rf_switch_configuration);

	lr1110_radio_pa_config_t paConfig = {
		.pa_sel = LR1110_RADIO_PA_SEL_LP,
		.pa_reg_supply = LR1110_RADIO_PA_REG_SUPPLY_DCDC,
		.pa_dutycycle = 0x04,
		.pa_hp_sel = 0x00,
	};
	lr1110_radio_set_pa_config(context, &paConfig);

	// Set packet type
	lr1110_radio_packet_types_t packet_type = LR1110_RADIO_PACKET_LORA;
	lr1110_radio_set_packet_type(context, packet_type);
}

static int lr1110_lora_init(struct device *dev)
{
	int ret;

	LOG_DBG("Initializing %s", DT_INST_LABEL(0));

	if (sx12xx_configure_pin(reset, GPIO_OUTPUT_INACTIVE) ||
	    sx12xx_configure_pin(busy, GPIO_INPUT) ||
	    sx12xx_configure_pin(dio1, GPIO_INPUT | GPIO_INT_DEBOUNCE)) {
		return -EIO;
	}

	dev_data.spi = device_get_binding(DT_INST_BUS_LABEL(0));
	if (!dev_data.spi) {
		LOG_ERR("Cannot get pointer to %s device",
			DT_INST_BUS_LABEL(0));
		return -EINVAL;
	}

#if HAVE_GPIO_CS
	dev_data.spi_cs.gpio_dev = device_get_binding(GPIO_CS_LABEL);
	if (!dev_data.spi_cs.gpio_dev) {
		LOG_ERR("Cannot get pointer to %s device", GPIO_CS_LABEL);
		return -EIO;
	}

	dev_data.spi_cs.gpio_pin = GPIO_CS_PIN;
	dev_data.spi_cs.gpio_dt_flags = GPIO_CS_FLAGS;
	dev_data.spi_cs.delay = 0U;

	dev_data.spi_cfg.cs = &dev_data.spi_cs;
#endif
	dev_data.spi_cfg.operation = SPI_WORD_SET(8) | SPI_TRANSFER_MSB;
	dev_data.spi_cfg.frequency = DT_INST_PROP(0, spi_max_frequency);
	dev_data.spi_cfg.slave = DT_INST_REG_ADDR(0);

	ret = sx12xx_init(dev);
	if (ret < 0) {
		LOG_ERR("Failed to initialize SX12xx common");
		return ret;
	}

	LOG_INF("Lora driver initialized");

	return 0;
}

static const struct lora_driver_api lr1110_lora_api = {
	.config = sx12xx_lora_config,
	.send = sx12xx_lora_send,
	.recv = sx12xx_lora_recv,
	.test_cw = sx12xx_lora_test_cw,
};

DEVICE_AND_API_INIT(lr1110_lora, DT_INST_LABEL(0), &lr1110_lora_init, NULL,
		    NULL, POST_KERNEL, CONFIG_LORA_INIT_PRIORITY,
		    &lr1110_lora_api);
