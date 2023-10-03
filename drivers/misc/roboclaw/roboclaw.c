/*
 * Copyright (c) 2023 Mathis Lécrivain
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#define DT_DRV_COMPAT basicmicro_roboclaw_serial

#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/uart.h>
#include <zephyr/drivers/misc/roboclaw/roboclaw.h>
#include <zephyr/sys/byteorder.h>
#include <zephyr/sys/crc.h>
#include <zephyr/sys_clock.h>

#include <zephyr/logging/log.h>

LOG_MODULE_REGISTER(roboclaw, CONFIG_ROBOCLAW_SERIAL_LOG_LEVEL);
/****************************************************************************************************
 *                                         Private defines
 ***************************************************************************************************/
#define CRC_POLY (0x1021)
#define CRC_SEED (0x0)

/****************************************************************************************************
 *                                          Private struct
 ***************************************************************************************************/
/**
 * @brief Build time configurations for the roboclaw UART peripheral.
 */
struct roboclaw_config {
	const struct device *uart_dev;
	uint8_t address;
	int timeout;
};

/****************************************************************************************************
 *                                          Private enum
 ***************************************************************************************************/
enum command {
	COMMAND_GET_M1_ENC = 16,
	COMMAND_GET_M2_ENC = 17,
	COMMAND_GET_VERSION = 21,
	COMMAND_SET_M1_DUTY = 32,
	COMMAND_SET_M2_DUTY = 33,
};

/****************************************************************************************************
 *                                          Private variable
 ***************************************************************************************************/

/****************************************************************************************************
 *                                          Private tab
 ***************************************************************************************************/

/****************************************************************************************************
 *                                     Private function prototypes
 ***************************************************************************************************/
static int roboclaw_init(const struct device *dev);
static int roboclaw_write(const struct device *dev, const uint8_t *buf, size_t size);
static int roboclaw_read(const struct device *dev, uint8_t *buf, size_t size);
static int roboclaw_write16(const struct device *dev, uint8_t cmd, uint16_t data);
static int roboclaw_read32(const struct device *dev, uint8_t cmd, uint32_t *data, uint8_t *status);

/****************************************************************************************************
 *                                     Public function definition
 ***************************************************************************************************/
int roboclaw_set_motor_duty(const struct device *dev, roboclaw_motor_id_t motor_id, int16_t duty)
{
	uint8_t cmd;

	switch (motor_id) {
	case MOTOR_ID_M1:
		cmd = COMMAND_SET_M1_DUTY;
		break;
	case MOTOR_ID_M2:
		cmd = COMMAND_SET_M2_DUTY;
		break;

	default:
		return -EINVAL;
	}

	return roboclaw_write16(dev, cmd, duty);
}

int roboclaw_get_encoder_counter(const struct device *dev, roboclaw_encoder_id_t id,
				 int32_t *counter, uint8_t *status)
{
	uint8_t cmd;

	switch (id) {
	case ENCODER_ID_M1:
		cmd = COMMAND_GET_M1_ENC;
		break;
	case ENCODER_ID_M2:
		cmd = COMMAND_GET_M2_ENC;
		break;
	default:
		return -EINVAL;
	}

	return roboclaw_read32(dev, cmd, counter, status);
}

/****************************************************************************************************
 *                                     Private function definition
 ***************************************************************************************************/
/**
 * @brief Initializes Roboclaw device
 *
 * @param dev Roboclaw device
 * @return 0 on success, negative error code otherwise
 */
static int roboclaw_init(const struct device *dev)
{
	struct roboclaw_config *conf = (struct roboclaw_config *)dev->config;

	/* Check if UART is ready */
	if (!device_is_ready(conf->uart_dev)) {
		return -ENODEV;
	}

	LOG_INF("Roboclaw initialized (address: 0x%x | timeout: %d)", conf->address, conf->timeout);

	return 0;
}

/**
 * @brief Write bytes on uart device
 *
 * @param dev Roboclaw device
 * @param buf output buffer
 * @param size size to write
 * @return int ret code
 */
static int roboclaw_write(const struct device *dev, const uint8_t *buf, size_t size)
{
	struct roboclaw_config *conf = (struct roboclaw_config *)dev->config;

	if (size == 0) {
		return 0;
	}

	for (size_t i = 0; i < size; i++) {
		uart_poll_out(conf->uart_dev, buf[i]);
	}

	return 0;
}

/**
 * @brief Read bytes from uart device
 *
 * @param dev Roboclaw device
 * @param buf input buffer
 * @param size size to read
 * @return int ret code
 */
static int roboclaw_read(const struct device *dev, uint8_t *buf, size_t size)
{
	int rc = 0;
	uint64_t end;
	struct roboclaw_config *conf = (struct roboclaw_config *)dev->config;

	/* Make sure we don't wait forever */
	end = sys_clock_timeout_end_calc(K_MSEC(conf->timeout));

	for (uint8_t i = 0; i < size && rc == 0; ++i) {
		do {
			rc = uart_poll_in(conf->uart_dev, &buf[i]);
		} while (rc == -1 && end > k_uptime_ticks());
	}

	/* -1 indicates we timed out */
	rc = rc == -1 ? -EAGAIN : rc;

	if (rc < 0) {
		LOG_ERR("Failed to read data (%d)", rc);
	}

	return rc;
}

static int roboclaw_write16(const struct device *dev, uint8_t cmd, uint16_t data)
{
	int rc = 0;
	uint8_t buf[6];
	uint8_t ack;
	uint16_t crc = 0;
	struct roboclaw_config *conf = (struct roboclaw_config *)dev->config;

	buf[0] = conf->address;
	buf[1] = cmd;

	/* Append data value in big endian format */
	sys_put_be16(data, &buf[2]);

	/* Compute CRC */
	crc = crc16(CRC_POLY, CRC_SEED, buf, 4);

	/* Append crc value in big endian format */
	sys_put_be16(crc, &buf[4]);

	/* Write buff */
	if ((rc = roboclaw_write(dev, buf, 6))) {
		return rc;
	}

	/* Read response */
	if ((rc = roboclaw_read(dev, &ack, sizeof(ack)))) {
		return rc;
	}

	if (ack != 0xFF) {
		return -EIO;
	}

	return 0;
}

static int roboclaw_read32(const struct device *dev, uint8_t cmd, uint32_t *data, uint8_t *status)
{
	int rc = 0;
	uint8_t buf[8];
	uint16_t ccrc = 0;
	uint16_t crc = 0;
	size_t rd_size = 0;
	struct roboclaw_config *conf = (struct roboclaw_config *)dev->config;

	buf[0] = conf->address;
	buf[1] = cmd;

	/* Write buff */
	if ((rc = roboclaw_write(dev, buf, 2))) {
		return rc;
	}

	if (status) {
		rd_size = sizeof(int32_t) + sizeof(uint8_t);
	} else {
		rd_size = sizeof(int32_t);
	}

	/* Read response */
	if ((rc = roboclaw_read(dev, buf, rd_size + sizeof(crc)))) {
		return rc;
	}

	/* Read crc in big endian format */
	ccrc = sys_get_be16(&buf[rd_size]);

	/* Compute CRC (Including input address/command in order to validate tx/rx chain)*/
	crc = CRC_SEED;
	crc = crc16(CRC_POLY, crc, &conf->address, sizeof(conf->address));
	crc = crc16(CRC_POLY, crc, &cmd, sizeof(cmd));
	crc = crc16(CRC_POLY, crc, buf, rd_size);

	if (ccrc != crc) {
		LOG_ERR("Bad crc (%x != %x)", ccrc, crc);
		return -EBADMSG;
	}

	/* read data in big endian format */
	*data = sys_get_be32(&buf[0]);

	if (status) {
		*status = buf[sizeof(uint32_t)];
	}

	return rc;
}

/****************************************************************************************************
 *                                    Driver definition
 ***************************************************************************************************/
#define INIT_ROBOCLAW_DEVICE(inst)                                                                 \
	static struct roboclaw_config roboclaw_config_##inst = {                                   \
		.uart_dev = DEVICE_DT_GET(DT_INST_BUS(inst)),                                      \
		.address = DT_INST_PROP(inst, address),                                            \
		.timeout = DT_INST_PROP(inst, timeout),                                            \
	};                                                                                         \
	DEVICE_DT_INST_DEFINE(inst, roboclaw_init, NULL, NULL, &roboclaw_config_##inst,            \
			      POST_KERNEL, 80, NULL);

DT_INST_FOREACH_STATUS_OKAY(INIT_ROBOCLAW_DEVICE);
