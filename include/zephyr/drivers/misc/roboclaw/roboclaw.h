/* grove_lcd.h - Public API for the Robolaw device */
/*
 * Copyright (c) 2023 Mathis Lécrivain
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#ifndef ZEPHYR_INCLUDE_ROBOCLAW_H_
#define ZEPHYR_INCLUDE_ROBOCLAW_H_

#include <stdint.h>

#include <zephyr/device.h>

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Roboclaw controller APIs
 * @defgroup roboclaw_driver Roboclaw controller APIs
 * @ingroup third_party
 * @{
 */

/****************************************************************************************************
 *                                         Public defines
 ***************************************************************************************************/

/****************************************************************************************************
 *                                          Public enum
 ***************************************************************************************************/
typedef enum {
	MOTOR_ID_M1,
	MOTOR_ID_M2,
} roboclaw_motor_id_t;

typedef enum {
	ENCODER_ID_M1,
	ENCODER_ID_M2,
} roboclaw_encoder_id_t;

/****************************************************************************************************
 *                                          Public struct
 ***************************************************************************************************/

/****************************************************************************************************
 *                                    Public function prototypes
 ***************************************************************************************************/
/**
 * @brief
 *
 * @param dev
 * @param motor_id
 * @param duty
 * @return int
 */
int roboclaw_set_motor_duty(const struct device *dev, roboclaw_motor_id_t motor_id, int16_t duty);

/**
 * @brief
 *
 * @param dev
 * @param id
 * @param counter
 * @param status
 * @return int
 */
int roboclaw_get_encoder_counter(const struct device *dev, roboclaw_encoder_id_t id,
				 int32_t *counter, uint8_t *status);

/**
 * @}
 */

#ifdef __cplusplus
}
#endif

#endif /* ZEPHYR_INCLUDE_ROBOCLAW_H_ */
