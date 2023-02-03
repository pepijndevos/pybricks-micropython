// SPDX-License-Identifier: MIT
// Copyright (c) 2020-2023 The Pybricks Authors

// IMU driver for STMicroelectronics LSM6DS3TR-C accel/gyro connected to STM32 MCU.


#include <pbdrv/config.h>

#if PBDRV_CONFIG_IMU_LSM6S3TR_C_STM32
#include <math.h>
#include <stdatomic.h>
#include <stdint.h>
#include <string.h>

#include <pbdrv/imu.h>

#include <contiki.h>
#include <lsm6ds3tr_c_reg.h>

#include STM32_HAL_H

#if defined(STM32L4)
#include <stm32l4xx_ll_i2c.h>
#endif

#include "../core.h"
#include "./imu_lsm6ds3tr_c_stm32.h"

typedef enum {
    /** Initialization is not complete yet. */
    IMU_INIT_STATE_BUSY,
    /** Initialization failed at some point. */
    IMU_INIT_STATE_FAILED,
    /** Initialization was successful. */
    IMU_INIT_STATE_COMPLETE,
} imu_init_state_t;

struct _pbdrv_imu_dev_t {
    /** Driver context for external library. */
    stmdev_ctx_t ctx;
    /** STM32 HAL I2C context. */
    I2C_HandleTypeDef hi2c;
    /** Scale factor to convert raw data to degrees per second. */
    float gyro_scale;
    /** Scale factor to convert raw data to m/s^2. */
    float accel_scale;
    /** Raw data. */
    int16_t data[6];
    /** Gyroscope rate offset */
    float gyro_bias[3];
    /** quaternion estimation */
    float q[4];
    uint32_t prev_sample_time;
    float ki;
    float kp;
    uint32_t sample_count;
    bool calibrating;
    /** Initialization state. */
    imu_init_state_t init_state;
    /** INT1 oneshot. */
    volatile bool int1;
};

/** The size of the data field in pbdrv_imu_dev_t in bytes. */
#define NUM_DATA_BYTES sizeof(((struct _pbdrv_imu_dev_t *)0)->data)

static pbdrv_imu_dev_t global_imu_dev;
PROCESS(pbdrv_imu_lsm6ds3tr_c_stm32_process, "LSM6DS3TR-C");

// REVISIT: For now, this driver takes complete ownership of the STM32 I2C
// subsystem. A shared I2C driver would be needed

void pbdrv_imu_lsm6ds3tr_c_stm32_handle_i2c_er_irq(void) {
    HAL_I2C_ER_IRQHandler(&global_imu_dev.hi2c);
}

void pbdrv_imu_lsm6ds3tr_c_stm32_handle_i2c_ev_irq(void) {
    HAL_I2C_EV_IRQHandler(&global_imu_dev.hi2c);
}

void HAL_I2C_MemTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    global_imu_dev.ctx.read_write_done = true;
    process_poll(&pbdrv_imu_lsm6ds3tr_c_stm32_process);
}

void HAL_I2C_MemRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    global_imu_dev.ctx.read_write_done = true;
    process_poll(&pbdrv_imu_lsm6ds3tr_c_stm32_process);
}

void HAL_I2C_MasterTxCpltCallback(I2C_HandleTypeDef *hi2c) {
    global_imu_dev.ctx.read_write_done = true;
    process_poll(&pbdrv_imu_lsm6ds3tr_c_stm32_process);
}

void HAL_I2C_MasterRxCpltCallback(I2C_HandleTypeDef *hi2c) {
    global_imu_dev.ctx.read_write_done = true;
    process_poll(&pbdrv_imu_lsm6ds3tr_c_stm32_process);
}

void HAL_I2C_ErrorCallback(I2C_HandleTypeDef *hi2c) {
    global_imu_dev.ctx.read_write_done = true;
    process_poll(&pbdrv_imu_lsm6ds3tr_c_stm32_process);
}

void pbdrv_imu_lsm6ds3tr_c_stm32_handle_int1_irq(void) {
    global_imu_dev.int1 = true;
    process_poll(&pbdrv_imu_lsm6ds3tr_c_stm32_process);
}

/**
 * Reset the I2C peripheral.
 *
 * Occasionally, I2C transactions will fail. The BUSY flag is stuck on and
 * HAL error flag is set to HAL_I2C_ERROR_AF. To recover, we just reset and
 * reinitialize the I2C peripheral.
 */
static void pbdrv_imu_lsm6ds3tr_c_stm32_i2c_reset(I2C_HandleTypeDef *hi2c) {
    I2C_TypeDef *I2C = hi2c->Instance;

    I2C->CR1 |= I2C_CR1_SWRST;
    I2C->CR1 &= ~I2C_CR1_SWRST;

    HAL_I2C_Init(hi2c);
}

static void pbdrv_imu_lsm6ds3tr_c_stm32_write_reg(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Write_IT(&global_imu_dev.hi2c, LSM6DS3TR_C_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, data, len);

    if (ret != HAL_OK) {
        // If there was an error, the interrupt will never come so we have to set the flag here.
        global_imu_dev.ctx.read_write_done = true;
    }
}

static void pbdrv_imu_lsm6ds3tr_c_stm32_read_reg(void *handle, uint8_t reg, uint8_t *data, uint16_t len) {
    HAL_StatusTypeDef ret = HAL_I2C_Mem_Read_IT(&global_imu_dev.hi2c, LSM6DS3TR_C_I2C_ADD_L, reg, I2C_MEMADD_SIZE_8BIT, data, len);

    if (ret != HAL_OK) {
        // If there was an error, the interrupt will never come so we have to set the flag here.
        global_imu_dev.ctx.read_write_done = true;
    }
}

static void mahony_update(pbdrv_imu_dev_t *imu_dev, float ax, float ay, float az, float gx, float gy, float gz, float dt) {
    float recip_norm;
    float vx, vy, vz;
    float ex, ey, ez; // error terms
    float qa, qb, qc;
    float q[4] = { imu_dev->q[0], imu_dev->q[1], imu_dev->q[2], imu_dev->q[3] };
    static float ix, iy, iz; // integral feedback terms
    float tmp;

    // Compute feedback only if accelerometer measurement valid (avoids NaN in accelerometer normalisation)
    tmp = ax * ax + ay * ay + az * az;

    if (tmp > 0.0f) {
        // Normalise accelerometer (assumed to measure the direction of gravity in body frame)
        recip_norm = 1.0f / sqrtf(tmp);
        ax *= recip_norm;
        ay *= recip_norm;
        az *= recip_norm;

        // Estimated direction of gravity in the body frame (factor of two divided out)
        vx = q[1] * q[3] - q[0] * q[2];
        vy = q[0] * q[1] + q[2] * q[3];
        vz = q[0] * q[0] - 0.5f + q[3] * q[3];

        // Error is cross product between estimated and measured direction of gravity in body frame
        // (half the actual magnitude)
        ex = ay * vz - az * vy;
        ey = az * vx - ax * vz;
        ez = ax * vy - ay * vx;

        // Compute and apply to gyro term the integral feedback, if enabled
        if (imu_dev->ki > 0.0f) {
            ix += imu_dev->ki * ex * dt; // integral error scaled by ki
            iy += imu_dev->ki * ey * dt;
            iz += imu_dev->ki * ez * dt;
            gx += ix; // apply integral feedback
            gy += iy;
            gz += iz;
        }

        // Apply proportional feedback to gyro term
        gx += imu_dev->kp * ex;
        gy += imu_dev->kp * ey;
        gz += imu_dev->kp * ez;
    }

    // Integrate rate of change of quaternion, q cross gyro term
    dt *= 0.5f;
    gx *= dt; // pre-multiply common factors
    gy *= dt;
    gz *= dt;
    qa = imu_dev->q[0];
    qb = imu_dev->q[1];
    qc = imu_dev->q[2];
    q[0] += -qb * gx - qc * gy - q[3] * gz;
    q[1] += qa * gx + qc * gz - q[3] * gy;
    q[2] += qa * gy - qb * gz + q[3] * gx;
    q[3] += qa * gz + qb * gy - qc * gx;

    // renormalise quaternion
    recip_norm = 1.0f / sqrtf(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    imu_dev->q[0] = q[0] * recip_norm;
    imu_dev->q[1] = q[1] * recip_norm;
    imu_dev->q[2] = q[2] * recip_norm;
    imu_dev->q[3] = q[3] * recip_norm;
}

static PT_THREAD(pbdrv_imu_lsm6ds3tr_c_stm32_init(struct pt *pt)) {
    const pbdrv_imu_lsm6s3tr_c_stm32_platform_data_t *pdata = &pbdrv_imu_lsm6s3tr_c_stm32_platform_data;
    pbdrv_imu_dev_t *imu_dev = &global_imu_dev;
    I2C_HandleTypeDef *hi2c = &imu_dev->hi2c;
    stmdev_ctx_t *ctx = &imu_dev->ctx;

    static struct pt child;
    static uint8_t id;
    static uint8_t rst;

    PT_BEGIN(pt);

    ctx->write_reg = pbdrv_imu_lsm6ds3tr_c_stm32_write_reg;
    ctx->read_reg = pbdrv_imu_lsm6ds3tr_c_stm32_read_reg;

    hi2c->Instance = pdata->i2c;
    #if defined(STM32L4)
    // HACK: This is hard-coded for Technic hub.
    // Clock is 5MHz, so these timing come out to 1 usec. When combined with
    // internal delays, this is slightly slower than 400kHz
    hi2c->Init.Timing = __LL_I2C_CONVERT_TIMINGS(0, 0, 0, 4, 4);
    #else
    hi2c->Init.ClockSpeed = 400000;
    #endif
    hi2c->Init.OwnAddress1 = 0;
    hi2c->Init.AddressingMode = I2C_ADDRESSINGMODE_7BIT;
    hi2c->Init.DualAddressMode = I2C_DUALADDRESS_DISABLE;
    hi2c->Init.GeneralCallMode = I2C_GENERALCALL_DISABLE;
    hi2c->Init.NoStretchMode = I2C_NOSTRETCH_DISABLE;

    HAL_I2C_Init(hi2c);

    PT_SPAWN(pt, &child, lsm6ds3tr_c_device_id_get(&child, ctx, &id));

    if (id != LSM6DS3TR_C_ID) {
        imu_dev->init_state = IMU_INIT_STATE_FAILED;
        PT_EXIT(pt);
    }

    // Init based on data polling example

    /*
     *  Restore default configuration
     */
    PT_SPAWN(pt, &child, lsm6ds3tr_c_reset_set(&child, ctx, PROPERTY_ENABLE));
    do {
        PT_SPAWN(pt, &child, lsm6ds3tr_c_reset_get(&child, ctx, &rst));
    } while (rst);

    /*
     * Set Output Data Rate
     */
    PT_SPAWN(pt, &child, lsm6ds3tr_c_xl_data_rate_set(&child, ctx, LSM6DS3TR_C_XL_ODR_1k66Hz));
    PT_SPAWN(pt, &child, lsm6ds3tr_c_gy_data_rate_set(&child, ctx, LSM6DS3TR_C_GY_ODR_1k66Hz));

    /*
     * Set scale
     */
    PT_SPAWN(pt, &child, lsm6ds3tr_c_xl_full_scale_set(&child, ctx, LSM6DS3TR_C_8g));
    imu_dev->accel_scale = lsm6ds3tr_c_from_fs8g_to_mg(1) * 9.81f;

    PT_SPAWN(pt, &child, lsm6ds3tr_c_gy_full_scale_set(&child, ctx, LSM6DS3TR_C_2000dps));
    imu_dev->gyro_scale = lsm6ds3tr_c_from_fs2000dps_to_mdps(1) / 1000.0f;

    // Configure INT1 to trigger when new gyro data is ready.
    PT_SPAWN(pt, &child, lsm6ds3tr_c_pin_int1_route_set(&child, ctx, (lsm6ds3tr_c_int1_route_t) {
        .int1_drdy_g = 1,
    }));

    // If we leave the default latched mode, sometimes we don't get the INT1 interrupt.
    PT_SPAWN(pt, &child, lsm6ds3tr_c_data_ready_mode_set(&child, ctx, LSM6DS3TR_C_DRDY_PULSED));

    // Enable rounding mode so we can get gyro + accel in continuous reads.
    PT_SPAWN(pt, &child, lsm6ds3tr_c_rounding_mode_set(&child, ctx, LSM6DS3TR_C_ROUND_GY_XL));

    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
        imu_dev->init_state = IMU_INIT_STATE_FAILED;
        PT_EXIT(pt);
    }

    // Sets initial values for Orientation estimation
    imu_dev->gyro_bias[0] = 0.0f;
    imu_dev->gyro_bias[1] = 0.0f;
    imu_dev->gyro_bias[2] = 0.0f;
    imu_dev->q[0] = 1.0f;
    imu_dev->q[1] = 0.0f;
    imu_dev->q[2] = 0.0f;
    imu_dev->q[3] = 0.0f;
    imu_dev->kp = 50.0f;
    imu_dev->ki = 0.0f;
    imu_dev->calibrating = false;

    imu_dev->prev_sample_time = pbdrv_clock_get_us();

    imu_dev->init_state = IMU_INIT_STATE_COMPLETE;

    PT_END(pt);
}

static void pbdrv_imu_update(pbdrv_imu_dev_t *imu_dev) {
    // Gets the time between loops of the estimate code
    uint32_t now = pbdrv_clock_get_us();
    float dt = (now - imu_dev->prev_sample_time) / 1000000.0f;
    imu_dev->prev_sample_time = now;

    float a[3]; // Scaled Accelerometer value array

    float g[3]; // Scaled Gyro value array

    // Gets scaled IMU values
    pbdrv_imu_accel_read(imu_dev, a);
    pbdrv_imu_gyro_read(imu_dev, g);

    if (imu_dev->calibrating) {
        imu_dev->gyro_bias[0] += g[0];
        imu_dev->gyro_bias[1] += g[1];
        imu_dev->gyro_bias[2] += g[2];
        imu_dev->sample_count++;
    } else {
        // Converts gyro values from deg/s to radians/s and applies gyro offset
        for (int i = 0; i < 3; i++) {
            g[i] = (g[i] - imu_dev->gyro_bias[i]) * 0.0174532925f;
        }
        mahony_update(imu_dev, a[0], a[1], a[2], g[0], g[1], g[2], dt);
    }
}

PROCESS_THREAD(pbdrv_imu_lsm6ds3tr_c_stm32_process, ev, data) {
    pbdrv_imu_dev_t *imu_dev = &global_imu_dev;
    I2C_HandleTypeDef *hi2c = &imu_dev->hi2c;

    static struct pt child;
    static uint8_t buf[NUM_DATA_BYTES];

    PROCESS_BEGIN();

    PROCESS_PT_SPAWN(&child, pbdrv_imu_lsm6ds3tr_c_stm32_init(&child));

    pbdrv_init_busy_down();

    if (imu_dev->init_state != IMU_INIT_STATE_COMPLETE) {
        // The IMU is not essential. It just won't be available if init fails.
        PROCESS_EXIT();
    }

retry:
    // Write the register address of the start of the gyro and accel data.
    buf[0] = LSM6DS3TR_C_OUTX_L_G;
    imu_dev->ctx.read_write_done = false;
    HAL_StatusTypeDef ret = HAL_I2C_Master_Seq_Transmit_IT(
        &imu_dev->hi2c, LSM6DS3TR_C_I2C_ADD_L, buf, 1, I2C_FIRST_FRAME);

    if (ret != HAL_OK) {
        pbdrv_imu_lsm6ds3tr_c_stm32_i2c_reset(hi2c);
        goto retry;
    }

    PROCESS_WAIT_UNTIL(imu_dev->ctx.read_write_done);

    if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
        pbdrv_imu_lsm6ds3tr_c_stm32_i2c_reset(hi2c);
        goto retry;
    }

    // Since we configured the IMU to enable "rounding" on the accel and gyro
    // data registers, we can just keep reading forever and it automatically
    // loops around. This way we don't have to keep writing the register
    // value each time we want to read new data. This saves CPU usage since
    // we have fewer interrupts per sample.

    for (;;) {
        PROCESS_WAIT_EVENT_UNTIL(atomic_exchange(&imu_dev->int1, false));

        imu_dev->ctx.read_write_done = false;
        ret = HAL_I2C_Master_Seq_Receive_IT(
            &imu_dev->hi2c, LSM6DS3TR_C_I2C_ADD_L, buf, NUM_DATA_BYTES, I2C_NEXT_FRAME);

        if (ret != HAL_OK) {
            pbdrv_imu_lsm6ds3tr_c_stm32_i2c_reset(hi2c);
            goto retry;
        }

        PROCESS_WAIT_UNTIL(imu_dev->ctx.read_write_done);

        if (HAL_I2C_GetError(hi2c) != HAL_I2C_ERROR_NONE) {
            pbdrv_imu_lsm6ds3tr_c_stm32_i2c_reset(hi2c);
            goto retry;
        }

        memcpy(&imu_dev->data[0], buf, NUM_DATA_BYTES);
        pbdrv_imu_update(imu_dev);
    }

    PROCESS_END();
}

// internal driver interface implementation

void pbdrv_imu_init(void) {
    pbdrv_init_busy_up();
    process_start(&pbdrv_imu_lsm6ds3tr_c_stm32_process);
}

// public driver interface implementation

pbio_error_t pbdrv_imu_get_imu(pbdrv_imu_dev_t **imu_dev) {
    *imu_dev = &global_imu_dev;

    if ((*imu_dev)->init_state == IMU_INIT_STATE_BUSY) {
        return PBIO_ERROR_AGAIN;
    }

    if ((*imu_dev)->init_state == IMU_INIT_STATE_FAILED) {
        return PBIO_ERROR_FAILED;
    }

    return PBIO_SUCCESS;
}

void pbdrv_imu_accel_read(pbdrv_imu_dev_t *imu_dev, float *values) {
    // Output is signed such that we have a right handed coordinate system where:
    // Forward acceleration is +X, upward acceleration is +Z and acceleration to the left is +Y.
    values[0] = PBDRV_CONFIG_IMU_LSM6S3TR_C_STM32_SIGN_X * imu_dev->data[3] * imu_dev->accel_scale;
    values[1] = PBDRV_CONFIG_IMU_LSM6S3TR_C_STM32_SIGN_Y * imu_dev->data[4] * imu_dev->accel_scale;
    values[2] = PBDRV_CONFIG_IMU_LSM6S3TR_C_STM32_SIGN_Z * imu_dev->data[5] * imu_dev->accel_scale;
}

void pbdrv_imu_gyro_read(pbdrv_imu_dev_t *imu_dev, float *values) {
    // Output is signed such that we have a right handed coordinate system
    // consistent with the coordinate system above. Positive rotations along
    // those axes then follow the right hand rule.
    values[0] = PBDRV_CONFIG_IMU_LSM6S3TR_C_STM32_SIGN_X * imu_dev->data[0] * imu_dev->gyro_scale;
    values[1] = PBDRV_CONFIG_IMU_LSM6S3TR_C_STM32_SIGN_Y * imu_dev->data[1] * imu_dev->gyro_scale;
    values[2] = PBDRV_CONFIG_IMU_LSM6S3TR_C_STM32_SIGN_Z * imu_dev->data[2] * imu_dev->gyro_scale;
}

void pbdrv_imu_quaternion_read(pbdrv_imu_dev_t *imu_dev, float *values) {
    values[0] = imu_dev->q[0];
    values[1] = imu_dev->q[1];
    values[2] = imu_dev->q[2];
    values[3] = imu_dev->q[3];
}

void pbdrv_imu_reset_heading(pbdrv_imu_dev_t *imu_dev) {
    imu_dev->q[0] = 1.0f;
    imu_dev->q[1] = 0.0f;
    imu_dev->q[2] = 0.0f;
    imu_dev->q[3] = 0.0f;
}

// Gathers calibration data for CalibrationTime seconds assuming the gyro is stationary, returns the bias and sets bias offsets.
void pbdrv_imu_start_gyro_calibration(pbdrv_imu_dev_t *imu_dev) {
    imu_dev->gyro_bias[0] = 0.0f;
    imu_dev->gyro_bias[1] = 0.0f;
    imu_dev->gyro_bias[2] = 0.0f;
    imu_dev->sample_count = 0;
    imu_dev->calibrating = true;
}

// Gathers calibration data for CalibrationTime seconds assuming the gyro is stationary, returns the bias and sets bias offsets.
void pbdrv_imu_stop_gyro_calibration(pbdrv_imu_dev_t *imu_dev, float *values) {
    imu_dev->calibrating = false;
    imu_dev->gyro_bias[0] /= imu_dev->sample_count;
    imu_dev->gyro_bias[1] /= imu_dev->sample_count;
    imu_dev->gyro_bias[2] /= imu_dev->sample_count;

    values[0] = imu_dev->gyro_bias[0];
    values[1] = imu_dev->gyro_bias[1];
    values[2] = imu_dev->gyro_bias[2];
}

// Gathers calibration data for CalibrationTime seconds, when the data has been collected it stores the offsets in the O
void pbdrv_imu_set_gyro_bias(pbdrv_imu_dev_t *imu_dev, float x, float y, float z) {
    imu_dev->gyro_bias[0] = x;
    imu_dev->gyro_bias[1] = y;
    imu_dev->gyro_bias[2] = z;
}

void pbdrv_imu_set_mahony_gains(pbdrv_imu_dev_t *imu_dev, float kp, float ki) {
    imu_dev->kp = kp;
    imu_dev->ki = ki;
}

#endif // PBDRV_CONFIG_IMU_LSM6S3TR_C_STM32
