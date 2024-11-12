#define _POSIX_C_SOURCE 200809L 

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <linux/i2c-dev.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <time.h>
#include <string.h>
#include <errno.h>
#include "mpu9150_driver.h"

static int i2c_fd = -1;
static float gyro_scale = 1.0f;
static float accel_scale = 1.0f;
static uint32_t last_read_time = 0;

// Error codes
#define MPU9150_SUCCESS        0
#define MPU9150_ERR_I2C      -1
#define MPU9150_ERR_INIT     -2
#define MPU9150_ERR_CONFIG   -3
#define MPU9150_ERR_SELFTEST -4

// I2C helper functions with error handling
static int i2c_write(uint8_t reg, uint8_t data) {
    uint8_t buf[2] = {reg, data};
    if (write(i2c_fd, buf, 2) != 2) {
        return MPU9150_ERR_I2C;
    }
    return MPU9150_SUCCESS;
}

static int i2c_read(uint8_t reg, uint8_t *data, size_t len) {
    if (write(i2c_fd, &reg, 1) != 1) {
        return MPU9150_ERR_I2C;
    }
    if (read(i2c_fd, data, len) != len) {
        return MPU9150_ERR_I2C;
    }
    return MPU9150_SUCCESS;
}

// Get current timestamp in milliseconds
static uint32_t get_timestamp_ms(void) {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return (ts.tv_sec * 1000) + (ts.tv_nsec / 1000000);
}

int mpu9150_init(const mpu9150_config_t *config) {
    // Open I2C device
    i2c_fd = open("/dev/i2c-1", O_RDWR);
    if (i2c_fd < 0) {
        return MPU9150_ERR_INIT;
    }

    // Set I2C slave address
    if (ioctl(i2c_fd, I2C_SLAVE, MPU9150_ADDR) < 0) {
        close(i2c_fd);
        return MPU9150_ERR_INIT;
    }

    // Reset device
    if (mpu9150_reset() != MPU9150_SUCCESS) {
        return MPU9150_ERR_INIT;
    }

    // Wait for reset to complete
    usleep(100000);  // 100ms

    // Verify device ID
    uint8_t who_am_i;
    if (i2c_read(MPU9150_WHO_AM_I, &who_am_i, 1) < 0 || who_am_i != 0x68) {
        return MPU9150_ERR_INIT;
    }

    // Wake up device
    if (i2c_write(MPU9150_PWR_MGMT_1, 0x00) < 0) {
        return MPU9150_ERR_INIT;
    }

    // Apply configuration
    if (config != NULL) {
        if (mpu9150_set_gyro_full_scale(config->gyro_fs) < 0 ||
            mpu9150_set_accel_full_scale(config->accel_fs) < 0 ||
            mpu9150_set_dlpf_bandwidth(config->dlpf_bandwidth) < 0 ||
            mpu9150_set_sample_rate(config->sample_rate_div) < 0 ||
            mpu9150_configure_fsync(config->fsync_config) < 0) {
            return MPU9150_ERR_CONFIG;
        }

        // Configure interrupts and FIFO if requested
        if (config->interrupt_enabled) {
            mpu9150_enable_interrupts(true);
        }
        
        if (config->fifo_enabled) {
            i2c_write(MPU9150_FIFO_EN, 0xF8);  // Enable accel, gyro, temp
        }
    }

    return MPU9150_SUCCESS;
}

int mpu9150_configure_fsync(uint8_t ext_sync_set) {
    uint8_t config;
    
    // Read current CONFIG register
    if (i2c_read(MPU9150_CONFIG, &config, 1) < 0) {
        return MPU9150_ERR_CONFIG;
    }

    // Clear and set EXT_SYNC_SET bits (bits 5:3)
    config &= ~(0x07 << 3);
    config |= (ext_sync_set & 0x07) << 3;

    // Write back to CONFIG register
    if (i2c_write(MPU9150_CONFIG, config) < 0) {
        return MPU9150_ERR_CONFIG;
    }

    return MPU9150_SUCCESS;
}

int mpu9150_set_sample_rate(uint8_t rate) {
    if (i2c_write(MPU9150_SMPLRT_DIV, rate) < 0) {
        return MPU9150_ERR_CONFIG;
    }
    return MPU9150_SUCCESS;
}

int mpu9150_read_sensor_data(mpu9150_data_t *data) {
    uint8_t raw_data[14];
    
    if (data == NULL) {
        return MPU9150_ERR_CONFIG;
    }

    // Read accelerometer, temperature and gyroscope data
    if (i2c_read(MPU9150_ACCEL_XOUT_H, raw_data, 14) < 0) {
        return MPU9150_ERR_I2C;
    }

    // Convert raw data to physical values
    data->accel[0] = (float)((int16_t)((raw_data[0] << 8) | raw_data[1])) * accel_scale;
    data->accel[1] = (float)((int16_t)((raw_data[2] << 8) | raw_data[3])) * accel_scale;
    data->accel[2] = (float)((int16_t)((raw_data[4] << 8) | raw_data[5])) * accel_scale;

    // Temperature conversion (datasheet formula)
    int16_t temp_raw = (int16_t)((raw_data[6] << 8) | raw_data[7]);
    data->temp = (float)temp_raw / 340.0f + 36.53f;

    data->gyro[0] = (float)((int16_t)((raw_data[8] << 8) | raw_data[9])) * gyro_scale;
    data->gyro[1] = (float)((int16_t)((raw_data[10] << 8) | raw_data[11])) * gyro_scale;
    data->gyro[2] = (float)((int16_t)((raw_data[12] << 8) | raw_data[13])) * gyro_scale;

    // Update timestamp
    data->timestamp = get_timestamp_ms();
    last_read_time = data->timestamp;

    return MPU9150_SUCCESS;
}

int mpu9150_reset(void) {
    // Set reset bit in PWR_MGMT_1 register
    if (i2c_write(MPU9150_PWR_MGMT_1, 0x80) < 0) {
        return MPU9150_ERR_CONFIG;
    }
    return MPU9150_SUCCESS;
}

void mpu9150_close(void) {
    if (i2c_fd >= 0) {
        close(i2c_fd);
        i2c_fd = -1;
    }
}

const char* mpu9150_get_error_string(int error_code) {
    switch (error_code) {
        case MPU9150_SUCCESS:
            return "Success";
        case MPU9150_ERR_I2C:
            return "I2C communication error";
        case MPU9150_ERR_INIT:
            return "Initialization error";
        case MPU9150_ERR_CONFIG:
            return "Configuration error";
        case MPU9150_ERR_SELFTEST:
            return "Self-test error";
        default:
            return "Unknown error";
    }
}