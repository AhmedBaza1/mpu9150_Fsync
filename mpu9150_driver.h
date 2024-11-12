#ifndef MPU9150_DRIVER_H
#define MPU9150_DRIVER_H

#define _POSIX_C_SOURCE 200809L 

#include <stdint.h>
#include <stdbool.h>
#include <time.h>   

// Error codes
#define MPU9150_SUCCESS        0
#define MPU9150_ERR_I2C      -1
#define MPU9150_ERR_INIT     -2
#define MPU9150_ERR_CONFIG   -3
#define MPU9150_ERR_SELFTEST -4
#define MPU9150_ERR_READ     -5
#define MPU9150_ERR_WRITE    -6
#define MPU9150_ERR_TIMEOUT  -7

// MPU-9150 Register Map
#define MPU9150_ADDR            0x68
#define MPU9150_WHO_AM_I       0x75
#define MPU9150_SMPLRT_DIV     0x19
#define MPU9150_CONFIG         0x1A
#define MPU9150_GYRO_CONFIG    0x1B
#define MPU9150_ACCEL_CONFIG   0x1C
#define MPU9150_FIFO_EN        0x23
#define MPU9150_INT_PIN_CFG    0x37
#define MPU9150_INT_ENABLE     0x38
#define MPU9150_INT_STATUS     0x3A
#define MPU9150_ACCEL_XOUT_H   0x3B
#define MPU9150_TEMP_OUT_H     0x41
#define MPU9150_GYRO_XOUT_H    0x43
#define MPU9150_PWR_MGMT_1     0x6B
#define MPU9150_PWR_MGMT_2     0x6C

// FSYNC Configuration Options
#define FSYNC_DISABLED          0
#define FSYNC_TEMP_OUT         1
#define FSYNC_GYRO_XOUT        2
#define FSYNC_GYRO_YOUT        3
#define FSYNC_GYRO_ZOUT        4
#define FSYNC_ACCEL_XOUT       5
#define FSYNC_ACCEL_YOUT       6
#define FSYNC_ACCEL_ZOUT       7

// Gyroscope Full Scale Range Options
typedef enum {
    GYRO_FS_250DPS  = 0,
    GYRO_FS_500DPS  = 1,
    GYRO_FS_1000DPS = 2,
    GYRO_FS_2000DPS = 3
} gyro_fs_sel_t;

// Accelerometer Full Scale Range Options
typedef enum {
    ACCEL_FS_2G  = 0,
    ACCEL_FS_4G  = 1,
    ACCEL_FS_8G  = 2,
    ACCEL_FS_16G = 3
} accel_fs_sel_t;

// DLPF Bandwidth Options
typedef enum {
    DLPF_BW_256HZ = 0,
    DLPF_BW_188HZ = 1,
    DLPF_BW_98HZ  = 2,
    DLPF_BW_44HZ  = 3,
    DLPF_BW_21HZ  = 4,
    DLPF_BW_10HZ  = 5,
    DLPF_BW_5HZ   = 6
} dlpf_bandwidth_t;

// Sensor Data Structure
typedef struct {
    float accel[3];    // X, Y, Z acceleration in g
    float gyro[3];     // X, Y, Z angular velocity in degrees/s
    float mag[3];      // X, Y, Z magnetic field in μT
    float temp;        // Temperature in °C
    uint32_t timestamp; // Timestamp in milliseconds
} mpu9150_data_t;

// Configuration Structure
typedef struct {
    gyro_fs_sel_t gyro_fs;
    accel_fs_sel_t accel_fs;
    dlpf_bandwidth_t dlpf_bandwidth;
    uint8_t sample_rate_div;
    uint8_t fsync_config;
    bool interrupt_enabled;
    bool fifo_enabled;
} mpu9150_config_t;

// Function declarations
int mpu9150_init(const mpu9150_config_t *config);
int mpu9150_configure_fsync(uint8_t ext_sync_set);
int mpu9150_set_sample_rate(uint8_t rate);
int mpu9150_set_gyro_full_scale(gyro_fs_sel_t fs_sel);
int mpu9150_set_accel_full_scale(accel_fs_sel_t fs_sel);
int mpu9150_set_dlpf_bandwidth(dlpf_bandwidth_t bandwidth);
int mpu9150_enable_interrupts(bool enable);
int mpu9150_read_sensor_data(mpu9150_data_t *data);
int mpu9150_self_test(void);
int mpu9150_reset(void);
void mpu9150_close(void);
uint32_t get_timestamp_ms(void);

// Error handling
const char* mpu9150_get_error_string(int error_code);

#endif