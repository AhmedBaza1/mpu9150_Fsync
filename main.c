#include <stdio.h>
#include <unistd.h>
#include <signal.h>
#include <stdbool.h>
#include "mpu9150_driver.h"

static volatile bool running = true;

void signal_handler(int signum) {
    running = false;
}

int main() {
    // Set up signal handling
    signal(SIGINT, signal_handler);
    signal(SIGTERM, signal_handler);

    // Configure MPU-9150
    mpu9150_config_t config = {
        .gyro_fs = GYRO_FS_250DPS,
        .accel_fs = ACCEL_FS_2G,
        .dlpf_bandwidth = DLPF_BW_44HZ,
        .sample_rate_div = 0,     // 1kHz sample rate
        .fsync_config = FSYNC_GYRO_XOUT,
        .interrupt_enabled = true,
        .fifo_enabled = true
    };

    // Initialize sensor
    int result = mpu9150_init(&config);
    if (result != MPU9150_SUCCESS) {
        printf("Failed to initialize MPU-9150: %s\n", 
               mpu9150_get_error_string(result));
        return -1;
    }

    printf("MPU-9150 initialized successfully\n");
    printf("FSYNC configured for GYRO_XOUT synchronization\n");
    printf("Sample rate: 1kHz\n");
    printf("Press Ctrl+C to exit\n\n");

    mpu9150_data_t sensor_data;
    uint32_t sample_count = 0;
    uint32_t start_time = get_timestamp_ms();

    // Main reading loop
    while (running) {
        result = mpu9150_read_sensor_data(&sensor_data);
        if (result != MPU9150_SUCCESS) {
            printf("Failed to read sensor data: %s\n", 
                   mpu9150_get_error_string(result));
            break;
        }

        sample_count++;
        
        // Print data every 100ms
        if ((sample_count % 100) == 0) {
            printf("Accel (g): X=%.2f Y=%.2f Z=%.2f | ", 
                   sensor_data.accel[0], sensor_data.accel[1], sensor_data.accel[2]);
            printf("Gyro (°/s): X=%.2f Y=%.2f Z=%.2f | ", 
                   sensor_data.gyro[0], sensor_data.gyro[1], sensor_data.gyro[2]);
            printf("Temp: %.1f°C\n", sensor_data.temp);
        }

        // Maintain approximately 1kHz sample rate
        usleep(900);  // Sleep for 900μs (allowing 100μs for processing)
    }

    // Calculate actual sample rate
    uint32_t elapsed_time = get_timestamp_ms() - start_time;
    float actual_rate = (float)sample_count / (elapsed_time / 1000.0f);
    printf("\nActual sample rate: %.2f Hz\n", actual_rate);

    mpu9150_close();
    return 0;
}