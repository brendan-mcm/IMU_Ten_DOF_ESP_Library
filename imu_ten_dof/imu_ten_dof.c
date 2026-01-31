#include <stdio.h>
#include <math.h>
#include "driver/i2c_master.h"
#include "imu_ten_dof.h"
#include "esp_log.h"

#define A_DEVID_VAL 0xe5 // hardcoded tes

// ID for debugging
static const char* TAG = "imu_ten_dof";

// i2c master bus handle
static i2c_master_bus_handle_t* gp_master_bus = NULL;

// ADXL345
static i2c_master_dev_handle_t g_accel_handle = NULL;
static float g_ax_offset = A_X_OFFSET;
static float g_ay_offset = A_Y_OFFSET;
static float g_az_offset = A_Z_OFFSET;

// ITG 3200
static i2c_master_dev_handle_t g_gyro_handle = NULL;
static float g_gx_offset = G_X_OFFSET;
static float g_gy_offset = G_Y_OFFSET;
static float g_gz_offset = G_Z_OFFSET;

void imu_init(i2c_master_bus_handle_t* master_bus_handle) {
    gp_master_bus = master_bus_handle;
}

void imu_accel_init(void) {
    i2c_device_config_t accel_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = A_I2C_ADDR,
        .scl_speed_hz = A_CLK_SPEED,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*gp_master_bus, &accel_cfg, &g_accel_handle));

    uint8_t write_buff[2]; // = A_DEVID; // address and byte to write
    uint8_t read_buff[6];

    write_buff[0] = A_DEVID;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(g_accel_handle, write_buff, 1, read_buff, 1, -1));
    if (read_buff[0] != A_DEVID_VAL) {
        ESP_LOGI(TAG, "Expected DEVID = 0x%x,  actual = 0x%x", A_DEVID_VAL, read_buff[0]);
    }

    write_buff[0] = A_PWR_CTRL;
    write_buff[1] = 0x00; // standby
    ESP_ERROR_CHECK(i2c_master_transmit(g_accel_handle, write_buff, 2, -1));
    write_buff[1] = 0x10; // enable autosleep
    ESP_ERROR_CHECK(i2c_master_transmit(g_accel_handle, write_buff, 2, -1));
    write_buff[1] = 0x08; // measure only
    ESP_ERROR_CHECK(i2c_master_transmit(g_accel_handle, write_buff, 2, -1));
    ESP_ERROR_CHECK(i2c_master_transmit_receive(g_accel_handle, write_buff, 1, read_buff, 1, -1));
    write_buff[0] = A_DATA_FMT;
    write_buff[1] = 0x02; // set to +- 8g
    ESP_ERROR_CHECK(i2c_master_transmit(g_accel_handle, write_buff, 2, -1));
}

void imu_accel_xyz (float xyz_buff[]) {
    uint8_t write_buff[1];
    uint8_t read_buff[6];

    write_buff[0] = A_DATA;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(g_accel_handle, write_buff, 1, read_buff, 6, -1));

    int16_t x_accel = ( ((int16_t)read_buff[1]) << 8) | read_buff[0];
    int16_t y_accel = ( ((int16_t)read_buff[3]) << 8) | read_buff[2];
    int16_t z_accel = ( ((int16_t)read_buff[5]) << 8) | read_buff[4];

    xyz_buff[0] = (x_accel - g_ax_offset) / A_LSB_G;
    xyz_buff[1] = (y_accel - g_ay_offset) / A_LSB_G;
    xyz_buff[2] = (z_accel - g_az_offset) / A_LSB_G;
}

// params: xyz arr in g, output buffer for roll pitch
void imu_accel_rp (float xyz_arr[], float rp_buff[]) {
    rp_buff[0] = atanf(xyz_arr[1] / sqrtf(xyz_arr[0] * xyz_arr[0] + xyz_arr[2] * xyz_arr[2])) * A_RAD_DEG; // roll
    rp_buff[1] = atanf(xyz_arr[0] / sqrtf(xyz_arr[1] * xyz_arr[1] + xyz_arr[2] * xyz_arr[2])) * A_RAD_DEG; // pitch
}

void imu_gyro_init(void) {
    i2c_device_config_t gyro_cfg = {
        .dev_addr_length = I2C_ADDR_BIT_LEN_7,
        .device_address = G_I2C_ADDR,
        .scl_speed_hz = G_CLK_SPEED,
    };
    ESP_ERROR_CHECK(i2c_master_bus_add_device(*gp_master_bus, &gyro_cfg, &g_gyro_handle));

    uint8_t write_buff[2];
    uint8_t read_buff[6];

    write_buff[0] = G_WHOAMI;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(g_gyro_handle, write_buff, 1, read_buff, 1, -1));
    if (read_buff[0] != G_I2C_ADDR) {
        ESP_LOGI(TAG, "Expected WHOAMI = 0x%x, actual = 0x%x", G_WHOAMI, read_buff[0]);
    }

    write_buff[0] = G_PWR_MGMT;
    write_buff[1] = 0x01; // On for all axes 
    ESP_ERROR_CHECK(i2c_master_transmit(g_gyro_handle, write_buff, 2, -1));
    write_buff[0] = G_DLPF;
    write_buff[1] = 0x1d; // 0x1d 1KHz sample with 10Hz LPF
    ESP_ERROR_CHECK(i2c_master_transmit(g_gyro_handle, write_buff, 2, -1));
}

void imu_gyro_cali(void) {
    ESP_LOGI(TAG, "Keep gyro still for 5 seconds", NULL);
    //
    uint16_t num_samples = 10000;
    // Reset offsets to 0
    g_gx_offset = 0;
    g_gy_offset = 0;
    g_gz_offset = 0;

    float accum_x = 0;
    float accum_y = 0;
    float accum_z = 0;

    float xyz_arr[3];
    for (uint16_t loop_counter = 0; loop_counter < num_samples; loop_counter++) {
        imu_gyro_xyz(xyz_arr);
        accum_x += xyz_arr[0];
        accum_y += xyz_arr[1];
        accum_z += xyz_arr[2];
    }

    g_gx_offset = accum_x / num_samples;
    g_gy_offset = accum_y / num_samples;
    g_gz_offset = accum_z / num_samples;

    ESP_LOGI(TAG, "Gyro calibration complete: x_offset = %f, y_offset = %f, z_offset = %f", g_gx_offset, g_gy_offset, g_gz_offset);

}

void imu_gyro_xyz(float xyz_buff[]) {
    uint8_t write_buff[1];
    uint8_t read_buff[6];

    write_buff[0] = G_DATA;
    ESP_ERROR_CHECK(i2c_master_transmit_receive(g_gyro_handle, write_buff, 1, read_buff, 6, -1));

    int16_t x_gyro = ( ((int16_t)read_buff[0]) << 8) | read_buff[1];
    int16_t y_gyro = ( ((int16_t)read_buff[2]) << 8) | read_buff[3];
    int16_t z_gyro = ( ((int16_t)read_buff[4]) << 8) | read_buff[5];

    xyz_buff[0] = x_gyro / G_LSB_DEG;
    xyz_buff[1] = -y_gyro / G_LSB_DEG; // define pos pitch as up
    xyz_buff[2] = z_gyro / G_LSB_DEG;

    xyz_buff[0] -= g_gx_offset;
    xyz_buff[1] -= g_gy_offset;
    xyz_buff[2] -= g_gz_offset;

    // round to nearest degree / s
    xyz_buff[0] = roundf(xyz_buff[0]);
    xyz_buff[1] = roundf(xyz_buff[1]);
    xyz_buff[2] = roundf(xyz_buff[2]);

}

