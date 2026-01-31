// For 10DOF Breakout Board from DFRobot
// Contains ADXL345 Accelerometer, I2G 32000 Gyro, VCM5883L Magnetometer, BMP085 Barometer
// Prefix A, G, M, B

#define X_GY 5

// ADXL345
#define A_I2C_ADDR 0x53
#define A_CLK_SPEED 400000 // supports fast mode

// Registers
#define A_DEVID 0x00
#define A_PWR_CTRL 0x2d
#define A_DATA 0x32 // X0
#define A_DATA_FMT 0x31 // data format ('b 0000 0010) for +- 8g

// Calculations
#define A_LSB_G 64 // 64 for 10 bit with +=8g (16g) range
#define A_X_OFFSET 2.0f // default values from 24JAN unless calibration called
#define A_Y_OFFSET -4.0f
#define A_Z_OFFSET -9.0f
#define A_RAD_DEG 57.2957795131f // pi / 180

// ITG 3200
#define G_I2C_ADDR 0x68
#define G_CLK_SPEED 400000 // supports fast mode

// Registers
#define G_WHOAMI 0x00 // WHO AM I
#define G_DLPF 0x16 // 0001 1101 0x1d for 10hz LPF with 1kHz sample rate
#define G_TEMP 0x1b // 16 bit high low
#define G_DATA 0x1d // 16 bit x high x low, y high ...
#define G_PWR_MGMT 0x3e // 0x01 for clock select PLL with X Gyro Reference
#define G_LSB_DEG 14.375f  // 2^16 / 4000 */sec BUT DATASHEET 14.375

// measured 27JAN w/ 40k samples avg w/ function (for default settings)
#define G_X_OFFSET 0.409707f
#define G_Y_OFFSET 2.094100f
#define G_Z_OFFSET -1.280741f


// Overall Functions
void imu_init(i2c_master_bus_handle_t* master_bus_handle);

// ADXL345
void imu_accel_init(void);
void imu_accel_xyz(float xyz_buff[]);
void imu_accel_rp (float xyz_arr[], float rp_buff[]);

// ITG3200
void imu_gyro_init(void);
void imu_gyro_cali(void);
void imu_gyro_xyz(float xyz_buff[]);
