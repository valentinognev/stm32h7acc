/*
 * Author: Jon Trulson <jtrulson@ics.com>
 * Copyright (c) 2016 Intel Corporation.
 *
 * This program and the accompanying materials are made available under the
 * terms of the The MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include <unistd.h>
#include <string.h>
#include <assert.h>
// #include <mraa/i2c.h>
// #include <mraa/spi.h>
// #include <mraa/gpio.h>

#include "bmi160.h"
#include "bosch_bmi160.h"
#include "spi.h"
#include "main.h"

#define uint8_t        uint8_t

// we have to do it the old skool way.  Note, this also means that
// only one instance of the bmi160 driver can be active at a time.

// static mraa_spi_context spiContext = NULL;
// /* this is used for chip-select when using SPI */
// static mraa_gpio_context gpioContext = NULL;

// whether we are doing I2C or SPI
static bool isSPI = true;

// // Our bmi160 info structure
// struct bmi160_t s_bmi160;

calData calibration;

// For SPI, these are our CS on/off functions, if needed
static void bmi160_cs_on(const bmi160_t *bmi160)
{    // if (gpioContext)        mraa_gpio_write(gpioContext, 0);
    LL_GPIO_ResetOutputPin(bmi160->cs_port, bmi160->cs_pin);
}

static void bmi160_cs_off(const bmi160_t *bmi160)
{    // if (gpioContext)        mraa_gpio_write(gpioContext, 1);
    LL_GPIO_SetOutputPin(bmi160->cs_port, bmi160->cs_pin);
}

// i2c bus read and write functions for use with the bmi driver code
int8_t bmi160_bus_read(const bmi160_t *bmi160, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    reg_addr |= 0x80; // needed for read

    uint8_t sbuf[cnt + 1], sbufout[cnt + 1];
    memset((char *)sbuf, 0, cnt + 1);
    memset((char *)sbufout, 0, cnt + 1);
    sbuf[0] = reg_addr;

    bmi160_cs_on(bmi160);
    SPI_TransmitReceive_DMA(sbuf, sbufout, cnt + 1);  
    bmi160_cs_off(bmi160);

    // now copy it into user buffer
    int i;
    for (i=0; i<cnt; i++)
        reg_data[i] = sbufout[i + 1];

    return 0;
}

int8_t bmi160_bus_write(const bmi160_t *bmi160, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    reg_addr &= 0x7f; // mask off 0x80 for writing

    uint8_t sbuf[cnt + 1],res[cnt + 1];
    memset((char *)sbuf, 0, cnt + 1);
    sbuf[0] = reg_addr;

    // copy in the data to write...
    int i;
    for (i=0; i<cnt; i++)
        sbuf[i + 1] = reg_data[i];

    bmi160_cs_on(bmi160);
    SPI_TransmitReceive_DMA(sbuf, res, cnt + 1);
    bmi160_cs_off(bmi160);

    return 0;
}

// delay for some milliseconds
void bmi160_delay_ms(uint32_t msek)
{
  HAL_Delay(msek);
}

uint8_t bmi160_init(bmi160_context *dev, GPIO_TypeDef* cs_port, int cs_pin, bool enable_mag)
{
    // zero out context
    //memset((void *)&dev, 0, sizeof(bmi160_context));

    // Only create cs context if we are actually using a valid pin.
    // A hardware controlled pin should specify cs as -1.
    // init the driver interface functions
    // Init our driver interface pointers
    dev->bmi160.cs_pin = cs_pin;
    dev->bmi160.cs_port = cs_port;
    dev->bmi160.spi = hspi1;
    dev->bmi160.mag_manual_enable = enable_mag;

    if (bmi160_init_bus(&(dev->bmi160)))
    {
        // printf("%s: bmi160_bus_init() failed.\n", __FUNCTION__);
        // bmi160_close(dev);
        return NULL;
    }

    // bmi160_init_bus will read the chip Id and deposit into our
    // interface struct.  So, check it out and make sure it's correct.
    if (dev->bmi160.chip_id != BMI160_CHIP_ID)
    {
        // printf("%s: Error: expected chip id %02x, but got %02x.\n", __FUNCTION__, BMI160_CHIP_ID, s_bmi160.chip_id);
        // bmi160_close(dev);
        return NULL;
    }

    dev->accelScale = 1.0;
    dev->gyroScale = 1.0;
    dev->magEnabled = false;

    // This should be interesting...
    const uint32_t C_BMI160_THIRTY_U8X = 30;

    bmi160_enable_magnetometer(dev, enable_mag);

    /* Set the accel mode as Normal write in the register 0x7E */
    bmi160_set_command_register(&(dev->bmi160), ACCEL_MODE_NORMAL);

    /* bmi160_delay_ms in ms */
    bmi160_delay_ms(C_BMI160_THIRTY_U8X);

    /* Set the gyro mode as Normal write in the register 0x7E */
    bmi160_set_command_register(&(dev->bmi160), GYRO_MODE_NORMAL);

    /* bmi160_delay_ms in ms */
    bmi160_delay_ms(C_BMI160_THIRTY_U8X);

    /* Set the accel bandwidth as OSRS4 */
    bmi160_set_accel_bw(&(dev->bmi160), BMI160_ACCEL_OSR4_AVG1);
    bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

    /* Set the gryo bandwidth as Normal */
    bmi160_set_gyro_bw(&(dev->bmi160), BMI160_GYRO_NORMAL_MODE);
    bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

    /* set gyro data rate as 200Hz */
    bmi160_set_gyro_output_data_rate(&(dev->bmi160), BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
    bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

    /* set accel data rate as 200Hz */
    bmi160_set_accel_output_data_rate(&(dev->bmi160), BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ, BMI160_ACCEL_OSR4_AVG1);
    bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

    bmi160_set_accelerometer_scale(dev, BMI160_ACC_RANGE_2G);
    bmi160_set_gyroscope_scale(dev, BMI160_GYRO_RANGE_125);

    return 1;
}

void bmi160_close(bmi160_context* dev)
{
    // assert(dev != NULL);

    // if (i2cContext)
    //     mraa_i2c_stop(i2cContext);
    // i2cContext = NULL;

    // if (spiContext)
    //     mraa_spi_stop(spiContext);
    // spiContext = NULL;

    // if (gpioContext)
    //     mraa_gpio_close(gpioContext);
    // gpioContext = NULL;

    // free(dev);
}

void bmi160_update(bmi160_context* dev)
{
    assert(dev != NULL); 

    struct bmi160_gyro_t gyroxyz;
    struct bmi160_accel_t accelxyz;
    struct bmi160_mag_xyz_s32_t magxyz;

    // read gyro data
    bmi160_read_gyro_xyz(&(dev->bmi160), &gyroxyz);

    // read accel data
    bmi160_read_accel_xyz(&(dev->bmi160), &accelxyz);

    // read mag data
    if (dev->magEnabled)
        bmi160_bmm150_mag_compensate_xyz(&(dev->bmi160), &magxyz);

    // read the sensor time
    uint32_t v_sensor_time;
    bmi160_get_sensor_time(&(dev->bmi160), &v_sensor_time);
    dev->sensorTime = (unsigned int)v_sensor_time;

    dev->accelX = (float)accelxyz.x;
    dev->accelY = (float)accelxyz.y;
    dev->accelZ = (float)accelxyz.z;

    dev->gyroX = (float)gyroxyz.x;
    dev->gyroY = (float)gyroxyz.y;
    dev->gyroZ = (float)gyroxyz.z;

    if (dev->magEnabled)
    {
        dev->magX = (float)magxyz.x;
        dev->magY = (float)magxyz.y;
        dev->magZ = (float)magxyz.z;
    }
}

void bmi160_update2(bmi160_context* dev) 
{
   	float aRes = 16.0 / 32768.0;			//ares value for full range (16g) readings
	float gRes = 2000.0 / 32768.0;			//gres value for full range (2000dps) readings

	int16_t IMUCount[6];                                          // used to read all 16 bytes at once from the accel/gyro
	uint8_t rawData[12];                                          // x/y/z accel register data stored here

    struct bmi160_gyro_t gyroxyz;
    struct bmi160_accel_t accelxyz;
    struct bmi160_mag_xyz_s32_t magxyz;

    // read gyro data
    bmi160_read_gyro_xyz(&(dev->bmi160), &gyroxyz);

    // read accel data
    bmi160_read_accel_xyz(&(dev->bmi160), &accelxyz);

    // read mag data
    if (dev->magEnabled)
        bmi160_bmm150_mag_compensate_xyz(&(dev->bmi160), &magxyz);

    // read the sensor time
    uint32_t v_sensor_time;
    bmi160_get_sensor_time(&(dev->bmi160), &v_sensor_time);

	float ax, ay, az, gx, gy, gz, mx, my, mz;

	// Calculate the accel value into actual g's per second
	ax = -((float)accelxyz.x * aRes - calibration.accelBias[0]);
	ay = -((float)accelxyz.y * aRes - calibration.accelBias[1]);
	az =   (float)accelxyz.z * aRes - calibration.accelBias[2];

	// Calculate the gyro value into actual degrees per second
	gx = -((float)gyroxyz.x * gRes - calibration.gyroBias[0]);
	gy = -((float)gyroxyz.y * gRes - calibration.gyroBias[1]);
	gz =   (float)gyroxyz.z * gRes - calibration.gyroBias[2];

	// Calculate the magnetometer value into actual direction
	mx = - (float)magxyz.x;// * mRes - calibration.magBias[0]);
	my = - (float)magxyz.y;// * mRes - calibration.magBias[1]);
	mz =   (float)magxyz.z;// * mRes - calibration.magBias[2];

    uint8_t geometryIndex = 0;
	switch (geometryIndex) {
	case 0:
		dev->accelX = ax;		dev->gyroX = gx;	  dev->magX = mx;
		dev->accelY = ay;		dev->gyroY = gy;      dev->magY = my;
		dev->accelZ = az;		dev->gyroZ = gz;      dev->magZ = mz;
		break;
	case 1:
		dev->accelX = -ay;		dev->gyroX = -gy;     dev->magX = -my;
		dev->accelY = ax;		dev->gyroY = gx;      dev->magY =  mx;
		dev->accelZ = az;		dev->gyroZ = gz;      dev->magZ =  mz;
		break;
	case 2:
		dev->accelX = -ax;		dev->gyroX = -gx;     dev->magX = -mx;
		dev->accelY = -ay;		dev->gyroY = -gy;     dev->magY = -my;
		dev->accelZ = az;		dev->gyroZ = gz;      dev->magZ =  mz;
		break;
	case 3:
		dev->accelX = ay;		dev->gyroX = gy;      dev->magX =  my;
		dev->accelY = -ax;		dev->gyroY = -gx;     dev->magY = -mx;
		dev->accelZ = az;		dev->gyroZ = gz;      dev->magZ =  mz;
		break;
	case 4:
		dev->accelX = -az;		dev->gyroX = -gz;     dev->magX = -mz;
		dev->accelY = -ay;		dev->gyroY = -gy;     dev->magY = -my;
		dev->accelZ = -ax;		dev->gyroZ = -gx;     dev->magZ = -mx;
		break;
	case 5:
		dev->accelX = -az;		dev->gyroX = -gz;     dev->magX = -mz;
		dev->accelY =  ax;		dev->gyroY =  gx;     dev->magY =  mx;
		dev->accelZ = -ay;		dev->gyroZ = -gy;     dev->magZ = -my;
		break;
	case 6:
		dev->accelX = -az;		dev->gyroX = -gz;     dev->magX = -mz;
		dev->accelY = ay;		dev->gyroY = gy;      dev->magY =  my;
		dev->accelZ = ax;		dev->gyroZ = gx;      dev->magZ =  mx;
		break;
	case 7:
		dev->accelX = -az;		dev->gyroX = -gz;     dev->magX = -mz;
		dev->accelY = -ax;		dev->gyroY = -gx;     dev->magY = -mx;
		dev->accelZ = ay;		dev->gyroZ = gy;      dev->magZ =  my;
		break;
	}
}

void bmi160_set_accelerometer_scale(bmi160_context* dev,
                                    BMI160_ACC_RANGE_T scale)
{
    assert(dev != NULL);

    int8_t v_range = BMI160_ACCEL_RANGE_2G;
    // store scaling factor

    switch (scale)
    {
    case BMI160_ACC_RANGE_2G:
        v_range = BMI160_ACCEL_RANGE_2G;
        dev->accelScale = 2.f / 32768.f;			//ares value for range (2g) readings
        break;

    case BMI160_ACC_RANGE_4G:
        v_range = BMI160_ACCEL_RANGE_4G;
        dev->accelScale = 4.f / 32768.f;			//ares value for range (4g) readings
        break;

    case BMI160_ACC_RANGE_8G:
        v_range = BMI160_ACCEL_RANGE_8G;
        dev->accelScale = 8.f / 32768.f;			//ares value for range (8g) readings
        break;

    case BMI160_ACC_RANGE_16G:
        v_range = BMI160_ACCEL_RANGE_16G;
        dev->accelScale = 16.f / 32768.f;			//ares value for full range (16g) readings
        break;

    default: // should never occur, but...
        dev->accelScale = 1.0;        // set a safe, though incorrect value
        printf("%s: internal error, unsupported scale.\n", __FUNCTION__);
        break;
    }

    bmi160_set_accel_range(&(dev->bmi160), v_range);

    return;
}

void bmi160_set_gyroscope_scale(bmi160_context* dev, BMI160_GYRO_RANGE_T scale)
{
    assert(dev != NULL);

    uint8_t v_range = BMI160_GYRO_RANGE_2000_DEG_SEC;

    // store scaling factor

    switch (scale)
    {
    case BMI160_GYRO_RANGE_125:
        v_range = BMI160_GYRO_RANGE_125_DEG_SEC;
        dev->gyroScale = 125.f / 32768.f;			//ares value for range (250dps) readings
        break;

    case BMI160_GYRO_RANGE_250:
        v_range = BMI160_GYRO_RANGE_250_DEG_SEC;
        dev->gyroScale = 250.f / 32768.f;			//ares value for range (250dps) readings
        break;

    case BMI160_GYRO_RANGE_500:
        v_range = BMI160_GYRO_RANGE_500_DEG_SEC;
        dev->gyroScale = 500.f / 32768.f;			//ares value for range (500dps) readings
        break;

    case BMI160_GYRO_RANGE_1000:
        v_range = BMI160_GYRO_RANGE_1000_DEG_SEC;
        dev->gyroScale = 1000.f / 32768.f;			//ares value for range (1000dps) readings
        break;

    case BMI160_GYRO_RANGE_2000:
        v_range = BMI160_GYRO_RANGE_2000_DEG_SEC;
        dev->gyroScale = 2000.f / 32768.f;			//ares value for full range (2000dps) readings
        break;

    default: // should never occur, but...
        dev->gyroScale = 1.0;        // set a safe, though incorrect value
        printf("%s: internal error, unsupported scale.\n", __FUNCTION__);
        break;
    }

    bmi160_set_gyro_range(&(dev->bmi160), v_range);

    return;
}

void bmi160_get_accelerometer(const bmi160_context* dev, float *x, float *y,
                              float *z)
{
    assert(dev != NULL);

    if (x)
        *x = dev->accelX * dev->accelScale;

    if (y)
        *y = dev->accelY * dev->accelScale;

    if (z)
        *z = dev->accelZ * dev->accelScale;
}

void bmi160_get_gyroscope(const bmi160_context* dev, float *x, float *y, float *z)
{
    assert(dev != NULL);

    if (x)
        *x = dev->gyroX * dev->gyroScale;

    if (y)
        *y = dev->gyroY * dev->gyroScale;

    if (z)
        *z = dev->gyroZ * dev->gyroScale;
}

void bmi160_get_magnetometer(const bmi160_context* dev, float *x, float *y,
                             float *z)
{
    assert(dev != NULL);

    if (x)
        *x = dev->magX;

    if (y)
        *y = dev->magY;

    if (z)
        *z = dev->magZ;
}

#if 0
float *bmi160_get_ccelerometer()
{
    float *values = new float[3]; // x, y, and then z

    getAccelerometer(&values[0], &values[1], &values[2]);

    return values;
}

float *bmi160_getGyroscope()
{
    float *values = new float[3]; // x, y, and then z

    getGyroscope(&values[0], &values[1], &values[2]);

    return values;
}

float *bmi160_getMagnetometer()
{
    float *values = new float[3]; // x, y, and then z

    getMagnetometer(&values[0], &values[1], &values[2]);

    return values;
}
#endif

void bmi160_enable_magnetometer(bmi160_context *dev, bool enable)
{
    assert(dev != NULL);

    // butchered from support example
    if (!enable)
    {
        bmi160_set_bmm150_mag_and_secondary_if_power_mode(&(dev->bmi160), MAG_SUSPEND_MODE);
        bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);
        bmi160_set_if_mode(&(dev->bmi160), 0x00);
        bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

        dev->magEnabled = false;
        dev->magX = 0;
        dev->magY = 0;
        dev->magZ = 0;
    }
    else
    {
        uint8_t v_bmm_chip_id_u8 = BMI160_INIT_VALUE;
        /* Init the magnetometer */
        bmi160_bmm150_mag_interface_init(&(dev->bmi160), &v_bmm_chip_id_u8);

        /* bmi160_delay_ms in ms*/
        bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

        dev->magEnabled = true;
    }
}

unsigned int bmi160_get_time(const bmi160_context *dev)
{
    assert(dev != NULL);

    return dev->sensorTime;
}

void bmi160_calibrateAccelGyro(const bmi160_context *dev, calData* cal)
{
	uint8_t data[12];
	uint16_t packet_count = 64; // How many sets of full gyro and accelerometer data for averaging;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 125.f / 32768.f;			
	float  accelsensitivity = 2.f / 32768.f;

	// reset device
    uint8_t buff = 0xB6;
	bmi160_bus_write(&(dev->bmi160), BMI160_CMD_COMMANDS_ADDR, &buff, 1); // Toggle softreset
	delay(100); // wait for reset

    buff = ACCEL_MODE_NORMAL;
	bmi160_bus_write(&(dev->bmi160), BMI160_CMD_COMMANDS_ADDR, &buff, 1);  // Start up accelerometer
	delay(200);
    buff = GYRO_MODE_NORMAL;
	bmi160_bus_write(&(dev->bmi160), BMI160_CMD_COMMANDS_ADDR, &buff, 1);  // Start up gyroscope
	delay(200);								  //wait until they're done starting up...
	
    buff = BMI160_ACCEL_RANGE_2G;
	bmi160_bus_write(&(dev->bmi160), BMI160_USER_ACCEL_RANGE_ADDR, &buff, 1);  // Set up Accel range. +-2G
    buff = BMI160_GYRO_RANGE_125_DEG_SEC;
	bmi160_bus_write(&(dev->bmi160), BMI160_USER_GYRO_RANGE_ADDR, &buff, 1);  // Set up Gyro range. +-125dps

    buff = BMI160_ACCEL_OUTPUT_DATA_RATE_400HZ;
	bmi160_bus_write(&(dev->bmi160), BMI160_USER_ACCEL_CONFIG_ADDR, &buff, 1);//0x2A);  // Set Accel ODR to 400hz, BWP mode to Oversample 1, LPF of ~162hz
    buff = BMI160_GYRO_OUTPUT_DATA_RATE_400HZ;
	bmi160_bus_write(&(dev->bmi160), BMI160_USER_GYRO_CONFIG_ADDR, &buff, 1);//0x2A);  // Set Gyro ODR to 400hz, BWP mode to Oversample 1, LPF of ~136hz

    struct bmi160_accel_t accel;
    struct bmi160_gyro_t gyro;
	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		//bmi160_bus_read(&(dev->bmi160), BMI160_GYR_X_L, 12, &data[0]);    // Read the 12 raw data registers into data array
        bmi160_read_accel_xyz(&(dev->bmi160), &accel);
        bmi160_read_gyro_xyz(&(dev->bmi160), &gyro);

		// gyro_temp[0] = ((int16_t)data[1] << 8) | data[0];		  // Turn the MSB and LSB into a signed 16-bit value
		// gyro_temp[1] = ((int16_t)data[3] << 8) | data[2];
		// gyro_temp[2] = ((int16_t)data[5] << 8) | data[4];

		// accel_temp[0] = ((int16_t)data[7] << 8) | data[6];
		// accel_temp[1] = ((int16_t)data[9] << 8) | data[8];
		// accel_temp[2] = ((int16_t)data[11] << 8) | data[10];

		accel_bias[0] += accel.x * accelsensitivity; // Sum individual signed 16-bit biases to get accumulated biases
		accel_bias[1] += accel.y * accelsensitivity;
		accel_bias[2] += accel.z * accelsensitivity;

		gyro_bias[0] += gyro.x * gyrosensitivity;
		gyro_bias[1] += gyro.y * gyrosensitivity;
		gyro_bias[2] += gyro.z * gyrosensitivity;

		delay(20);
	}

	accel_bias[0] /= packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= packet_count;
	accel_bias[2] /= packet_count;

	gyro_bias[0] /= packet_count;
	gyro_bias[1] /= packet_count;
	gyro_bias[2] /= packet_count;

    uint8_t geometryIndex = 0;
	switch (geometryIndex) 
    {
	case 0:
	case 1:
	case 2:
	case 3:
		if (accel_bias[2] > 0.f) {
			accel_bias[2] -= 1.f; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[2] += 1.f;
		}
		break;
	case 4:
	case 6:
		if (accel_bias[0] > 0.f) {
			accel_bias[0] -= 1.f; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[0] += 1.f;
		}
		break;
	case 5:
	case 7:
		if (accel_bias[1] > 0.f) {
			accel_bias[1] -= 1.f; // Remove gravity from the z-axis accelerometer bias calculation
		}
		else {
			accel_bias[1] += 1.f;
		}
		break;
	}
	// Output scaled accelerometer biases for display in the main program
	cal->accelBias[0] = (float)accel_bias[0];
	cal->accelBias[1] = (float)accel_bias[1];
	cal->accelBias[2] = (float)accel_bias[2];
	// Output scaled gyro biases for display in the main program
	cal->gyroBias[0] = (float)gyro_bias[0];
	cal->gyroBias[1] = (float)gyro_bias[1];
	cal->gyroBias[2] = (float)gyro_bias[2];
	cal->valid = true;
}