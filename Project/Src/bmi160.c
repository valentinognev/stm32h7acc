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
#include "main.h"

// #include <upm_utilities.h>

// we have to do it the old skool way.  Note, this also means that
// only one instance of the bmi160 driver can be active at a time.

// static mraa_spi_context spiContext = NULL;
// /* this is used for chip-select when using SPI */
// static mraa_gpio_context gpioContext = NULL;

// whether we are doing I2C or SPI
static bool isSPI = true;

// Our bmi160 info structure
struct bmi160_t s_bmi160;

// For SPI, these are our CS on/off functions, if needed
static void bmi160_cs_on()
{
    // if (gpioContext)        mraa_gpio_write(gpioContext, 0);
    LL_GPIO_ResetOutputPin(ACCL1_CS_GPIO_Port, ACCL1_CS_Pin);
}

static void bmi160_cs_off()
{
    // if (gpioContext)        mraa_gpio_write(gpioContext, 1);
    LL_GPIO_SetOutputPin(ACCL1_CS_GPIO_Port, ACCL1_CS_Pin);
}

// i2c bus read and write functions for use with the bmi driver code
int8_t bmi160_bus_read(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    reg_addr |= 0x80; // needed for read

    uint8_t sbuf[cnt + 1], sbufout[cnt + 1];
    memset((char *)sbuf, 0, cnt + 1);
    memset((char *)sbufout, 0, cnt + 1);
    sbuf[0] = reg_addr;

    bmi160_cs_on();
    SPI_TransmitReceive_DMA(sbuf, sbufout, cnt + 1);  
    bmi160_cs_off();

    // now copy it into user buffer
    int i;
    for (i=0; i<cnt; i++)
        reg_data[i] = sbufout[i + 1];

    return 0;
}

int8_t bmi160_bus_write(uint8_t dev_addr, uint8_t reg_addr, uint8_t *reg_data, uint8_t cnt)
{
    reg_addr &= 0x7f; // mask off 0x80 for writing

    uint8_t sbuf[cnt + 1],res[cnt + 1];
    memset((char *)sbuf, 0, cnt + 1);
    sbuf[0] = reg_addr;

    // copy in the data to write...
    int i;
    for (i=0; i<cnt; i++)
        sbuf[i + 1] = reg_data[i];

    bmi160_cs_on();
    SPI_TransmitReceive_DMA(sbuf, res, cnt + 1);
    bmi160_cs_off();

    return 0;
}

// delay for some milliseconds
void bmi160_delay_ms(uint32_t msek)
{
  HAL_Delay(msek);
}


bmi160_context bmi160_init(unsigned int bus, int address, int cs_pin,
                           bool enable_mag)
{
    bmi160_context dev = (bmi160_context)malloc(sizeof(struct _bmi160_context));

    if (!dev)
        return NULL;

    // zero out context
    memset((void *)dev, 0, sizeof(struct _bmi160_context));

    // make sure MRAA is initialized
        // we are doing SPI
    isSPI = true;

    // Only create cs context if we are actually using a valid pin.
    // A hardware controlled pin should specify cs as -1.
    // init the driver interface functions
    // Init our driver interface pointers
    if (bmi160_init_bus(&s_bmi160))
    {
        // printf("%s: bmi160_bus_init() failed.\n", __FUNCTION__);
        // bmi160_close(dev);
        return NULL;
    }

    // bmi160_init_bus will read the chip Id and deposit into our
    // interface struct.  So, check it out and make sure it's correct.
    if (s_bmi160.chip_id != BMI160_CHIP_ID)
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
    bmi160_set_command_register(ACCEL_MODE_NORMAL);

    /* bmi160_delay_ms in ms */
    bmi160_delay_ms(C_BMI160_THIRTY_U8X);

    /* Set the gyro mode as Normal write in the register 0x7E */
    bmi160_set_command_register(GYRO_MODE_NORMAL);

    /* bmi160_delay_ms in ms */
    bmi160_delay_ms(C_BMI160_THIRTY_U8X);

    /* Set the accel bandwidth as OSRS4 */
    bmi160_set_accel_bw(BMI160_ACCEL_OSR4_AVG1);
    bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

    /* Set the gryo bandwidth as Normal */
    bmi160_set_gyro_bw(BMI160_GYRO_NORMAL_MODE);
    bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

    /* set gyro data rate as 200Hz */
    bmi160_set_gyro_output_data_rate(BMI160_GYRO_OUTPUT_DATA_RATE_200HZ);
    bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

    /* set accel data rate as 200Hz */
    bmi160_set_accel_output_data_rate(BMI160_ACCEL_OUTPUT_DATA_RATE_200HZ,
                                      BMI160_ACCEL_OSR4_AVG1);
    bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

    bmi160_set_accelerometer_scale(dev, BMI160_ACC_RANGE_2G);
    bmi160_set_gyroscope_scale(dev, BMI160_GYRO_RANGE_125);

    return dev;
}

void bmi160_close(bmi160_context dev)
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

void bmi160_update(const bmi160_context dev)
{
    assert(dev != NULL);

    struct bmi160_gyro_t gyroxyz;
    struct bmi160_accel_t accelxyz;
    struct bmi160_mag_xyz_s32_t magxyz;

    // read gyro data
    bmi160_read_gyro_xyz(&gyroxyz);

    // read accel data
    bmi160_read_accel_xyz(&accelxyz);

    // read mag data
    if (dev->magEnabled)
        bmi160_bmm150_mag_compensate_xyz(&magxyz);

    // read the sensor time
    uint32_t v_sensor_time;
    bmi160_get_sensor_time(&v_sensor_time);
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

void BMI160::update() {
	int16_t IMUCount[6];                                          // used to read all 16 bytes at once from the accel/gyro
	uint8_t rawData[12];                                          // x/y/z accel register data stored here

	readBytesI2C(wire, IMUAddress, BMI160_GYR_X_L, 12, &rawData[0]);    // Read the 12 raw data registers into data array

	IMUCount[0] = ((int16_t)rawData[1] << 8) | rawData[0];		  // Turn the MSB and LSB into a signed 16-bit value
	IMUCount[1] = ((int16_t)rawData[3] << 8) | rawData[2];
	IMUCount[2] = ((int16_t)rawData[5] << 8) | rawData[4];
	IMUCount[3] = ((int16_t)rawData[7] << 8) | rawData[6];
	IMUCount[4] = ((int16_t)rawData[9] << 8) | rawData[8];
	IMUCount[5] = ((int16_t)rawData[11] << 8) | rawData[10];

	float ax, ay, az, gx, gy, gz;

	// Calculate the accel value into actual g's per second
	ax = -((float)IMUCount[3] * aRes - calibration.accelBias[0]);
	ay = -((float)IMUCount[4] * aRes - calibration.accelBias[1]);
	az = (float)IMUCount[5] * aRes - calibration.accelBias[2];

	// Calculate the gyro value into actual degrees per second
	gx = -((float)IMUCount[0] * gRes - calibration.gyroBias[0]);
	gy = -((float)IMUCount[1] * gRes - calibration.gyroBias[1]);
	gz = (float)IMUCount[2] * gRes - calibration.gyroBias[2];

	switch (geometryIndex) {
	case 0:
		accel.accelX = ax;		gyro.gyroX = gx;
		accel.accelY = ay;		gyro.gyroY = gy;
		accel.accelZ = az;		gyro.gyroZ = gz;
		break;
	case 1:
		accel.accelX = -ay;		gyro.gyroX = -gy;
		accel.accelY = ax;		gyro.gyroY = gx;
		accel.accelZ = az;		gyro.gyroZ = gz;
		break;
	case 2:
		accel.accelX = -ax;		gyro.gyroX = -gx;
		accel.accelY = -ay;		gyro.gyroY = -gy;
		accel.accelZ = az;		gyro.gyroZ = gz;
		break;
	case 3:
		accel.accelX = ay;		gyro.gyroX = gy;
		accel.accelY = -ax;		gyro.gyroY = -gx;
		accel.accelZ = az;		gyro.gyroZ = gz;
		break;
	case 4:
		accel.accelX = -az;		gyro.gyroX = -gz;
		accel.accelY = -ay;		gyro.gyroY = -gy;
		accel.accelZ = -ax;		gyro.gyroZ = -gx;
		break;
	case 5:
		accel.accelX = -az;		gyro.gyroX = -gz;
		accel.accelY = ax;		gyro.gyroY = gx;
		accel.accelZ = -ay;		gyro.gyroZ = -gy;
		break;
	case 6:
		accel.accelX = -az;		gyro.gyroX = -gz;
		accel.accelY = ay;		gyro.gyroY = gy;
		accel.accelZ = ax;		gyro.gyroZ = gx;
		break;
	case 7:
		accel.accelX = -az;		gyro.gyroX = -gz;
		accel.accelY = -ax;		gyro.gyroY = -gx;
		accel.accelZ = ay;		gyro.gyroZ = gy;
		break;
	}

	uint8_t buf[2];
	readBytesI2C(wire, IMUAddress, BMI160_TEMPERATURE_0, 2, &buf[0]);
	float temp = ((((int16_t)buf[1]) << 8) | buf[0]);
	temperature = (temp / 512) + 23.f;
}

void bmi160_set_accelerometer_scale(const bmi160_context dev,
                                    BMI160_ACC_RANGE_T scale)
{
    assert(dev != NULL);

    int8_t v_range = BMI160_ACCEL_RANGE_2G;
    // store scaling factor

    switch (scale)
    {
    case BMI160_ACC_RANGE_2G:
        v_range = BMI160_ACCEL_RANGE_2G;
        dev->accelScale = 16384.0;
        break;

    case BMI160_ACC_RANGE_4G:
        v_range = BMI160_ACCEL_RANGE_4G;
        dev->accelScale = 8192.0;
        break;

    case BMI160_ACC_RANGE_8G:
        v_range = BMI160_ACCEL_RANGE_8G;
        dev->accelScale = 4096.0;
        break;

    case BMI160_ACC_RANGE_16G:
        v_range = BMI160_ACCEL_RANGE_16G;
        dev->accelScale = 2048.0;
        break;

    default: // should never occur, but...
        dev->accelScale = 1.0;        // set a safe, though incorrect value
        printf("%s: internal error, unsupported scale.\n", __FUNCTION__);
        break;
    }

    bmi160_set_accel_range(v_range);

    return;
}

void bmi160_set_gyroscope_scale(const bmi160_context dev,
                                BMI160_GYRO_RANGE_T scale)
{
    assert(dev != NULL);

    uint8_t v_range = BMI160_GYRO_RANGE_2000_DEG_SEC;

    // store scaling factor

    switch (scale)
    {
    case BMI160_GYRO_RANGE_125:
        v_range = BMI160_GYRO_RANGE_125_DEG_SEC;
        dev->gyroScale = 262.4;
        break;

    case BMI160_GYRO_RANGE_250:
        v_range = BMI160_GYRO_RANGE_250_DEG_SEC;
        dev->gyroScale = 131.2;
        break;

    case BMI160_GYRO_RANGE_500:
        v_range = BMI160_GYRO_RANGE_500_DEG_SEC;
        dev->gyroScale = 65.6;
        break;

    case BMI160_GYRO_RANGE_1000:
        v_range = BMI160_GYRO_RANGE_1000_DEG_SEC;
        dev->gyroScale = 32.8;
        break;

    case BMI160_GYRO_RANGE_2000:
        v_range = BMI160_GYRO_RANGE_2000_DEG_SEC;
        dev->gyroScale = 16.4;
        break;

    default: // should never occur, but...
        dev->gyroScale = 1.0;        // set a safe, though incorrect value
        printf("%s: internal error, unsupported scale.\n", __FUNCTION__);
        break;
    }

    bmi160_set_gyro_range(v_range);

    return;
}

void bmi160_get_accelerometer(const bmi160_context dev, float *x, float *y,
                              float *z)
{
    assert(dev != NULL);

    if (x)
        *x = dev->accelX / dev->accelScale;

    if (y)
        *y = dev->accelY / dev->accelScale;

    if (z)
        *z = dev->accelZ / dev->accelScale;
}

void bmi160_get_gyroscope(const bmi160_context dev, float *x, float *y,
                          float *z)
{
    assert(dev != NULL);

    if (x)
        *x = dev->gyroX / dev->gyroScale;

    if (y)
        *y = dev->gyroY / dev->gyroScale;

    if (z)
        *z = dev->gyroZ / dev->gyroScale;
}

void bmi160_get_magnetometer(const bmi160_context dev, float *x, float *y,
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

void bmi160_enable_magnetometer(const bmi160_context dev, bool enable)
{
    assert(dev != NULL);

    // butchered from support example
    if (!enable)
    {
        bmi160_set_bmm150_mag_and_secondary_if_power_mode(MAG_SUSPEND_MODE);
        bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);
        bmi160_set_if_mode(0x00);
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
        bmi160_bmm150_mag_interface_init(&v_bmm_chip_id_u8);

        /* bmi160_delay_ms in ms*/
        bmi160_delay_ms(BMI160_GEN_READ_WRITE_DELAY);

        dev->magEnabled = true;
    }
}

unsigned int bmi160_get_time(const bmi160_context dev)
{
    assert(dev != NULL);

    return dev->sensorTime;
}

void BMI160::calibrateAccelGyro(calData* cal)
{
	uint8_t data[12];
	uint16_t packet_count = 64; // How many sets of full gyro and accelerometer data for averaging;
	float gyro_bias[3] = { 0, 0, 0 }, accel_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 125.f / 32768.f;			
	float  accelsensitivity = 2.f / 32768.f;

	// reset device
	writeByteI2C(wire, IMUAddress, BMI160_CMD, 0xB6); // Toggle softreset
	delay(100); // wait for reset

	writeByteI2C(wire, IMUAddress, BMI160_CMD, 0x11);  // Start up accelerometer
	delay(200);
	writeByteI2C(wire, IMUAddress, BMI160_CMD, 0x15);  // Start up gyroscope
	delay(200);								  //wait until they're done starting up...
	

	writeByteI2C(wire, IMUAddress, BMI160_ACC_RANGE, 0x03);  // Set up Accel range. +-2G
	writeByteI2C(wire, IMUAddress, BMI160_GYR_RANGE, 0x04);  // Set up Gyro range. +-125dps

	writeByteI2C(wire, IMUAddress, BMI160_ACC_CONF, 0x2A);  // Set Accel ODR to 400hz, BWP mode to Oversample 1, LPF of ~162hz
	writeByteI2C(wire, IMUAddress, BMI160_GYR_CONF, 0x2A);  // Set Gyro ODR to 400hz, BWP mode to Oversample 1, LPF of ~136hz

	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 }, gyro_temp[3] = { 0, 0, 0 };

		readBytesI2C(wire, IMUAddress, BMI160_GYR_X_L, 12, &data[0]);    // Read the 12 raw data registers into data array

		gyro_temp[0] = ((int16_t)data[1] << 8) | data[0];		  // Turn the MSB and LSB into a signed 16-bit value
		gyro_temp[1] = ((int16_t)data[3] << 8) | data[2];
		gyro_temp[2] = ((int16_t)data[5] << 8) | data[4];

		accel_temp[0] = ((int16_t)data[7] << 8) | data[6];
		accel_temp[1] = ((int16_t)data[9] << 8) | data[8];
		accel_temp[2] = ((int16_t)data[11] << 8) | data[10];


		accel_bias[0] += accel_temp[0] * accelsensitivity; // Sum individual signed 16-bit biases to get accumulated biases
		accel_bias[1] += accel_temp[1] * accelsensitivity;
		accel_bias[2] += accel_temp[2] * accelsensitivity;

		gyro_bias[0] += gyro_temp[0] * gyrosensitivity;
		gyro_bias[1] += gyro_temp[1] * gyrosensitivity;
		gyro_bias[2] += gyro_temp[2] * gyrosensitivity;

		delay(20);
	}

	accel_bias[0] /= packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= packet_count;
	accel_bias[2] /= packet_count;

	gyro_bias[0] /= packet_count;
	gyro_bias[1] /= packet_count;
	gyro_bias[2] /= packet_count;

	switch (geometryIndex) {
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