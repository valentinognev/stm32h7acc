/*
 * Author: Jon Trulson <jtrulson@ics.com>
 * Copyright (c) 2017 Intel Corporation.
 *
 * The MIT License
 *
 * This program and the accompanying materials are made available under the
 * terms of the The MIT License which is available at
 * https://opensource.org/licenses/MIT.
 *
 * SPDX-License-Identifier: MIT
 */

#include <unistd.h>
#include <assert.h>

//#include "upm_utilities.h"
#include "bma250e.h"
#include "main.h"

calData calacc;

extern SPI_HandleTypeDef hspi1;

// macro for converting a uint8_t low/high pair into a float
#define INT16_TO_FLOAT(h, l) \
    (float)( (int16_t)( (l) | ((h) << 8) ) )

// SPI CS on and off functions
static void _csOn(const bma250e_context dev)
{
    assert(dev != NULL);

    // if (dev->gpioCS)        mraa_gpio_write(dev->gpioCS, 0);
    LL_GPIO_ResetOutputPin(ACCL1_CS_GPIO_Port, ACCL1_CS_Pin);
}

static void _csOff(const bma250e_context dev)
{
    assert(dev != NULL);

    // if (dev->gpioCS)        mraa_gpio_write(dev->gpioCS, 1);
    LL_GPIO_SetOutputPin(ACCL1_CS_GPIO_Port, ACCL1_CS_Pin);
}

// init
bma250e_context bma250e_init(int bus, int addr, int cs)
{
    bma250e_context dev = (bma250e_context)malloc(sizeof(struct _bma250e_context));

    if (!dev)
        return NULL;

    // zero out context
    memset((void *)dev, 0, sizeof(struct _bma250e_context));

    if (addr < 0)
        dev->isSPI = true;

    // check the chip id
    uint16_t chipID = bma250e_get_chip_id(dev);  

    // check the various chips id's and set appropriate capabilities.
    // Bail if the chip id is unknown.
    switch (chipID)
    {
    case 0xf9: // standalone bma250e
        dev->resolution = BMA250E_RESOLUTION_10BITS;
        dev->fifoAvailable = true;

        break;

    case 0xfa: // bmx055, bmi055 variants, 12b resolution
        dev->resolution = BMA250E_RESOLUTION_12BITS;
        dev->fifoAvailable = true;

        break;

    case 0x03: // bmc050 variant, no FIFO, 12b resolution
        dev->resolution = BMA250E_RESOLUTION_12BITS;
        dev->fifoAvailable = false;

        break;

    default:
        // printf("%s: invalid chip id: %02x.  Expected f9, fa, or 03\n",  __FUNCTION__, chipID);
        bma250e_close(dev);
        return NULL;
    }

    bma250e_reset(dev);
 
    // call devinit with default options
    if (bma250e_devinit(dev, BMA250E_POWER_MODE_NORMAL, BMA250E_RANGE_16G, BMA250E_BW_62_5))
    {
        printf("%s: bma250e_devinit() failed.\n", __FUNCTION__);
        bma250e_close(dev);
        return NULL;
    }

    return dev;
}

void bma250e_close(bma250e_context dev)
{
    // assert(dev != NULL);

    // bma250e_uninstall_isr(dev, BMA250E_INTERRUPT_INT1);
    // bma250e_uninstall_isr(dev, BMA250E_INTERRUPT_INT2);

    // if (dev->i2c)
    //     mraa_i2c_stop(dev->i2c);
    // if (dev->spi)
    //     mraa_spi_stop(dev->spi);
    // if (dev->gpioCS)
    //     mraa_gpio_close(dev->gpioCS);

    // free(dev);
}

void bma250e_calibrateAccel(bma250e_context dev, calData* calacc)
{
	uint8_t data[12]; // data array to hold accelerometer and gyro x, y, z, data
	uint16_t packet_count = 64; // How many sets of full gyro and accelerometer data for averaging;
	float accel_bias[3] = { 0, 0, 0 };

	float  accelsensitivity = 2.f / 2048.f;				//ares value for full range (16g) readings (12 bit)

    bma250e_reset(dev);

    if (bma250e_set_power_mode(dev, BMA250E_POWER_MODE_NORMAL) // Setting the accelerometer into normal mode)
    || bma250e_set_range(dev, BMA250E_RANGE_2G)   // Setting accelerometer into 2g range, maximum sensitivity
    || bma250e_set_bandwidth(dev, BMA250E_BW_125)              // Setting the accelerometer lpf bandwidth to 125hz
    || bma250e_enable_register_shadowing(dev, true)
    || bma250e_enable_output_filtering(dev, true)
    || bma250e_fifo_config(dev, BMA250E_FIFO_MODE_BYPASS, BMA250E_FIFO_DATA_SEL_XYZ)
    || bma250e_set_interrupt_map1(dev, BMA250E_INT_MAP_1_INT1_DATA)
    || bma250e_set_interrupt_enable1(dev, BMA250E_INT_STATUS_1_DATA)) // enable data ready interrupt
    {
        // printf("%s: failed to set configuration parameters.\n", __FUNCTION__);
        return UPM_ERROR_OPERATION_FAILED;
    }
   
	for (int i = 0; i < packet_count; i++)
	{
		int16_t accel_temp[3] = { 0, 0, 0 };
        memset(data, 0, 12);

		bma250e_read_regs(dev, BMA250E_REG_FIFO_DATA, &data[0], 6);       // Read the 7 raw accelerometer data registers into data array
		
		accel_temp[0] = ((data[1] << 8) | (data[0] & 0xF0)) >> 4;  // Form signed 16-bit integer for each sample
		accel_temp[1] = ((data[3] << 8) | (data[2] & 0xF0)) >> 4;
		accel_temp[2] = ((data[5] << 8) | (data[4] & 0xF0)) >> 4;

        float x = accel_temp[0] * accelsensitivity;
        float y = accel_temp[1] * accelsensitivity;
        float z = accel_temp[2] * accelsensitivity;
		accel_bias[0] += accel_temp[0] * accelsensitivity; // Sum individual signed 16-bit biases to get accumulated biases
		accel_bias[1] += accel_temp[1] * accelsensitivity;
		accel_bias[2] += accel_temp[2] * accelsensitivity;
		HAL_Delay(20);
	}

	accel_bias[0] /= packet_count; // Normalize sums to get average count biases
	accel_bias[1] /= packet_count;
	accel_bias[2] /= packet_count;

    int geometryIndex=3;
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
	calacc->accelBias[0] = (float)accel_bias[0];
	calacc->accelBias[1] = (float)accel_bias[1];
	calacc->accelBias[2] = (float)accel_bias[2];
}

upm_result_t bma250e_devinit(const bma250e_context dev, BMA250E_POWER_MODE_T pwr, 
                                 BMA250E_RANGE_T range, BMA250E_BW_T bw)
{
    assert(dev != NULL);

    bma250e_calibrateAccel(dev, &calacc);

    if (bma250e_set_power_mode(dev, pwr))
    {
        printf("%s: bma250e_set_power_mode() failed.\n", __FUNCTION__);
        return UPM_ERROR_OPERATION_FAILED;
    }

    HAL_Delay(50); // 50ms, in case we are waking up

    // set our range and bandwidth, make sure register shadowing is
    // enabled, enable output filtering, and set our FIFO config
    if (bma250e_set_range(dev, range)
        || bma250e_set_bandwidth(dev, bw)
        || bma250e_enable_register_shadowing(dev, true)
        || bma250e_enable_output_filtering(dev, true)
        || bma250e_fifo_config(dev, BMA250E_FIFO_MODE_BYPASS, BMA250E_FIFO_DATA_SEL_XYZ)
        || bma250e_set_interrupt_map1(dev, BMA250E_INT_MAP_1_INT1_DATA)
        || bma250e_set_interrupt_enable1(dev, BMA250E_INT_STATUS_1_DATA)) // enable data ready interrupt
    {
        // printf("%s: failed to set configuration parameters.\n", __FUNCTION__);
        return UPM_ERROR_OPERATION_FAILED;
    }

     bma250e_enable_fifo(dev, true);

    // make sure low power mode LPM2 is enabled in case we go to low
    // power or suspend mode. LPM1 mode (the default) requires register
    // writes to be drastically slowed down when enabled, which we
    // cannot handle.
    //bma250e_set_low_power_mode2(dev);

    uint8_t testRange = bma250e_read_reg(dev, BMA250E_REG_PMU_RANGE);
    uint8_t testBandwidth = bma250e_read_reg(dev, BMA250E_REG_PMU_BW);
    uint8_t testShadowFilter = bma250e_read_reg(dev, BMA250E_REG_ACC_HBW);
    uint8_t testFIFOConfig = bma250e_read_reg(dev, BMA250E_REG_FIFO_CONFIG_0);
    uint8_t testIntMap1 = bma250e_get_interrupt_map1(dev);
    uint8_t testInt1 = bma250e_get_interrupt_enable1(dev);


    // settle
    HAL_Delay(50);

    return UPM_SUCCESS;
}

upm_result_t bma250e_update(const bma250e_context dev)
{
    assert(dev != NULL);

    int bufLen = 7; // max, non-FIFO
    uint8_t startReg = BMA250E_REG_ACCD_X_LSB;

    if (dev->useFIFO)
    {
        bufLen = 6;
        startReg = BMA250E_REG_FIFO_DATA;
    }

    uint8_t buf[bufLen];

    if (bma250e_read_regs(dev, startReg, buf, bufLen) != bufLen)
    {
        // printf("%s: bma250e_read_regs() failed to read %d bytes\n", __FUNCTION__, bufLen);
        return UPM_ERROR_OPERATION_FAILED;
    }

    uint8_t mask = 0, shift = 0;
    float divisor = 1;

    switch (dev->resolution)
    {
    case BMA250E_RESOLUTION_10BITS:
        mask = _BMA250E_ACCD10_LSB_MASK;
        shift = _BMA250E_ACCD10_LSB_SHIFT;
        divisor = 64.0;

        break;

    case BMA250E_RESOLUTION_12BITS:
        mask = _BMA250E_ACCD12_LSB_MASK;
        shift = _BMA250E_ACCD12_LSB_SHIFT;
        divisor = 16.0;

        break;
    }

    // x                       msb     lsb
    dev->accX = INT16_TO_FLOAT(buf[1], (buf[0] & (mask << shift)));
    // y
    dev->accY = INT16_TO_FLOAT(buf[3], (buf[2] & (mask << shift)));
    // z
    dev->accZ = INT16_TO_FLOAT(buf[5], (buf[4] & (mask << shift)));

    dev->accX/=divisor;
    dev->accY/=divisor;
    dev->accZ/=divisor;
    // get the temperature...

    int8_t temp = 0;
    if (dev->useFIFO)
    {
        // we have to read temperature separately...
        temp = (int8_t)bma250e_read_reg(dev, BMA250E_REG_TEMP);
    }
    else
    {
        // we already got it
        temp = (int8_t)buf[6];
    }

    // .5K/LSB, 23C center point
    dev->temperature = ((float)temp / 2.0) + 23.0;

    return UPM_SUCCESS;
}


void bma250e_update2(const bma250e_context dev, float *ax, float *ay, float *az, float *temperature) 
{
	int16_t AccelCount[3];                                        // used to read all 6 bytes at once from the BMI055 accel
	uint8_t rawDataAccel[7];                                          // x/y/z accel register data stored here

	bma250e_read_regs(dev, BMA250E_REG_FIFO_DATA, &rawDataAccel[0], 7);       // Read the 7 raw accelerometer data registers into data array
    uint8_t testRange = bma250e_read_reg(dev, BMA250E_REG_PMU_RANGE);
    
	//accel registers
	AccelCount[0] = ((rawDataAccel[1] << 8) | (rawDataAccel[0] & 0xF0)) >> 4;		  // Turn the MSB and LSB into a signed 12-bit value
	AccelCount[1] = ((rawDataAccel[3] << 8) | (rawDataAccel[2] & 0xF0)) >> 4;	      // praise sign extension, making this code clean and simple.
	AccelCount[2] = ((rawDataAccel[5] << 8) | (rawDataAccel[4] & 0xF0)) >> 4;

	// Calculate the accel value into actual g's per second
    float ax_ = AccelCount[0] * (float)dev->accScale;
    float ay_ = AccelCount[1] * (float)dev->accScale;
    float az_ = AccelCount[2] * (float)dev->accScale;
	*ax = AccelCount[0] * (float)dev->accScale - calacc.accelBias[0];
	*ay = AccelCount[1] * (float)dev->accScale - calacc.accelBias[1];
	*az = AccelCount[2] * (float)dev->accScale - calacc.accelBias[2];

    int geometryIndex=3;
	switch (geometryIndex) {
	case 0:
		*ax = *ax;
		*ay = *ay;
		*az = *az;
		break;
	case 1:
		*ax = -*ay;
		*ay = *ax;
		*az = *az;
		break;
	case 2:
		*ax = -*ax;
		*ay = -*ay;
		*az = *az;
		break;
	case 3:
		*ax = *ay;
		*ay = -*ax;
		*az = *az;
		break;
	case 4:
		*ax = -*az;
		*ay = -*ay;
		*az = -*ax;
		break;
	case 5:
		*ax = -*az;
		*ay = *ax;
		*az = -*ay;
		break;
	case 6:
		*ax = -*az;
		*ay = *ay;
		*az = *ax;
		break;
	case 7:
		*ax = -*az;
		*ay = -*ax;
		*az = *ay;
		break;
	}

	// Calculate the temperature value into actual deg c
	*temperature = -((rawDataAccel[6] * -0.5f) * (86.5f - -40.5f) / (float)(128.f) - 40.5f) - 20.f;
}

void bma250e_enable_fifo(const bma250e_context dev, bool useFIFO)
{
    assert(dev != NULL);

    if (dev->fifoAvailable)
        dev->useFIFO = useFIFO;
}

uint8_t bma250e_read_reg(const bma250e_context dev, const uint8_t reg_)
{
    assert(dev != NULL);

    uint8_t reg = reg_| 0x80; // needed for read
    uint8_t pktin[2] = {reg, 0};
    uint8_t pktout[2] = {0, 0};

    _csOn(dev);
    SPI_TransmitReceive_DMA(pktin, pktout, 2);
    _csOff(dev);

    return pktout[1];
}

int8_t bma250e_read_regs(const bma250e_context dev, const uint8_t reg_, int8_t *buffer, int len)
{
    assert(dev != NULL);

    uint8_t reg = reg_| 0x80; // needed for read

    uint8_t sbuf[len + 2];
    memset((uint8_t *)sbuf, 0, sizeof(uint8_t)*(len + 2));
    sbuf[0] = reg;

    _csOn(dev);
    SPI_TransmitReceive_DMA(sbuf, sbuf, len + 1);
    _csOff(dev);

    // now copy it into user buffer
    for (int i=0; i<len; i++)
        buffer[i] = sbuf[1+i];

    return len;
}

upm_result_t bma250e_write_reg(const bma250e_context dev,
                               uint8_t reg, uint8_t val)
{
    assert(dev != NULL);

    reg &= 0x7f; // mask off 0x80 for writing
    uint8_t pkt[2] = {reg, val}, buf[2]={0,0};

    _csOn(dev);
    SPI_TransmitReceive_DMA(pkt, buf, 2);
    _csOff(dev);

    return UPM_SUCCESS;
}

uint8_t bma250e_get_chip_id(const bma250e_context dev)
{
    assert(dev != NULL);

    return bma250e_read_reg(dev, BMA250E_REG_CHIP_ID);
}

void bma250e_get_accelerometer(const bma250e_context dev,
                               float *x, float *y, float *z)
{
    assert(dev != NULL);

    if (x)
        *x = (dev->accX * dev->accScale) / 1000.0;

    if (y)
        *y = (dev->accY * dev->accScale) / 1000.0;

    if (z)
        *z = (dev->accZ * dev->accScale) / 1000.0;
}

float bma250e_get_temperature(const bma250e_context dev)
{
    assert(dev != NULL);

    return dev->temperature;
}

upm_result_t bma250e_reset(const bma250e_context dev)
{
    assert(dev != NULL);

    if (bma250e_write_reg(dev, BMA250E_REG_SOFTRESET, BMA250E_RESET_BYTE))
        return UPM_ERROR_OPERATION_FAILED;

    HAL_Delay(1);

    return UPM_SUCCESS;
}

upm_result_t bma250e_set_range(const bma250e_context dev,
                               BMA250E_RANGE_T range)
{
    assert(dev != NULL);

    if (bma250e_write_reg(dev, BMA250E_REG_PMU_RANGE, range))
        return UPM_ERROR_OPERATION_FAILED;

    switch (dev->resolution)
    {
    case BMA250E_RESOLUTION_10BITS:
        switch(range)
        {
        case BMA250E_RANGE_2G:
            dev->accScale = 2.f/512.f; // milli-gravities
            break;

        case BMA250E_RANGE_4G:
            dev->accScale = 4.f/512.f;
            break;

        case BMA250E_RANGE_8G:
            dev->accScale = 8.f/512.f;
            break;

        case BMA250E_RANGE_16G:
            dev->accScale = 16.f/512.f;
            break;
        }

        break;

    case BMA250E_RESOLUTION_12BITS:
        switch(range)
        {
        case BMA250E_RANGE_2G:
            dev->accScale = 2.f/2048.f; // milli-gravities
            break;

        case BMA250E_RANGE_4G:
            dev->accScale = 4.f/2048.f;
            break;

        case BMA250E_RANGE_8G:
            dev->accScale = 8.f/2048.f;
            break;

        case BMA250E_RANGE_16G:
            dev->accScale = 16.f/2048.f;
            break;
        }

        break;
    }

    return UPM_SUCCESS;
}

upm_result_t bma250e_set_bandwidth(const bma250e_context dev,
                                   BMA250E_BW_T bw)
{
    assert(dev != NULL);

    if (bma250e_write_reg(dev, BMA250E_REG_PMU_BW, bw))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bma250e_set_power_mode(const bma250e_context dev,
                                    BMA250E_POWER_MODE_T power)
{
    assert(dev != NULL);

    // mask off reserved bits first
    uint8_t reg = bma250e_read_reg(dev, BMA250E_REG_PMU_LPW) & ~_BMA250E_PMU_LPW_RESERVED_MASK;

    reg &= ~(_BMA250E_PMU_LPW_POWER_MODE_MASK << _BMA250E_PMU_LPW_POWER_MODE_SHIFT);
    reg |= (power << _BMA250E_PMU_LPW_POWER_MODE_SHIFT);

//    if (bma250e_write_reg(dev, BMA250E_REG_PMU_LPW, power))
    if (bma250e_write_reg(dev, BMA250E_REG_PMU_LPW, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bma250e_fifo_set_watermark(const bma250e_context dev, int wm)
{
    assert(dev != NULL);

    if (!dev->fifoAvailable)
        return UPM_ERROR_NOT_SUPPORTED;

    // mask off illegal values
    uint8_t reg = ((uint8_t)wm) & _BMA250E_FIFO_CONFIG_0_WATER_MARK_MASK;

    if (bma250e_write_reg(dev, BMA250E_REG_FIFO_CONFIG_0, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bma250e_fifo_config(const bma250e_context dev,
                                 BMA250E_FIFO_MODE_T mode,
                                 BMA250E_FIFO_DATA_SEL_T axes)
{
    assert(dev != NULL);

    if (!dev->fifoAvailable)
        return UPM_ERROR_NOT_SUPPORTED;

    uint8_t reg = ( (mode << _BMA250E_FIFO_CONFIG_1_FIFO_MODE_SHIFT) |
                    (axes << _BMA250E_FIFO_CONFIG_1_FIFO_DATA_SHIFT) );

    if (bma250e_write_reg(dev, BMA250E_REG_FIFO_CONFIG_1, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bma250e_set_self_test(const bma250e_context dev,
                                   bool sign, bool amp,
                                   BMA250E_SELFTTEST_AXIS_T axis)
{
    assert(dev != NULL);

    uint8_t reg = (axis << _BMA250E_PMU_SELFTTEST_AXIS_SHIFT);

    if (amp)
        reg |= BMA250E_PMU_SELFTTEST_AMP;

    if (sign)
        reg |= BMA250E_PMU_SELFTTEST_SIGN;

    if (bma250e_write_reg(dev, BMA250E_REG_PMU_SELFTEST, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_enable0(const bma250e_context dev)
{
    assert(dev != NULL);

    return (bma250e_read_reg(dev, BMA250E_REG_INT_EN_0)
            & ~_BMA250E_INT_EN_0_RESERVED_BITS);
}

upm_result_t bma250e_set_interrupt_enable0(const bma250e_context dev,
                                           uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMA250E_INT_EN_0_RESERVED_BITS;

    if (bma250e_write_reg(dev, BMA250E_REG_INT_EN_0, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_enable1(const bma250e_context dev)
{
    assert(dev != NULL);

    return (bma250e_read_reg(dev, BMA250E_REG_INT_EN_1) & ~_BMA250E_INT_EN_1_RESERVED_BITS);
}

upm_result_t bma250e_set_interrupt_enable1(const bma250e_context dev,
                                           uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMA250E_INT_EN_1_RESERVED_BITS;

    if (bma250e_write_reg(dev, BMA250E_REG_INT_EN_1, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_enable2(const bma250e_context dev)
{
    assert(dev != NULL);

    return (bma250e_read_reg(dev, BMA250E_REG_INT_EN_2)
            & ~_BMA250E_INT_EN_2_RESERVED_BITS);
}

upm_result_t bma250e_set_interrupt_enable2(const bma250e_context dev,
                                           uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMA250E_INT_EN_2_RESERVED_BITS;

    if (bma250e_write_reg(dev, BMA250E_REG_INT_EN_2, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_map0(const bma250e_context dev)
{
    assert(dev != NULL);

    return bma250e_read_reg(dev, BMA250E_REG_INT_MAP_0);
}

upm_result_t bma250e_set_interrupt_map0(const bma250e_context dev, uint8_t bits)
{
    assert(dev != NULL);

    if (bma250e_write_reg(dev, BMA250E_REG_INT_MAP_0, bits))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_map1(const bma250e_context dev)
{
    assert(dev != NULL);

    return (bma250e_read_reg(dev, BMA250E_REG_INT_MAP_1) & ~_BMA250E_INT_MAP_1_INT1_RESERVED_BITS);
}

upm_result_t bma250e_set_interrupt_map1(const bma250e_context dev, uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMA250E_INT_MAP_1_INT1_RESERVED_BITS;

    if (bma250e_write_reg(dev, BMA250E_REG_INT_MAP_1, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_map2(const bma250e_context dev)
{
    assert(dev != NULL);

    return bma250e_read_reg(dev, BMA250E_REG_INT_MAP_2);
}

upm_result_t bma250e_set_interrupt_map2(const bma250e_context dev, uint8_t bits)
{
    assert(dev != NULL);

    if (bma250e_write_reg(dev, BMA250E_REG_INT_MAP_2, bits))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_src(const bma250e_context dev)
{
    assert(dev != NULL);

    return (bma250e_read_reg(dev, BMA250E_REG_INT_SRC)
            & ~_BMA250E_INT_SRC_RESERVED_BITS);
}

upm_result_t bma250e_set_interrupt_src(const bma250e_context dev, uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMA250E_INT_SRC_RESERVED_BITS;

    if (bma250e_write_reg(dev, BMA250E_REG_INT_SRC, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_output_control(const bma250e_context dev)
{
    assert(dev != NULL);

    return (bma250e_read_reg(dev, BMA250E_REG_INT_OUT_CTRL)
            & ~_BMA250E_INT_OUT_CTRL_INT1_RESERVED_BITS);
}

upm_result_t bma250e_set_interrupt_output_control(const bma250e_context dev,
                                                  uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMA250E_INT_OUT_CTRL_INT1_RESERVED_BITS;

    if (bma250e_write_reg(dev, BMA250E_REG_INT_OUT_CTRL, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bma250e_clear_interrupt_latches(const bma250e_context dev)
{
    assert(dev != NULL);

    uint8_t reg =
        (bma250e_read_reg(dev, BMA250E_REG_INT_RST_LATCH)
         & ~_BMA250E_INT_RST_LATCH_RESERVED_BITS);

    reg |= BMA250E_INT_RST_LATCH_RESET_INT;

    if (bma250e_write_reg(dev, BMA250E_REG_INT_RST_LATCH, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

BMA250E_RST_LATCH_T bma250e_get_interrupt_latch_behavior(
    const bma250e_context dev)
{
    assert(dev != NULL);

    uint8_t reg = (bma250e_read_reg(dev, BMA250E_REG_INT_RST_LATCH)
                   & ~_BMA250E_INT_RST_LATCH_RESERVED_BITS);

    reg &= (_BMA250E_INT_RST_LATCH_MASK << _BMA250E_INT_RST_LATCH_SHIFT);

    return (BMA250E_RST_LATCH_T)reg;
}

upm_result_t bma250e_set_interrupt_latch_behavior(const bma250e_context dev,
                                                  BMA250E_RST_LATCH_T latch)
{
    assert(dev != NULL);

    uint8_t reg =
        (bma250e_read_reg(dev, BMA250E_REG_INT_RST_LATCH)
         & ~_BMA250E_INT_RST_LATCH_RESERVED_BITS);

    reg &= ~(_BMA250E_INT_RST_LATCH_MASK << _BMA250E_INT_RST_LATCH_SHIFT);
    reg |= (latch << _BMA250E_INT_RST_LATCH_SHIFT);

    if (bma250e_write_reg(dev, BMA250E_REG_INT_RST_LATCH, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bma250e_enable_register_shadowing(const bma250e_context dev, bool shadow)
{
    assert(dev != NULL);

    uint8_t reg = (bma250e_read_reg(dev, BMA250E_REG_ACC_HBW) & ~_BMA250E_ACC_HBW_RESERVED_BITS);

    if (shadow)
        reg &= ~BMA250E_ACC_HBW_SHADOW_DIS;
    else
        reg |= BMA250E_ACC_HBW_SHADOW_DIS;

    if (bma250e_write_reg(dev, BMA250E_REG_ACC_HBW, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bma250e_enable_output_filtering(const bma250e_context dev,
                                             bool filter)
{
    assert(dev != NULL);

    uint8_t reg = (bma250e_read_reg(dev, BMA250E_REG_ACC_HBW) & ~_BMA250E_ACC_HBW_RESERVED_BITS);

    if (filter)
        reg &= ~BMA250E_ACC_HBW_DATA_HIGH_BW;
    else
        reg |= BMA250E_ACC_HBW_DATA_HIGH_BW;

    if (bma250e_write_reg(dev, BMA250E_REG_ACC_HBW, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bma250e_get_interrupt_status0(const bma250e_context dev)
{
    assert(dev != NULL);

    return bma250e_read_reg(dev, BMA250E_REG_INT_STATUS_0);
}

uint8_t bma250e_get_interrupt_status1(const bma250e_context dev)
{
    assert(dev != NULL);

    return (bma250e_read_reg(dev, BMA250E_REG_INT_STATUS_1) & ~_BMA250E_INT_STATUS_1_RESERVED_BITS);
}

uint8_t bma250e_get_interrupt_status2(const bma250e_context dev)
{
    assert(dev != NULL);

    return bma250e_read_reg(dev, BMA250E_REG_INT_STATUS_2);
}

uint8_t bma250e_get_interrupt_status3_bits(const bma250e_context dev)
{
    assert(dev != NULL);

    // filter out the orientation bitfield..
    return (bma250e_read_reg(dev, BMA250E_REG_INT_STATUS_3) & ~(_BMA250E_INT_STATUS_3_ORIENT_MASK << _BMA250E_INT_STATUS_3_ORIENT_SHIFT));
}

BMA250E_ORIENT_T bma250e_get_interrupt_status3_orientation(
    const bma250e_context dev)
{
    assert(dev != NULL);

    // grab just the orientation bitfield
    uint8_t reg = (bma250e_read_reg(dev, BMA250E_REG_INT_STATUS_3)
                   & (_BMA250E_INT_STATUS_3_ORIENT_MASK
                      << _BMA250E_INT_STATUS_3_ORIENT_SHIFT));

    reg >>= _BMA250E_INT_STATUS_3_ORIENT_SHIFT;

    return (BMA250E_ORIENT_T)reg;
}

upm_result_t bma250e_set_low_power_mode2(const bma250e_context dev)
{
    assert(dev != NULL);

    uint8_t reg = (bma250e_read_reg(dev, BMA250E_REG_PMU_LOW_POWER)
                   & ~_BMA250E_LOW_POWER_RESERVED_BITS);

    // we simply set the low power mode to 2.  Low power mode 1 slows
    // down register write accesses, and we can't handle that.  In the
    // words of the late Admiral Akbar: "We cannot handle firepower of
    // that magnitude!" :(

    reg |= BMA250E_LOW_POWER_LOWPOWER_MODE;

    if (bma250e_write_reg(dev, BMA250E_REG_PMU_LOW_POWER, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

// upm_result_t bma250e_install_isr(const bma250e_context dev,
//                                  BMA250E_INTERRUPT_PINS_T intr, int gpio,
//                                  mraa_gpio_edge_t level,
//                                  void (*isr)(void *), void *arg)
// {
//     assert(dev != NULL);

//     // delete any existing ISR and GPIO context for this interrupt
//     bma250e_uninstall_isr(dev, intr);

//     mraa_gpio_context gpio_isr = NULL;

//     // create gpio context
//     if (!(gpio_isr = mraa_gpio_init(gpio)))
//     {
//         printf("%s: mraa_gpio_init() failed.\n", __FUNCTION__);
//         return UPM_ERROR_OPERATION_FAILED;
//     }

//     mraa_gpio_dir(gpio_isr, MRAA_GPIO_IN);

//     if (mraa_gpio_isr(gpio_isr, level, isr, arg))
//     {
//         mraa_gpio_close(gpio_isr);
//         printf("%s: mraa_gpio_isr() failed.\n", __FUNCTION__);
//         return UPM_ERROR_OPERATION_FAILED;
//     }

//     switch (intr)
//     {
//     case BMA250E_INTERRUPT_INT1:
//         dev->gpio1 = gpio_isr;
//         break;

//     case BMA250E_INTERRUPT_INT2:
//         dev->gpio2 = gpio_isr;
//         break;
//     }

//     return UPM_SUCCESS;
// }

// void bma250e_uninstall_isr(const bma250e_context dev,
//                            BMA250E_INTERRUPT_PINS_T intr)
// {
//     assert(dev != NULL);

//     switch (intr)
//     {
//     case BMA250E_INTERRUPT_INT1:
//         if (dev->gpio1)
//         {
//             mraa_gpio_isr_exit(dev->gpio1);
//             mraa_gpio_close(dev->gpio1);
//             dev->gpio1 = NULL;
//         }
//         break;

//     case BMA250E_INTERRUPT_INT2:
//         if (dev->gpio2)
//         {
//             mraa_gpio_isr_exit(dev->gpio2);
//             mraa_gpio_close(dev->gpio2);
//             dev->gpio2 = NULL;
//         }
//         break;
//     }
// }
