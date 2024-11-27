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
#include "bmg160.h"

calData calgyro;

// macro for converting a uint8_t low/high pair into a float
#define INT16_TO_FLOAT(h, l) \
    (float)( (int16_t)( (l) | ((h) << 8) ) )

// SPI CS on and off functions
static void _csOn(const bmg160_context dev)
{
    assert(dev != NULL);

    //if (dev->gpioCS)        mraa_gpio_write(dev->gpioCS, 0);
    LL_GPIO_ResetOutputPin(GYRO1_CS_GPIO_Port, GYRO1_CS_Pin);
}

static void _csOff(const bmg160_context dev)
{
    assert(dev != NULL);

    //if (dev->gpioCS)        mraa_gpio_write(dev->gpioCS, 1);
    LL_GPIO_SetOutputPin(GYRO1_CS_GPIO_Port, GYRO1_CS_Pin);
}

// init
bmg160_context bmg160_init(int bus, int addr, int cs)
{
    bmg160_context dev = (bmg160_context)malloc(sizeof(struct _bmg160_context));

    if (!dev)
        return NULL;

    // zero out context
    memset((void *)dev, 0, sizeof(struct _bmg160_context));

    if (addr < 0)
        dev->isSPI = true;

    // Only create cs context if we are actually using a valid pin.
    // A hardware controlled pin should specify cs as -1.

    uint8_t chipID = bmg160_get_chip_id(dev);
    if (chipID != BMG160_CHIPID)
    {
        // printf("%s: invalid chip id: %02x.  Expected %02x\n",
        //        __FUNCTION__, chipID, BMG160_CHIPID);
        // bmg160_close(dev);
        return NULL;
    }
    bmg160_reset(dev);
    // call devinit with default options
    if (bmg160_devinit(dev, BMG160_POWER_MODE_NORMAL, BMG160_RANGE_2000, BMG160_BW_400_47))
    {
        // printf("%s: bmg160_devinit() failed.\n", __FUNCTION__);
        // bmg160_close(dev);
        return NULL;
    }

    return dev;
}

void bmg160_close(bmg160_context dev)
{
    // assert(dev != NULL);

    // bmg160_uninstall_isr(dev, BMG160_INTERRUPT_INT1);
    // bmg160_uninstall_isr(dev, BMG160_INTERRUPT_INT2);

    // if (dev->i2c)
    //     mraa_i2c_stop(dev->i2c);
    // if (dev->spi)
    //     mraa_spi_stop(dev->spi);
    // if (dev->gpioCS)
    //     mraa_gpio_close(dev->gpioCS);

    // free(dev);
}

void bmg160_calibrateAccel(bmg160_context dev, calData* calgyro)
{
	uint8_t data[12]; // data array to hold gyro x, y, z, data
	uint16_t packet_count = 64; // How many sets of full gyro and gyro data for averaging;
	float gyro_bias[3] = { 0, 0, 0 };

	float  gyrosensitivity = 125.f / 32768.f;			//gres value for full range (2000dps) readings (16 bit)

    bmg160_reset(dev);

    if (bmg160_set_power_mode(dev, BMG160_POWER_MODE_NORMAL) // Setting the gyro into normal mode)
    || bmg160_set_range(dev, BMG160_RANGE_125)       // setting gyro into 125dps range, maximum sensitivity
    || bmg160_set_bandwidth(dev, BMG160_BW_400_47)   // setting the gyro lpf bandwidth to 47hz ;;;;;;;;;;;; THIS LIMITS ODR TO 400HZ
    || bmg160_enable_register_shadowing(dev, true)
    || bmg160_enable_output_filtering(dev, true)
    || bmg160_fifo_config(dev, BMG160_FIFO_MODE_BYPASS, BMG160_FIFO_DATA_SEL_XYZ)
    || bmg160_set_interrupt_map1(dev, BMG160_INT_MAP_1_INT1_DATA)) // enable data ready interrupt
    {
        // printf("%s: failed to set configuration parameters.\n", __FUNCTION__);
        return UPM_ERROR_OPERATION_FAILED;
    }
   
	for (int i = 0; i < packet_count; i++)
	{
		int16_t gyro_temp[3] = { 0, 0, 0 };

		bmg160_read_regs(dev, BMG160_REG_FIFO_DATA, &data[0], 6);       // Read the 7 raw gyro data registers into data array
		
		gyro_temp[0] = ((data[1] << 8) | (data[0] & 0xF0)) >> 4;  // Form signed 16-bit integer for each sample
		gyro_temp[1] = ((data[3] << 8) | (data[2] & 0xF0)) >> 4;
		gyro_temp[2] = ((data[5] << 8) | (data[4] & 0xF0)) >> 4;

		gyro_bias[0] += gyro_temp[0] * gyrosensitivity; // Sum individual signed 16-bit biases to get accumulated biases
		gyro_bias[1] += gyro_temp[1] * gyrosensitivity;
		gyro_bias[2] += gyro_temp[2] * gyrosensitivity;
		HAL_Delay(20);
	}

	gyro_bias[0] /= packet_count; // Normalize sums to get average count biases
	gyro_bias[1] /= packet_count;
	gyro_bias[2] /= packet_count;

    int geometryIndex=3;
	switch (geometryIndex) {
	case 0:
	case 1:
	case 2:
	case 3:
		if (gyro_bias[2] > 0.f) {
			gyro_bias[2] -= 1.f; // Remove gravity from the z-axis gyro bias calculation
		}
		else {
			gyro_bias[2] += 1.f;
		}
		break;
	case 4:
	case 6:
		if (gyro_bias[0] > 0.f) {
			gyro_bias[0] -= 1.f; // Remove gravity from the z-axis gyro bias calculation
		}
		else {
			gyro_bias[0] += 1.f;
		}
		break;
	case 5:
	case 7:
		if (gyro_bias[1] > 0.f) {
			gyro_bias[1] -= 1.f; // Remove gravity from the z-axis gyro bias calculation
		}
		else {
			gyro_bias[1] += 1.f;
		}
		break;
	}
	// Output scaled gyro biases for display in the main program
	calgyro->gyroBias[0] = (float)gyro_bias[0];
	calgyro->gyroBias[1] = (float)gyro_bias[1];
	calgyro->gyroBias[2] = (float)gyro_bias[2];
}

upm_result_t bmg160_devinit(const bmg160_context dev,
                            BMG160_POWER_MODE_T pwr,
                            BMG160_RANGE_T range,
                            BMG160_BW_T bw)
{
    assert(dev != NULL);

    if (bmg160_set_power_mode(dev, pwr))
    {
        printf("%s: bmg160_set_power_mode() failed.\n", __FUNCTION__);
        return UPM_ERROR_OPERATION_FAILED;
    }

    HAL_Delay(50); // 50ms, in case we are waking up
    
    bmg160_calibrateAccel(dev, &calgyro);

    // set our range and bandwidth, make sure register shadowing is
    // enabled, enable output filtering, and set our FIFO config

    if (bmg160_set_range(dev, range)
        || bmg160_set_bandwidth(dev, bw)
        || bmg160_enable_register_shadowing(dev, true)
        || bmg160_enable_output_filtering(dev, true)
        || bmg160_fifo_config(dev, BMG160_FIFO_MODE_BYPASS,
                              BMG160_FIFO_DATA_SEL_XYZ))
    {
        // printf("%s: failed to set configuration parameters.\n", __FUNCTION__);
        return UPM_ERROR_OPERATION_FAILED;
    }

    bmg160_enable_fifo(dev, true);

    // settle
    HAL_Delay(50);

    return UPM_SUCCESS;
}

void bmg160_update(const bmg160_context dev, float *gx, float *gy, float *gz) 
{
	int16_t GyroCount[3];										  // used to read all 6 bytes at once from the BMI055 gyro
	uint8_t rawDataGyro[6];

	bmg160_read_regs(dev, BMG160_REG_FIFO_DATA, &rawDataGyro[0], 6);   // Read the 6 raw gyroscope data registers into data array

	//gyro registers
	GyroCount[0] = (rawDataGyro[1] << 8) | (rawDataGyro[0]);
	GyroCount[1] = (rawDataGyro[3] << 8) | (rawDataGyro[2]);
	GyroCount[2] = (rawDataGyro[5] << 8) | (rawDataGyro[4]);

	// Calculate the gyro value into actual degrees per second
	*gx = GyroCount[0] * (float)dev->gyrScale - calgyro.gyroBias[0];
	*gy = GyroCount[1] * (float)dev->gyrScale - calgyro.gyroBias[1];
	*gz = GyroCount[2] * (float)dev->gyrScale - calgyro.gyroBias[2];

    int geometryIndex=3;
	switch (geometryIndex) 
    {
	case 0:
		*gx = *gx;
		*gy = *gy;
		*gz = *gz;
		break;
	case 1:
		*gx = -*gy;
		*gy = *gx;
		*gz = *gz;
		break;
	case 2:
		*gx = -*gx;
		*gy = -*gy;
		*gz = *gz;
		break;
	case 3:
		*gx = *gy;
		*gy = -*gx;
		*gz = *gz;
		break;
	case 4:
		*gx = -*gz;
		*gy = -*gy;
		*gz = -*gx;
		break;
	case 5:
		*gx = -*gz;
		*gy = *gx;
		*gz = -*gy;
		break;
	case 6:
		*gx = -*gz;
		*gy = *gy;
		*gz = *gx;
		break;
	case 7:
		*gx = -*gz;
		*gy = -*gx;
		*gz = *gy;
		break;
    }
}

void bmg160_enable_fifo(const bmg160_context dev, bool useFIFO)
{
    assert(dev != NULL);

    dev->useFIFO = useFIFO;
}

uint8_t bmg160_read_reg(const bmg160_context dev, uint8_t reg)
{
    assert(dev != NULL);


    reg |= 0x80; // needed for read
    uint8_t pktin[2] = {reg, 0};
    uint8_t pktout[2] = {0, 0};

    _csOn(dev);
    SPI_TransmitReceive_DMA(&pktin, &pktout, 1);
    _csOff(dev);

    return pktout[1];
}

int bmg160_read_regs(const bmg160_context dev, uint8_t reg, uint8_t *buffer, int len)
{
    assert(dev != NULL);

    reg |= 0x80; // needed for read

    uint8_t sbuf[len + 2];
    memset((char *)sbuf, 0, sizeof(uint8_t)*(len + 2));
    sbuf[0] = reg;

    // We need to do it this way for edison - ie: use a single
    // transfer rather than breaking it up into two like we used to.
    // This means a buffer copy is now required, but that's the way
    // it goes.

    _csOn(dev);
    SPI_TransmitReceive_DMA(sbuf, sbuf, len + 1);
    _csOff(dev);

    // now copy it into user buffer
    for (int i=0; i<len; i++)
        buffer[i] = sbuf[1+i];
 
    return len;
}

upm_result_t bmg160_write_reg(const bmg160_context dev,
                              uint8_t reg, uint8_t val)
{
    assert(dev != NULL);

    reg &= 0x7f; // mask off 0x80 for writing
    uint8_t pkt[2] = {reg, val}, buf[2]={0,0};

    _csOn(dev);
    SPI_TransmitReceive_DMA(pkt, buf, 1);
    _csOff(dev);

    return UPM_SUCCESS;
}

uint8_t bmg160_get_chip_id(const bmg160_context dev)
{
    assert(dev != NULL);

    return bmg160_read_reg(dev, BMG160_REG_CHIP_ID);
}

void bmg160_get_gyroscope(const bmg160_context dev,
                          float *x, float *y, float *z)
{
    assert(dev != NULL);

    if (x)
        *x = (dev->gyrX * dev->gyrScale);

    if (y)
        *y = (dev->gyrY * dev->gyrScale);

    if (z)
        *z = (dev->gyrZ * dev->gyrScale);
}

float bmg160_get_temperature(const bmg160_context dev)
{
    assert(dev != NULL);

    return dev->temperature;
}

upm_result_t bmg160_reset(const bmg160_context dev)
{
    assert(dev != NULL);

    if (bmg160_write_reg(dev, BMG160_REG_SOFTRESET, BMG160_RESET_BYTE))
        return UPM_ERROR_OPERATION_FAILED;

    HAL_Delay(1);

    return UPM_SUCCESS;
}

upm_result_t bmg160_set_range(const bmg160_context dev,
                              BMG160_RANGE_T range)
{
    assert(dev != NULL);

    // we also have to write a fixed '0x10' to the high-order bits for
    // some reason (according to datasheet)
    uint8_t reg = range | (_BMG160_GYR_RANGE_FIXED_VALUE << _BMG160_GYR_RANGE_FIXED_SHIFT);
    if (bmg160_write_reg(dev, BMG160_REG_GYR_RANGE, reg))
        return UPM_ERROR_OPERATION_FAILED;

    switch(range)
    {
    case BMG160_RANGE_125:
        dev->gyrScale = 125.f / 32768.f; // milli-degrees
        break;

    case BMG160_RANGE_250:
        dev->gyrScale = 250.f / 32768.f;
        break;

    case BMG160_RANGE_500:
        dev->gyrScale = 500.f / 32768.f;
        break;

    case BMG160_RANGE_1000:
        dev->gyrScale = 1000.f / 32768.f;
        break;

    case BMG160_RANGE_2000:
        dev->gyrScale = 2000.f / 32768.f;
        break;
    }

    return UPM_SUCCESS;
}

upm_result_t bmg160_set_bandwidth(const bmg160_context dev,
                                  BMG160_BW_T bw)
{
    assert(dev != NULL);

    if (bmg160_write_reg(dev, BMG160_REG_GYR_BW, bw))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bmg160_set_power_mode(const bmg160_context dev,
                                   BMG160_POWER_MODE_T power)
{
    assert(dev != NULL);

    // mask off reserved bits first
    uint8_t reg = bmg160_read_reg(dev, BMG160_REG_LPM1) & ~_BMG160_LPM1_RESERVED_BITS;

    reg &= ~(_BMG160_LPM1_POWER_MODE_MASK << _BMG160_LPM1_POWER_MODE_SHIFT);
    reg |= (power << _BMG160_LPM1_POWER_MODE_SHIFT);

//    if (bmg160_write_reg(dev, BMG160_REG_LPM1, power))
    if (bmg160_write_reg(dev, BMG160_REG_LPM1, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bmg160_fifo_set_watermark(const bmg160_context dev, int wm)
{
    assert(dev != NULL);

    // mask off illegal values
    uint8_t reg = ((uint8_t)wm) & _BMG160_FIFO_CONFIG_0_WATER_MARK_MASK;

    if (bmg160_write_reg(dev, BMG160_REG_FIFO_CONFIG_0, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bmg160_fifo_config(const bmg160_context dev,
                                BMG160_FIFO_MODE_T mode,
                                BMG160_FIFO_DATA_SEL_T axes)
{
    assert(dev != NULL);

    uint8_t reg = ( (mode << _BMG160_FIFO_CONFIG_1_FIFO_MODE_SHIFT) |
                    (axes << _BMG160_FIFO_CONFIG_1_FIFO_DATA_SHIFT) );

    if (bmg160_write_reg(dev, BMG160_REG_FIFO_CONFIG_1, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bmg160_get_interrupt_enable0(const bmg160_context dev)
{
    assert(dev != NULL);

    return (bmg160_read_reg(dev, BMG160_REG_INT_EN_0)
            & ~_BMG160_INT_EN_0_RESERVED_BITS);
}

upm_result_t bmg160_set_interrupt_enable0(const bmg160_context dev,
                                          uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMG160_INT_EN_0_RESERVED_BITS;

    if (bmg160_write_reg(dev, BMG160_REG_INT_EN_0, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bmg160_get_interrupt_map0(const bmg160_context dev)
{
    assert(dev != NULL);

    return bmg160_read_reg(dev, BMG160_REG_INT_MAP_0)
        & ~_BMG160_INT_MAP_0_RESERVED_BITS;
}

upm_result_t bmg160_set_interrupt_map0(const bmg160_context dev, uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMG160_INT_MAP_0_RESERVED_BITS;

    if (bmg160_write_reg(dev, BMG160_REG_INT_MAP_0, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bmg160_get_interrupt_map1(const bmg160_context dev)
{
    assert(dev != NULL);

    return bmg160_read_reg(dev, BMG160_REG_INT_MAP_1);
}

upm_result_t bmg160_set_interrupt_map1(const bmg160_context dev, uint8_t bits)
{
    assert(dev != NULL);

    if (bmg160_write_reg(dev, BMG160_REG_INT_MAP_1, bits))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

// REG_INT_EN1, for some strange reason
uint8_t bmg160_get_interrupt_src(const bmg160_context dev)
{
    assert(dev != NULL);

    return (bmg160_read_reg(dev, BMG160_REG_INT_EN_1)
            & ~_BMG160_INT_EN_1_INT1_RESERVED_BITS);
}

upm_result_t bmg160_set_interrupt_src(const bmg160_context dev, uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMG160_INT_EN_1_INT1_RESERVED_BITS;

    if (bmg160_write_reg(dev, BMG160_REG_INT_EN_1, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bmg160_get_interrupt_output_control(const bmg160_context dev)
{
    assert(dev != NULL);

    return (bmg160_read_reg(dev, BMG160_REG_INT_EN_1)
            & ~_BMG160_INT_EN_1_INT1_RESERVED_BITS);
}

upm_result_t bmg160_set_interrupt_output_control(const bmg160_context dev,
                                                 uint8_t bits)
{
    assert(dev != NULL);

    uint8_t reg = bits & ~_BMG160_INT_EN_1_INT1_RESERVED_BITS;

    if (bmg160_write_reg(dev, BMG160_REG_INT_EN_1, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bmg160_clear_interrupt_latches(const bmg160_context dev)
{
    assert(dev != NULL);

    uint8_t reg = bmg160_read_reg(dev, BMG160_REG_INT_RST_LATCH) & ~_BMG160_INT_RST_LATCH_RESERVED_BITS;

    reg |= BMG160_INT_RST_LATCH_RESET_INT;

    if (bmg160_write_reg(dev, BMG160_REG_INT_RST_LATCH, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

BMG160_RST_LATCH_T bmg160_get_interrupt_latch_behavior(const bmg160_context dev)
{
    assert(dev != NULL);

    uint8_t reg = bmg160_read_reg(dev, BMG160_REG_INT_RST_LATCH)
        & ~_BMG160_INT_RST_LATCH_RESERVED_BITS;

    reg &= (_BMG160_INT_RST_LATCH_MASK << _BMG160_INT_RST_LATCH_SHIFT);

    return (BMG160_RST_LATCH_T)reg;
}

upm_result_t bmg160_set_interrupt_latch_behavior(const bmg160_context dev,
                                                 BMG160_RST_LATCH_T latch)
{
    assert(dev != NULL);

    uint8_t reg = bmg160_read_reg(dev, BMG160_REG_INT_RST_LATCH) & ~_BMG160_INT_RST_LATCH_RESERVED_BITS;

    reg &= ~(_BMG160_INT_RST_LATCH_MASK << _BMG160_INT_RST_LATCH_SHIFT);
    reg |= (latch << _BMG160_INT_RST_LATCH_SHIFT);

    if (bmg160_write_reg(dev, BMG160_REG_INT_RST_LATCH, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bmg160_enable_register_shadowing(const bmg160_context dev,
                                              bool shadow)
{
    assert(dev != NULL);

    uint8_t reg = bmg160_read_reg(dev, BMG160_REG_RATE_HBW) & ~_BMG160_RATE_HBW_RESERVED_BITS;

    if (shadow)
        reg &= ~BMG160_RATE_HBW_SHADOW_DIS;
    else
        reg |= BMG160_RATE_HBW_SHADOW_DIS;

    if (bmg160_write_reg(dev, BMG160_REG_RATE_HBW, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

upm_result_t bmg160_enable_output_filtering(const bmg160_context dev,
                                            bool filter)
{
    assert(dev != NULL);

    uint8_t reg = bmg160_read_reg(dev, BMG160_REG_RATE_HBW) & ~_BMG160_RATE_HBW_RESERVED_BITS;

    if (filter)
        reg &= ~BMG160_RATE_HBW_DATA_HIGH_BW;
    else
        reg |= BMG160_RATE_HBW_DATA_HIGH_BW;

    if (bmg160_write_reg(dev, BMG160_REG_RATE_HBW, reg))
        return UPM_ERROR_OPERATION_FAILED;

    return UPM_SUCCESS;
}

uint8_t bmg160_get_interrupt_status0(const bmg160_context dev)
{
    assert(dev != NULL);

    return (bmg160_read_reg(dev, BMG160_REG_INT_STATUS_0)
            & ~_BMG160_INT_STATUS_0_RESERVED_BITS);
}

uint8_t bmg160_get_interrupt_status1(const bmg160_context dev)
{
    assert(dev != NULL);

    return (bmg160_read_reg(dev, BMG160_REG_INT_STATUS_1)
            & ~_BMG160_INT_STATUS_1_RESERVED_BITS);
}

uint8_t bmg160_get_interrupt_status2(const bmg160_context dev)
{
    assert(dev != NULL);

    return (bmg160_read_reg(dev, BMG160_REG_INT_STATUS_2)
            & ~_BMG160_INT_STATUS_2_RESERVED_BITS);
}

uint8_t bmg160_get_interrupt_status3(const bmg160_context dev)
{
    assert(dev != NULL);

    return (bmg160_read_reg(dev, BMG160_REG_INT_STATUS_3)
            & ~_BMG160_INT_STATUS_3_RESERVED_BITS);
}

// upm_result_t bmg160_install_isr(const bmg160_context dev,
//                                 BMG160_INTERRUPT_PINS_T intr, int gpio,
//                                 mraa_gpio_edge_t level,
//                                 void (*isr)(void *), void *arg)
// {
//     assert(dev != NULL);

//     // delete any existing ISR and GPIO context for this interrupt
//     bmg160_uninstall_isr(dev, intr);

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
//     case BMG160_INTERRUPT_INT1:
//         dev->gpio1 = gpio_isr;
//         break;

//     case BMG160_INTERRUPT_INT2:
//         dev->gpio2 = gpio_isr;
//         break;
//     }

//     return UPM_SUCCESS;
// }

// void bmg160_uninstall_isr(const bmg160_context dev,
//                           BMG160_INTERRUPT_PINS_T intr)
// {
//     assert(dev != NULL);

//     switch (intr)
//     {
//     case BMG160_INTERRUPT_INT1:
//         if (dev->gpio1)
//         {
//             mraa_gpio_isr_exit(dev->gpio1);
//             mraa_gpio_close(dev->gpio1);
//             dev->gpio1 = NULL;
//         }
//         break;

//     case BMG160_INTERRUPT_INT2:
//         if (dev->gpio2)
//         {
//             mraa_gpio_isr_exit(dev->gpio2);
//             mraa_gpio_close(dev->gpio2);
//             dev->gpio2 = NULL;
//         }
//         break;
//     }
// }
