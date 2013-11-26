/*
* Copyright (C) 2012 Invensense, Inc.
*
* This software is licensed under the terms of the GNU General Public
* License version 2, as published by the Free Software Foundation, and
* may be copied, distributed, and modified under those terms.
*
* This program is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
* GNU General Public License for more details.
*
*/

/**
 *  @addtogroup  DRIVERS
 *  @brief       Hardware drivers.
 *
 *  @{
 *      @file    inv_slave_bma250.c
 *      @brief   A sysfs device driver for Invensense devices
<<<<<<< HEAD
 *      @details This file is part of invensense mpu driver code
=======
 *      @details This file is part of inv_gyro driver code
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
 *
 */

#include <linux/module.h>
#include <linux/init.h>
#include <linux/slab.h>
#include <linux/i2c.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sysfs.h>
#include <linux/jiffies.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/kfifo.h>
#include <linux/poll.h>
#include <linux/miscdevice.h>
#include <linux/spinlock.h>

#include "inv_mpu_iio.h"
<<<<<<< HEAD
#define BMA250_CHIP_ID			3
#define BMA250_RANGE_SET		0
#define BMA250_BW_SET			4

/* range and bandwidth */
#define BMA250_RANGE_2G                 3
#define BMA250_RANGE_4G                 5
#define BMA250_RANGE_8G                 8
#define BMA250_RANGE_16G                12
#define BMA250_RANGE_MAX                4
#define BMA250_RANGE_MASK               0xF0

#define BMA250_BW_7_81HZ        0x08
#define BMA250_BW_15_63HZ       0x09
#define BMA250_BW_31_25HZ       0x0A
#define BMA250_BW_62_50HZ       0x0B
#define BMA250_BW_125HZ         0x0C
#define BMA250_BW_250HZ         0x0D
#define BMA250_BW_500HZ         0x0E
#define BMA250_BW_1000HZ        0x0F
#define BMA250_MAX_BW_SIZE      8
#define BMA250_BW_REG_MASK      0xE0

/*      register definitions */
#define BMA250_X_AXIS_LSB_REG                   0x02
#define BMA250_RANGE_SEL_REG                    0x0F
#define BMA250_BW_SEL_REG                       0x10
#define BMA250_MODE_CTRL_REG                    0x11

/* mode settings */
#define BMA250_MODE_NORMAL     0
#define BMA250_MODE_LOWPOWER   1
#define BMA250_MODE_SUSPEND    2
#define BMA250_MODE_MAX        3
#define BMA250_MODE_MASK       0x3F
#define BMA250_BIT_SUSPEND     0x80
#define BMA250_BIT_LP          0x40
=======
#define BMA250_CHIP_ID			(3)
#define BMA250_RANGE_SET		(0)
#define BMA250_BW_SET			(4)

/* range and bandwidth */

#define BMA250_RANGE_2G                 (3)
#define BMA250_RANGE_4G                 (5)
#define BMA250_RANGE_8G                 (8)
#define BMA250_RANGE_16G                (12)
#define BMA250_RANGE_MAX                (4)
#define BMA250_RANGE_MASK               (0xF0)

#define BMA250_BW_7_81HZ        (0x08)
#define BMA250_BW_15_63HZ       (0x09)
#define BMA250_BW_31_25HZ       (0x0A)
#define BMA250_BW_62_50HZ       (0x0B)
#define BMA250_BW_125HZ         (0x0C)
#define BMA250_BW_250HZ         (0x0D)
#define BMA250_BW_500HZ         (0x0E)
#define BMA250_BW_1000HZ        (0x0F)
#define BMA250_MAX_BW_SIZE      (8)
#define BMA250_BW_REG_MASK      (0xE0)

/*      register definitions */
#define BMA250_X_AXIS_LSB_REG                   (0x02)
#define BMA250_RANGE_SEL_REG                    (0x0F)
#define BMA250_BW_SEL_REG                       (0x10)
#define BMA250_MODE_CTRL_REG                    (0x11)

/* mode settings */
#define BMA250_MODE_NORMAL      (0)
#define BMA250_MODE_LOWPOWER    (1)
#define BMA250_MODE_SUSPEND     (2)
#define BMA250_MODE_MAX         (3)
#define BMA250_MODE_MASK        (0x3F)
#define BMA250_BIT_SUSPEND     (0x80)
#define BMA250_BIT_LP          (0x40)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

struct bma_property {
	int range;
	int bandwidth;
	int mode;
};

static struct bma_property bma_static_property = {
	.range = BMA250_RANGE_SET,
	.bandwidth = BMA250_BW_SET,
	.mode = BMA250_MODE_SUSPEND
};

<<<<<<< HEAD
static int bma250_set_bandwidth(struct inv_mpu_iio_s *st, u8 bw)
{
	int res;
	u8 data;
	int bandwidth;
	switch (bw) {
	case 0:
		bandwidth = BMA250_BW_7_81HZ;
		break;
	case 1:
		bandwidth = BMA250_BW_15_63HZ;
		break;
	case 2:
		bandwidth = BMA250_BW_31_25HZ;
		break;
	case 3:
		bandwidth = BMA250_BW_62_50HZ;
		break;
	case 4:
		bandwidth = BMA250_BW_125HZ;
		break;
	case 5:
		bandwidth = BMA250_BW_250HZ;
		break;
	case 6:
		bandwidth = BMA250_BW_500HZ;
		break;
	case 7:
		bandwidth = BMA250_BW_1000HZ;
		break;
	default:
		return -EINVAL;
=======
static int bma250_set_bandwidth(struct inv_gyro_state_s *st, unsigned char BW)
{
	int res = 0;
	unsigned char data;
	int Bandwidth = 0;
	if (BW >= BMA250_MAX_BW_SIZE)
		return -1;
	switch (BW) {
	case 0:
		Bandwidth = BMA250_BW_7_81HZ;
		break;
	case 1:
		Bandwidth = BMA250_BW_15_63HZ;
		break;
	case 2:
		Bandwidth = BMA250_BW_31_25HZ;
		break;
	case 3:
		Bandwidth = BMA250_BW_62_50HZ;
		break;
	case 4:
		Bandwidth = BMA250_BW_125HZ;
		break;
	case 5:
		Bandwidth = BMA250_BW_250HZ;
		break;
	case 6:
		Bandwidth = BMA250_BW_500HZ;
		break;
	case 7:
		Bandwidth = BMA250_BW_1000HZ;
		break;
	default:
		break;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	}
	res = inv_secondary_read(BMA250_BW_SEL_REG, 1, &data);
	if (res)
		return res;
	data &= BMA250_BW_REG_MASK;
<<<<<<< HEAD
	data |= bandwidth;
=======
	data |= Bandwidth;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	res = inv_secondary_write(BMA250_BW_SEL_REG, data);
	return res;
}

<<<<<<< HEAD
static int bma250_set_range(struct inv_mpu_iio_s *st, u8 range)
{
	int res;
	u8 orig, data;
	switch (range) {
=======
static int bma250_set_range(struct inv_gyro_state_s *st, unsigned char Range)
{
	int res = 0;
	unsigned char orig, data = 0;
	if (Range >= BMA250_RANGE_MAX)
		return -1;
	switch (Range) {
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	case 0:
		data  = BMA250_RANGE_2G;
		break;
	case 1:
		data  = BMA250_RANGE_4G;
		break;
	case 2:
		data  = BMA250_RANGE_8G;
		break;
	case 3:
		data  = BMA250_RANGE_16G;
		break;
	default:
<<<<<<< HEAD
		return -EINVAL;
=======
		break;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	}
	res = inv_secondary_read(BMA250_RANGE_SEL_REG, 1, &orig);
	if (res)
		return res;
	orig &= BMA250_RANGE_MASK;
	data |= orig;
	res = inv_secondary_write(BMA250_RANGE_SEL_REG, data);
<<<<<<< HEAD
	if (res)
		return res;
	bma_static_property.range = range;

	return 0;
}

static int setup_slave_bma250(struct inv_mpu_iio_s *st)
{
	int result;
	u8 data[2];
	result = set_3050_bypass(st, true);
=======
	bma_static_property.range = Range;
	return res;
}

static int setup_slave_bma250(struct inv_gyro_state_s *st)
{
	int result;
	unsigned char data[2];
	result = set_3050_bypass(st, 1);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (result)
		return result;
	/*read secondary i2c ID register */
	result = inv_secondary_read(0, 1, data);
	if (result)
		return result;
	if (BMA250_CHIP_ID != data[0])
<<<<<<< HEAD
		return -EINVAL;
	result = set_3050_bypass(st, false);
=======
		return result;
	result = set_3050_bypass(st, 0);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (result)
		return result;
	/*AUX(accel), slave address is set inside set_3050_bypass*/
	/* bma250 x axis LSB register address is 2 */
	result = inv_i2c_single_write(st, REG_3050_AUX_BST_ADDR,
					BMA250_X_AXIS_LSB_REG);
<<<<<<< HEAD

	return result;
}

static int bma250_set_mode(struct inv_mpu_iio_s *st, u8 mode)
{
	int res;
	u8 data;

=======
	return result;
}

static int bma250_set_mode(struct inv_gyro_state_s *st, unsigned char Mode)
{
	int res = 0;
	unsigned char data = 0;

	if (Mode >= BMA250_RANGE_MASK)
		return -1;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	res = inv_secondary_read(BMA250_MODE_CTRL_REG, 1, &data);
	if (res)
		return res;
	data &= BMA250_MODE_MASK;
<<<<<<< HEAD
	switch (mode) {
=======
	switch (Mode) {
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	case BMA250_MODE_NORMAL:
		break;
	case BMA250_MODE_LOWPOWER:
		data |= BMA250_BIT_LP;
		break;
	case BMA250_MODE_SUSPEND:
		data |= BMA250_BIT_SUSPEND;
		break;
	default:
<<<<<<< HEAD
		return -EINVAL;
	}
	res = inv_secondary_write(BMA250_MODE_CTRL_REG, data);
	if (res)
		return res;
	bma_static_property.mode = mode;

	return 0;
}

static int suspend_slave_bma250(struct inv_mpu_iio_s *st)
=======
		break;
	}
	res = inv_secondary_write(BMA250_MODE_CTRL_REG, data);
	bma_static_property.mode = Mode;
	return res;
}
static int suspend_slave_bma250(struct inv_gyro_state_s *st)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
{
	int result;
	if (bma_static_property.mode == BMA250_MODE_SUSPEND)
		return 0;
	/*set to bypass mode */
<<<<<<< HEAD
	result = set_3050_bypass(st, true);
=======
	result = set_3050_bypass(st, 1);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (result)
		return result;
	bma250_set_mode(st, BMA250_MODE_SUSPEND);
	/* no need to recover to non-bypass mode because we need it now */
<<<<<<< HEAD

	return 0;
}

static int resume_slave_bma250(struct inv_mpu_iio_s *st)
=======
	return result;
}
static int resume_slave_bma250(struct inv_gyro_state_s *st)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
{
	int result;
	if (bma_static_property.mode == BMA250_MODE_NORMAL)
		return 0;
	/*set to bypass mode */
<<<<<<< HEAD
	result = set_3050_bypass(st, true);
	if (result)
		return result;
	result = bma250_set_mode(st, BMA250_MODE_NORMAL);
	/* recover bypass mode */
	result |= set_3050_bypass(st, false);

	return result ? (-EINVAL) : 0;
}

static int combine_data_slave_bma250(u8 *in, short *out)
=======
	result = set_3050_bypass(st, 1);
	if (result)
		return result;
	bma250_set_mode(st, BMA250_MODE_NORMAL);
	/* recover bypass mode */
	result = set_3050_bypass(st, 0);
	return result;
}
static int combine_data_slave_bma250(unsigned char *in, short *out)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
{
	out[0] = le16_to_cpup((__le16 *)(&in[0]));
	out[1] = le16_to_cpup((__le16 *)(&in[2]));
	out[2] = le16_to_cpup((__le16 *)(&in[4]));
<<<<<<< HEAD

	return 0;
}

static int get_mode_slave_bma250(void)
{
	switch (bma_static_property.mode) {
	case BMA250_MODE_SUSPEND:
		return INV_MODE_SUSPEND;
	case BMA250_MODE_NORMAL:
		return INV_MODE_NORMAL;
	default:
		return -EINVAL;
	}
}

=======
	return 0;
}
static int get_mode_slave_bma250(struct inv_gyro_state_s *st)
{
	if (bma_static_property.mode == BMA250_MODE_SUSPEND)
		return 0;
	else if (bma_static_property.mode == BMA250_MODE_NORMAL)
		return 1;
	return -1;
};
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
/**
 *  set_lpf_bma250() - set lpf value
 */

<<<<<<< HEAD
static int set_lpf_bma250(struct inv_mpu_iio_s *st, int rate)
=======
static int set_lpf_bma250(struct inv_gyro_state_s *st, int rate)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
{
	const short hz[] = {1000, 500, 250, 125, 62, 31, 15, 7};
	const int   d[] = {7, 6, 5, 4, 3, 2, 1, 0};
	int i, h, data, result;
	h = (rate >> 1);
	i = 0;
<<<<<<< HEAD
	while ((h < hz[i]) && (i < ARRAY_SIZE(hz) - 1))
		i++;
	data = d[i];

	result = set_3050_bypass(st, true);
	if (result)
		return result;
	result = bma250_set_bandwidth(st, (u8) data);
	result |= set_3050_bypass(st, false);

	return result ? (-EINVAL) : 0;
=======
	while ((h < hz[i]) && (i < ARRAY_SIZE(hz)))
		i++;
	if (i == ARRAY_SIZE(hz))
		i -= 1;
	data = d[i];

	result = set_3050_bypass(st, 1);
	if (result)
		return result;
	result = bma250_set_bandwidth(st, (unsigned char) data);
	result |= set_3050_bypass(st, 0);

	return result;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}
/**
 *  set_fs_bma250() - set range value
 */

<<<<<<< HEAD
static int set_fs_bma250(struct inv_mpu_iio_s *st, int fs)
{
	int result;
	result = set_3050_bypass(st, true);
	if (result)
		return result;
	result = bma250_set_range(st, (u8) fs);
	result |= set_3050_bypass(st, false);

	return result ? (-EINVAL) : 0;
=======
static int set_fs_bma250(struct inv_gyro_state_s *st, int fs)
{
	int result;
	result = set_3050_bypass(st, 1);
	if (result)
		return result;
	result = bma250_set_range(st, (unsigned char) fs);
	result |= set_3050_bypass(st, 0);
	if (result)
		return -EINVAL;
	return result;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}

static struct inv_mpu_slave slave_bma250 = {
	.suspend = suspend_slave_bma250,
	.resume  = resume_slave_bma250,
	.setup   = setup_slave_bma250,
	.combine_data = combine_data_slave_bma250,
	.get_mode = get_mode_slave_bma250,
	.set_lpf = set_lpf_bma250,
	.set_fs  = set_fs_bma250
};

<<<<<<< HEAD
int inv_register_mpu3050_slave(struct inv_mpu_iio_s *st)
{
	st->mpu_slave = &slave_bma250;

=======
int inv_register_bma250_slave(struct inv_gyro_state_s *st)
{
	st->mpu_slave = &slave_bma250;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	return 0;
}
/**
 *  @}
 */

