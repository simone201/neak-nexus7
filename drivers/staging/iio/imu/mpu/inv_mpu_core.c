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
<<<<<<< HEAD
 *      @file    inv_mpu_core.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This driver currently works for the
 *               MPU3050/MPU6050/MPU9150/MPU6500/MPU9250 devices.
 */
#define pr_fmt(fmt) KBUILD_MODNAME ": " fmt
=======
 *      @file    inv_gyro.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This driver currently works for the ITG3500, MPU6050, MPU9150
 *               MPU3050
 */
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

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
<<<<<<< HEAD

#include "inv_mpu_iio.h"
#include "../../sysfs.h"
#include "../../inv_test/inv_counters.h"

s64 get_time_ns(void)
{
	struct timespec ts;
	ktime_get_ts(&ts);
	return timespec_to_ns(&ts);
}

static const short AKM8975_ST_Lower[3] = {-100, -100, -1000};
static const short AKM8975_ST_Upper[3] = {100, 100, -300};

static const short AKM8972_ST_Lower[3] = {-50, -50, -500};
static const short AKM8972_ST_Upper[3] = {50, 50, -100};

static const short AKM8963_ST_Lower[3] = {-200, -200, -3200};
static const short AKM8963_ST_Upper[3] = {200, 200, -800};

/* This is for compatibility for power state. Should remove once HAL
   does not use power_state sysfs entry */
static bool fake_asleep;

=======
#include "inv_mpu_iio.h"
#include "../../sysfs.h"
#define CHECK_DMP	do \
	{ \
		if ((st->chip_config.is_asleep) || \
		(0 == st->chip_config.firmware_loaded)) \
			return -EPERM; \
		result = kstrtoul(buf, 10, (long unsigned int *)&data); \
		if (result) \
			return result; \
	} while (0);
static void inv_setup_reg(struct inv_reg_map_s *reg)
{
	reg->who_am_i		= 0x75;
	reg->sample_rate_div	= 0x19;
	reg->lpf		= 0x1A;
	reg->product_id		= 0x0C;
	reg->bank_sel		= 0x6D;
	reg->user_ctrl		= 0x6A;
	reg->fifo_en		= 0x23;
	reg->gyro_config	= 0x1B;
	reg->accl_config	= 0x1C;
	reg->fifo_count_h	= 0x72;
	reg->fifo_r_w		= 0x74;
	reg->raw_gyro		= 0x43;
	reg->raw_accl		= 0x3B;
	reg->temperature	= 0x41;
	reg->int_enable		= 0x38;
	reg->int_status		= 0x3A;
	reg->pwr_mgmt_1		= 0x6B;
	reg->pwr_mgmt_2		= 0x6C;
	reg->mem_start_addr	= 0x6E;
	reg->mem_r_w		= 0x6F;
	reg->prgm_strt_addrh	= 0x70;
};
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
static const struct inv_hw_s hw_info[INV_NUM_PARTS] = {
	{119, "ITG3500"},
	{ 63, "MPU3050"},
	{117, "MPU6050"},
<<<<<<< HEAD
	{118, "MPU9150"},
	{119, "MPU6500"},
	{118, "MPU9250"},
};

static void inv_setup_reg(struct inv_reg_map_s *reg)
{
	reg->sample_rate_div	= REG_SAMPLE_RATE_DIV;
	reg->lpf		= REG_CONFIG;
	reg->bank_sel		= REG_BANK_SEL;
	reg->user_ctrl		= REG_USER_CTRL;
	reg->fifo_en		= REG_FIFO_EN;
	reg->gyro_config	= REG_GYRO_CONFIG;
	reg->accl_config	= REG_ACCEL_CONFIG;
	reg->fifo_count_h	= REG_FIFO_COUNT_H;
	reg->fifo_r_w		= REG_FIFO_R_W;
	reg->raw_gyro		= REG_RAW_GYRO;
	reg->raw_accl		= REG_RAW_ACCEL;
	reg->temperature	= REG_TEMPERATURE;
	reg->int_enable		= REG_INT_ENABLE;
	reg->int_status		= REG_INT_STATUS;
	reg->pwr_mgmt_1		= REG_PWR_MGMT_1;
	reg->pwr_mgmt_2		= REG_PWR_MGMT_2;
	reg->mem_start_addr	= REG_MEM_START_ADDR;
	reg->mem_r_w		= REG_MEM_RW;
	reg->prgm_strt_addrh	= REG_PRGM_STRT_ADDRH;
};

=======
	{118, "MPU9150"}
};
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
/**
 *  inv_i2c_read() - Read one or more bytes from the device registers.
 *  @st:	Device driver instance.
 *  @reg:	First device register to be read from.
 *  @length:	Number of bytes to read.
 *  @data:	Data read from device.
<<<<<<< HEAD
 *  NOTE:This is not re-implementation of i2c_smbus_read because i2c
 *       address could be specified in this case. We could have two different
 *       i2c address due to secondary i2c interface.
 */
int inv_i2c_read_base(struct inv_mpu_iio_s *st, u16 i2c_addr,
	u8 reg, u16 length, u8 *data)
=======
 *  NOTE: The slave register will not increment when reading from the FIFO.
 */
int inv_i2c_read_base(struct inv_gyro_state_s *st, unsigned short i2c_addr,
	unsigned char reg, unsigned short length, unsigned char *data)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
{
	struct i2c_msg msgs[2];
	int res;

	if (!data)
		return -EINVAL;

	msgs[0].addr = i2c_addr;
	msgs[0].flags = 0;	/* write */
	msgs[0].buf = &reg;
	msgs[0].len = 1;

	msgs[1].addr = i2c_addr;
	msgs[1].flags = I2C_M_RD;
	msgs[1].buf = data;
	msgs[1].len = length;

	res = i2c_transfer(st->sl_handle, msgs, 2);
<<<<<<< HEAD

	if (res < 2) {
		if (res >= 0)
			res = -EIO;
	} else
		res = 0;

	INV_I2C_INC_MPUWRITE(3);
	INV_I2C_INC_MPUREAD(length);
#if CONFIG_DYNAMIC_DEBUG
	{
		char *read = 0;
		pr_debug("%s RD%02X%02X%02X -> %s%s\n", st->hw->name,
			 i2c_addr, reg, length,
			 wr_pr_debug_begin(data, length, read),
			 wr_pr_debug_end(read));
	}
#endif
	return res;
=======
	if (res < 2) {
		if (res >= 0)
			res = -EIO;
		return res;
	} else
		return 0;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}

/**
 *  inv_i2c_single_write() - Write a byte to a device register.
 *  @st:	Device driver instance.
 *  @reg:	Device register to be written to.
 *  @data:	Byte to write to device.
<<<<<<< HEAD
 *  NOTE:This is not re-implementation of i2c_smbus_write because i2c
 *       address could be specified in this case. We could have two different
 *       i2c address due to secondary i2c interface.
 */
int inv_i2c_single_write_base(struct inv_mpu_iio_s *st,
	u16 i2c_addr, u8 reg, u8 data)
{
	u8 tmp[2];
	struct i2c_msg msg;
	int res;
=======
 */
int inv_i2c_single_write_base(struct inv_gyro_state_s *st,
	unsigned short i2c_addr, unsigned char reg, unsigned char data)
{
	unsigned char tmp[2];
	struct i2c_msg msg;
	int res;

>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	tmp[0] = reg;
	tmp[1] = data;

	msg.addr = i2c_addr;
	msg.flags = 0;	/* write */
	msg.buf = tmp;
	msg.len = 2;

<<<<<<< HEAD
	pr_debug("%s WR%02X%02X%02X\n", st->hw->name, i2c_addr, reg, data);
	INV_I2C_INC_MPUWRITE(3);

=======
	/*printk(KERN_ERR "WS%02X%02X%02X\n", i2c_addr, reg, data);*/
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	res = i2c_transfer(st->sl_handle, &msg, 1);
	if (res < 1) {
		if (res == 0)
			res = -EIO;
		return res;
	} else
		return 0;
}
<<<<<<< HEAD

static int inv_switch_engine(struct inv_mpu_iio_s *st, bool en, u32 mask)
{
	struct inv_reg_map_s *reg;
	u8 data, mgmt_1;
	int result;
	reg = &st->reg;
	/* switch clock needs to be careful. Only when gyro is on, can
	   clock source be switched to gyro. Otherwise, it must be set to
	   internal clock */
	if (BIT_PWR_GYRO_STBY == mask) {
		result = inv_i2c_read(st, reg->pwr_mgmt_1, 1, &mgmt_1);
		if (result)
			return result;

		mgmt_1 &= ~BIT_CLK_MASK;
	}

	if ((BIT_PWR_GYRO_STBY == mask) && (!en)) {
		/* turning off gyro requires switch to internal clock first.
		   Then turn off gyro engine */
		mgmt_1 |= INV_CLK_INTERNAL;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1,
						mgmt_1);
		if (result)
			return result;
	}

	result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &data);
	if (result)
		return result;
	if (en)
		data &= (~mask);
	else
		data |= mask;
	result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
	if (result)
		return result;

	if ((BIT_PWR_GYRO_STBY == mask) && en) {
		/* only gyro on needs sensor up time */
		msleep(SENSOR_UP_TIME);
		/* after gyro is on & stable, switch internal clock to PLL */
		mgmt_1 |= INV_CLK_PLL;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_1,
						mgmt_1);
		if (result)
			return result;
	}
	if ((BIT_PWR_ACCL_STBY == mask) && en)
		msleep(REG_UP_TIME);

	return 0;
}

/**
 *  inv_lpa_freq() - store current low power frequency setting.
 */
static int inv_lpa_freq(struct inv_mpu_iio_s *st, int lpa_freq)
{
	unsigned long result;
	u8 d;
	struct inv_reg_map_s *reg;
	/* this mapping makes 6500 and 6050 setting close */
	/* 2, 4, 6, 7 corresponds to 0.98, 3.91, 15.63, 31.25 */
	const u8 mpu6500_lpa_mapping[] = {2, 4, 6, 7};

	if (lpa_freq > MAX_LPA_FREQ_PARAM)
		return -EINVAL;

	if (INV_MPU6500 == st->chip_type) {
		d = mpu6500_lpa_mapping[lpa_freq];
		result = inv_i2c_single_write(st, REG_6500_LP_ACCEL_ODR, d);
		if (result)
			return result;
	} else {
		reg = &st->reg;
		result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &d);
		if (result)
			return result;
		d &= ~BIT_LPA_FREQ;
		d |= (u8)(lpa_freq << LPA_FREQ_SHIFT);
		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, d);
		if (result)
			return result;
	}
	st->chip_config.lpa_freq = lpa_freq;

	return 0;
}

static int set_power_itg(struct inv_mpu_iio_s *st, bool power_on)
{
	struct inv_reg_map_s *reg;
	u8 data;
	int result;

	if ((!power_on) == st->chip_config.is_asleep)
		return 0;
	reg = &st->reg;
	if (power_on)
		data = 0;
	else
		data = BIT_SLEEP;
	result = inv_i2c_single_write(st, reg->pwr_mgmt_1, data);
	if (result)
		return result;

	if (power_on) {
		if (INV_MPU6500 == st->chip_type)
			msleep(POWER_UP_TIME);
		else
			msleep(REG_UP_TIME);
	}

	st->chip_config.is_asleep = !power_on;

=======
static int set_power_itg(struct inv_gyro_state_s *st,
	unsigned char power_on)
{
	struct inv_reg_map_s *reg;
	unsigned char data;
	int result;

	reg = &st->reg;
	if (power_on)
		data = 0;
	else
		data = BIT_SLEEP;
	if (st->chip_config.lpa_mode)
		data |= BIT_CYCLE;
	if (st->chip_config.gyro_enable) {
		result = inv_i2c_single_write(st,
			reg->pwr_mgmt_1, data | INV_CLK_PLL);
		if (result)
			return result;
		st->chip_config.clk_src = INV_CLK_PLL;
	} else {
		result = inv_i2c_single_write(st,
			reg->pwr_mgmt_1, data | INV_CLK_INTERNAL);
		if (result)
			return result;
		st->chip_config.clk_src = INV_CLK_INTERNAL;
	}

	if (power_on) {
		msleep(POWER_UP_TIME);
		data = 0;
		if (0 == st->chip_config.accl_enable)
			data |= BIT_PWR_ACCL_STBY;
		if (0 == st->chip_config.gyro_enable)
			data |= BIT_PWR_GYRO_STBY;
		data |= (st->chip_config.lpa_freq << LPA_FREQ_SHIFT);

		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
		msleep(POWER_UP_TIME);
		st->chip_config.is_asleep = 0;
	} else
		st->chip_config.is_asleep = 1;
	return 0;
}
/**
 *  inv_set_power_state() - Turn device on/off.
 *  @st:	Device driver instance.
 *  @power_on:	1 to turn on, 0 to suspend.
 */
int inv_set_power_state(struct inv_gyro_state_s *st,
	unsigned char power_on)
{
	if (INV_MPU3050 == st->chip_type)
		return set_power_mpu3050(st, power_on);
	else
		return set_power_itg(st, power_on);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	return 0;
}

/**
 *  inv_init_config() - Initialize hardware, disable FIFO.
 *  @indio_dev:	Device driver instance.
 *  Initial configuration:
 *  FSR: +/- 2000DPS
 *  DLPF: 42Hz
 *  FIFO rate: 50Hz
<<<<<<< HEAD
=======
 *  Clock source: Gyro PLL
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
 */
static int inv_init_config(struct iio_dev *indio_dev)
{
	struct inv_reg_map_s *reg;
	int result;
<<<<<<< HEAD
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);


	reg = &st->reg;
=======
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	if (st->chip_config.is_asleep)
		return -EPERM;
	reg = &st->reg;
	result = set_inv_enable(indio_dev, 0);
	if (result)
		return result;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

	result = inv_i2c_single_write(st, reg->gyro_config,
		INV_FSR_2000DPS << GYRO_CONFIG_FSR_SHIFT);
	if (result)
		return result;
<<<<<<< HEAD

=======
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	st->chip_config.fsr = INV_FSR_2000DPS;

	result = inv_i2c_single_write(st, reg->lpf, INV_FILTER_42HZ);
	if (result)
		return result;
	st->chip_config.lpf = INV_FILTER_42HZ;

	result = inv_i2c_single_write(st, reg->sample_rate_div,
<<<<<<< HEAD
					ONE_K_HZ / INIT_FIFO_RATE - 1);
	if (result)
		return result;
	st->chip_config.fifo_rate = INIT_FIFO_RATE;
	st->chip_config.new_fifo_rate = INIT_FIFO_RATE;
	st->irq_dur_ns            = INIT_DUR_TIME;
	st->chip_config.prog_start_addr = DMP_START_ADDR;
	st->chip_config.dmp_output_rate = INIT_DMP_OUTPUT_RATE;
	st->self_test.samples = INIT_ST_SAMPLES;
	st->self_test.threshold = INIT_ST_THRESHOLD;
	if (INV_ITG3500 != st->chip_type) {
=======
					ONE_K_HZ/INIT_FIFO_RATE - 1);
	if (result)
		return result;
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG,
				st->plat_data.int_config & (~BIT_BYPASS_EN));
	if (result)
		return result;
	st->chip_config.fifo_rate = INIT_FIFO_RATE;
	st->irq_dur_us            = INIT_DUR_TIME;
	st->chip_config.prog_start_addr = DMP_START_ADDR;
	st->chip_config.gyro_enable = 1;
	st->chip_config.gyro_fifo_enable = 1;
	if (INV_ITG3500 != st->chip_type) {
		st->chip_config.accl_enable = 1;
		st->chip_config.accl_fifo_enable = 1;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		st->chip_config.accl_fs = INV_FS_02G;
		result = inv_i2c_single_write(st, reg->accl_config,
			(INV_FS_02G << ACCL_CONFIG_FSR_SHIFT));
		if (result)
			return result;
		st->tap.time = INIT_TAP_TIME;
		st->tap.thresh = INIT_TAP_THRESHOLD;
		st->tap.min_count = INIT_TAP_MIN_COUNT;
<<<<<<< HEAD
		st->smd.threshold = MPU_INIT_SMD_THLD;
		st->smd.delay     = MPU_INIT_SMD_DELAY_THLD;
		st->smd.delay2    = MPU_INIT_SMD_DELAY2_THLD;

		result = inv_i2c_single_write(st, REG_ACCEL_MOT_DUR,
						INIT_MOT_DUR);
		if (result)
			return result;
		st->mot_int.mot_dur = INIT_MOT_DUR;

		result = inv_i2c_single_write(st, REG_ACCEL_MOT_THR,
						INIT_MOT_THR);
		if (result)
			return result;
		st->mot_int.mot_thr = INIT_MOT_THR;
	}

	return 0;
}

/**
 *  inv_compass_scale_show() - show compass scale.
 */
static int inv_compass_scale_show(struct inv_mpu_iio_s *st, int *scale)
=======
	}
	return 0;
}
/**
 *  inv_compass_scale_show() - show compass scale.
 */
static int inv_compass_scale_show(struct inv_gyro_state_s *st, int *scale)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
{
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id)
		*scale = DATA_AKM8975_SCALE;
	else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id)
		*scale = DATA_AKM8972_SCALE;
	else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id)
		if (st->compass_scale)
			*scale = DATA_AKM8963_SCALE1;
		else
			*scale = DATA_AKM8963_SCALE0;
	else
		return -EINVAL;
<<<<<<< HEAD

	return IIO_VAL_INT;
}

/**
 *  inv_sensor_show() - Read gyro/accel data directly from registers.
 */
static int inv_sensor_show(struct inv_mpu_iio_s  *st, int reg, int axis,
					int *val)
{
	int ind, result;
	u8 d[2];

	ind = (axis - IIO_MOD_X) * 2;
	result = i2c_smbus_read_i2c_block_data(st->client,
					       reg + ind, 2, d);
	if (result != 2)
		return -EINVAL;
	*val = (short)be16_to_cpup((__be16 *)(d));

=======
	*scale *= (1L << 15);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	return IIO_VAL_INT;
}

/**
 *  mpu_read_raw() - read raw method.
 */
static int mpu_read_raw(struct iio_dev *indio_dev,
<<<<<<< HEAD
			struct iio_chan_spec const *chan,
			int *val, int *val2, long mask)
{
	struct inv_mpu_iio_s  *st = iio_priv(indio_dev);
	int result;

	switch (mask) {
	case 0:
		/* if enabled, power is on already */
		if (!st->chip_config.enable)
			return -EBUSY;
		switch (chan->type) {
		case IIO_ANGL_VEL:
			if (!st->chip_config.gyro_enable)
				return -EPERM;
			return inv_sensor_show(st, st->reg.raw_gyro,
						chan->channel2, val);
		case IIO_ACCEL:
			if (!st->chip_config.accl_enable)
				return -EPERM;
			return inv_sensor_show(st, st->reg.raw_accl,
						chan->channel2, val);
		case IIO_MAGN:
			if (!st->chip_config.compass_enable)
				return -EPERM;
			*val = st->raw_compass[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		case IIO_QUATERNION:
			if (!(st->chip_config.dmp_on
				&& st->chip_config.quaternion_on))
				return -EPERM;
			if (IIO_MOD_R == chan->channel2)
				*val = st->raw_quaternion[0];
			else
				*val = st->raw_quaternion[chan->channel2 -
							  IIO_MOD_X + 1];
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
		{
			const s16 gyro_scale[] = {250, 500, 1000, 2000};

			*val = gyro_scale[st->chip_config.fsr];

			return IIO_VAL_INT;
		}
		case IIO_ACCEL:
		{
			const s16 accel_scale[] = {2, 4, 8, 16};
			*val = accel_scale[st->chip_config.accl_fs] *
					st->chip_info.multi;
			return IIO_VAL_INT;
		}
		case IIO_MAGN:
			return inv_compass_scale_show(st, val);
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_CALIBBIAS:
		if (st->chip_config.self_test_run_once == 0) {
			/* This can only be run when enable is zero */
			if (st->chip_config.enable)
				return -EBUSY;
			mutex_lock(&indio_dev->mlock);

			result = inv_power_up_self_test(st);
			if (result)
				goto error_info_calibbias;
			result = inv_do_test(st, 0,  st->gyro_bias,
				st->accel_bias);
			if (result)
				goto error_info_calibbias;
			st->chip_config.self_test_run_once = 1;
error_info_calibbias:
			/* Reset Accel and Gyro full scale range
			   back to default value */
			inv_recover_setting(st);
			mutex_unlock(&indio_dev->mlock);
		}

		switch (chan->type) {
		case IIO_ANGL_VEL:
			*val = st->gyro_bias[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		case IIO_ACCEL:
			*val = st->accel_bias[chan->channel2 - IIO_MOD_X] *
					st->chip_info.multi;
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_ACCEL:
			*val = st->input_accel_bias[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		default:
			return -EINVAL;
		}
=======
			      struct iio_chan_spec const *chan,
			      int *val,
			      int *val2,
			      long mask) {
	struct inv_gyro_state_s  *st = iio_priv(indio_dev);
	int result;
	if (st->chip_config.is_asleep)
		return -EINVAL;
	switch (mask) {
	case 0:
		if (chan->type == IIO_ANGL_VEL) {
			*val = st->raw_gyro[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_ACCEL) {
			*val = st->raw_accel[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_MAGN) {
			*val = st->raw_compass[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		return -EINVAL;
	case IIO_CHAN_INFO_SCALE:
		if (chan->type == IIO_ANGL_VEL) {
			*val = (1 << st->chip_config.fsr)*GYRO_DPS_SCALE;
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_ACCEL) {
			*val = (2 << st->chip_config.accl_fs);
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_MAGN)
			return inv_compass_scale_show(st, val);
		return -EINVAL;
	case IIO_CHAN_INFO_CALIBBIAS:
		if (st->chip_config.self_test_run_once == 0) {
			result = inv_do_test(st, 0,  st->gyro_bias,
				st->accel_bias);
			if (result)
				return result;
			st->chip_config.self_test_run_once = 1;
		}

		if (chan->type == IIO_ANGL_VEL) {
			*val = st->gyro_bias[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		if (chan->type == IIO_ACCEL) {
			*val = st->accel_bias[chan->channel2 - IIO_MOD_X];
			return IIO_VAL_INT;
		}
		return -EINVAL;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	default:
		return -EINVAL;
	}
}

/**
 *  inv_write_fsr() - Configure the gyro's scale range.
 */
<<<<<<< HEAD
static int inv_write_fsr(struct inv_mpu_iio_s *st, int fsr)
=======
static int inv_write_fsr(struct inv_gyro_state_s *st, int fsr)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
{
	struct inv_reg_map_s *reg;
	int result;
	reg = &st->reg;
	if ((fsr < 0) || (fsr > MAX_GYRO_FS_PARAM))
		return -EINVAL;
	if (fsr == st->chip_config.fsr)
		return 0;

<<<<<<< HEAD
	if (INV_MPU3050 == st->chip_type)
		result = inv_i2c_single_write(st, reg->lpf,
			(fsr << GYRO_CONFIG_FSR_SHIFT) | st->chip_config.lpf);
	else
		result = inv_i2c_single_write(st, reg->gyro_config,
			fsr << GYRO_CONFIG_FSR_SHIFT);

	if (result)
		return result;
	st->chip_config.fsr = fsr;

=======
	if (INV_MPU3050 == st->chip_type) {
		result = inv_i2c_single_write(st, reg->lpf,
			(fsr << GYRO_CONFIG_FSR_SHIFT) | st->chip_config.lpf);
	} else {
		result = inv_i2c_single_write(st, reg->gyro_config,
			fsr << GYRO_CONFIG_FSR_SHIFT);
	}
	if (result)
		return result;
	st->chip_config.fsr = fsr;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	return 0;
}

/**
 *  inv_write_accel_fs() - Configure the accelerometer's scale range.
 */
<<<<<<< HEAD
static int inv_write_accel_fs(struct inv_mpu_iio_s *st, int fs)
{
	int result;
	struct inv_reg_map_s *reg;

	reg = &st->reg;
=======
static int inv_write_accel_fs(struct inv_gyro_state_s *st, int fs)
{
	int result;
	struct inv_reg_map_s *reg;
	reg = &st->reg;

>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (fs < 0 || fs > MAX_ACCL_FS_PARAM)
		return -EINVAL;
	if (fs == st->chip_config.accl_fs)
		return 0;
<<<<<<< HEAD
	if (INV_MPU3050 == st->chip_type)
		result = st->mpu_slave->set_fs(st, fs);
	else
		result = inv_i2c_single_write(st, reg->accl_config,
				(fs << ACCL_CONFIG_FSR_SHIFT));
	if (result)
		return result;

	st->chip_config.accl_fs = fs;

	return 0;
}

/**
 *  inv_write_compass_scale() - Configure the compass's scale range.
 */
static int inv_write_compass_scale(struct inv_mpu_iio_s  *st, int data)
=======
	if (INV_MPU3050 == st->chip_type) {
		result = st->mpu_slave->set_fs(st, fs);
		if (result)
			return result;
	} else {
		result = inv_i2c_single_write(st, reg->accl_config,
				(fs << ACCL_CONFIG_FSR_SHIFT));
		if (result)
			return result;
	}
	/* reset fifo because the data could be mixed with old bad data */
	st->chip_config.accl_fs = fs;
	return 0;
}
/**
 *  inv_write_compass_scale() - Configure the compass's scale range.
 */
static int inv_write_compass_scale(struct inv_gyro_state_s  *st, int data)
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
{
	char d, en;
	int result;
	if (COMPASS_ID_AK8963 != st->plat_data.sec_slave_id)
		return 0;
<<<<<<< HEAD
	en = !!data;
	if (st->compass_scale == en)
		return 0;
	d = (DATA_AKM_MODE_SM | (st->compass_scale << AKM8963_SCALE_SHIFT));
=======
	if (data)
		en = 1;
	else
		en = 0;
	if (st->compass_scale == en)
		return 0;
	d = (1 | (st->compass_scale << AKM8963_SCALE_SHIFT));
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	result = inv_i2c_single_write(st, REG_I2C_SLV1_DO, d);
	if (result)
		return result;
	st->compass_scale = en;
<<<<<<< HEAD

	return 0;
=======
	return 0;

>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}

/**
 *  mpu_write_raw() - write raw method.
 */
static int mpu_write_raw(struct iio_dev *indio_dev,
			       struct iio_chan_spec const *chan,
			       int val,
			       int val2,
			       long mask) {
<<<<<<< HEAD
	struct inv_mpu_iio_s  *st = iio_priv(indio_dev);
	int result;

	if (st->chip_config.enable)
		return -EBUSY;
	mutex_lock(&indio_dev->mlock);
	result = st->set_power_state(st, true);
	if (result) {
		mutex_unlock(&indio_dev->mlock);
		return result;
	}

	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		switch (chan->type) {
		case IIO_ANGL_VEL:
			result = inv_write_fsr(st, val);
			break;
		case IIO_ACCEL:
			result = inv_write_accel_fs(st, val);
			break;
		case IIO_MAGN:
			result = inv_write_compass_scale(st, val);
			break;
		default:
			result = -EINVAL;
			break;
		}
		break;
	case IIO_CHAN_INFO_OFFSET:
		switch (chan->type) {
		case IIO_ACCEL:
			if (!st->chip_config.firmware_loaded) {
				result = -EPERM;
				goto error_write_raw;
			}
			result = inv_set_accel_bias_dmp(st);
			if (result)
				goto error_write_raw;
			st->input_accel_bias[chan->channel2 - IIO_MOD_X] = val;
			result = 0;
			break;
		default:
			result = -EINVAL;
			break;
		}
		break;
	default:
		result = -EINVAL;
		break;
	}

error_write_raw:
	result |= st->set_power_state(st, false);
	mutex_unlock(&indio_dev->mlock);

	return result;
=======
	struct inv_gyro_state_s  *st = iio_priv(indio_dev);
	int result;
	if (st->chip_config.is_asleep)
		return -EPERM;
	switch (mask) {
	case IIO_CHAN_INFO_SCALE:
		result = -EINVAL;
		if (chan->type == IIO_ANGL_VEL)
			result = inv_write_fsr(st, val);
		if (chan->type == IIO_ACCEL)
			result = inv_write_accel_fs(st, val);
		if (chan->type == IIO_MAGN)
			result = inv_write_compass_scale(st, val);
		return result;
	default:
		return -EINVAL;
	}
	return 0;
}

/**
 *  inv_set_lpf() - set low pass filer based on fifo rate.
 */
static int inv_set_lpf(struct inv_gyro_state_s *st, int rate)
{
	const short hz[] = {188, 98, 42, 20, 10, 5};
	const int   d[] = {INV_FILTER_188HZ, INV_FILTER_98HZ,
			INV_FILTER_42HZ, INV_FILTER_20HZ,
			INV_FILTER_10HZ, INV_FILTER_5HZ};
	int i, h, data, result;
	struct inv_reg_map_s *reg;
	reg = &st->reg;
	h = (rate >> 1);
	i = 0;
	while ((h < hz[i]) && (i < ARRAY_SIZE(d)))
		i++;
	if (i == ARRAY_SIZE(d))
		i -= 1;
	data = d[i];
	if (INV_MPU3050 == st->chip_type) {
		if (st->mpu_slave != NULL) {
			result = st->mpu_slave->set_lpf(st, rate);
			if (result)
				return result;
		}
		result = inv_i2c_single_write(st, reg->lpf, data |
			(st->chip_config.fsr << GYRO_CONFIG_FSR_SHIFT));
		if (result)
			return result;
	} else
		result = inv_i2c_single_write(st, reg->lpf, data);
	if (result)
		return result;
	st->chip_config.lpf = data;
	return 0;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}

/**
 *  inv_fifo_rate_store() - Set fifo rate.
 */
<<<<<<< HEAD
static int inv_fifo_rate_store(struct inv_mpu_iio_s *st, int fifo_rate)
{
	if ((fifo_rate < MIN_FIFO_RATE) || (fifo_rate > MAX_FIFO_RATE))
		return -EINVAL;
	if (fifo_rate == st->chip_config.fifo_rate)
		return 0;

	if (st->chip_config.has_compass) {
		st->compass_divider = COMPASS_RATE_SCALE * fifo_rate /
					ONE_K_HZ;
		if (st->compass_divider > 0)
			st->compass_divider -= 1;
		st->compass_counter = 0;
	}
	st->irq_dur_ns = (ONE_K_HZ / fifo_rate) * NSEC_PER_MSEC;
	st->chip_config.new_fifo_rate = fifo_rate;

	return 0;
}

/**
 *  inv_reg_dump_show() - Register dump for testing.
 */
static ssize_t inv_reg_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ii;
	char data;
	ssize_t bytes_printed = 0;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);

	mutex_lock(&indio_dev->mlock);
	if (!st->chip_config.enable)
		st->set_power_state(st, true);
	for (ii = 0; ii < st->hw->num_reg; ii++) {
		/* don't read fifo r/w register */
		if (ii == st->reg.fifo_r_w)
			data = 0;
		else
			inv_i2c_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf + bytes_printed, "%#2x: %#2x\n",
					 ii, data);
	}
	if (!st->chip_config.enable)
		st->set_power_state(st, false);
	mutex_unlock(&indio_dev->mlock);

	return bytes_printed;
}

int write_be32_key_to_mem(struct inv_mpu_iio_s *st,
					u32 data, int key)
{
	cpu_to_be32s(&data);
	return mem_w_key(key, sizeof(data), (u8 *)&data);
}

/**
 * inv_quaternion_on() -  calling this function will store
 *                                 current quaternion on
 */
static int inv_quaternion_on(struct inv_mpu_iio_s *st,
				 struct iio_buffer *ring, bool en)
{
	st->chip_config.quaternion_on = en;
	if (!en) {
		clear_bit(INV_MPU_SCAN_QUAT_R, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_Z, ring->scan_mask);
	}

	return 0;
}

/**
 * inv_dmp_attr_store() -  calling this function will store current
 *                        dmp parameter settings
 */
static ssize_t inv_dmp_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result, data;

	mutex_lock(&indio_dev->mlock);
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto dmp_attr_store_fail;
	}
	if (this_attr->address <= ATTR_DMP_DISPLAY_ORIENTATION_ON) {
		if (!st->chip_config.firmware_loaded) {
			result = -EINVAL;
			goto dmp_attr_store_fail;
		}
		result = st->set_power_state(st, true);
		if (result)
			goto dmp_attr_store_fail;
	}

	result = kstrtoint(buf, 10, &data);
	if (result)
		goto dmp_attr_store_fail;
	switch (this_attr->address) {
	case ATTR_DMP_SMD_ENABLE:
	{
		u8 on[] = {0, 1};
		u8 off[] = {0, 0};
		u8 *d;
		if (data)
			d = on;
		else
			d = off;
		result = mem_w_key(KEY_SMD_ENABLE, ARRAY_SIZE(on), d);
		if (result)
			goto dmp_attr_store_fail;
		st->chip_config.smd_enable = !!data;
	}
		break;
	case ATTR_DMP_SMD_THLD:
		if (data < 0 || data > SHRT_MAX)
			goto dmp_attr_store_fail;
		result = write_be32_key_to_mem(st, data << 16,
						KEY_SMD_ACCEL_THLD);
		if (result)
			goto dmp_attr_store_fail;
		st->smd.threshold = data;
		break;
	case ATTR_DMP_SMD_DELAY_THLD:
		if (data < 0 || data > INT_MAX / MPU_DEFAULT_DMP_FREQ)
			goto dmp_attr_store_fail;
		result = write_be32_key_to_mem(st, data * MPU_DEFAULT_DMP_FREQ,
						KEY_SMD_DELAY_THLD);
		if (result)
			goto dmp_attr_store_fail;
		st->smd.delay = data;
		break;
	case ATTR_DMP_SMD_DELAY_THLD2:
		if (data < 0 || data > INT_MAX / MPU_DEFAULT_DMP_FREQ)
			goto dmp_attr_store_fail;
		result = write_be32_key_to_mem(st, data * MPU_DEFAULT_DMP_FREQ,
						KEY_SMD_DELAY2_THLD);
		if (result)
			goto dmp_attr_store_fail;
		st->smd.delay2 = data;
		break;
	case ATTR_DMP_TAP_ON:
		result = inv_enable_tap_dmp(st, !!data);
		if (result)
			goto dmp_attr_store_fail;
		st->chip_config.tap_on = !!data;
		break;
	case ATTR_DMP_TAP_THRESHOLD: {
		const char ax[] = {INV_TAP_AXIS_X, INV_TAP_AXIS_Y,
							INV_TAP_AXIS_Z};
		int i;
		if (data < 0 || data > USHRT_MAX) {
			result = -EINVAL;
			goto dmp_attr_store_fail;
		}
		for (i = 0; i < ARRAY_SIZE(ax); i++) {
			result = inv_set_tap_threshold_dmp(st, ax[i], data);
			if (result)
				goto dmp_attr_store_fail;
		}
		st->tap.thresh = data;
		break;
	}
	case ATTR_DMP_TAP_MIN_COUNT:
		if (data < 0 || data > USHRT_MAX) {
			result = -EINVAL;
			goto dmp_attr_store_fail;
		}
		result = inv_set_min_taps_dmp(st, data);
		if (result)
			goto dmp_attr_store_fail;
		st->tap.min_count = data;
		break;
	case ATTR_DMP_TAP_TIME:
		if (data < 0 || data > USHRT_MAX) {
			result = -EINVAL;
			goto dmp_attr_store_fail;
		}
		result = inv_set_tap_time_dmp(st, data);
		if (result)
			goto dmp_attr_store_fail;
		st->tap.time = data;
		break;
	case ATTR_DMP_DISPLAY_ORIENTATION_ON:
		result = inv_set_display_orient_interrupt_dmp(st, !!data);
		if (result)
			goto dmp_attr_store_fail;
		st->chip_config.display_orient_on = !!data;
		break;
	/* from here, power of chip is not turned on */
	case ATTR_DMP_ON:
		st->chip_config.dmp_on = !!data;
		break;
	case ATTR_DMP_INT_ON:
		st->chip_config.dmp_int_on = !!data;
		break;
	case ATTR_DMP_EVENT_INT_ON:
		st->chip_config.dmp_event_int_on = !!data;
		break;
	case ATTR_DMP_OUTPUT_RATE:
		if (data <= 0 || data > MAX_DMP_OUTPUT_RATE) {
			result = -EINVAL;
			goto dmp_attr_store_fail;
		}
		st->chip_config.dmp_output_rate = data;
		if (st->chip_config.has_compass) {
			st->compass_dmp_divider = COMPASS_RATE_SCALE * data /
							ONE_K_HZ;
			if (st->compass_dmp_divider > 0)
				st->compass_dmp_divider -= 1;
			st->compass_counter = 0;
		}
		break;
	case ATTR_DMP_QUATERNION_ON:
		result = inv_quaternion_on(st, indio_dev->buffer, !!data);
		break;
#ifdef CONFIG_INV_TESTING
	case ATTR_DEBUG_SMD_ENABLE_TESTP1:
	{
		u8 d[] = {0x42};
		result = st->set_power_state(st, true);
		if (result)
			goto dmp_attr_store_fail;
		result = mem_w_key(KEY_SMD_ENABLE_TESTPT1, ARRAY_SIZE(d), d);
		if (result)
			goto dmp_attr_store_fail;
	}
		break;
	case ATTR_DEBUG_SMD_ENABLE_TESTP2:
	{
		u8 d[] = {0x42};
		result = st->set_power_state(st, true);
		if (result)
			goto dmp_attr_store_fail;
		result = mem_w_key(KEY_SMD_ENABLE_TESTPT2, ARRAY_SIZE(d), d);
		if (result)
			goto dmp_attr_store_fail;
	}
		break;
#endif
	default:
		result = -EINVAL;
		goto dmp_attr_store_fail;
	}

dmp_attr_store_fail:
	if ((this_attr->address <= ATTR_DMP_DISPLAY_ORIENTATION_ON) &&
					(!st->chip_config.enable))
		result |= st->set_power_state(st, false);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;
=======
static ssize_t inv_fifo_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long fifo_rate;
	unsigned char data;
	int result;
	struct inv_gyro_state_s *st;
	struct inv_reg_map_s *reg;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	reg = &st->reg;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (kstrtoul(buf, 10, &fifo_rate))
		return -EINVAL;
	if ((fifo_rate < MIN_FIFO_RATE) || (fifo_rate > MAX_FIFO_RATE))
		return -EINVAL;
	if (fifo_rate == st->chip_config.fifo_rate)
		return count;
	if (st->chip_config.has_compass) {
		data = COMPASS_RATE_SCALE*fifo_rate/ONE_K_HZ;
		if (data > 0)
			data -= 1;
		st->compass_divider = data;
		st->compass_counter = 0;
		/* I2C_MST_DLY is set according to sample rate,
		   AKM cannot be read or set at sample rate higher than 100Hz*/
		result = inv_i2c_single_write(st, REG_I2C_SLV4_CTRL, data);
		if (result)
			return result;
	}
	data = ONE_K_HZ / fifo_rate - 1;
	result = inv_i2c_single_write(st, reg->sample_rate_div, data);
	if (result)
		return result;
	st->chip_config.fifo_rate = fifo_rate;
	result = inv_set_lpf(st, fifo_rate);
	if (result)
		return result;
	st->irq_dur_us = (data + 1) * ONE_K_HZ;
	st->last_isr_time = iio_get_time_ns();
	return count;
}
/**
 *  inv_fifo_rate_show() - Get the current sampling rate.
 */
static ssize_t inv_fifo_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.fifo_rate);
}

/**
 *  inv_power_state_store() - Turn device on/off.
 */
static ssize_t inv_power_state_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	int result;
	unsigned long power_state;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	if (kstrtoul(buf, 10, &power_state))
		return -EINVAL;
	if (!power_state == st->chip_config.is_asleep)
		return count;
	result = inv_set_power_state(st, power_state);
	return count;
}

/**
 *  inv_power_state_show() - Check if the device is on or in sleep mode.
 */
static ssize_t inv_power_state_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return sprintf(buf, "0\n");
	else
		return sprintf(buf, "1\n");
}

/**
 * inv_firmware_loaded_store() -  calling this function will change
 *                        firmware load
 */
static ssize_t inv_firmware_loaded_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned long data, result;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data != 0)
		return -EINVAL;
	st->chip_config.firmware_loaded = 0;
	st->chip_config.dmp_on = 0;
	st->chip_config.quaternion_on = 0;
	return count;
}
/**
 * inv_firmware_loaded_show() -  calling this function will show current
 *                        firmware load status
 */
static ssize_t inv_firmware_loaded_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.firmware_loaded);
}

/**
 *  inv_lpa_mode_store() - store current low power settings
 */
static ssize_t inv_lpa_mode_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned long result, lpa_mode;
	unsigned char d;
	struct inv_reg_map_s *reg;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &lpa_mode);
	if (result)
		return result;

	reg = &st->reg;
	result = inv_i2c_read(st, reg->pwr_mgmt_1, 1, &d);
	if (result)
		return result;
	d &= ~BIT_CYCLE;
	if (lpa_mode)
		d |= BIT_CYCLE;
	result = inv_i2c_single_write(st, reg->pwr_mgmt_1, d);
	if (result)
		return result;
	st->chip_config.lpa_mode = lpa_mode;
	return count;
}
/**
 *  inv_lpa_mode_show() - show current low power settings
 */
static ssize_t inv_lpa_mode_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.lpa_mode);
}

/**
 *  inv_lpa_freq_store() - store current low power frequency setting.
 */
static ssize_t inv_lpa_freq_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned long result, lpa_freq;
	unsigned char d;
	struct inv_reg_map_s *reg;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &lpa_freq);
	if (result)
		return result;
	if (lpa_freq > MAX_LPA_FREQ_PARAM)
		return -EINVAL;
	reg = &st->reg;
	result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &d);
	if (result)
		return result;
	d &= ~BIT_LPA_FREQ;
	d |= (unsigned char)(lpa_freq << LPA_FREQ_SHIFT);
	result = inv_i2c_single_write(st, reg->pwr_mgmt_2, d);
	if (result)
		return result;
	st->chip_config.lpa_freq = lpa_freq;
	return count;
}
/**
 *  inv_lpa_freq_show() - show current low power frequency setting
 */
static ssize_t inv_lpa_freq_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	switch (st->chip_config.lpa_freq) {
	case 0:
		return sprintf(buf, "1.25\n");
	case 1:
		return sprintf(buf, "5\n");
	case 2:
		return sprintf(buf, "20\n");
	case 3:
		return sprintf(buf, "40\n");
	default:
		return sprintf(buf, "0\n");
	}
}
/**
 * inv_dmp_on_store() -  calling this function will store current dmp on
 */
static ssize_t inv_dmp_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	st->chip_config.dmp_on = !!data;
	return count;
}

/**
 * inv_dmp_on_show() -  calling this function will show current dmp_on
 */
static ssize_t inv_dmp_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_on);
}
/**
 * inv_dmp_int_on_store() -  calling this function will store current dmp int on
 */
static ssize_t inv_dmp_int_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	st->chip_config.dmp_int_on = !!data;
	return count;
}

/**
 * inv_dmp_int_on_show() -  calling this function will show current dmp_int_on
 */
static ssize_t inv_dmp_int_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_int_on);
}

/**
 * inv_dmp_output_rate_store() -  calling this function store dmp_output_rate
 */
static ssize_t inv_dmp_output_rate_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	unsigned int result, data;
	st = iio_priv(indio_dev);

	CHECK_DMP
	if (0 == data)
		return -EINVAL;
	result = inv_set_fifo_rate(st, data);
	if (result)
		return result;
	st->chip_config.dmp_output_rate = data;
	return count;
}

/**
 * inv_dmp_output_rate_show() -  calling this shows dmp_output_rate
 */
static ssize_t inv_dmp_output_rate_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.dmp_output_rate);
}

/**
 * inv_orientation_on_store() -  calling this function will store
 *                                 current orientation on
 */
static ssize_t inv_orientation_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data, en;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	en = !!data;
	result = inv_enable_orientation_dmp(st, en);
	if (result)
		return result;
	st->chip_config.orientation_on = en;
	return count;
}
/**
 * inv_orientation_on_show() -  calling this function will show
 *				current orientation_on
 */
static ssize_t inv_orientation_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.orientation_on);
}

/**
 * inv_display_orient_on_store() -  calling this function will store
 *                                 current display_orient on
 */
static ssize_t inv_display_orient_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data, en;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	en = !!data;
	result = inv_set_display_orient_interrupt_dmp(st, en);
	if (result)
		return result;
	st->chip_config.display_orient_on = en;
	return count;
}
/**
 * inv_display_orient_on_show() -  calling this function will show
 *				current display_orient_on
 */
static ssize_t inv_display_orient_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.display_orient_on);
}

/**
 * inv_quaternion_on_store() -  calling this function will store
 *                                 current quaternion on
 */
static ssize_t inv_quaternion_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data, en;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct iio_buffer *ring = indio_dev->buffer;
	st = iio_priv(indio_dev);

	CHECK_DMP
	en = !!data;
	result = inv_send_quaternion(st, en);
	if (result)
		return result;
	st->chip_config.quaternion_on = en;
	if (0 == en) {
		clear_bit(INV_MPU_SCAN_QUAT_R, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_QUAT_Z, ring->scan_mask);
	}

	return count;
}
/**
 * inv_quaternion_on_show() -  calling this function will show
 *				current orientation_on
 */
static ssize_t inv_quaternion_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.quaternion_on);
}

/**
 * inv_tap_on_store() -  calling this function will store current tap on
 */
static ssize_t inv_tap_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	st->chip_config.tap_on = !!data;
	result = inv_enable_tap_dmp(st, st->chip_config.tap_on);
	return count;
}

/**
 * inv_tap_on_show() -  calling this function will show current tap_on
 */
static ssize_t inv_tap_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.tap_on);
}
/**
 * inv_tap_time_store() -  calling this function will store current tap time
 */
static ssize_t inv_tap_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	result = inv_set_tap_time_dmp(st, data);
	if (result)
		return result;
	st->tap.time = data;
	return count;
}
/**
 * inv_tap_time_show() -  calling this function will show current tap time
 */
static ssize_t inv_tap_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->tap.time);
}

/**
 * inv_tap_min_count_store() -  calling this function will store tap count
 */
static ssize_t inv_tap_min_count_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	result = inv_set_min_taps_dmp(st, data);
	if (result)
		return result;
	st->tap.min_count = data;
	return count;
}
/**
 * inv_tap_min_count_show() -  calling this function show minimum count
 */
static ssize_t inv_tap_min_count_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->tap.min_count);
}

/**
 * inv_tap_threshold_store() -  calling this function will store tap threshold
 */
static ssize_t inv_tap_threshold_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned int result, data;
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	CHECK_DMP
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_X, data);
	if (result)
		return result;
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_Y, data);
	if (result)
		return result;
	result = inv_set_tap_threshold_dmp(st, INV_TAP_AXIS_Z, data);
	if (result)
		return result;

	st->tap.thresh = data;
	return count;
}
/**
 * inv_tap_thresh_show() -  calling this function show current tap threshold
 */
static ssize_t inv_tap_threshold_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->tap.thresh);
}
/**
 *  inv_clk_src_show() - Show the device's clock source.
 */
static ssize_t inv_clk_src_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	switch (st->chip_config.clk_src) {
	case INV_CLK_INTERNAL:
		return sprintf(buf, "INTERNAL\n");
	case INV_CLK_PLL:
		return sprintf(buf, "Gyro PLL\n");
	default:
		return -EPERM;
	}
}
/**
 *  inv_reg_dump_show() - Register dump for testing.
 *  TODO: Only for testing.
 */
static ssize_t inv_reg_dump_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int ii;
	char data;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	ssize_t bytes_printed = 0;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	for (ii = 0; ii < st->hw->num_reg; ii++) {
		inv_i2c_read(st, ii, 1, &data);
		bytes_printed += sprintf(buf+bytes_printed, "%#2x: %#2x\n",
			ii, data);
	}
	return bytes_printed;
}

/**
 * inv_self_test_show() - self test result. 0 for fail; 1 for success.
 *                        calling this function will trigger self test
 *                        and return test result.
 */
static ssize_t inv_self_test_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int result;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (INV_MPU3050 == st->chip_type)
		result = 0;
	else
		result = inv_hw_self_test(st);
	return sprintf(buf, "%d\n", result);
}
/**
 * inv_key_show() -  calling this function will show the key
 *
 */
static ssize_t inv_key_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	unsigned char *key;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	key = st->plat_data.key;
	return sprintf(buf,
	"%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x%02x\n",
		key[0],  key[1],  key[2],  key[3],
		key[4],  key[5],  key[6],  key[7],
		key[8],  key[9],  key[10], key[11],
		key[12], key[13], key[14], key[15]);
}
/**
 * inv_gyro_matrix_show() - show orientation matrix
 */
static ssize_t inv_gyro_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	m = st->plat_data.orientation;
	return sprintf(buf,
	"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}
/**
 * inv_accl_matrix_show() - show orientation matrix
 */
static ssize_t inv_accl_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_ACCEL)
		m = st->plat_data.secondary_orientation;
	else
		m = st->plat_data.orientation;
	return sprintf(buf,
	"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}
/**
 * inv_compass_matrix_show() - show orientation matrix
 */
static ssize_t inv_compass_matrix_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	signed char *m;
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_COMPASS)
		m = st->plat_data.secondary_orientation;
	else
		return -1;
	return sprintf(buf,
	"%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
		m[0],  m[1],  m[2],  m[3], m[4], m[5], m[6], m[7], m[8]);
}

/**
 * inv_flick_lower_store() -  calling this function will store current
 *                        flick lower bound
 */
static ssize_t inv_flick_lower_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtol(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;

	result = mem_w_key(KEY_FLICK_LOWER, 4, p);
	if (result)
		return result;
	st->flick.lower = data;
	return count;
}

/**
 * inv_flick_lower_show() -  calling this function will show current
 *                        flick lower bound
 */
static ssize_t inv_flick_lower_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->flick.lower);
}
/**
 * inv_flick_upper_store() -  calling this function will store current
 *                        flick upper bound
 */
static ssize_t inv_flick_upper_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_UPPER, 4, p);
	if (result)
		return result;
	st->flick.upper = data;
	return count;
}

/**
 * inv_flick_upper_show() -  calling this function will show current
 *                        flick upper bound
 */
static ssize_t inv_flick_upper_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->flick.upper);
}
/**
 * inv_flick_counter_store() -  calling this function will store current
 *                        flick counter value
 */
static ssize_t inv_flick_counter_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_COUNTER, 4, p);
	if (result)
		return result;
	st->flick.counter = data;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

	return count;
}

/**
<<<<<<< HEAD
 * inv_attr_show() -  calling this function will show current
 *                        dmp parameters.
 */
static ssize_t inv_attr_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int result;
	s8 *m;

	switch (this_attr->address) {
	case ATTR_DMP_SMD_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.smd_enable);
	case ATTR_DMP_SMD_THLD:
		return sprintf(buf, "%d\n", st->smd.threshold);
	case ATTR_DMP_SMD_DELAY_THLD:
		return sprintf(buf, "%d\n", st->smd.delay);
	case ATTR_DMP_SMD_DELAY_THLD2:
		return sprintf(buf, "%d\n", st->smd.delay2);
	case ATTR_DMP_TAP_ON:
		return sprintf(buf, "%d\n", st->chip_config.tap_on);
	case ATTR_DMP_TAP_THRESHOLD:
		return sprintf(buf, "%d\n", st->tap.thresh);
	case ATTR_DMP_TAP_MIN_COUNT:
		return sprintf(buf, "%d\n", st->tap.min_count);
	case ATTR_DMP_TAP_TIME:
		return sprintf(buf, "%d\n", st->tap.time);
	case ATTR_DMP_DISPLAY_ORIENTATION_ON:
		return sprintf(buf, "%d\n",
			st->chip_config.display_orient_on);

	case ATTR_DMP_ON:
		return sprintf(buf, "%d\n", st->chip_config.dmp_on);
	case ATTR_DMP_INT_ON:
		return sprintf(buf, "%d\n", st->chip_config.dmp_int_on);
	case ATTR_DMP_EVENT_INT_ON:
		return sprintf(buf, "%d\n", st->chip_config.dmp_event_int_on);
	case ATTR_DMP_OUTPUT_RATE:
		return sprintf(buf, "%d\n",
				st->chip_config.dmp_output_rate);
	case ATTR_DMP_QUATERNION_ON:
		return sprintf(buf, "%d\n", st->chip_config.quaternion_on);

	case ATTR_MOTION_LPA_ON:
		return sprintf(buf, "%d\n", st->mot_int.mot_on);
	case ATTR_MOTION_LPA_FREQ:{
		const char *f[] = {"1.25", "5", "20", "40"};
		return sprintf(buf, "%s\n", f[st->chip_config.lpa_freq]);
	}
	case ATTR_MOTION_LPA_DURATION:
		return sprintf(buf, "%d\n", st->mot_int.mot_dur);
	case ATTR_MOTION_LPA_THRESHOLD:
		return sprintf(buf, "%d\n", st->mot_int.mot_thr);

	case ATTR_SELF_TEST_SAMPLES:
		return sprintf(buf, "%d\n", st->self_test.samples);
	case ATTR_SELF_TEST_THRESHOLD:
		return sprintf(buf, "%d\n", st->self_test.threshold);
	case ATTR_GYRO_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.gyro_enable);
	case ATTR_ACCL_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.accl_enable);
	case ATTR_COMPASS_ENABLE:
		return sprintf(buf, "%d\n", st->chip_config.compass_enable);
	case ATTR_POWER_STATE:
		return sprintf(buf, "%d\n", !fake_asleep);
	case ATTR_FIRMWARE_LOADED:
		return sprintf(buf, "%d\n", st->chip_config.firmware_loaded);
	case ATTR_SAMPLING_FREQ:
		return sprintf(buf, "%d\n", st->chip_config.new_fifo_rate);

	case ATTR_SELF_TEST:
		if (st->chip_config.enable)
			return -EBUSY;
		mutex_lock(&indio_dev->mlock);
		if (INV_MPU3050 == st->chip_type)
			result = 1;
		else
			result = inv_hw_self_test(st);
		mutex_unlock(&indio_dev->mlock);
		return sprintf(buf, "%d\n", result);

	case ATTR_GYRO_MATRIX:
		m = st->plat_data.orientation;
		return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	case ATTR_ACCL_MATRIX:
		if (st->plat_data.sec_slave_type == SECONDARY_SLAVE_TYPE_ACCEL)
			m = st->plat_data.secondary_orientation;
		else
			m = st->plat_data.orientation;
		return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	case ATTR_COMPASS_MATRIX:
		if (st->plat_data.sec_slave_type ==
				SECONDARY_SLAVE_TYPE_COMPASS)
			m = st->plat_data.secondary_orientation;
		else
			return -ENODEV;
		return sprintf(buf, "%d,%d,%d,%d,%d,%d,%d,%d,%d\n",
			m[0], m[1], m[2], m[3], m[4], m[5], m[6], m[7], m[8]);
	case ATTR_SECONDARY_NAME:{
	const char *n[] = {"0", "AK8975", "AK8972", "AK8963", "BMA250"};
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id)
		return sprintf(buf, "%s\n", n[1]);
	else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id)
		return sprintf(buf, "%s\n", n[2]);
	else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id)
		return sprintf(buf, "%s\n", n[3]);
	else if (ACCEL_ID_BMA250 == st->plat_data.sec_slave_id)
		return sprintf(buf, "%s\n", n[4]);
	else
		return sprintf(buf, "%s\n", n[0]);
	}

#ifdef CONFIG_INV_TESTING
	case ATTR_REG_WRITE:
		return sprintf(buf, "1\n");
	case ATTR_DEBUG_SMD_EXE_STATE:
	{
		u8 d[2];

		result = st->set_power_state(st, true);
		mpu_memory_read(st, st->i2c_addr,
				inv_dmp_get_address(KEY_SMD_EXE_STATE), 2, d);
		return sprintf(buf, "%d\n", (short)be16_to_cpup((__be16 *)(d)));
	}
	case ATTR_DEBUG_SMD_DELAY_CNTR:
	{
		u8 d[4];

		result = st->set_power_state(st, true);
		mpu_memory_read(st, st->i2c_addr,
				inv_dmp_get_address(KEY_SMD_DELAY_CNTR), 4, d);
		return sprintf(buf, "%d\n", (int)be32_to_cpup((__be32 *)(d)));
	}
#endif
	default:
		return -EPERM;
	}
}

/**
 * inv_dmp_display_orient_show() -  calling this function will
 *			show orientation This event must use poll.
 */
static ssize_t inv_dmp_display_orient_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct inv_mpu_iio_s *st = iio_priv(dev_get_drvdata(dev));
	return sprintf(buf, "%d\n", st->display_orient_data);
}

/**
 * inv_accel_motion_show() -  calling this function showes motion interrupt.
 *                         This event must use poll.
 */
static ssize_t inv_accel_motion_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}

/**
 * inv_smd_show() -  calling this function showes smd interrupt.
 *                         This event must use poll.
 */
static ssize_t inv_smd_show(struct device *dev,
=======
 * inv_flick_counter_show() -  calling this function will show current
 *                        flick counter value
 */
static ssize_t inv_flick_counter_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->flick.counter);
}

/**
 * inv_flick_int_on_store() -  calling this function will store current
 *                        flick interrupt on value
 */
static ssize_t inv_flick_int_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long result, data;
	unsigned char d[4];
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data)
		/* Use interrupt to signal when gesture was observed */
		d[0] = DIND40+4;
	else
		d[0] = DINAA0+8;
	result = mem_w_key(KEY_CGNOTICE_INTR, 1, d);
	if (result)
		return result;
	st->chip_config.flick_int_on = data;
	return count;
}

/**
 * inv_flick_int_on_show() -  calling this function will show current
 *                        flick interrupt on value
 */
static ssize_t inv_flick_int_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.flick_int_on);
}
/**
 * inv_flick_axis_store() -  calling this function will store current
 *                        flick axis value
 */
static ssize_t inv_flick_axis_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long result, data;
	unsigned char d[4];
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;

	if (data == 0)
		d[0] = DINBC2;
	else if (data == 2)
		d[2] = DINBC6;
	else
		d[0] = DINBC4;
	result = mem_w_key(KEY_CFG_FLICK_IN, 1, d);
	if (result)
		return result;
	st->flick.axis = data;

	return count;
}

/**
 * inv_flick_axis_show() -  calling this function will show current
 *                        flick axis value
 */
static ssize_t inv_flick_axis_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->flick.axis);
}
/**
 * inv_flick_msg_on_store() -  calling this function will store current
 *                        flick message on value
 */
static ssize_t inv_flick_msg_on_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;
	if (data)
		data = DATA_MSG_ON;
	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_FLICK_MSG, 4, p);
	if (result)
		return result;
	st->flick.msg_on = data;

	return count;
}

/**
 * inv_flick_msg_on_show() -  calling this function will show current
 *                        flick message on value
 */
static ssize_t inv_flick_msg_on_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->flick.msg_on);
}

/**
 * inv_pedometer_steps_store() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_steps_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_PEDSTD_STEPCTR, 4, p);
	if (result)
		return result;

	return count;
}

/**
 * inv_pedometer_steps_show() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_steps_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int result, data;
	unsigned char d[4];
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
		inv_dmp_get_address(KEY_D_PEDSTD_STEPCTR), 4, d);
	if (result)
		return result;
	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%d\n", data);
}
/**
 * inv_pedometer_time_store() -  calling this function will store current
 *                        pedometer time into MPU memory
 */
static ssize_t inv_pedometer_time_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	unsigned int result, data, out;
	unsigned char *p;
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = kstrtoul(buf, 10, (long unsigned int *)&data);
	if (result)
		return result;

	out = cpu_to_be32p(&data);
	p = (unsigned char *)&out;
	result = mem_w_key(KEY_D_PEDSTD_TIMECTR, 4, p);
	if (result)
		return result;
	return count;
}
/**
 * inv_pedometer_time_show() -  calling this function will store current
 *                        pedometer steps into MPU memory
 */
static ssize_t inv_pedometer_time_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	int result, data;
	unsigned char d[4];
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	if (st->chip_config.is_asleep)
		return -EPERM;
	result = mpu_memory_read(st->sl_handle, st->i2c_addr,
		inv_dmp_get_address(KEY_D_PEDSTD_TIMECTR), 4, d);
	if (result)
		return result;
	data = be32_to_cpup((int *)d);
	return sprintf(buf, "%d\n", data*20);
}

/**
 * inv_dmp_flick_show() -  calling this function will show flick event.
 *                         This event must use poll.
 */
static ssize_t inv_dmp_flick_show(struct device *dev,
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	struct device_attribute *attr, char *buf)
{
	return sprintf(buf, "1\n");
}
<<<<<<< HEAD
=======
/**
 * inv_dmp_orient_show() -  calling this function will show orientation
 *                         This event must use poll.
 */
static ssize_t inv_dmp_orient_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->orient_data);
}

/**
 * inv_dmp_display_orient_show() -  calling this function will
 *			show orientation This event must use poll.
 */
static ssize_t inv_dmp_display_orient_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->display_orient_data);
}
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

/**
 * inv_dmp_tap_show() -  calling this function will show tap
 *                         This event must use poll.
 */
static ssize_t inv_dmp_tap_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
<<<<<<< HEAD
	struct inv_mpu_iio_s *st = iio_priv(dev_get_drvdata(dev));
	return sprintf(buf, "%d\n", st->tap_data);
}

=======
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->tap_data);
}
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
/**
 *  inv_temperature_show() - Read temperature data directly from registers.
 */
static ssize_t inv_temperature_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
<<<<<<< HEAD

	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
	struct inv_reg_map_s *reg;
	int result, cur_scale, cur_off;
	short temp;
	long scale_t;
	u8 data[2];
	const long scale[] = {3834792L, 3158064L, 3340827L};
	const long offset[] = {5383314L, 2394184L, 1376256L};

	reg = &st->reg;
	mutex_lock(&indio_dev->mlock);
	if (!st->chip_config.enable)
		result = st->set_power_state(st, true);
	else
		result = 0;
	if (result) {
		mutex_unlock(&indio_dev->mlock);
		return result;
	}
	result = inv_i2c_read(st, reg->temperature, 2, data);
	if (!st->chip_config.enable)
		result |= st->set_power_state(st, false);
	mutex_unlock(&indio_dev->mlock);
	if (result) {
		pr_err("Could not read temperature register.\n");
		return result;
	}
	temp = (signed short)(be16_to_cpup((short *)&data[0]));
	switch (st->chip_type) {
	case INV_MPU3050:
		cur_scale = scale[0];
		cur_off   = offset[0];
		break;
	case INV_MPU6050:
		cur_scale = scale[1];
		cur_off   = offset[1];
		break;
	case INV_MPU6500:
		cur_scale = scale[2];
		cur_off   = offset[2];
		break;
	default:
		return -EINVAL;
	};
	scale_t = cur_off +
		inv_q30_mult((int)temp << MPU_TEMP_SHIFT, cur_scale);

	INV_I2C_INC_TEMPREAD(1);

	return sprintf(buf, "%ld %lld\n", scale_t, get_time_ns());
}

/**
 * inv_firmware_loaded() -  calling this function will change
 *                        firmware load
 */
static int inv_firmware_loaded(struct inv_mpu_iio_s *st, int data)
{
	if (data)
		return -EINVAL;
	st->chip_config.firmware_loaded = 0;
	st->chip_config.dmp_on = 0;
	st->chip_config.quaternion_on = 0;

	return 0;
}

static int inv_switch_gyro_engine(struct inv_mpu_iio_s *st, bool en)
{
	return inv_switch_engine(st, en, BIT_PWR_GYRO_STBY);
}

static int inv_switch_accl_engine(struct inv_mpu_iio_s *st, bool en)
{
	return inv_switch_engine(st, en, BIT_PWR_ACCL_STBY);
}

/**
 *  inv_gyro_enable() - Enable/disable gyro.
 */
static int inv_gyro_enable(struct inv_mpu_iio_s *st,
				 struct iio_buffer *ring, bool en)
{
	if (en == st->chip_config.gyro_enable)
		return 0;
	if (!en) {
=======
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct inv_reg_map_s *reg;
	int result;
	short temp;
	long scale_t;
	unsigned char data[2];
	reg = &st->reg;

	if (st->chip_config.is_asleep)
		return -EPERM;
	result = inv_i2c_read(st, reg->temperature, 2, data);
	if (result) {
		printk(KERN_ERR "Could not read temperature register.\n");
		return result;
	}
	temp = (signed short)(be16_to_cpup((short *)&data[0]));

	if (INV_MPU3050 == st->chip_type)
		scale_t = MPU3050_TEMP_OFFSET +
			inv_q30_mult((long)temp << MPU_TEMP_SHIFT,
				MPU3050_TEMP_SCALE);
	else
		scale_t = MPU6050_TEMP_OFFSET +
			inv_q30_mult((long)temp << MPU_TEMP_SHIFT,
				MPU6050_TEMP_SCALE);
	return sprintf(buf, "%ld %lld\n", scale_t, iio_get_time_ns());
}
static int inv_switch_gyro_engine(struct inv_gyro_state_s *st, int en)
{
	struct inv_reg_map_s *reg;
	unsigned char data, p;
	int result;
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type) {
		if (en) {
			data = INV_CLK_PLL;
			p = (BITS_3050_POWER1 | data);
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
			p = (BITS_3050_POWER2 | data);
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
			p = data;
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
		} else {
			p = BITS_3050_GYRO_STANDBY;
			result = inv_i2c_single_write(st, reg->pwr_mgmt_1, p);
			if (result)
				return result;
		}
	} else {
		result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &data);
		if (result)
			return result;
		if (en)
			data &= (~BIT_PWR_GYRO_STBY);
		else
			data |= BIT_PWR_GYRO_STBY;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
		msleep(SENSOR_UP_TIME);
	}
	if (en)
		st->chip_config.clk_src = INV_CLK_PLL;
	else
		st->chip_config.clk_src = INV_CLK_INTERNAL;

	return 0;
}
static int inv_switch_accl_engine(struct inv_gyro_state_s *st, int en)
{
	struct inv_reg_map_s *reg;
	unsigned char data;
	int result;
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type) {
		if (NULL == st->mpu_slave)
			return -EPERM;
		if (en)
			result = st->mpu_slave->resume(st);
		else
			result = st->mpu_slave->suspend(st);
		if (result)
			return result;
	} else {
		result = inv_i2c_read(st, reg->pwr_mgmt_2, 1, &data);
		if (result)
			return result;
		if (en)
			data &= (~BIT_PWR_ACCL_STBY);
		else
			data |= BIT_PWR_ACCL_STBY;
		result = inv_i2c_single_write(st, reg->pwr_mgmt_2, data);
		if (result)
			return result;
		msleep(SENSOR_UP_TIME);
	}
	return 0;
}

/**
 *  inv_gyro_enable_store() - Enable/disable gyro.
 */
static ssize_t inv_gyro_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data,  en;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int result;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (st->chip_config.enable)
		return -EPERM;

	result = kstrtoul(buf, 10, &data);
	if (result)
		return -EINVAL;
	en = !!data;
	if (en == st->chip_config.gyro_enable)
		return count;
	result = inv_switch_gyro_engine(st, en);
	if (result)
		return result;

	if (0 == en) {
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		st->chip_config.gyro_fifo_enable = 0;
		clear_bit(INV_MPU_SCAN_GYRO_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_GYRO_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_GYRO_Z, ring->scan_mask);
	}
	st->chip_config.gyro_enable = en;
<<<<<<< HEAD

	return 0;
}

/**
 *  inv_accl_enable() - Enable/disable accl.
 */
static ssize_t inv_accl_enable(struct inv_mpu_iio_s *st,
				 struct iio_buffer *ring, bool en)
{
	if (en == st->chip_config.accl_enable)
		return 0;
	if (!en) {
=======
	return count;
}
/**
 *  inv_gyro_enable_show() - Check if the FIFO and ring buffer are enabled.
 */
static ssize_t inv_gyro_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.gyro_enable);
}

/**
 *  inv_accl_enable_store() - Enable/disable accl.
 */
static ssize_t inv_accl_enable_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long en, data;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	int result;

	if (st->chip_config.is_asleep)
		return -EPERM;
	if (st->chip_config.enable)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return -EINVAL;
	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.accl_enable)
		return count;
	result = inv_switch_accl_engine(st, en);
	if (result)
		return result;
	st->chip_config.accl_enable = en;
	if (0 == en) {
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		st->chip_config.accl_fifo_enable = 0;
		clear_bit(INV_MPU_SCAN_ACCL_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_ACCL_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_ACCL_Z, ring->scan_mask);
	}
<<<<<<< HEAD
	st->chip_config.accl_enable = en;

	return 0;
}

/**
 * inv_compass_enable() -  calling this function will store compass
 *                         enable
 */
static ssize_t inv_compass_enable(struct inv_mpu_iio_s *st,
				 struct iio_buffer *ring, bool en)
{
	if (en == st->chip_config.compass_enable)
		return 0;
	if (!en) {
		st->chip_config.compass_fifo_enable = 0;
		clear_bit(INV_MPU_SCAN_MAGN_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_MAGN_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_MAGN_Z, ring->scan_mask);
	}
	st->chip_config.compass_enable = en;

	return 0;
}

/**
 * inv_attr_store() -  calling this function will store current
 *                        non-dmp parameter settings
 */
static ssize_t inv_attr_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	struct iio_dev_attr *this_attr = to_iio_dev_attr(attr);
	int data;
	u8  d;
	int result;

	mutex_lock(&indio_dev->mlock);
	if (st->chip_config.enable) {
		result = -EBUSY;
		goto attr_store_fail;
	}
	if (this_attr->address <= ATTR_MOTION_LPA_THRESHOLD) {
		result = st->set_power_state(st, true);
		if (result)
			goto attr_store_fail;
	}

	result = kstrtoint(buf, 10, &data);
	if (result)
		goto attr_store_fail;
	switch (this_attr->address) {
	case ATTR_MOTION_LPA_ON:
		if (INV_MPU6500 == st->chip_type) {
			if (data)
				/* enable and put in MPU6500 mode */
				d = BIT_ACCEL_INTEL_ENABLE
					| BIT_ACCEL_INTEL_MODE;
			else
				d = 0;
			result = inv_i2c_single_write(st,
						REG_6500_ACCEL_INTEL_CTRL, d);
			if (result)
				goto attr_store_fail;
		}
		st->mot_int.mot_on = !!data;
		st->chip_config.lpa_mode = !!data;
		break;
	case ATTR_MOTION_LPA_FREQ:
		result = inv_lpa_freq(st, data);
		break;
	case ATTR_MOTION_LPA_DURATION:
		if (INV_MPU6500 != st->chip_type) {
			result = inv_i2c_single_write(st, REG_ACCEL_MOT_DUR,
					      MPU6050_MOTION_DUR_DEFAULT);
			if (result)
				goto attr_store_fail;
		}
		st->mot_int.mot_dur = data;
		break;
	case ATTR_MOTION_LPA_THRESHOLD:
		if ((data > MPU6XXX_MAX_MOTION_THRESH) || (data < 0)) {
			result = -EINVAL;
			goto attr_store_fail;
		}
		d = (u8)(data >> MPU6XXX_MOTION_THRESH_SHIFT);
		data = (d << MPU6XXX_MOTION_THRESH_SHIFT);
		result = inv_i2c_single_write(st, REG_ACCEL_MOT_THR, d);
		if (result)
			goto attr_store_fail;
		st->mot_int.mot_thr = data;
		break;
	/* from now on, power is not turned on */
	case ATTR_SELF_TEST_SAMPLES:
		if (data > ST_MAX_SAMPLES || data < 0) {
			result = -EINVAL;
			goto attr_store_fail;
		}
		st->self_test.samples = data;
		break;
	case ATTR_SELF_TEST_THRESHOLD:
		if (data > ST_MAX_THRESHOLD || data < 0) {
			result = -EINVAL;
			goto attr_store_fail;
		}
		st->self_test.threshold = data;
	case ATTR_GYRO_ENABLE:
		result = st->gyro_en(st, ring, !!data);
		break;
	case ATTR_ACCL_ENABLE:
		result = st->accl_en(st, ring, !!data);
		break;
	case ATTR_COMPASS_ENABLE:
		result = inv_compass_enable(st, ring, !!data);
		break;
	case ATTR_POWER_STATE:
		fake_asleep = !data;
		break;
	case ATTR_FIRMWARE_LOADED:
		result = inv_firmware_loaded(st, data);
		break;
	case ATTR_SAMPLING_FREQ:
		result = inv_fifo_rate_store(st, data);
		break;
	default:
		result = -EINVAL;
		goto attr_store_fail;
	};

attr_store_fail:
	if ((this_attr->address <= ATTR_MOTION_LPA_THRESHOLD) &&
					(!st->chip_config.enable))
		result |= st->set_power_state(st, false);
	mutex_unlock(&indio_dev->mlock);
	if (result)
		return result;

	return count;
}

#ifdef CONFIG_INV_TESTING
/**
 * inv_reg_write_store() - register write command for testing.
 *                         Format: WSRRDD, where RR is the register in hex,
 *                                         and DD is the data in hex.
 */
static ssize_t inv_reg_write_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
	u32 result;
	u8 wreg, wval;
	int temp;
	char local_buf[10];

	if ((buf[0] != 'W' && buf[0] != 'w') ||
	    (buf[1] != 'S' && buf[1] != 's'))
		return -EINVAL;
	if (strlen(buf) < 6)
		return -EINVAL;

	strncpy(local_buf, buf, 7);
	local_buf[6] = 0;
	result = sscanf(&local_buf[4], "%x", &temp);
	if (result == 0)
		return -EINVAL;
	wval = temp;
	local_buf[4] = 0;
	sscanf(&local_buf[2], "%x", &temp);
	if (result == 0)
		return -EINVAL;
	wreg = temp;

	result = inv_i2c_single_write(st, wreg, wval);
	if (result)
		return result;

	return count;
}
#endif /* CONFIG_INV_TESTING */

#define INV_MPU_CHAN(_type, _channel2, _index)                \
	{                                                         \
		.type = _type,                                        \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.info_mask =  (IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT | \
				IIO_CHAN_INFO_SCALE_SHARED_BIT),              \
		.scan_index = _index,                                 \
		.scan_type  = IIO_ST('s', 16, 16, 0)                  \
	}

#define INV_ACCL_CHAN(_type, _channel2, _index)                \
	{                                                         \
		.type = _type,                                        \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.info_mask =  (IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT | \
				IIO_CHAN_INFO_SCALE_SHARED_BIT |     \
				IIO_CHAN_INFO_OFFSET_SEPARATE_BIT),  \
		.scan_index = _index,                                 \
		.scan_type  = IIO_ST('s', 16, 16, 0)                  \
	}

#define INV_MPU_QUATERNION_CHAN(_channel2, _index)            \
	{                                                         \
		.type = IIO_QUATERNION,                               \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.scan_index = _index,                                 \
		.scan_type  = IIO_ST('s', 32, 32, 0)                  \
	}

#define INV_MPU_MAGN_CHAN(_channel2, _index)                  \
	{                                                         \
		.type = IIO_MAGN,                                     \
		.modified = 1,                                        \
		.channel2 = _channel2,                                \
		.info_mask =  IIO_CHAN_INFO_SCALE_SHARED_BIT,         \
		.scan_index = _index,                                 \
		.scan_type  = IIO_ST('s', 16, 16, 0)                  \
	}

static const struct iio_chan_spec inv_mpu_channels[] = {
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU_SCAN_TIMESTAMP),
	INV_MPU_CHAN(IIO_ANGL_VEL, IIO_MOD_X, INV_MPU_SCAN_GYRO_X),
	INV_MPU_CHAN(IIO_ANGL_VEL, IIO_MOD_Y, INV_MPU_SCAN_GYRO_Y),
	INV_MPU_CHAN(IIO_ANGL_VEL, IIO_MOD_Z, INV_MPU_SCAN_GYRO_Z),

	INV_ACCL_CHAN(IIO_ACCEL, IIO_MOD_X, INV_MPU_SCAN_ACCL_X),
	INV_ACCL_CHAN(IIO_ACCEL, IIO_MOD_Y, INV_MPU_SCAN_ACCL_Y),
	INV_ACCL_CHAN(IIO_ACCEL, IIO_MOD_Z, INV_MPU_SCAN_ACCL_Z),

	INV_MPU_QUATERNION_CHAN(IIO_MOD_R, INV_MPU_SCAN_QUAT_R),
	INV_MPU_QUATERNION_CHAN(IIO_MOD_X, INV_MPU_SCAN_QUAT_X),
	INV_MPU_QUATERNION_CHAN(IIO_MOD_Y, INV_MPU_SCAN_QUAT_Y),
	INV_MPU_QUATERNION_CHAN(IIO_MOD_Z, INV_MPU_SCAN_QUAT_Z),

	INV_MPU_MAGN_CHAN(IIO_MOD_X, INV_MPU_SCAN_MAGN_X),
	INV_MPU_MAGN_CHAN(IIO_MOD_Y, INV_MPU_SCAN_MAGN_Y),
	INV_MPU_MAGN_CHAN(IIO_MOD_Z, INV_MPU_SCAN_MAGN_Z),
};


/*constant IIO attribute */
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("10 20 50 100 200 500");

/* special sysfs */
static DEVICE_ATTR(reg_dump, S_IRUGO, inv_reg_dump_show, NULL);
static DEVICE_ATTR(temperature, S_IRUGO, inv_temperature_show, NULL);

/* event based sysfs, needs poll to read */
static DEVICE_ATTR(event_tap, S_IRUGO, inv_dmp_tap_show, NULL);
static DEVICE_ATTR(event_display_orientation, S_IRUGO,
	inv_dmp_display_orient_show, NULL);
static DEVICE_ATTR(event_accel_motion, S_IRUGO, inv_accel_motion_show, NULL);
static DEVICE_ATTR(event_smd, S_IRUGO, inv_smd_show, NULL);

/* DMP sysfs with power on/off */
static IIO_DEVICE_ATTR(smd_enable, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_SMD_ENABLE);
static IIO_DEVICE_ATTR(smd_threshold, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_SMD_THLD);
static IIO_DEVICE_ATTR(smd_delay_threshold, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_SMD_DELAY_THLD);
static IIO_DEVICE_ATTR(smd_delay_threshold2, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_SMD_DELAY_THLD2);
static IIO_DEVICE_ATTR(tap_on, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_TAP_ON);
static IIO_DEVICE_ATTR(tap_threshold, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_TAP_THRESHOLD);
static IIO_DEVICE_ATTR(tap_min_count, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_TAP_MIN_COUNT);
static IIO_DEVICE_ATTR(tap_time, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_TAP_TIME);
static IIO_DEVICE_ATTR(display_orientation_on, S_IRUGO | S_IWUSR,
	inv_attr_show, inv_dmp_attr_store, ATTR_DMP_DISPLAY_ORIENTATION_ON);

/* DMP sysfs without power on/off */
static IIO_DEVICE_ATTR(dmp_on, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_dmp_attr_store, ATTR_DMP_ON);
static IIO_DEVICE_ATTR(dmp_int_on, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_dmp_attr_store, ATTR_DMP_INT_ON);
static IIO_DEVICE_ATTR(dmp_event_int_on, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_dmp_attr_store, ATTR_DMP_EVENT_INT_ON);
static IIO_DEVICE_ATTR(dmp_output_rate, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_dmp_attr_store, ATTR_DMP_OUTPUT_RATE);
static IIO_DEVICE_ATTR(quaternion_on, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_dmp_attr_store, ATTR_DMP_QUATERNION_ON);

/* non DMP sysfs with power on/off */
static IIO_DEVICE_ATTR(motion_lpa_on, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_MOTION_LPA_ON);
static IIO_DEVICE_ATTR(motion_lpa_freq, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_MOTION_LPA_FREQ);
static IIO_DEVICE_ATTR(motion_lpa_duration, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_MOTION_LPA_DURATION);
static IIO_DEVICE_ATTR(motion_lpa_threshold, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_MOTION_LPA_THRESHOLD);

/* non DMP sysfs without power on/off */
static IIO_DEVICE_ATTR(self_test_samples, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_SELF_TEST_SAMPLES);
static IIO_DEVICE_ATTR(self_test_threshold, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_SELF_TEST_THRESHOLD);
static IIO_DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_GYRO_ENABLE);
static IIO_DEVICE_ATTR(accl_enable, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_ACCL_ENABLE);
static IIO_DEVICE_ATTR(compass_enable, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_COMPASS_ENABLE);
static IIO_DEVICE_ATTR(power_state, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_POWER_STATE);
static IIO_DEVICE_ATTR(firmware_loaded, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_FIRMWARE_LOADED);
static IIO_DEVICE_ATTR(sampling_frequency, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_attr_store, ATTR_SAMPLING_FREQ);

/* show method only sysfs but with power on/off */
static IIO_DEVICE_ATTR(self_test, S_IRUGO, inv_attr_show, NULL,
	ATTR_SELF_TEST);

/* show method only sysfs */
static IIO_DEVICE_ATTR(gyro_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_GYRO_MATRIX);
static IIO_DEVICE_ATTR(accl_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_ACCL_MATRIX);
static IIO_DEVICE_ATTR(compass_matrix, S_IRUGO, inv_attr_show, NULL,
	ATTR_COMPASS_MATRIX);
static IIO_DEVICE_ATTR(secondary_name, S_IRUGO, inv_attr_show, NULL,
	ATTR_SECONDARY_NAME);

#ifdef CONFIG_INV_TESTING
static IIO_DEVICE_ATTR(reg_write, S_IRUGO | S_IWUSR, inv_attr_show,
	inv_reg_write_store, ATTR_REG_WRITE);
/* smd debug related sysfs */
static IIO_DEVICE_ATTR(debug_smd_enable_testp1, S_IWUSR, NULL,
	inv_dmp_attr_store, ATTR_DEBUG_SMD_ENABLE_TESTP1);
static IIO_DEVICE_ATTR(debug_smd_enable_testp2, S_IWUSR, NULL,
	inv_dmp_attr_store, ATTR_DEBUG_SMD_ENABLE_TESTP2);
static IIO_DEVICE_ATTR(debug_smd_exe_state, S_IRUGO, inv_attr_show,
	NULL, ATTR_DEBUG_SMD_EXE_STATE);
static IIO_DEVICE_ATTR(debug_smd_delay_cntr, S_IRUGO, inv_attr_show,
	NULL, ATTR_DEBUG_SMD_DELAY_CNTR);
#endif

static const struct attribute *inv_gyro_attributes[] = {
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
	&dev_attr_reg_dump.attr,
	&dev_attr_temperature.attr,
	&iio_dev_attr_self_test_samples.dev_attr.attr,
	&iio_dev_attr_self_test_threshold.dev_attr.attr,
	&iio_dev_attr_gyro_enable.dev_attr.attr,
	&iio_dev_attr_power_state.dev_attr.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_dev_attr_self_test.dev_attr.attr,
	&iio_dev_attr_gyro_matrix.dev_attr.attr,
	&iio_dev_attr_secondary_name.dev_attr.attr,
#ifdef CONFIG_INV_TESTING
	&iio_dev_attr_reg_write.dev_attr.attr,
	&iio_dev_attr_debug_smd_enable_testp1.dev_attr.attr,
	&iio_dev_attr_debug_smd_enable_testp2.dev_attr.attr,
	&iio_dev_attr_debug_smd_exe_state.dev_attr.attr,
	&iio_dev_attr_debug_smd_delay_cntr.dev_attr.attr,
#endif
};

static const struct attribute *inv_mpu6050_attributes[] = {
	&dev_attr_event_display_orientation.attr,
	&dev_attr_event_tap.attr,
	&dev_attr_event_accel_motion.attr,
	&dev_attr_event_smd.attr,
	&iio_dev_attr_smd_enable.dev_attr.attr,
	&iio_dev_attr_smd_threshold.dev_attr.attr,
	&iio_dev_attr_smd_delay_threshold.dev_attr.attr,
	&iio_dev_attr_smd_delay_threshold2.dev_attr.attr,
	&iio_dev_attr_tap_on.dev_attr.attr,
	&iio_dev_attr_tap_threshold.dev_attr.attr,
	&iio_dev_attr_tap_min_count.dev_attr.attr,
	&iio_dev_attr_tap_time.dev_attr.attr,
	&iio_dev_attr_display_orientation_on.dev_attr.attr,
	&iio_dev_attr_dmp_on.dev_attr.attr,
	&iio_dev_attr_dmp_int_on.dev_attr.attr,
	&iio_dev_attr_dmp_event_int_on.dev_attr.attr,
	&iio_dev_attr_dmp_output_rate.dev_attr.attr,
	&iio_dev_attr_quaternion_on.dev_attr.attr,
	&iio_dev_attr_motion_lpa_on.dev_attr.attr,
	&iio_dev_attr_motion_lpa_freq.dev_attr.attr,
	&iio_dev_attr_motion_lpa_duration.dev_attr.attr,
	&iio_dev_attr_motion_lpa_threshold.dev_attr.attr,
	&iio_dev_attr_accl_enable.dev_attr.attr,
	&iio_dev_attr_firmware_loaded.dev_attr.attr,
	&iio_dev_attr_accl_matrix.dev_attr.attr,
};

static const struct attribute *inv_compass_attributes[] = {
	&iio_dev_attr_compass_enable.dev_attr.attr,
	&iio_dev_attr_compass_matrix.dev_attr.attr,
};

static const struct attribute *inv_mpu3050_attributes[] = {
	&iio_dev_attr_accl_enable.dev_attr.attr,
	&iio_dev_attr_accl_matrix.dev_attr.attr,
=======
	return count;
}
/**
 *  inv_accl_enable_show() - Check if the FIFO and ring buffer are enabled.
 */
static ssize_t inv_accl_enable_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	return sprintf(buf, "%d\n", st->chip_config.accl_enable);
}

/**
 * inv_compass_en_store() -  calling this function will store compass
 *                         enable
 */
static ssize_t inv_compass_en_store(struct device *dev,
	struct device_attribute *attr, const char *buf, size_t count)
{
	unsigned long data, result, en;
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	struct iio_buffer *ring = indio_dev->buffer;
	if (st->chip_config.is_asleep)
		return -EPERM;
	if (st->chip_config.enable)
		return -EPERM;
	result = kstrtoul(buf, 10, &data);
	if (result)
		return result;
	if (data)
		en = 1;
	else
		en = 0;
	if (en == st->chip_config.compass_enable)
		return count;
	st->chip_config.compass_enable = en;
	if (0 == en) {
		st->chip_config.compass_fifo_enable = 0;
		clear_bit(INV_MPU_SCAN_MAGN_X, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_MAGN_Y, ring->scan_mask);
		clear_bit(INV_MPU_SCAN_MAGN_Z, ring->scan_mask);
	}

	return count;
}
/**
 * inv_compass_en_show() -  calling this function will show compass
 *                         enable status
 */
static ssize_t inv_compass_en_show(struct device *dev,
	struct device_attribute *attr, char *buf)
{
	struct iio_dev *indio_dev = dev_get_drvdata(dev);
	struct inv_gyro_state_s *st = iio_priv(indio_dev);

	return sprintf(buf, "%d\n", st->chip_config.compass_enable);
}

static const struct iio_chan_spec gyro_channels[] = {
	/*there is only one gyro, with modifier X, Y, Z
	So it is not indexed. no modifier name, only simple, x, y,z
	the scale should be shared while bias is not so each
	axis has different bias*/
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU_SCAN_TIMESTAMP)
};

static const struct iio_chan_spec gyro_accel_channels[] = {
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_R,
		.scan_index = INV_MPU_SCAN_QUAT_R,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.scan_index = INV_MPU_SCAN_QUAT_X,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.scan_index = INV_MPU_SCAN_QUAT_Y,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.scan_index = INV_MPU_SCAN_QUAT_Z,
		.scan_type = IIO_ST('s', 32, 32, 0)
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU_SCAN_TIMESTAMP)
};
static const struct iio_chan_spec gyro_accel_compass_channels[] = {
	{
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ANGL_VEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_GYRO_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_ACCEL,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_CALIBBIAS_SEPARATE_BIT |
		IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_ACCL_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_MAGN_X,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_MAGN_Y,
		.scan_type = IIO_ST('s', 16, 16, 0)
	}, {
		.type = IIO_MAGN,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.info_mask = IIO_CHAN_INFO_SCALE_SHARED_BIT,
		.scan_index = INV_MPU_SCAN_MAGN_Z,
		.scan_type = IIO_ST('s', 16, 16, 0)
	},
	{
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_R,
		.scan_index = INV_MPU_SCAN_QUAT_R,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_X,
		.scan_index = INV_MPU_SCAN_QUAT_X,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_Y,
		.scan_index = INV_MPU_SCAN_QUAT_Y,
		.scan_type = IIO_ST('s', 32, 32, 0)
	}, {
		.type = IIO_QUATERNION,
		.modified = 1,
		.channel2 = IIO_MOD_Z,
		.scan_index = INV_MPU_SCAN_QUAT_Z,
		.scan_type = IIO_ST('s', 32, 32, 0)
	},
	IIO_CHAN_SOFT_TIMESTAMP(INV_MPU_SCAN_TIMESTAMP)
};

static struct inv_chip_chan_info chip_channel_info[] = {
	{
		.channels = gyro_channels,
		.num_channels = ARRAY_SIZE(gyro_channels),
	},
	{
		.channels = gyro_accel_channels,
		.num_channels = ARRAY_SIZE(gyro_accel_channels),
	},
	{
		.channels = gyro_accel_compass_channels,
		.num_channels = ARRAY_SIZE(gyro_accel_compass_channels),
	}
};

/*constant IIO attribute */
static IIO_CONST_ATTR_SAMP_FREQ_AVAIL("10 50 100 200 500");
static IIO_DEV_ATTR_SAMP_FREQ(S_IRUGO | S_IWUSR, inv_fifo_rate_show,
	inv_fifo_rate_store);
static DEVICE_ATTR(temperature, S_IRUGO, inv_temperature_show, NULL);
static DEVICE_ATTR(clock_source, S_IRUGO, inv_clk_src_show, NULL);
static DEVICE_ATTR(power_state, S_IRUGO | S_IWUSR, inv_power_state_show,
	inv_power_state_store);
static DEVICE_ATTR(firmware_loaded, S_IRUGO | S_IWUSR,
	inv_firmware_loaded_show, inv_firmware_loaded_store);
static DEVICE_ATTR(lpa_mode, S_IRUGO | S_IWUSR, inv_lpa_mode_show,
	inv_lpa_mode_store);
static DEVICE_ATTR(lpa_freq, S_IRUGO | S_IWUSR, inv_lpa_freq_show,
	inv_lpa_freq_store);
static DEVICE_ATTR(reg_dump, S_IRUGO, inv_reg_dump_show, NULL);
static DEVICE_ATTR(self_test, S_IRUGO, inv_self_test_show, NULL);
static DEVICE_ATTR(key, S_IRUGO, inv_key_show, NULL);
static DEVICE_ATTR(gyro_matrix, S_IRUGO, inv_gyro_matrix_show, NULL);
static DEVICE_ATTR(accl_matrix, S_IRUGO, inv_accl_matrix_show, NULL);
static DEVICE_ATTR(compass_matrix, S_IRUGO, inv_compass_matrix_show, NULL);
static DEVICE_ATTR(flick_lower, S_IRUGO | S_IWUSR, inv_flick_lower_show,
	inv_flick_lower_store);
static DEVICE_ATTR(flick_upper, S_IRUGO | S_IWUSR, inv_flick_upper_show,
	inv_flick_upper_store);
static DEVICE_ATTR(flick_counter, S_IRUGO | S_IWUSR, inv_flick_counter_show,
	inv_flick_counter_store);
static DEVICE_ATTR(flick_message_on, S_IRUGO | S_IWUSR, inv_flick_msg_on_show,
	inv_flick_msg_on_store);
static DEVICE_ATTR(flick_int_on, S_IRUGO | S_IWUSR, inv_flick_int_on_show,
	inv_flick_int_on_store);
static DEVICE_ATTR(flick_axis, S_IRUGO | S_IWUSR, inv_flick_axis_show,
	inv_flick_axis_store);
static DEVICE_ATTR(dmp_on, S_IRUGO | S_IWUSR, inv_dmp_on_show,
	inv_dmp_on_store);
static DEVICE_ATTR(dmp_int_on, S_IRUGO | S_IWUSR, inv_dmp_int_on_show,
	inv_dmp_int_on_store);
static DEVICE_ATTR(dmp_output_rate, S_IRUGO | S_IWUSR,
	inv_dmp_output_rate_show, inv_dmp_output_rate_store);
static DEVICE_ATTR(orientation_on, S_IRUGO | S_IWUSR,
	inv_orientation_on_show, inv_orientation_on_store);
static DEVICE_ATTR(quaternion_on, S_IRUGO | S_IWUSR,
	inv_quaternion_on_show, inv_quaternion_on_store);
static DEVICE_ATTR(display_orientation_on, S_IRUGO | S_IWUSR,
	inv_display_orient_on_show, inv_display_orient_on_store);
static DEVICE_ATTR(tap_on, S_IRUGO | S_IWUSR, inv_tap_on_show,
	inv_tap_on_store);
static DEVICE_ATTR(tap_time, S_IRUGO | S_IWUSR, inv_tap_time_show,
	inv_tap_time_store);
static DEVICE_ATTR(tap_min_count, S_IRUGO | S_IWUSR, inv_tap_min_count_show,
	inv_tap_min_count_store);
static DEVICE_ATTR(tap_threshold, S_IRUGO | S_IWUSR, inv_tap_threshold_show,
	inv_tap_threshold_store);
static DEVICE_ATTR(pedometer_time, S_IRUGO | S_IWUSR, inv_pedometer_time_show,
	inv_pedometer_time_store);
static DEVICE_ATTR(pedometer_steps, S_IRUGO | S_IWUSR,
		inv_pedometer_steps_show, inv_pedometer_steps_store);
static DEVICE_ATTR(event_flick, S_IRUGO, inv_dmp_flick_show, NULL);
static DEVICE_ATTR(event_orientation, S_IRUGO, inv_dmp_orient_show, NULL);
static DEVICE_ATTR(event_tap, S_IRUGO, inv_dmp_tap_show, NULL);
static DEVICE_ATTR(event_display_orientation, S_IRUGO,
			inv_dmp_display_orient_show, NULL);
static DEVICE_ATTR(gyro_enable, S_IRUGO | S_IWUSR, inv_gyro_enable_show,
	inv_gyro_enable_store);
static DEVICE_ATTR(accl_enable, S_IRUGO | S_IWUSR, inv_accl_enable_show,
	inv_accl_enable_store);
static DEVICE_ATTR(compass_enable, S_IRUGO | S_IWUSR, inv_compass_en_show,
	inv_compass_en_store);

static const struct attribute *inv_gyro_attributes[] = {
	&dev_attr_gyro_enable.attr,
	&dev_attr_temperature.attr,
	&dev_attr_clock_source.attr,
	&dev_attr_power_state.attr,
	&dev_attr_reg_dump.attr,
	&dev_attr_self_test.attr,
	&dev_attr_key.attr,
	&dev_attr_gyro_matrix.attr,
	&iio_dev_attr_sampling_frequency.dev_attr.attr,
	&iio_const_attr_sampling_frequency_available.dev_attr.attr,
};

static const struct attribute *inv_mpu6050_attributes[] = {
	&dev_attr_accl_enable.attr,
	&dev_attr_accl_matrix.attr,
	&dev_attr_firmware_loaded.attr,
	&dev_attr_lpa_mode.attr,
	&dev_attr_lpa_freq.attr,
	&dev_attr_flick_lower.attr,
	&dev_attr_flick_upper.attr,
	&dev_attr_flick_counter.attr,
	&dev_attr_flick_message_on.attr,
	&dev_attr_flick_int_on.attr,
	&dev_attr_flick_axis.attr,
	&dev_attr_dmp_on.attr,
	&dev_attr_dmp_int_on.attr,
	&dev_attr_dmp_output_rate.attr,
	&dev_attr_orientation_on.attr,
	&dev_attr_quaternion_on.attr,
	&dev_attr_display_orientation_on.attr,
	&dev_attr_tap_on.attr,
	&dev_attr_tap_time.attr,
	&dev_attr_tap_min_count.attr,
	&dev_attr_tap_threshold.attr,
	&dev_attr_pedometer_time.attr,
	&dev_attr_pedometer_steps.attr,
	&dev_attr_event_flick.attr,
	&dev_attr_event_orientation.attr,
	&dev_attr_event_display_orientation.attr,
	&dev_attr_event_tap.attr,
};

static const struct attribute *inv_compass_attributes[] = {
	&dev_attr_compass_matrix.attr,
	&dev_attr_compass_enable.attr,
};

static const struct attribute *inv_mpu3050_attributes[] = {
	&dev_attr_accl_matrix.attr,
	&dev_attr_accl_enable.attr,
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
};

static struct attribute *inv_attributes[ARRAY_SIZE(inv_gyro_attributes) +
				ARRAY_SIZE(inv_mpu6050_attributes) +
				ARRAY_SIZE(inv_compass_attributes) + 1];
<<<<<<< HEAD

=======
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
static const struct attribute_group inv_attribute_group = {
	.name = "mpu",
	.attrs = inv_attributes
};

static const struct iio_info mpu_info = {
	.driver_module = THIS_MODULE,
	.read_raw = &mpu_read_raw,
	.write_raw = &mpu_write_raw,
	.attrs = &inv_attribute_group,
};

/**
 *  inv_setup_compass() - Configure compass.
 */
<<<<<<< HEAD
static int inv_setup_compass(struct inv_mpu_iio_s *st)
{
	int result;
	u8 data[4];

	if (INV_MPU6050 == st->chip_type) {
		result = inv_i2c_read(st, REG_YGOFFS_TC, 1, data);
		if (result)
			return result;
		data[0] &= ~BIT_I2C_MST_VDDIO;
		if (st->plat_data.level_shifter)
			data[0] |= BIT_I2C_MST_VDDIO;
		/*set up VDDIO register */
		result = inv_i2c_single_write(st, REG_YGOFFS_TC, data[0]);
		if (result)
			return result;
	}
	/* set to bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG,
				st->plat_data.int_config | BIT_BYPASS_EN);
=======
static int inv_setup_compass(struct inv_gyro_state_s *st)
{
	int result;
	unsigned char data[4];

	result = inv_i2c_read(st, REG_YGOFFS_TC, 1, data);
	if (result)
		return result;
	data[0] &= ~BIT_I2C_MST_VDDIO;
	if (st->plat_data.level_shifter)
		data[0] |= BIT_I2C_MST_VDDIO;
	/*set up VDDIO register */
	result = inv_i2c_single_write(st, REG_YGOFFS_TC, data[0]);
	if (result)
		return result;
	/* set to bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG, BIT_BYPASS_EN);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (result)
		return result;
	/*read secondary i2c ID register */
	result = inv_secondary_read(REG_AKM_ID, 1, data);
	if (result)
		return result;
	if (data[0] != DATA_AKM_ID)
		return -ENXIO;
	/*set AKM to Fuse ROM access mode */
<<<<<<< HEAD
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_FR);
=======
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PW_FR);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (result)
		return result;
	result = inv_secondary_read(REG_AKM_SENSITIVITY, THREE_AXIS,
					st->chip_info.compass_sens);
	if (result)
		return result;
	/*revert to power down mode */
<<<<<<< HEAD
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PD);
	if (result)
		return result;
	pr_debug("%s senx=%d, seny=%d, senz=%d\n",
		 st->hw->name,
		 st->chip_info.compass_sens[0],
		 st->chip_info.compass_sens[1],
		 st->chip_info.compass_sens[2]);
	/*restore to non-bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG,
			st->plat_data.int_config);
=======
	result = inv_secondary_write(REG_AKM_MODE, DATA_AKM_MODE_PW_DN);
	if (result)
		return result;
	pr_err("senx=%d, seny=%d,senz=%d\n",
		st->chip_info.compass_sens[0],
		st->chip_info.compass_sens[1],
		st->chip_info.compass_sens[2]);
	/*restore to non-bypass mode */
	result = inv_i2c_single_write(st, REG_INT_PIN_CFG, 0);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (result)
		return result;

	/*setup master mode and master clock and ES bit*/
	result = inv_i2c_single_write(st, REG_I2C_MST_CTRL, BIT_WAIT_FOR_ES);
	if (result)
		return result;
	/* slave 0 is used to read data from compass */
	/*read mode */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_ADDR, BIT_I2C_READ|
		st->plat_data.secondary_i2c_addr);
	if (result)
		return result;
	/* AKM status register address is 2 */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_REG, REG_AKM_STATUS);
	if (result)
		return result;
	/* slave 0 is enabled at the beginning, read 8 bytes from here */
	result = inv_i2c_single_write(st, REG_I2C_SLV0_CTRL, BIT_SLV_EN |
				NUM_BYTES_COMPASS_SLAVE);
	if (result)
		return result;
	/*slave 1 is used for AKM mode change only*/
	result = inv_i2c_single_write(st, REG_I2C_SLV1_ADDR,
		st->plat_data.secondary_i2c_addr);
	if (result)
		return result;
	/* AKM mode register address is 0x0A */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_REG, REG_AKM_MODE);
	if (result)
		return result;
	/* slave 1 is enabled, byte length is 1 */
	result = inv_i2c_single_write(st, REG_I2C_SLV1_CTRL, BIT_SLV_EN | 1);
	if (result)
		return result;
	/* output data for slave 1 is fixed, single measure mode*/
	st->compass_scale = 1;
<<<<<<< HEAD
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id) {
		st->compass_st_upper = AKM8975_ST_Upper;
		st->compass_st_lower = AKM8975_ST_Lower;
		data[0] = DATA_AKM_MODE_SM;
	} else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id) {
		st->compass_st_upper = AKM8972_ST_Upper;
		st->compass_st_lower = AKM8972_ST_Lower;
		data[0] = DATA_AKM_MODE_SM;
	} else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id) {
		st->compass_st_upper = AKM8963_ST_Upper;
		st->compass_st_lower = AKM8963_ST_Lower;
		data[0] = DATA_AKM_MODE_SM |
			  (st->compass_scale << AKM8963_SCALE_SHIFT);
=======
	data[0] = 1;
	if (COMPASS_ID_AK8975 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8975_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8975_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8975_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8975_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8975_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8975_ST_Z_LW;
	} else if (COMPASS_ID_AK8972 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8972_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8972_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8972_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8972_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8972_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8972_ST_Z_LW;
	} else if (COMPASS_ID_AK8963 == st->plat_data.sec_slave_id) {
		st->compass_st_upper[0] = DATA_AKM8963_ST_X_UP;
		st->compass_st_upper[1] = DATA_AKM8963_ST_Y_UP;
		st->compass_st_upper[2] = DATA_AKM8963_ST_Z_UP;
		st->compass_st_lower[0] = DATA_AKM8963_ST_X_LW;
		st->compass_st_lower[1] = DATA_AKM8963_ST_Y_LW;
		st->compass_st_lower[2] = DATA_AKM8963_ST_Z_LW;
		data[0] |= (st->compass_scale << AKM8963_SCALE_SHIFT);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	}
	result = inv_i2c_single_write(st, REG_I2C_SLV1_DO, data[0]);
	if (result)
		return result;
	/* slave 0 and 1 timer action is enabled every sample*/
	result = inv_i2c_single_write(st, REG_I2C_MST_DELAY_CTRL,
				BIT_SLV0_DLY_EN | BIT_SLV1_DLY_EN);
	return result;
}

<<<<<<< HEAD
static void inv_setup_func_ptr(struct inv_mpu_iio_s *st)
{
	if (st->chip_type == INV_MPU3050) {
		st->set_power_state    = set_power_mpu3050;
		st->switch_gyro_engine = inv_switch_3050_gyro_engine;
		st->switch_accl_engine = inv_switch_3050_accl_engine;
		st->init_config        = inv_init_config_mpu3050;
		st->setup_reg          = inv_setup_reg_mpu3050;
	} else {
		st->set_power_state    = set_power_itg;
		st->switch_gyro_engine = inv_switch_gyro_engine;
		st->switch_accl_engine = inv_switch_accl_engine;
		st->init_config        = inv_init_config;
		st->setup_reg          = inv_setup_reg;
		/*MPU6XXX special functions */
		st->compass_en         = inv_compass_enable;
		st->quaternion_en      = inv_quaternion_on;
		st->gyro_en            = inv_gyro_enable;
		st->accl_en            = inv_accl_enable;
	}
}

static int inv_detect_6xxx(struct inv_mpu_iio_s *st)
{
	int result;
	u8 d;

	result = inv_i2c_read(st, REG_WHOAMI, 1, &d);
	if (result)
		return result;
	if (d == MPU6500_ID) {
		st->chip_type = INV_MPU6500;
		strcpy(st->name, "mpu6500");
	} else {
		strcpy(st->name, "mpu6050");
	}

	return 0;
}

/**
 *  inv_check_chip_type() - check and setup chip type.
 */
static int inv_check_chip_type(struct inv_mpu_iio_s *st,
		const struct i2c_device_id *id)
{
	struct inv_reg_map_s *reg;
	int result;
	int t_ind;

=======
/**
 *  inv_check_chip_type() - check and setup chip type.
 */
static int inv_check_chip_type(struct inv_gyro_state_s *st,
		const struct i2c_device_id *id)
{
	struct inv_reg_map_s *reg;
	int result, chan_index;
	int t_ind;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (!strcmp(id->name, "itg3500"))
		st->chip_type = INV_ITG3500;
	else if (!strcmp(id->name, "mpu3050"))
		st->chip_type = INV_MPU3050;
	else if (!strcmp(id->name, "mpu6050"))
		st->chip_type = INV_MPU6050;
	else if (!strcmp(id->name, "mpu9150"))
<<<<<<< HEAD
		st->chip_type = INV_MPU6050;
	else if (!strcmp(id->name, "mpu6500"))
		st->chip_type = INV_MPU6500;
	else if (!strcmp(id->name, "mpu9250"))
		st->chip_type = INV_MPU6500;
	else if (!strcmp(id->name, "mpu6xxx"))
		st->chip_type = INV_MPU6050;
	else
		return -EPERM;
	inv_setup_func_ptr(st);
	st->hw  = &hw_info[st->chip_type];
	st->mpu_slave = NULL;
	reg = &st->reg;
	st->setup_reg(reg);
	/* reset to make sure previous state are not there */
	result = inv_i2c_single_write(st, reg->pwr_mgmt_1, BIT_H_RESET);
	if (result)
		return result;
	msleep(POWER_UP_TIME);
	/* toggle power state */
	result = st->set_power_state(st, false);
	if (result)
		return result;

	result = st->set_power_state(st, true);
	if (result)
		return result;

	if (!strcmp(id->name, "mpu6xxx")) {
		/* for MPU6500, reading register need more time */
		msleep(POWER_UP_TIME);
		result = inv_detect_6xxx(st);
		if (result)
			return result;
	}

	switch (st->chip_type) {
	case INV_ITG3500:
		st->num_channels = INV_CHANNEL_NUM_GYRO;
		break;
	case INV_MPU6050:
	case INV_MPU6500:
		if (SECONDARY_SLAVE_TYPE_COMPASS ==
		    st->plat_data.sec_slave_type) {
			st->chip_config.has_compass = 1;
			st->num_channels =
				INV_CHANNEL_NUM_GYRO_ACCL_QUANTERNION_MAGN;
		} else {
			st->chip_config.has_compass = 0;
			st->num_channels =
				INV_CHANNEL_NUM_GYRO_ACCL_QUANTERNION;
		}
		break;
	case INV_MPU3050:
		if (SECONDARY_SLAVE_TYPE_ACCEL ==
		    st->plat_data.sec_slave_type) {
			if (ACCEL_ID_BMA250 == st->plat_data.sec_slave_id)
				inv_register_mpu3050_slave(st);
			st->num_channels = INV_CHANNEL_NUM_GYRO_ACCL;
		} else {
			st->num_channels = INV_CHANNEL_NUM_GYRO;
		}
		break;
	default:
		result = st->set_power_state(st, false);
		return -ENODEV;
	}
	switch (st->chip_type) {
	case INV_MPU6050:
		result = inv_get_silicon_rev_mpu6050(st);
		break;
	case INV_MPU6500:
		result = inv_get_silicon_rev_mpu6500(st);
		break;
	default:
		result = 0;
		break;
	}
	if (result) {
		pr_err("read silicon rev error\n");
		st->set_power_state(st, false);
		return result;
	}
	/* turn off the gyro engine after OTP reading */
	result = st->switch_gyro_engine(st, false);
	if (result)
		return result;
	result = st->switch_accl_engine(st, false);
	if (result)
		return result;
	if (st->chip_config.has_compass) {
		result = inv_setup_compass(st);
		if (result) {
			pr_err("compass setup failed\n");
			st->set_power_state(st, false);
=======
		st->chip_type = INV_MPU9150;
	else
		return -EPERM;
	st->hw  = (struct inv_hw_s *)(hw_info  + st->chip_type);
	st->mpu_slave = NULL;
	chan_index = CHAN_INDEX_GYRO;
	if (INV_MPU9150 == st->chip_type) {
		st->plat_data.sec_slave_type = SECONDARY_SLAVE_TYPE_COMPASS;
		st->plat_data.sec_slave_id = COMPASS_ID_AK8975;
		st->chip_config.has_compass = 1;
		chan_index = CHAN_INDEX_GYRO_ACCL_MAGN;
	}
	if (SECONDARY_SLAVE_TYPE_ACCEL == st->plat_data.sec_slave_type) {
		if (ACCEL_ID_BMA250 == st->plat_data.sec_slave_id)
			inv_register_bma250_slave(st);
		chan_index = CHAN_INDEX_GYRO_ACCL;
	}
	if (SECONDARY_SLAVE_TYPE_COMPASS == st->plat_data.sec_slave_type)
		st->chip_config.has_compass = 1;
	else
		st->chip_config.has_compass = 0;
	if (INV_MPU6050 == st->chip_type) {
		if (st->chip_config.has_compass)
			chan_index = CHAN_INDEX_GYRO_ACCL_MAGN;
		else
			chan_index = CHAN_INDEX_GYRO_ACCL;
	}
	st->chan_info = &chip_channel_info[chan_index];
	reg = &st->reg;
	if (INV_MPU3050 == st->chip_type)
		inv_setup_reg_mpu3050(reg);
	else
		inv_setup_reg(reg);
	st->chip_config.gyro_enable = 1;
	result = inv_set_power_state(st, 1);
	if (result)
		return result;

	if (INV_ITG3500 != st->chip_type && INV_MPU3050 != st->chip_type) {
		result = inv_get_silicon_rev_mpu6050(st);
		if (result) {
			inv_i2c_single_write(st, reg->pwr_mgmt_1,
					BIT_SLEEP | INV_CLK_PLL);
			return result;
		}
	}
	if (st->chip_config.has_compass) {
		result = inv_setup_compass(st);
		if (result) {
			inv_i2c_single_write(st, reg->pwr_mgmt_1,
					BIT_SLEEP | INV_CLK_PLL);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
			return result;
		}
	}

	t_ind = 0;
	memcpy(&inv_attributes[t_ind], inv_gyro_attributes,
<<<<<<< HEAD
		sizeof(inv_gyro_attributes));
	t_ind += ARRAY_SIZE(inv_gyro_attributes);

	if (INV_MPU3050 == st->chip_type && st->mpu_slave != NULL) {
		memcpy(&inv_attributes[t_ind], inv_mpu3050_attributes,
		       sizeof(inv_mpu3050_attributes));
=======
			sizeof(inv_gyro_attributes));
	t_ind = ARRAY_SIZE(inv_gyro_attributes);

	if (INV_MPU3050 == st->chip_type && st->mpu_slave != NULL) {
		memcpy(&inv_attributes[t_ind], inv_mpu3050_attributes,
				sizeof(inv_mpu3050_attributes));
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		t_ind += ARRAY_SIZE(inv_mpu3050_attributes);
		inv_attributes[t_ind] = NULL;
		return 0;
	}

<<<<<<< HEAD
	if ((INV_MPU6050 == st->chip_type) || (INV_MPU6500 == st->chip_type)) {
		memcpy(&inv_attributes[t_ind], inv_mpu6050_attributes,
		       sizeof(inv_mpu6050_attributes));
		t_ind += ARRAY_SIZE(inv_mpu6050_attributes);
	}

	if (st->chip_config.has_compass) {
		memcpy(&inv_attributes[t_ind], inv_compass_attributes,
		       sizeof(inv_compass_attributes));
		t_ind += ARRAY_SIZE(inv_compass_attributes);
	}
	inv_attributes[t_ind] = NULL;

=======
	if (chan_index > CHAN_INDEX_GYRO) {
		memcpy(&inv_attributes[t_ind], inv_mpu6050_attributes,
				sizeof(inv_mpu6050_attributes));
		t_ind += ARRAY_SIZE(inv_mpu6050_attributes);
	}

	if (chan_index > CHAN_INDEX_GYRO_ACCL) {
		memcpy(&inv_attributes[t_ind], inv_compass_attributes,
				sizeof(inv_compass_attributes));
		t_ind += ARRAY_SIZE(inv_compass_attributes);
	}
	inv_attributes[t_ind] = NULL;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	return 0;
}

/**
 *  inv_create_dmp_sysfs() - create binary sysfs dmp entry.
 */
static const struct bin_attribute dmp_firmware = {
	.attr = {
		.name = "dmp_firmware",
		.mode = S_IRUGO | S_IWUSR
	},
	.size = 4096,
	.read = inv_dmp_firmware_read,
	.write = inv_dmp_firmware_write,
};

static int inv_create_dmp_sysfs(struct iio_dev *ind)
{
	int result;
	result = sysfs_create_bin_file(&ind->dev.kobj, &dmp_firmware);
<<<<<<< HEAD

=======
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	return result;
}

/**
 *  inv_mpu_probe() - probe function.
 */
static int inv_mpu_probe(struct i2c_client *client,
	const struct i2c_device_id *id)
{
<<<<<<< HEAD
	struct inv_mpu_iio_s *st;
	struct iio_dev *indio_dev;
	int result;

	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENOSYS;
		pr_err("I2c function error\n");
=======
	struct inv_gyro_state_s *st;
	struct iio_dev *indio_dev;
	int result, reg_done;
	if (!i2c_check_functionality(client->adapter, I2C_FUNC_I2C)) {
		result = -ENODEV;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		goto out_no_free;
	}
	indio_dev = iio_allocate_device(sizeof(*st));
	if (indio_dev == NULL) {
<<<<<<< HEAD
		pr_err("memory allocation failed\n");
		result =  -ENOMEM;
		goto out_no_free;
	}
	st = iio_priv(indio_dev);
	st->client = client;
=======
		result =  -ENOMEM;
		goto out_no_free;
	}
	reg_done = 0;
	st = iio_priv(indio_dev);
	st->i2c = client;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	st->sl_handle = client->adapter;
	st->i2c_addr = client->addr;
	st->plat_data =
		*(struct mpu_platform_data *)dev_get_platdata(&client->dev);
	/* power is turned on inside check chip type*/
	result = inv_check_chip_type(st, id);
	if (result)
		goto out_free;
<<<<<<< HEAD

	result = st->init_config(indio_dev);
=======
	if (INV_MPU3050 == st->chip_type)
		result = inv_init_config_mpu3050(indio_dev);
	else
		result = inv_init_config(indio_dev);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (result) {
		dev_err(&client->adapter->dev,
			"Could not initialize device.\n");
		goto out_free;
	}
<<<<<<< HEAD
	result = st->set_power_state(st, false);
=======
	result = inv_set_power_state(st, 1);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	if (result) {
		dev_err(&client->adapter->dev,
			"%s could not be turned off.\n", st->hw->name);
		goto out_free;
	}

	/* Make state variables available to all _show and _store functions. */
	i2c_set_clientdata(client, indio_dev);
	indio_dev->dev.parent = &client->dev;
<<<<<<< HEAD
	if (!strcmp(id->name, "mpu6xxx"))
		indio_dev->name = st->name;
	else
		indio_dev->name = id->name;
	indio_dev->channels = inv_mpu_channels;
	indio_dev->num_channels = st->num_channels;

=======
	indio_dev->name = id->name;
	indio_dev->channels = st->chan_info->channels;
	indio_dev->num_channels = st->chan_info->num_channels;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	indio_dev->info = &mpu_info;
	indio_dev->modes = INDIO_DIRECT_MODE;
	indio_dev->currentmode = INDIO_DIRECT_MODE;

	result = inv_mpu_configure_ring(indio_dev);
<<<<<<< HEAD
	if (result) {
		pr_err("configure ring buffer fail\n");
		goto out_free;
	}
	result = iio_buffer_register(indio_dev, indio_dev->channels,
					indio_dev->num_channels);
	if (result) {
		pr_err("ring buffer register fail\n");
		goto out_unreg_ring;
	}
	st->irq = client->irq;
	result = inv_mpu_probe_trigger(indio_dev);
	if (result) {
		pr_err("trigger probe fail\n");
		goto out_remove_ring;
	}

	/* Tell the i2c counter, we have an IRQ */
	INV_I2C_SETIRQ(IRQ_MPU, client->irq);

	result = iio_device_register(indio_dev);
	if (result) {
		pr_err("IIO device register fail\n");
		goto out_remove_trigger;
	}

	if (INV_MPU6050 == st->chip_type ||
	    INV_MPU6500 == st->chip_type) {
		result = inv_create_dmp_sysfs(indio_dev);
		if (result) {
			pr_err("create dmp sysfs failed\n");
			goto out_unreg_iio;
		}
=======
	if (result)
		goto out_free;
	result = iio_buffer_register(indio_dev, st->chan_info->channels,
					st->chan_info->num_channels);
	if (result)
		goto out_unreg_ring;
	st->irq = client->irq;
	result = inv_mpu_probe_trigger(indio_dev);
	if (result)
		goto out_remove_ring;

	result = iio_device_register(indio_dev);
	if (result)
		goto out_remove_trigger;
	if (INV_MPU6050 == st->chip_type || INV_MPU9150 == st->chip_type) {
		result = inv_create_dmp_sysfs(indio_dev);
		if (result)
			goto out_unreg_iio;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	}

	INIT_KFIFO(st->timestamps);
	spin_lock_init(&st->time_stamp_lock);
<<<<<<< HEAD
	dev_info(&client->dev, "%s is ready to go!\n",
					indio_dev->name);

=======
	pr_info("%s: Probe name %s\n", __func__, id->name);
	dev_info(&client->adapter->dev, "%s is ready to go!\n", st->hw->name);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	return 0;
out_unreg_iio:
	iio_device_unregister(indio_dev);
out_remove_trigger:
	if (indio_dev->modes & INDIO_BUFFER_TRIGGERED)
		inv_mpu_remove_trigger(indio_dev);
out_remove_ring:
	iio_buffer_unregister(indio_dev);
out_unreg_ring:
	inv_mpu_unconfigure_ring(indio_dev);
out_free:
	iio_free_device(indio_dev);
out_no_free:
	dev_err(&client->adapter->dev, "%s failed %d\n", __func__, result);
<<<<<<< HEAD

	return -EIO;
}

static void inv_mpu_shutdown(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
	struct inv_reg_map_s *reg;
	int result;

	reg = &st->reg;
	dev_dbg(&client->adapter->dev, "Shutting down %s...\n", st->hw->name);

	/* reset to make sure previous state are not there */
	result = inv_i2c_single_write(st, reg->pwr_mgmt_1, BIT_H_RESET);
	if (result)
		dev_err(&client->adapter->dev, "Failed to reset %s\n",
			st->hw->name);
	msleep(POWER_UP_TIME);
	/* turn off power to ensure gyro engine is off */
	result = st->set_power_state(st, false);
	if (result)
		dev_err(&client->adapter->dev, "Failed to turn off %s\n",
			st->hw->name);
}

=======
	return -EIO;
}

>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
/**
 *  inv_mpu_remove() - remove function.
 */
static int inv_mpu_remove(struct i2c_client *client)
{
	struct iio_dev *indio_dev = i2c_get_clientdata(client);
<<<<<<< HEAD
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
	kfifo_free(&st->timestamps);
	iio_device_unregister(indio_dev);
	if (indio_dev->modes & INDIO_BUFFER_TRIGGERED)
		inv_mpu_remove_trigger(indio_dev);
=======
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
	kfifo_free(&st->timestamps);
	iio_device_unregister(indio_dev);
	inv_mpu_remove_trigger(indio_dev);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	iio_buffer_unregister(indio_dev);
	inv_mpu_unconfigure_ring(indio_dev);
	iio_free_device(indio_dev);

	dev_info(&client->adapter->dev, "inv-mpu-iio module removed.\n");
<<<<<<< HEAD

	return 0;
}

#ifdef CONFIG_PM
static int inv_mpu_resume(struct device *dev)
{
	struct inv_mpu_iio_s *st =
			iio_priv(i2c_get_clientdata(to_i2c_client(dev)));
	pr_debug("%s inv_mpu_resume\n", st->hw->name);
	return st->set_power_state(st, true);
}

static int inv_mpu_suspend(struct device *dev)
{
	struct inv_mpu_iio_s *st =
			iio_priv(i2c_get_clientdata(to_i2c_client(dev)));
	pr_debug("%s inv_mpu_suspend\n", st->hw->name);
	return st->set_power_state(st, false);
}
static const struct dev_pm_ops inv_mpu_pmops = {
	SET_SYSTEM_SLEEP_PM_OPS(inv_mpu_suspend, inv_mpu_resume)
};
#define INV_MPU_PMOPS (&inv_mpu_pmops)
#else
#define INV_MPU_PMOPS NULL
#endif /* CONFIG_PM */

static const u16 normal_i2c[] = { I2C_CLIENT_END };
=======
	return 0;
}
static const unsigned short normal_i2c[] = { I2C_CLIENT_END };
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
/* device id table is used to identify what device can be
 * supported by this driver
 */
static const struct i2c_device_id inv_mpu_id[] = {
	{"itg3500", INV_ITG3500},
	{"mpu3050", INV_MPU3050},
	{"mpu6050", INV_MPU6050},
	{"mpu9150", INV_MPU9150},
<<<<<<< HEAD
	{"mpu6500", INV_MPU6500},
	{"mpu9250", INV_MPU9250},
	{"mpu6xxx", INV_MPU6XXX},
=======
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	{}
};

MODULE_DEVICE_TABLE(i2c, inv_mpu_id);

static struct i2c_driver inv_mpu_driver = {
	.class = I2C_CLASS_HWMON,
	.probe		=	inv_mpu_probe,
	.remove		=	inv_mpu_remove,
<<<<<<< HEAD
	.shutdown	=	inv_mpu_shutdown,
=======
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	.id_table	=	inv_mpu_id,
	.driver = {
		.owner	=	THIS_MODULE,
		.name	=	"inv-mpu-iio",
<<<<<<< HEAD
		.pm     =       INV_MPU_PMOPS,
=======
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	},
	.address_list = normal_i2c,
};

static int __init inv_mpu_init(void)
{
	int result = i2c_add_driver(&inv_mpu_driver);
	if (result) {
<<<<<<< HEAD
		pr_err("failed\n");
=======
		pr_err("%s failed\n", __func__);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
		return result;
	}
	return 0;
}

static void __exit inv_mpu_exit(void)
{
	i2c_del_driver(&inv_mpu_driver);
}

module_init(inv_mpu_init);
module_exit(inv_mpu_exit);

MODULE_AUTHOR("Invensense Corporation");
MODULE_DESCRIPTION("Invensense device driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("inv-mpu-iio");
<<<<<<< HEAD

/**
 *  @}
 */
=======
/**
 *  @}
 */

>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
