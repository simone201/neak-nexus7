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
 *      @file    inv_mpu_trigger.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This file is part of inv mpu iio driver code
=======
 *      @file    inv_mpu3050.c
 *      @brief   A sysfs device driver for Invensense devices
 *      @details This file is part of inv_gyro driver code
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
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

#include "../../iio.h"
#include "../../sysfs.h"
#include "../../trigger.h"
<<<<<<< HEAD

#include "inv_mpu_iio.h"

/**
 * inv_mpu_data_rdy_trigger_set_state() set data ready interrupt state
=======
#include "inv_mpu_iio.h"

/**
 * inv_mpu_data_rdy_trigger_set_state() set datardy interrupt state
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
 **/
static int inv_mpu_data_rdy_trigger_set_state(struct iio_trigger *trig,
						bool state)
{
	struct iio_dev *indio_dev = trig->private_data;

	dev_dbg(&indio_dev->dev, "%s (%d)\n", __func__, state);
<<<<<<< HEAD

	return 0;
=======
	return set_inv_enable(indio_dev, state);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}

static const struct iio_trigger_ops inv_mpu_trigger_ops = {
	.owner = THIS_MODULE,
	.set_trigger_state = &inv_mpu_data_rdy_trigger_set_state,
};

int inv_mpu_probe_trigger(struct iio_dev *indio_dev)
{
	int ret;
<<<<<<< HEAD
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
=======
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

	st->trig = iio_allocate_trigger("%s-dev%d",
					indio_dev->name,
					indio_dev->id);
<<<<<<< HEAD
	if (st->trig == NULL)
		return -ENOMEM;
	st->trig->dev.parent = &st->client->dev;
=======
	if (st->trig == NULL) {
		ret = -ENOMEM;
		goto error_ret;
	}

	/* select default trigger */
	st->trig->dev.parent = &st->i2c->dev;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
	st->trig->private_data = indio_dev;
	st->trig->ops = &inv_mpu_trigger_ops;
	ret = iio_trigger_register(st->trig);

<<<<<<< HEAD
	if (ret) {
		iio_free_trigger(st->trig);
		return -EPERM;
	}
	indio_dev->trig = st->trig;

	return 0;
=======
	/* select default trigger */
	indio_dev->trig = st->trig;
	if (ret)
		goto error_ret;

	return 0;

error_ret:
	return ret;
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a
}

void inv_mpu_remove_trigger(struct iio_dev *indio_dev)
{
<<<<<<< HEAD
	struct inv_mpu_iio_s *st = iio_priv(indio_dev);
=======
	struct inv_gyro_state_s *st = iio_priv(indio_dev);
>>>>>>> 990270e2da9e7ed84fad1e9e95c3b83ed206249a

	iio_trigger_unregister(st->trig);
	iio_free_trigger(st->trig);
}
/**
 *  @}
 */

