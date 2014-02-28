/*
 * Copyright (c) 2014, The Linux Foundation. All rights reserved.
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 and
 * only version 2 as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#define pr_fmt(fmt) "cache-hwmon: " fmt

#include <linux/kernel.h>
#include <linux/sizes.h>
#include <linux/module.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/ktime.h>
#include <linux/time.h>
#include <linux/err.h>
#include <linux/errno.h>
#include <linux/mutex.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/devfreq.h>
#include "governor.h"
#include "governor_cache_hwmon.h"

<<<<<<< HEAD
struct cache_hwmon_node {
	unsigned int cycles_per_low_req;
	unsigned int cycles_per_med_req;
	unsigned int cycles_per_high_req;
	unsigned int min_busy;
	unsigned int max_busy;
	unsigned int tolerance_mrps;
	unsigned int guard_band_mhz;
	unsigned int decay_rate;
	unsigned long prev_mhz;
	ktime_t prev_ts;
	bool mon_started;
	struct list_head list;
	void *orig_data;
	struct cache_hwmon *hw;
	struct attribute_group *attr_grp;
};

static LIST_HEAD(cache_hwmon_list);
static DEFINE_MUTEX(list_lock);

static int use_cnt;
static DEFINE_MUTEX(state_lock);

=======
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
#define show_attr(name) \
static ssize_t show_##name(struct device *dev,				\
			struct device_attribute *attr, char *buf)	\
{									\
<<<<<<< HEAD
	struct devfreq *df = to_devfreq(dev);				\
	struct cache_hwmon_node *hw = df->data;				\
	return snprintf(buf, PAGE_SIZE, "%u\n", hw->name);		\
=======
	return snprintf(buf, PAGE_SIZE, "%u\n", name);			\
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
}

#define store_attr(name, _min, _max) \
static ssize_t store_##name(struct device *dev,				\
			struct device_attribute *attr, const char *buf,	\
			size_t count)					\
{									\
	int ret;							\
	unsigned int val;						\
<<<<<<< HEAD
	struct devfreq *df = to_devfreq(dev);				\
	struct cache_hwmon_node *hw = df->data;				\
=======
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
	ret = sscanf(buf, "%u", &val);					\
	if (ret != 1)							\
		return -EINVAL;						\
	val = max(val, _min);						\
	val = min(val, _max);						\
<<<<<<< HEAD
	hw->name = val;							\
=======
	name = val;							\
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
	return count;							\
}

#define gov_attr(__attr, min, max)	\
show_attr(__attr)			\
store_attr(__attr, min, max)		\
static DEVICE_ATTR(__attr, 0644, show_##__attr, store_##__attr)

<<<<<<< HEAD
#define MIN_MS	10U
#define MAX_MS	500U

static struct cache_hwmon_node *find_hwmon_node(struct devfreq *df)
{
	struct cache_hwmon_node *node, *found = NULL;

	mutex_lock(&list_lock);
	list_for_each_entry(node, &cache_hwmon_list, list)
		if (node->hw->dev == df->dev.parent ||
		    node->hw->of_node == df->dev.parent->of_node) {
			found = node;
			break;
		}
	mutex_unlock(&list_lock);

	return found;
}

static unsigned long measure_mrps_and_set_irq(struct cache_hwmon_node *node,
=======

static struct cache_hwmon *hw;
static unsigned int cycles_per_low_req;
static unsigned int cycles_per_med_req = 20;
static unsigned int cycles_per_high_req = 35;
static unsigned int min_busy = 100;
static unsigned int max_busy = 100;
static unsigned int tolerance_mrps = 5;
static unsigned int guard_band_mhz = 100;
static unsigned int decay_rate = 90;

#define MIN_MS	10U
#define MAX_MS	500U
static unsigned int sample_ms = 50;
static unsigned long prev_mhz;
static ktime_t prev_ts;

static unsigned long measure_mrps_and_set_irq(struct devfreq *df,
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
			struct mrps_stats *stat)
{
	ktime_t ts;
	unsigned int us;
<<<<<<< HEAD
	struct cache_hwmon *hw = node->hw;
=======
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10

	/*
	 * Since we are stopping the counters, we don't want this short work
	 * to be interrupted by other tasks and cause the measurements to be
	 * wrong. Not blocking interrupts to avoid affecting interrupt
	 * latency and since they should be short anyway because they run in
	 * atomic context.
	 */
	preempt_disable();

	ts = ktime_get();
<<<<<<< HEAD
	us = ktime_to_us(ktime_sub(ts, node->prev_ts));
	if (!us)
		us = 1;

	hw->meas_mrps_and_set_irq(hw, node->tolerance_mrps, us, stat);
	node->prev_ts = ts;

	preempt_enable();

	dev_dbg(hw->df->dev.parent,
		"stat H=%3lu, M=%3lu, L=%3lu, T=%3lu, b=%3u, f=%4lu, us=%d\n",
		 stat->mrps[HIGH], stat->mrps[MED], stat->mrps[LOW],
		 stat->mrps[HIGH] + stat->mrps[MED] + stat->mrps[LOW],
		 stat->busy_percent, hw->df->previous_freq / 1000, us);
=======
	us = ktime_to_us(ktime_sub(ts, prev_ts));
	if (!us)
		us = 1;

	hw->meas_mrps_and_set_irq(df, tolerance_mrps, us, stat);
	prev_ts = ts;

	preempt_enable();

	pr_debug("stat H=%3lu, M=%3lu, T=%3lu, b=%3u, f=%4lu, us=%d\n",
		 stat->high, stat->med, stat->high + stat->med,
		 stat->busy_percent, df->previous_freq / 1000, us);
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10

	return 0;
}

<<<<<<< HEAD
static void compute_cache_freq(struct cache_hwmon_node *node,
		struct mrps_stats *mrps, unsigned long *freq)
=======
static void compute_cache_freq(struct mrps_stats *mrps, unsigned long *freq)
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
{
	unsigned long new_mhz;
	unsigned int busy;

<<<<<<< HEAD
	new_mhz = mrps->mrps[HIGH] * node->cycles_per_high_req
		+ mrps->mrps[MED] * node->cycles_per_med_req
		+ mrps->mrps[LOW] * node->cycles_per_low_req;

	busy = max(node->min_busy, mrps->busy_percent);
	busy = min(node->max_busy, busy);
=======
	new_mhz = mrps->high * cycles_per_high_req
		+ mrps->med * cycles_per_med_req
		+ mrps->low * cycles_per_low_req;

	busy = max(min_busy, mrps->busy_percent);
	busy = min(max_busy, busy);
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10

	new_mhz *= 100;
	new_mhz /= busy;

<<<<<<< HEAD
	if (new_mhz < node->prev_mhz) {
		new_mhz = new_mhz * node->decay_rate + node->prev_mhz
				* (100 - node->decay_rate);
		new_mhz /= 100;
	}
	node->prev_mhz = new_mhz;

	new_mhz += node->guard_band_mhz;
=======
	if (new_mhz < prev_mhz) {
		new_mhz = new_mhz * decay_rate + prev_mhz * (100 - decay_rate);
		new_mhz /= 100;
	}
	prev_mhz = new_mhz;

	new_mhz += guard_band_mhz;
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
	*freq = new_mhz * 1000;
}

#define TOO_SOON_US	(1 * USEC_PER_MSEC)
<<<<<<< HEAD
int update_cache_hwmon(struct cache_hwmon *hwmon)
{
	struct cache_hwmon_node *node;
	struct devfreq *df;
=======
static irqreturn_t mon_intr_handler(int irq, void *dev)
{
	struct devfreq *df = dev;
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
	ktime_t ts;
	unsigned int us;
	int ret;

<<<<<<< HEAD
	if (!hwmon)
		return -EINVAL;
	df = hwmon->df;
	if (!df)
		return -ENODEV;
	node = df->data;
	if (!node)
		return -ENODEV;
	if (!node->mon_started)
		return -EBUSY;

	dev_dbg(df->dev.parent, "Got update request\n");
=======
	if (!hw->is_valid_irq(df))
		return IRQ_NONE;

	pr_debug("Got interrupt\n");
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
	devfreq_monitor_stop(df);

	/*
	 * Don't recalc cache freq if the interrupt comes right after a
	 * previous cache freq calculation.  This is done for two reasons:
	 *
	 * 1. Sampling the cache request during a very short duration can
	 *    result in a very inaccurate measurement due to very short
	 *    bursts.
	 * 2. This can only happen if the limit was hit very close to the end
	 *    of the previous sample period. Which means the current cache
	 *    request estimate is not very off and doesn't need to be
	 *    readjusted.
	 */
	ts = ktime_get();
<<<<<<< HEAD
	us = ktime_to_us(ktime_sub(ts, node->prev_ts));
=======
	us = ktime_to_us(ktime_sub(ts, prev_ts));
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
	if (us > TOO_SOON_US) {
		mutex_lock(&df->lock);
		ret = update_devfreq(df);
		if (ret)
<<<<<<< HEAD
			dev_err(df->dev.parent,
				"Unable to update freq on request!\n");
=======
			pr_err("Unable to update freq on IRQ!\n");
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
		mutex_unlock(&df->lock);
	}

	devfreq_monitor_start(df);

<<<<<<< HEAD
	return 0;
=======
	return IRQ_HANDLED;
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
}

static int devfreq_cache_hwmon_get_freq(struct devfreq *df,
					unsigned long *freq,
					u32 *flag)
{
	struct mrps_stats stat;
<<<<<<< HEAD
	struct cache_hwmon_node *node = df->data;

	memset(&stat, 0, sizeof(stat));
	measure_mrps_and_set_irq(node, &stat);
	compute_cache_freq(node, &stat, freq);
=======

	measure_mrps_and_set_irq(df, &stat);
	compute_cache_freq(&stat, freq);
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10

	return 0;
}

gov_attr(cycles_per_low_req, 1U, 100U);
gov_attr(cycles_per_med_req, 1U, 100U);
gov_attr(cycles_per_high_req, 1U, 100U);
gov_attr(min_busy, 1U, 100U);
gov_attr(max_busy, 1U, 100U);
gov_attr(tolerance_mrps, 0U, 100U);
gov_attr(guard_band_mhz, 0U, 500U);
gov_attr(decay_rate, 0U, 100U);

static struct attribute *dev_attr[] = {
	&dev_attr_cycles_per_low_req.attr,
	&dev_attr_cycles_per_med_req.attr,
	&dev_attr_cycles_per_high_req.attr,
	&dev_attr_min_busy.attr,
	&dev_attr_max_busy.attr,
	&dev_attr_tolerance_mrps.attr,
	&dev_attr_guard_band_mhz.attr,
	&dev_attr_decay_rate.attr,
	NULL,
};

static struct attribute_group dev_attr_group = {
	.name = "cache_hwmon",
	.attrs = dev_attr,
};

static int start_monitoring(struct devfreq *df)
{
	int ret;
	struct mrps_stats mrps;
<<<<<<< HEAD
	struct device *dev = df->dev.parent;
	struct cache_hwmon_node *node;
	struct cache_hwmon *hw;

	node = find_hwmon_node(df);
	if (!node) {
		dev_err(dev, "Unable to find HW monitor!\n");
		return -ENODEV;
	}
	hw = node->hw;
	hw->df = df;
	node->orig_data = df->data;
	df->data = node;

	node->prev_ts = ktime_get();
	node->prev_mhz = 0;
	mrps.mrps[HIGH] = (df->previous_freq / 1000) - node->guard_band_mhz;
	mrps.mrps[HIGH] /= node->cycles_per_high_req;
	mrps.mrps[MED] = mrps.mrps[LOW] = 0;

	ret = hw->start_hwmon(hw, &mrps);
	if (ret) {
		dev_err(dev, "Unable to start HW monitor!\n");
		goto err_start;
	}

	devfreq_monitor_start(df);
	node->mon_started = true;

	ret = sysfs_create_group(&df->dev.kobj, &dev_attr_group);
	if (ret) {
		dev_err(dev, "Error creating sys entries!\n");
=======

	prev_ts = ktime_get();
	prev_mhz = 0;
	mrps.high = (df->previous_freq / 1000) - guard_band_mhz;
	mrps.high /= cycles_per_high_req;

	ret = hw->start_hwmon(df, &mrps);
	if (ret) {
		pr_err("Unable to start HW monitor!\n");
		return ret;
	}

	devfreq_monitor_start(df);

	ret = request_threaded_irq(hw->irq, NULL, mon_intr_handler,
			  IRQF_ONESHOT | IRQF_SHARED,
			  "cache_hwmon", df);
	if (ret) {
		pr_err("Unable to register interrupt handler!\n");
		goto req_irq_fail;
	}

	ret = sysfs_create_group(&df->dev.kobj, &dev_attr_group);
	if (ret) {
		pr_err("Error creating sys entries!\n");
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
		goto sysfs_fail;
	}

	return 0;

sysfs_fail:
<<<<<<< HEAD
	node->mon_started = false;
	devfreq_monitor_stop(df);
	hw->stop_hwmon(hw);
err_start:
	df->data = node->orig_data;
	node->orig_data = NULL;
	hw->df = NULL;
=======
	disable_irq(hw->irq);
	free_irq(hw->irq, df);
req_irq_fail:
	devfreq_monitor_stop(df);
	hw->stop_hwmon(df);
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
	return ret;
}

static void stop_monitoring(struct devfreq *df)
{
<<<<<<< HEAD
	struct cache_hwmon_node *node = df->data;
	struct cache_hwmon *hw = node->hw;

	sysfs_remove_group(&df->dev.kobj, &dev_attr_group);
	node->mon_started = false;
	devfreq_monitor_stop(df);
	hw->stop_hwmon(hw);
	df->data = node->orig_data;
	node->orig_data = NULL;
	hw->df = NULL;
=======
	sysfs_remove_group(&df->dev.kobj, &dev_attr_group);
	disable_irq(hw->irq);
	free_irq(hw->irq, df);
	devfreq_monitor_stop(df);
	hw->stop_hwmon(df);
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
}

static int devfreq_cache_hwmon_ev_handler(struct devfreq *df,
					unsigned int event, void *data)
{
	int ret;
<<<<<<< HEAD
	unsigned int sample_ms;
=======
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10

	switch (event) {
	case DEVFREQ_GOV_START:
		sample_ms = df->profile->polling_ms;
		sample_ms = max(MIN_MS, sample_ms);
		sample_ms = min(MAX_MS, sample_ms);
		df->profile->polling_ms = sample_ms;

		ret = start_monitoring(df);
		if (ret)
			return ret;

<<<<<<< HEAD
		dev_dbg(df->dev.parent, "Enabled Cache HW monitor governor\n");
=======
		pr_debug("Enabled Cache HW monitor governor\n");
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
		break;

	case DEVFREQ_GOV_STOP:
		stop_monitoring(df);
<<<<<<< HEAD
		dev_dbg(df->dev.parent, "Disabled Cache HW monitor governor\n");
=======
		pr_debug("Disabled Cache HW monitor governor\n");
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
		break;

	case DEVFREQ_GOV_INTERVAL:
		sample_ms = *(unsigned int *)data;
		sample_ms = max(MIN_MS, sample_ms);
		sample_ms = min(MAX_MS, sample_ms);
		devfreq_interval_update(df, &sample_ms);
		break;
	}

	return 0;
}

static struct devfreq_governor devfreq_cache_hwmon = {
	.name = "cache_hwmon",
	.get_target_freq = devfreq_cache_hwmon_get_freq,
	.event_handler = devfreq_cache_hwmon_ev_handler,
};

<<<<<<< HEAD
int register_cache_hwmon(struct device *dev, struct cache_hwmon *hwmon)
{
	int ret = 0;
	struct cache_hwmon_node *node;

	if (!hwmon->dev && !hwmon->of_node)
		return -EINVAL;

	node = devm_kzalloc(dev, sizeof(*node), GFP_KERNEL);
	if (!node) {
		dev_err(dev, "Unable to register gov. Out of memory!\n");
		return -ENOMEM;
	}

	node->cycles_per_med_req = 20;
	node->cycles_per_high_req = 35;
	node->min_busy = 100;
	node->max_busy = 100;
	node->tolerance_mrps = 5;
	node->guard_band_mhz = 100;
	node->decay_rate = 90;
	node->hw = hwmon;
	node->attr_grp = &dev_attr_group;

	mutex_lock(&state_lock);
	if (!use_cnt) {
		ret = devfreq_add_governor(&devfreq_cache_hwmon);
		if (!ret)
			use_cnt++;
	}
	mutex_unlock(&state_lock);

	if (!ret) {
		dev_info(dev, "Cache HWmon governor registered.\n");
	} else {
		dev_err(dev, "Failed to add Cache HWmon governor\n");
		return ret;
	}

	mutex_lock(&list_lock);
	list_add_tail(&node->list, &cache_hwmon_list);
	mutex_unlock(&list_lock);

	return ret;
=======
int register_cache_hwmon(struct cache_hwmon *hwmon)
{
	int ret;

	hw = hwmon;
	ret = devfreq_add_governor(&devfreq_cache_hwmon);
	if (ret) {
		pr_err("devfreq governor registration failed\n");
		return ret;
	}

	return 0;
>>>>>>> 529524b... devfreq: Backport MSM devfreq features from 3.10
}

MODULE_DESCRIPTION("HW monitor based cache freq driver");
MODULE_LICENSE("GPL v2");
