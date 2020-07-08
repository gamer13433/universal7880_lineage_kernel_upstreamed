/*
 * sec-core.c
 *
 * Copyright (c) 2012 Samsung Electronics Co., Ltd
 *              http://www.samsung.com
 *
 *  This program is free software; you can redistribute  it and/or modify it
 *  under  the terms of  the GNU General  Public License as published by the
 *  Free Software Foundation;  either version 2 of the  License, or (at your
 *  option) any later version.
 *
 */

#include <linux/module.h>
#include <linux/moduleparam.h>
#include <linux/init.h>
#include <linux/err.h>
#include <linux/slab.h>
#include <linux/i2c.h>
<<<<<<< HEAD
#include <linux/of_gpio.h>
#include <linux/of_irq.h>
#include <linux/irqnr.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/rtc.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/irq.h>
#include <linux/mfd/samsung/rtc.h>
#include <linux/regmap.h>

#if defined(CONFIG_SOC_EXYNOS7420) || defined(CONFIG_SOC_EXYNOS7890)
#ifdef CONFIG_EXYNOS_MBOX
#include <mach/apm-exynos.h>
#endif
#else
#ifdef CONFIG_EXYNOS_MBOX
#include <linux/apm-exynos.h>
#include <linux/mailbox-exynos.h>
#endif
#endif

#ifdef CONFIG_EXYNOS_MBOX
enum {
	PMIC,
	RTC,
};
unsigned int apm_status = 0;
static DEFINE_MUTEX(sec_lock);
#endif

static struct mfd_cell s5m8751_devs[] = {
=======
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/pm_runtime.h>
#include <linux/mutex.h>
#include <linux/mfd/core.h>
#include <linux/mfd/samsung/core.h>
#include <linux/mfd/samsung/irq.h>
#include <linux/mfd/samsung/s2mpa01.h>
#include <linux/mfd/samsung/s2mps11.h>
#include <linux/mfd/samsung/s2mps14.h>
#include <linux/mfd/samsung/s2mpu02.h>
#include <linux/mfd/samsung/s5m8763.h>
#include <linux/mfd/samsung/s5m8767.h>
#include <linux/regulator/machine.h>
#include <linux/regmap.h>

static const struct mfd_cell s5m8751_devs[] = {
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	{
		.name = "s5m8751-pmic",
	}, {
		.name = "s5m-charger",
	}, {
		.name = "s5m8751-codec",
	},
};

<<<<<<< HEAD
static struct mfd_cell s5m8763_devs[] = {
=======
static const struct mfd_cell s5m8763_devs[] = {
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	{
		.name = "s5m8763-pmic",
	}, {
		.name = "s5m-rtc",
	}, {
		.name = "s5m-charger",
	},
};

<<<<<<< HEAD
static struct mfd_cell s5m8767_devs[] = {
=======
static const struct mfd_cell s5m8767_devs[] = {
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	{
		.name = "s5m8767-pmic",
	}, {
		.name = "s5m-rtc",
<<<<<<< HEAD
	},
};

static struct mfd_cell s2mps11_devs[] = {
	{
		.name = "s2mps11-pmic",
	}, {
		.name = "s2m-rtc",
	},
};

static struct mfd_cell s2mps13_devs[] = {
	{
		.name = "s2mps13-pmic",
	}, {
		.name = "s2m-rtc",
	},
};

static struct mfd_cell s2mps15_devs[] = {
	{
		.name = "s2mps15-pmic",
	}, {
		.name = "s2m-rtc",
	},
};

static struct mfd_cell s2mpu03_devs[] = {
	{
		.name = "s2mpu03-pmic",
	}, {
		.name = "s2m-rtc",
	},
};

static struct mfd_cell s2mps16_devs[] = {
	{
		.name = "s2mps16-pmic",
	}, {
		.name = "s2m-rtc",
	},
};

static struct mfd_cell s2mpu05_devs[] = {
	{
		.name = "s2mpu05-pmic",
	}, {
		.name = "s2m-rtc",
	},
};

#ifdef CONFIG_OF
static struct of_device_id sec_dt_match[] = {
	{	.compatible = "samsung,s5m8767-pmic",
		.data = (void *)S5M8767X,
	},
	{	.compatible = "samsung,s2mps13-pmic",
		.data = (void *)S2MPS13X,
	},
	{	.compatible = "samsung,s2mps11-pmic",
		.data = (void *)S2MPS11X,
	},
	{	.compatible = "samsung,s2mps15-pmic",
		.data = (void *)S2MPS15X,
	},
	{	.compatible = "samsung,s2mpu03-pmic",
		.data = (void *)S2MPU03X,
	},
	{	.compatible = "samsung,s2mps16-pmic",
		.data = (void *)S2MPS16X,
	},
	{	.compatible = "samsung,s2mpu05-pmic",
		.data = (void *)S2MPU05X,
	},
	{},
};
#endif

#ifdef CONFIG_EXYNOS_MBOX
static int exynos_regulator_apm_notifier(struct notifier_block *notifier,
						unsigned long pm_event, void *v)
{
	switch (pm_event) {
		case APM_READY:
			apm_status = true;
			break;
		case APM_SLEEP:
		case APM_TIMEOUT:
			apm_status = false;
			break;
	}

	return NOTIFY_OK;
}

static struct notifier_block exynos_apm_notifier = {
	.notifier_call = exynos_regulator_apm_notifier,
};
#endif

int sec_reg_read(struct sec_pmic_dev *sec_pmic, u32 reg, void *dest)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);

	if (!apm_status) {
		ret = regmap_read(sec_pmic->regmap, reg, dest);
	} else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_read(PMIC, reg, dest);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0) {
			ret = regmap_read(sec_pmic->regmap, reg, dest);
		}
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_read(sec_pmic->regmap, reg, dest);
#endif
}
EXPORT_SYMBOL_GPL(sec_reg_read);

int sec_bulk_read(struct sec_pmic_dev *sec_pmic, u32 reg, int count, u8 *buf)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);

	if (!apm_status)
		ret = regmap_bulk_read(sec_pmic->regmap, reg, buf, count);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_bulk_read(PMIC, reg, buf, count);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0) {
			ret = regmap_bulk_read(sec_pmic->regmap, reg, buf, count);
		}
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_bulk_read(sec_pmic->regmap, reg, buf, count);
#endif
}
EXPORT_SYMBOL_GPL(sec_bulk_read);

int sec_reg_write(struct sec_pmic_dev *sec_pmic, u32 reg, u32 value)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);
	if (!apm_status)
		ret = regmap_write(sec_pmic->regmap, reg, value);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_write(PMIC, reg, value);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0) {
			ret = regmap_write(sec_pmic->regmap, reg, value);
		}
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_write(sec_pmic->regmap, reg, value);
#endif
}
EXPORT_SYMBOL_GPL(sec_reg_write);

int sec_bulk_write(struct sec_pmic_dev *sec_pmic, u32 reg, int count, u8 *buf)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);
	if (!apm_status)
		ret = regmap_raw_write(sec_pmic->regmap, reg, buf, count);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_bulk_write(PMIC, reg, buf, count);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0) {
			ret = regmap_raw_write(sec_pmic->regmap, reg, buf, count);
		}
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_raw_write(sec_pmic->regmap, reg, buf, count);
#endif
}
EXPORT_SYMBOL_GPL(sec_bulk_write);

int sec_reg_update(struct sec_pmic_dev *sec_pmic, u32 reg, u32 val, u32 mask)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);
	if (!apm_status)
		ret = regmap_update_bits(sec_pmic->regmap, reg, mask, val);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_update_bits(PMIC, reg, mask, val);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0) {
			ret = regmap_update_bits(sec_pmic->regmap, reg, mask, val);
		}
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_update_bits(sec_pmic->regmap, reg, mask, val);
#endif
}
EXPORT_SYMBOL_GPL(sec_reg_update);

int sec_rtc_read(struct sec_pmic_dev *sec_pmic, u32 reg, void *dest)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);
	if (!apm_status)
		ret = regmap_read(sec_pmic->rtc_regmap, reg, dest);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_read(RTC, reg, dest);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0)
			ret = regmap_read(sec_pmic->rtc_regmap, reg, dest);
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_read(sec_pmic->rtc_regmap, reg, dest);
#endif
}
EXPORT_SYMBOL_GPL(sec_rtc_read);

int sec_rtc_bulk_read(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);
	if (!apm_status)
		ret = regmap_bulk_read(sec_pmic->rtc_regmap, reg, buf, count);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_bulk_read(RTC, reg, buf, count);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0)
			ret = regmap_bulk_read(sec_pmic->rtc_regmap, reg, buf, count);
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_bulk_read(sec_pmic->rtc_regmap, reg, buf, count);
#endif
}
EXPORT_SYMBOL_GPL(sec_rtc_bulk_read);

int sec_rtc_write(struct sec_pmic_dev *sec_pmic, u32 reg, u32 value)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);
	if (!apm_status)
		ret = regmap_write(sec_pmic->rtc_regmap, reg, value);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_write(RTC, reg, value);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0)
			ret = regmap_write(sec_pmic->rtc_regmap, reg, value);
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_write(sec_pmic->rtc_regmap, reg, value);
#endif
}
EXPORT_SYMBOL_GPL(sec_rtc_write);

int sec_rtc_bulk_write(struct sec_pmic_dev *sec_pmic, u32 reg, int count,
		u8 *buf)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);
	if (!apm_status)
		ret = regmap_raw_write(sec_pmic->rtc_regmap, reg, buf, count);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_bulk_write(RTC, reg, buf, count);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0)
			ret = regmap_raw_write(sec_pmic->rtc_regmap, reg, buf, count);
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_raw_write(sec_pmic->rtc_regmap, reg, buf, count);
#endif
}
EXPORT_SYMBOL_GPL(sec_rtc_bulk_write);

int sec_rtc_update(struct sec_pmic_dev *sec_pmic, u32 reg, u32 val,
		u32 mask)
{
#ifdef CONFIG_EXYNOS_MBOX
	int ret;

	mutex_lock(&sec_lock);
	if (!apm_status)
		ret = regmap_update_bits(sec_pmic->rtc_regmap, reg, mask, val);
	else {
		mutex_lock(&sec_pmic->iolock);
		ret = sec_pmic->ops->apm_update_bits(RTC, reg, mask, val);
		mutex_unlock(&sec_pmic->iolock);
		if (ret < 0)
			ret = regmap_update_bits(sec_pmic->rtc_regmap, reg, mask, val);
	}
	mutex_unlock(&sec_lock);

	return ret;
#else
	return regmap_update_bits(sec_pmic->rtc_regmap, reg, mask, val);
#endif
}
EXPORT_SYMBOL_GPL(sec_rtc_update);

#ifdef CONFIG_EXYNOS_MBOX
void sec_core_lock(void)
{
	mutex_lock(&sec_lock);
}
EXPORT_SYMBOL_GPL(sec_core_lock);

void sec_core_unlock(void)
{
	mutex_unlock(&sec_lock);
}
EXPORT_SYMBOL_GPL(sec_core_unlock);
#endif

static struct regmap_config sec_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};


#ifdef CONFIG_OF
static struct sec_platform_data *sec_pmic_i2c_parse_dt_pdata(
					struct device *dev)
{
	struct sec_platform_data *pdata;
	struct device_node *np = dev->of_node;
	int ret;
	u32 val;

	pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
	if (!pdata) {
		dev_err(dev, "failed to allocate platform data\n");
		return ERR_PTR(-ENOMEM);
	}
	dev->platform_data = pdata;
	pdata->irq_base = -1;

	/* WTSR, SMPL */
	pdata->wtsr_smpl = devm_kzalloc(dev, sizeof(*pdata->wtsr_smpl),
			GFP_KERNEL);
	if (!pdata->wtsr_smpl)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np, "wtsr_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->wtsr_smpl->wtsr_en = !!val;

	ret = of_property_read_u32(np, "smpl_en", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->wtsr_smpl->smpl_en = !!val;

	ret = of_property_read_u32(np, "wtsr_timer_val",
			&pdata->wtsr_smpl->wtsr_timer_val);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np, "smpl_timer_val",
			&pdata->wtsr_smpl->smpl_timer_val);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np, "check_jigon", &val);
	if (ret)
		return ERR_PTR(ret);
	pdata->wtsr_smpl->check_jigon = !!val;

	if (of_get_property(np, "ten-bit-address", NULL))
		pdata->ten_bit_address = true;

	/* init time */
	pdata->init_time = devm_kzalloc(dev, sizeof(*pdata->init_time),
			GFP_KERNEL);
	if (!pdata->init_time)
		return ERR_PTR(-ENOMEM);

	ret = of_property_read_u32(np, "init_time,sec",
			&pdata->init_time->tm_sec);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np, "init_time,min",
			&pdata->init_time->tm_min);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np, "init_time,hour",
			&pdata->init_time->tm_hour);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np, "init_time,mday",
			&pdata->init_time->tm_mday);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np, "init_time,mon",
			&pdata->init_time->tm_mon);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np, "init_time,year",
			&pdata->init_time->tm_year);
	if (ret)
		return ERR_PTR(ret);

	ret = of_property_read_u32(np, "init_time,wday",
			&pdata->init_time->tm_wday);
	if (ret)
		return ERR_PTR(ret);

	/* rtc optimize */
	ret = of_property_read_u32(np, "osc-bias-up", &val);
	if (!ret)
		pdata->osc_bias_up = val;
	else
		pdata->osc_bias_up = -1;

	ret = of_property_read_u32(np, "rtc_cap_sel", &val);
	if (!ret)
		pdata->cap_sel = val;
	else
		pdata->cap_sel = -1;

	ret = of_property_read_u32(np, "rtc_osc_xin", &val);
	if (!ret)
		pdata->osc_xin = val;
	else
		pdata->osc_xin = -1;

	ret = of_property_read_u32(np, "rtc_osc_xout", &val);
	if (!ret)
		pdata->osc_xout = val;
	else
		pdata->osc_xout = -1;

	return pdata;
=======
	}, {
		.name = "s5m8767-clk",
		.of_compatible = "samsung,s5m8767-clk",
	}
};

static const struct mfd_cell s2mps11_devs[] = {
	{
		.name = "s2mps11-pmic",
	}, {
		.name = "s2mps11-clk",
		.of_compatible = "samsung,s2mps11-clk",
	}
};

static const struct mfd_cell s2mps14_devs[] = {
	{
		.name = "s2mps14-pmic",
	}, {
		.name = "s2mps14-rtc",
	}, {
		.name = "s2mps14-clk",
		.of_compatible = "samsung,s2mps14-clk",
	}
};

static const struct mfd_cell s2mpa01_devs[] = {
	{
		.name = "s2mpa01-pmic",
	},
};

static const struct mfd_cell s2mpu02_devs[] = {
	{ .name = "s2mpu02-pmic", },
	{ .name = "s2mpu02-rtc", },
	{
		.name = "s2mpu02-clk",
		.of_compatible = "samsung,s2mpu02-clk",
	}
};

#ifdef CONFIG_OF
static const struct of_device_id sec_dt_match[] = {
	{	.compatible = "samsung,s5m8767-pmic",
		.data = (void *)S5M8767X,
	}, {
		.compatible = "samsung,s2mps11-pmic",
		.data = (void *)S2MPS11X,
	}, {
		.compatible = "samsung,s2mps14-pmic",
		.data = (void *)S2MPS14X,
	}, {
		.compatible = "samsung,s2mpa01-pmic",
		.data = (void *)S2MPA01,
	}, {
		.compatible = "samsung,s2mpu02-pmic",
		.data = (void *)S2MPU02,
	}, {
		/* Sentinel */
	},
};
#endif

static bool s2mpa01_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case S2MPA01_REG_INT1M:
	case S2MPA01_REG_INT2M:
	case S2MPA01_REG_INT3M:
		return false;
	default:
		return true;
	}
}

static bool s2mps11_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case S2MPS11_REG_INT1M:
	case S2MPS11_REG_INT2M:
	case S2MPS11_REG_INT3M:
		return false;
	default:
		return true;
	}
}

static bool s2mpu02_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case S2MPU02_REG_INT1M:
	case S2MPU02_REG_INT2M:
	case S2MPU02_REG_INT3M:
		return false;
	default:
		return true;
	}
}

static bool s5m8763_volatile(struct device *dev, unsigned int reg)
{
	switch (reg) {
	case S5M8763_REG_IRQM1:
	case S5M8763_REG_IRQM2:
	case S5M8763_REG_IRQM3:
	case S5M8763_REG_IRQM4:
		return false;
	default:
		return true;
	}
}

static const struct regmap_config sec_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,
};

static const struct regmap_config s2mpa01_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = S2MPA01_REG_LDO_OVCB4,
	.volatile_reg = s2mpa01_volatile,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_config s2mps11_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = S2MPS11_REG_L38CTRL,
	.volatile_reg = s2mps11_volatile,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_config s2mps14_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = S2MPS14_REG_LDODSCH3,
	.volatile_reg = s2mps11_volatile,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_config s2mpu02_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = S2MPU02_REG_DVSDATA,
	.volatile_reg = s2mpu02_volatile,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_config s5m8763_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = S5M8763_REG_LBCNFG2,
	.volatile_reg = s5m8763_volatile,
	.cache_type = REGCACHE_FLAT,
};

static const struct regmap_config s5m8767_regmap_config = {
	.reg_bits = 8,
	.val_bits = 8,

	.max_register = S5M8767_REG_LDO28CTRL,
	.volatile_reg = s2mps11_volatile,
	.cache_type = REGCACHE_FLAT,
};

#ifdef CONFIG_OF
/*
 * Only the common platform data elements for s5m8767 are parsed here from the
 * device tree. Other sub-modules of s5m8767 such as pmic, rtc , charger and
 * others have to parse their own platform data elements from device tree.
 *
 * The s5m8767 platform data structure is instantiated here and the drivers for
 * the sub-modules need not instantiate another instance while parsing their
 * platform data.
 */
static struct sec_platform_data *sec_pmic_i2c_parse_dt_pdata(
					struct device *dev)
{
	struct sec_platform_data *pd;

	pd = devm_kzalloc(dev, sizeof(*pd), GFP_KERNEL);
	if (!pd) {
		dev_err(dev, "could not allocate memory for pdata\n");
		return ERR_PTR(-ENOMEM);
	}

	/*
	 * ToDo: the 'wakeup' member in the platform data is more of a linux
	 * specfic information. Hence, there is no binding for that yet and
	 * not parsed here.
	 */

	return pd;
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
}
#else
static struct sec_platform_data *sec_pmic_i2c_parse_dt_pdata(
					struct device *dev)
{
	return NULL;
}
#endif

<<<<<<< HEAD
static inline int sec_i2c_get_driver_data(struct i2c_client *i2c,
=======
static inline unsigned long sec_i2c_get_driver_data(struct i2c_client *i2c,
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
						const struct i2c_device_id *id)
{
#ifdef CONFIG_OF
	if (i2c->dev.of_node) {
		const struct of_device_id *match;
<<<<<<< HEAD
=======

>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
		match = of_match_node(sec_dt_match, i2c->dev.of_node);
		return (unsigned long)match->data;
	}
#endif
<<<<<<< HEAD
	return (int)id->driver_data;
=======
	return id->driver_data;
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
}

static int sec_pmic_probe(struct i2c_client *i2c,
			    const struct i2c_device_id *id)
{
<<<<<<< HEAD
	struct sec_platform_data *pdata = i2c->dev.platform_data;
	struct sec_pmic_dev *sec_pmic;
	int ret;
=======
	struct sec_platform_data *pdata = dev_get_platdata(&i2c->dev);
	const struct regmap_config *regmap;
	const struct mfd_cell *sec_devs;
	struct sec_pmic_dev *sec_pmic;
	unsigned long device_type;
	int ret, num_sec_devs;
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

	sec_pmic = devm_kzalloc(&i2c->dev, sizeof(struct sec_pmic_dev),
				GFP_KERNEL);
	if (sec_pmic == NULL)
		return -ENOMEM;

	i2c_set_clientdata(i2c, sec_pmic);
	sec_pmic->dev = &i2c->dev;
	sec_pmic->i2c = i2c;
	sec_pmic->irq = i2c->irq;
<<<<<<< HEAD
	sec_pmic->type = sec_i2c_get_driver_data(i2c, id);

	mutex_init(&sec_pmic->iolock);
=======
	device_type = sec_i2c_get_driver_data(i2c, id);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

	if (sec_pmic->dev->of_node) {
		pdata = sec_pmic_i2c_parse_dt_pdata(sec_pmic->dev);
		if (IS_ERR(pdata)) {
			ret = PTR_ERR(pdata);
			return ret;
		}
<<<<<<< HEAD
		pdata->device_type = sec_pmic->type;
	}

	if (pdata == NULL)
		return -ENOMEM;

=======
		pdata->device_type = device_type;
	}
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	if (pdata) {
		sec_pmic->device_type = pdata->device_type;
		sec_pmic->ono = pdata->ono;
		sec_pmic->irq_base = pdata->irq_base;
<<<<<<< HEAD
		sec_pmic->wakeup = true;
		sec_pmic->pdata = pdata;
		sec_pmic->irq = i2c->irq;
	}

	sec_pmic->regmap = devm_regmap_init_i2c(i2c, &sec_regmap_config);
	if (IS_ERR(sec_pmic->regmap)) {
		ret = PTR_ERR(sec_pmic->regmap);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
			ret);
		return ret;
	}

	sec_pmic->rtc = i2c_new_dummy(i2c->adapter, RTC_I2C_ADDR);
	if (!sec_pmic->rtc) {
		dev_err(&i2c->dev, "Failed to allocate I2C for RTC\n");
		return -ENODEV;
	}

	if (pdata->ten_bit_address)
		sec_pmic->rtc->flags |= I2C_CLIENT_TEN;

	i2c_set_clientdata(sec_pmic->rtc, sec_pmic);
	sec_pmic->rtc_regmap = devm_regmap_init_i2c(sec_pmic->rtc,
						&sec_regmap_config);
	if (IS_ERR(sec_pmic->rtc_regmap)) {
		ret = PTR_ERR(sec_pmic->rtc_regmap);
		dev_err(&sec_pmic->rtc->dev, "Failed to allocate register map: %d\n",
=======
		sec_pmic->wakeup = pdata->wakeup;
		sec_pmic->pdata = pdata;
	}

	switch (sec_pmic->device_type) {
	case S2MPA01:
		regmap = &s2mpa01_regmap_config;
		break;
	case S2MPS11X:
		regmap = &s2mps11_regmap_config;
		break;
	case S2MPS14X:
		regmap = &s2mps14_regmap_config;
		break;
	case S5M8763X:
		regmap = &s5m8763_regmap_config;
		break;
	case S5M8767X:
		regmap = &s5m8767_regmap_config;
		break;
	case S2MPU02:
		regmap = &s2mpu02_regmap_config;
		break;
	default:
		regmap = &sec_regmap_config;
		break;
	}

	sec_pmic->regmap_pmic = devm_regmap_init_i2c(i2c, regmap);
	if (IS_ERR(sec_pmic->regmap_pmic)) {
		ret = PTR_ERR(sec_pmic->regmap_pmic);
		dev_err(&i2c->dev, "Failed to allocate register map: %d\n",
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
			ret);
		return ret;
	}

	if (pdata && pdata->cfg_pmic_irq)
		pdata->cfg_pmic_irq();

	sec_irq_init(sec_pmic);
<<<<<<< HEAD
#ifdef CONFIG_EXYNOS_MBOX
	sec_pmic->ops = &exynos_apm_function_ops;
	register_apm_notifier(&exynos_apm_notifier);
#endif
=======

>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	pm_runtime_set_active(sec_pmic->dev);

	switch (sec_pmic->device_type) {
	case S5M8751X:
<<<<<<< HEAD
		ret = mfd_add_devices(sec_pmic->dev, -1, s5m8751_devs,
				      ARRAY_SIZE(s5m8751_devs), NULL, 0, NULL);
		break;
	case S5M8763X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s5m8763_devs,
				      ARRAY_SIZE(s5m8763_devs), NULL, 0, NULL);
		break;
	case S5M8767X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s5m8767_devs,
				      ARRAY_SIZE(s5m8767_devs), NULL, 0, NULL);
		break;
	case S2MPS11X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s2mps11_devs,
				      ARRAY_SIZE(s2mps11_devs), NULL, 0, NULL);
		break;
	case S2MPS13X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s2mps13_devs,
				      ARRAY_SIZE(s2mps13_devs), NULL, 0, NULL);
		break;
	case S2MPS15X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s2mps15_devs,
				      ARRAY_SIZE(s2mps15_devs), NULL, 0, NULL);
		break;
	case S2MPU03X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s2mpu03_devs,
				      ARRAY_SIZE(s2mpu03_devs), NULL, 0, NULL);
		break;
	case S2MPS16X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s2mps16_devs,
				      ARRAY_SIZE(s2mps16_devs), NULL, 0, NULL);
		break;
	case S2MPU05X:
		ret = mfd_add_devices(sec_pmic->dev, -1, s2mpu05_devs,
				      ARRAY_SIZE(s2mpu05_devs), NULL, 0, NULL);
=======
		sec_devs = s5m8751_devs;
		num_sec_devs = ARRAY_SIZE(s5m8751_devs);
		break;
	case S5M8763X:
		sec_devs = s5m8763_devs;
		num_sec_devs = ARRAY_SIZE(s5m8763_devs);
		break;
	case S5M8767X:
		sec_devs = s5m8767_devs;
		num_sec_devs = ARRAY_SIZE(s5m8767_devs);
		break;
	case S2MPA01:
		sec_devs = s2mpa01_devs;
		num_sec_devs = ARRAY_SIZE(s2mpa01_devs);
		break;
	case S2MPS11X:
		sec_devs = s2mps11_devs;
		num_sec_devs = ARRAY_SIZE(s2mps11_devs);
		break;
	case S2MPS14X:
		sec_devs = s2mps14_devs;
		num_sec_devs = ARRAY_SIZE(s2mps14_devs);
		break;
	case S2MPU02:
		sec_devs = s2mpu02_devs;
		num_sec_devs = ARRAY_SIZE(s2mpu02_devs);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
		break;
	default:
		/* If this happens the probe function is problem */
		BUG();
	}
<<<<<<< HEAD

	if (ret < 0)
		goto err;

	return ret;

err:
	mfd_remove_devices(sec_pmic->dev);
	sec_irq_exit(sec_pmic);
	i2c_unregister_device(sec_pmic->rtc);
#ifdef CONFIG_EXYNOS_MBOX
	unregister_apm_notifier(&exynos_apm_notifier);
#endif
=======
	ret = mfd_add_devices(sec_pmic->dev, -1, sec_devs, num_sec_devs, NULL,
			      0, NULL);
	if (ret)
		goto err_mfd;

	device_init_wakeup(sec_pmic->dev, sec_pmic->wakeup);

	return ret;

err_mfd:
	sec_irq_exit(sec_pmic);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	return ret;
}

static int sec_pmic_remove(struct i2c_client *i2c)
{
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

	mfd_remove_devices(sec_pmic->dev);
	sec_irq_exit(sec_pmic);
<<<<<<< HEAD
	regmap_exit(sec_pmic->rtc_regmap);
	regmap_exit(sec_pmic->regmap);
	i2c_unregister_device(sec_pmic->rtc);
	return 0;
}

static const struct i2c_device_id sec_pmic_id[] = {
	{ "sec_pmic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sec_pmic_id);

#ifdef CONFIG_PM
static int sec_suspend(struct device *dev)
=======
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int sec_pmic_suspend(struct device *dev)
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

<<<<<<< HEAD
	if (sec_pmic->wakeup)
		enable_irq_wake(sec_pmic->irq);

	disable_irq(sec_pmic->irq);
	if (sec_pmic->device_type == S2MPS15X && sec_pmic->adc_en)
		sec_reg_write(sec_pmic, 0x5C, 0x00);
	else if (sec_pmic->device_type == S2MPS16X && sec_pmic->adc_en)
		sec_reg_write(sec_pmic, 0x6B, 0x00);
	return 0;
}

static int sec_resume(struct device *dev)
=======
	if (device_may_wakeup(dev))
		enable_irq_wake(sec_pmic->irq);
	/*
	 * PMIC IRQ must be disabled during suspend for RTC alarm
	 * to work properly.
	 * When device is woken up from suspend, an
	 * interrupt occurs before resuming I2C bus controller.
	 * The interrupt is handled by regmap_irq_thread which tries
	 * to read RTC registers. This read fails (I2C is still
	 * suspended) and RTC Alarm interrupt is disabled.
	 */
	disable_irq(sec_pmic->irq);

	switch (sec_pmic->device_type) {
	case S2MPS14X:
	case S2MPU02:
		regulator_suspend_prepare(PM_SUSPEND_MEM);
		break;
	default:
		break;
	}

	return 0;
}

static int sec_pmic_resume(struct device *dev)
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
{
	struct i2c_client *i2c = container_of(dev, struct i2c_client, dev);
	struct sec_pmic_dev *sec_pmic = i2c_get_clientdata(i2c);

<<<<<<< HEAD
	if (sec_pmic->wakeup)
		disable_irq_wake(sec_pmic->irq);

	enable_irq(sec_pmic->irq);
	if (sec_pmic->device_type == S2MPS15X && sec_pmic->adc_en)
		sec_reg_write(sec_pmic, 0x5C, 0x80);
	else if (sec_pmic->device_type == S2MPS16X && sec_pmic->adc_en)
		sec_reg_write(sec_pmic, 0x6B, 0x80);
	return 0;
}
#else
#define sec_suspend	NULL
#define sec_resume	NULL
#endif /* CONFIG_PM */

const struct dev_pm_ops sec_pmic_apm = {
	.suspend = sec_suspend,
	.resume = sec_resume,
};
=======
	if (device_may_wakeup(dev))
		disable_irq_wake(sec_pmic->irq);
	enable_irq(sec_pmic->irq);

	return 0;
}
#endif /* CONFIG_PM_SLEEP */

static SIMPLE_DEV_PM_OPS(sec_pmic_pm_ops, sec_pmic_suspend, sec_pmic_resume);

static const struct i2c_device_id sec_pmic_id[] = {
	{ "sec_pmic", 0 },
	{ }
};
MODULE_DEVICE_TABLE(i2c, sec_pmic_id);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

static struct i2c_driver sec_pmic_driver = {
	.driver = {
		   .name = "sec_pmic",
		   .owner = THIS_MODULE,
<<<<<<< HEAD
		   .of_match_table = of_match_ptr(sec_dt_match),
		   .pm = &sec_pmic_apm,
=======
		   .pm = &sec_pmic_pm_ops,
		   .of_match_table = of_match_ptr(sec_dt_match),
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	},
	.probe = sec_pmic_probe,
	.remove = sec_pmic_remove,
	.id_table = sec_pmic_id,
};

static int __init sec_pmic_init(void)
{
	return i2c_add_driver(&sec_pmic_driver);
}

subsys_initcall(sec_pmic_init);

static void __exit sec_pmic_exit(void)
{
	i2c_del_driver(&sec_pmic_driver);
}
module_exit(sec_pmic_exit);

MODULE_AUTHOR("Sangbeom Kim <sbkim73@samsung.com>");
<<<<<<< HEAD
MODULE_DESCRIPTION("Core support for the SAMSUNG MFD");
=======
MODULE_DESCRIPTION("Core support for the S5M MFD");
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
MODULE_LICENSE("GPL");
