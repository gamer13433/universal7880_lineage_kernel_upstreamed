/*
 * PCIe host controller driver for Samsung EXYNOS SoCs
 *
 * Copyright (C) 2013 Samsung Electronics Co., Ltd.
 *		http://www.samsung.com
 *
 * Author: Jingoo Han <jg1.han@samsung.com>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2 as
 * published by the Free Software Foundation.
 */

#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/gpio.h>
#include <linux/interrupt.h>
#include <linux/kernel.h>
<<<<<<< HEAD
#include <linux/mfd/syscon.h>
=======
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
#include <linux/module.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/platform_device.h>
<<<<<<< HEAD
#include <linux/regmap.h>
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>
#include <linux/exynos-pci-noti.h>
#include <linux/pm_qos.h>

#include <soc/samsung/exynos-pm.h>
#include <soc/samsung/exynos-powermode.h>

#include "pcie-designware.h"
#include "pci-exynos.h"

#if defined(CONFIG_SOC_EXYNOS8890)
#include "pci-exynos8890.c"
#include "pci-exynos8890_cal.c"
#endif

static struct exynos_pcie g_pcie[MAX_RC_NUM];
#ifdef CONFIG_PM_DEVFREQ
static struct pm_qos_request exynos_pcie_int_qos[MAX_RC_NUM];
#endif

#ifdef CONFIG_CPU_IDLE
static int exynos_pci_lpa_event(struct notifier_block *nb, unsigned long event, void *data);
#endif
static void exynos_pcie_resumed_phydown(struct pcie_port *pp);
static void exynos_pcie_assert_phy_reset(struct pcie_port *pp);
static int exynos_pcie_rd_own_conf(struct pcie_port *pp, int where, int size, u32 *val);
void exynos_pcie_send_pme_turn_off(struct exynos_pcie *exynos_pcie);
=======
#include <linux/resource.h>
#include <linux/signal.h>
#include <linux/types.h>

#include "pcie-designware.h"

#define to_exynos_pcie(x)	container_of(x, struct exynos_pcie, pp)

struct exynos_pcie {
	void __iomem		*elbi_base;
	void __iomem		*phy_base;
	void __iomem		*block_base;
	int			reset_gpio;
	struct clk		*clk;
	struct clk		*bus_clk;
	struct pcie_port	pp;
};

/* PCIe ELBI registers */
#define PCIE_IRQ_PULSE			0x000
#define IRQ_INTA_ASSERT			(0x1 << 0)
#define IRQ_INTB_ASSERT			(0x1 << 2)
#define IRQ_INTC_ASSERT			(0x1 << 4)
#define IRQ_INTD_ASSERT			(0x1 << 6)
#define PCIE_IRQ_LEVEL			0x004
#define PCIE_IRQ_SPECIAL		0x008
#define PCIE_IRQ_EN_PULSE		0x00c
#define PCIE_IRQ_EN_LEVEL		0x010
#define IRQ_MSI_ENABLE			(0x1 << 2)
#define PCIE_IRQ_EN_SPECIAL		0x014
#define PCIE_PWR_RESET			0x018
#define PCIE_CORE_RESET			0x01c
#define PCIE_CORE_RESET_ENABLE		(0x1 << 0)
#define PCIE_STICKY_RESET		0x020
#define PCIE_NONSTICKY_RESET		0x024
#define PCIE_APP_INIT_RESET		0x028
#define PCIE_APP_LTSSM_ENABLE		0x02c
#define PCIE_ELBI_RDLH_LINKUP		0x064
#define PCIE_ELBI_LTSSM_ENABLE		0x1
#define PCIE_ELBI_SLV_AWMISC		0x11c
#define PCIE_ELBI_SLV_ARMISC		0x120
#define PCIE_ELBI_SLV_DBI_ENABLE	(0x1 << 21)

/* PCIe Purple registers */
#define PCIE_PHY_GLOBAL_RESET		0x000
#define PCIE_PHY_COMMON_RESET		0x004
#define PCIE_PHY_CMN_REG		0x008
#define PCIE_PHY_MAC_RESET		0x00c
#define PCIE_PHY_PLL_LOCKED		0x010
#define PCIE_PHY_TRSVREG_RESET		0x020
#define PCIE_PHY_TRSV_RESET		0x024

/* PCIe PHY registers */
#define PCIE_PHY_IMPEDANCE		0x004
#define PCIE_PHY_PLL_DIV_0		0x008
#define PCIE_PHY_PLL_BIAS		0x00c
#define PCIE_PHY_DCC_FEEDBACK		0x014
#define PCIE_PHY_PLL_DIV_1		0x05c
#define PCIE_PHY_COMMON_POWER		0x064
#define PCIE_PHY_COMMON_PD_CMN		(0x1 << 3)
#define PCIE_PHY_TRSV0_EMP_LVL		0x084
#define PCIE_PHY_TRSV0_DRV_LVL		0x088
#define PCIE_PHY_TRSV0_RXCDR		0x0ac
#define PCIE_PHY_TRSV0_POWER		0x0c4
#define PCIE_PHY_TRSV0_PD_TSV		(0x1 << 7)
#define PCIE_PHY_TRSV0_LVCC		0x0dc
#define PCIE_PHY_TRSV1_EMP_LVL		0x144
#define PCIE_PHY_TRSV1_RXCDR		0x16c
#define PCIE_PHY_TRSV1_POWER		0x184
#define PCIE_PHY_TRSV1_PD_TSV		(0x1 << 7)
#define PCIE_PHY_TRSV1_LVCC		0x19c
#define PCIE_PHY_TRSV2_EMP_LVL		0x204
#define PCIE_PHY_TRSV2_RXCDR		0x22c
#define PCIE_PHY_TRSV2_POWER		0x244
#define PCIE_PHY_TRSV2_PD_TSV		(0x1 << 7)
#define PCIE_PHY_TRSV2_LVCC		0x25c
#define PCIE_PHY_TRSV3_EMP_LVL		0x2c4
#define PCIE_PHY_TRSV3_RXCDR		0x2ec
#define PCIE_PHY_TRSV3_POWER		0x304
#define PCIE_PHY_TRSV3_PD_TSV		(0x1 << 7)
#define PCIE_PHY_TRSV3_LVCC		0x31c
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

static inline void exynos_elb_writel(struct exynos_pcie *pcie, u32 val, u32 reg)
{
	writel(val, pcie->elbi_base + reg);
}

static inline u32 exynos_elb_readl(struct exynos_pcie *pcie, u32 reg)
{
	return readl(pcie->elbi_base + reg);
}

static inline void exynos_phy_writel(struct exynos_pcie *pcie, u32 val, u32 reg)
{
	writel(val, pcie->phy_base + reg);
}

static inline u32 exynos_phy_readl(struct exynos_pcie *pcie, u32 reg)
{
	return readl(pcie->phy_base + reg);
}

static inline void exynos_blk_writel(struct exynos_pcie *pcie, u32 val, u32 reg)
{
	writel(val, pcie->block_base + reg);
}

static inline u32 exynos_blk_readl(struct exynos_pcie *pcie, u32 reg)
{
	return readl(pcie->block_base + reg);
}

<<<<<<< HEAD
static ssize_t show_pcie(struct device *dev,
		struct device_attribute *attr,
		char *buf)
{

	return snprintf(buf, PAGE_SIZE, "0: send pme turn off message\n");
}

static ssize_t store_pcie(struct device *dev,
		struct device_attribute *attr,
		const char *buf, size_t count)
{
	int enable;
	u32 val;
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	if (sscanf(buf, "%10d", &enable) != 1)
		return -EINVAL;

	if (enable == 0) {
		exynos_elb_writel(exynos_pcie, 0x1, PCIE_APP_REQ_EXIT_L1);
		val = exynos_elb_readl(exynos_pcie, PCIE_APP_REQ_EXIT_L1_MODE);
		val &= ~APP_REQ_EXIT_L1_MODE;
		exynos_elb_writel(exynos_pcie, val, PCIE_APP_REQ_EXIT_L1_MODE);
		gpio_set_value(exynos_pcie->perst_gpio, 0);
		val = exynos_elb_readl(exynos_pcie, PCIE_APP_REQ_EXIT_L1_MODE);
		val |= APP_REQ_EXIT_L1_MODE;
		exynos_elb_writel(exynos_pcie, val, PCIE_APP_REQ_EXIT_L1_MODE);
		exynos_elb_writel(exynos_pcie, 0x0, PCIE_APP_REQ_EXIT_L1);
#ifdef CONFIG_PCI_EXYNOS_TEST
	} else if (enable == 1) {
		gpio_set_value(exynos_pcie->bt_gpio, 1);
		gpio_set_value(exynos_pcie->wlan_gpio, 1);
		mdelay(100);
		exynos_pcie_poweron(exynos_pcie->ch_num);
	} else if (enable == 2) {
		exynos_pcie_poweroff(exynos_pcie->ch_num);
		gpio_set_value(exynos_pcie->bt_gpio, 0);
		gpio_set_value(exynos_pcie->wlan_gpio, 0);
	} else if (enable == 3) {
		exynos_pcie_send_pme_turn_off(exynos_pcie);
#endif
	}

	return count;
}

static DEVICE_ATTR(pcie_sysfs, S_IWUSR | S_IWGRP | S_IRUSR | S_IRGRP,
			show_pcie, store_pcie);

static inline int create_pcie_sys_file(struct device *dev)
{
	return device_create_file(dev, &dev_attr_pcie_sysfs);
}

static inline void remove_pcie_sys_file(struct device *dev)
{
	device_remove_file(dev, &dev_attr_pcie_sysfs);
}

static void __maybe_unused exynos_pcie_notify_callback(struct pcie_port *pp, int event)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	if (exynos_pcie->event_reg && exynos_pcie->event_reg->callback &&
			(exynos_pcie->event_reg->events & event)) {
		struct exynos_pcie_notify *notify = &exynos_pcie->event_reg->notify;
		notify->event = event;
		notify->user = exynos_pcie->event_reg->user;
		dev_info(pp->dev, "Callback for the event : %d\n", event);
		exynos_pcie->event_reg->callback(notify);
	} else
		dev_info(pp->dev,
			"Client driver does not have registration of the event : %d\n", event);
}

int exynos_pcie_dump_link_down_status(int ch_num)
{
	struct pcie_port *pp = &g_pcie[ch_num].pp;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	if (exynos_pcie->state == STATE_LINK_UP) {
		dev_info(pp->dev, "LTSSM: 0x%08x\n",
				readl(exynos_pcie->elbi_base + PCIE_ELBI_RDLH_LINKUP));
		dev_info(pp->dev, "LTSSM_H: 0x%08x\n",
				readl(exynos_pcie->elbi_base + PCIE_CXPL_DEBUG_INFO_H));

		if (exynos_pcie->pcie_changed) {
			dev_info(pp->dev, "DMA_MONITOR1: 0x%08x\n",
					readl(exynos_pcie->elbi_base + PCIE_DMA_MONITOR1));
			dev_info(pp->dev, "DMA_MONITOR2: 0x%08x\n",
					readl(exynos_pcie->elbi_base + PCIE_DMA_MONITOR2));
			dev_info(pp->dev, "DMA_MONITOR3: 0x%08x\n",
					readl(exynos_pcie->elbi_base + PCIE_DMA_MONITOR3));
		}
	} else
		dev_info(pp->dev, "PCIE link state is %d\n", exynos_pcie->state);

	return 0;
}
EXPORT_SYMBOL(exynos_pcie_dump_link_down_status);

void exynos_pcie_print_link_history(struct pcie_port *pp)
{
	struct device *dev = pp->dev;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	u32 history_buffer[32];
	int i;

	for (i = 31; i >= 0; i--)
		history_buffer[i] = exynos_elb_readl(exynos_pcie,
				PCIE_HISTORY_REG(i));
	for (i = 31; i >= 0; i--)
		dev_info(dev, "LTSSM: 0x%02x, L1sub: 0x%x, D state: 0x%x\n",
				LTSSM_STATE(history_buffer[i]),
				L1SUB_STATE(history_buffer[i]),
				PM_DSTATE(history_buffer[i]));
}

int exynos_pcie_establish_link(struct pcie_port *pp)
{
	struct device *dev = pp->dev;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	u32 val;
	int count = 0, try_cnt = 0;

retry:
	if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
		/* avoid checking rx elecidle when access DBI */
		writel(readl(exynos_pcie->phy_pcs_base + 0xEC) | (0x1 << 3), exynos_pcie->phy_pcs_base + 0xEC);

		writel(0x0, exynos_pcie->elbi_base + PCIE_SOFT_CORE_RESET);
		udelay(20);
		writel(0x1, exynos_pcie->elbi_base + PCIE_SOFT_CORE_RESET);
	}

	/* set #PERST high */
	gpio_set_value(exynos_pcie->perst_gpio, 1);
	usleep_range(18000, 20000);

	if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
		/* APP_REQ_EXIT_L1_MODE : BIT0 (0x0 : S/W mode, 0x1 : H/W mode) */
		val = exynos_elb_readl(exynos_pcie, PCIE_APP_REQ_EXIT_L1_MODE);
		val |= APP_REQ_EXIT_L1_MODE;
		exynos_elb_writel(exynos_pcie, val, PCIE_APP_REQ_EXIT_L1_MODE);
		exynos_elb_writel(exynos_pcie, PCIE_LINKDOWN_RST_MANUAL,
				  PCIE_LINKDOWN_RST_CTRL_SEL);

#ifndef CONFIG_SOC_EXYNOS8890_EVT1
		/* setting for not-gating core clock in L2 state */
		val = exynos_elb_readl(exynos_pcie, PCIE_SOFT_AUXCLK_SEL_CTRL);
		val &= ~CORE_CLK_GATING;
		exynos_elb_writel(exynos_pcie, val, PCIE_SOFT_AUXCLK_SEL_CTRL);
#endif

		/* Q-Channel support and L1 NAK enable when AXI pending */
		if (exynos_pcie->pcie_changed) {
			val = exynos_elb_readl(exynos_pcie, PCIE_QCH_SEL);
			val &= ~CLOCK_GATING_MASK;
			val |= CLOCK_GATING_IN_L12;
			exynos_elb_writel(exynos_pcie, val, PCIE_QCH_SEL);
		}
	}

	exynos_pcie_assert_phy_reset(pp);

	exynos_elb_writel(exynos_pcie, 0x1, PCIE_L1_BUG_FIX_ENABLE);
=======
static void exynos_pcie_sideband_dbi_w_mode(struct pcie_port *pp, bool on)
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	if (on) {
		val = exynos_elb_readl(exynos_pcie, PCIE_ELBI_SLV_AWMISC);
		val |= PCIE_ELBI_SLV_DBI_ENABLE;
		exynos_elb_writel(exynos_pcie, val, PCIE_ELBI_SLV_AWMISC);
	} else {
		val = exynos_elb_readl(exynos_pcie, PCIE_ELBI_SLV_AWMISC);
		val &= ~PCIE_ELBI_SLV_DBI_ENABLE;
		exynos_elb_writel(exynos_pcie, val, PCIE_ELBI_SLV_AWMISC);
	}
}

static void exynos_pcie_sideband_dbi_r_mode(struct pcie_port *pp, bool on)
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	if (on) {
		val = exynos_elb_readl(exynos_pcie, PCIE_ELBI_SLV_ARMISC);
		val |= PCIE_ELBI_SLV_DBI_ENABLE;
		exynos_elb_writel(exynos_pcie, val, PCIE_ELBI_SLV_ARMISC);
	} else {
		val = exynos_elb_readl(exynos_pcie, PCIE_ELBI_SLV_ARMISC);
		val &= ~PCIE_ELBI_SLV_DBI_ENABLE;
		exynos_elb_writel(exynos_pcie, val, PCIE_ELBI_SLV_ARMISC);
	}
}

static void exynos_pcie_assert_core_reset(struct pcie_port *pp)
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	val = exynos_elb_readl(exynos_pcie, PCIE_CORE_RESET);
	val &= ~PCIE_CORE_RESET_ENABLE;
	exynos_elb_writel(exynos_pcie, val, PCIE_CORE_RESET);
	exynos_elb_writel(exynos_pcie, 0, PCIE_PWR_RESET);
	exynos_elb_writel(exynos_pcie, 0, PCIE_STICKY_RESET);
	exynos_elb_writel(exynos_pcie, 0, PCIE_NONSTICKY_RESET);
}

static void exynos_pcie_deassert_core_reset(struct pcie_port *pp)
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	val = exynos_elb_readl(exynos_pcie, PCIE_CORE_RESET);
	val |= PCIE_CORE_RESET_ENABLE;

	exynos_elb_writel(exynos_pcie, val, PCIE_CORE_RESET);
	exynos_elb_writel(exynos_pcie, 1, PCIE_STICKY_RESET);
	exynos_elb_writel(exynos_pcie, 1, PCIE_NONSTICKY_RESET);
	exynos_elb_writel(exynos_pcie, 1, PCIE_APP_INIT_RESET);
	exynos_elb_writel(exynos_pcie, 0, PCIE_APP_INIT_RESET);
	exynos_blk_writel(exynos_pcie, 1, PCIE_PHY_MAC_RESET);
}

static void exynos_pcie_assert_phy_reset(struct pcie_port *pp)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	exynos_blk_writel(exynos_pcie, 0, PCIE_PHY_MAC_RESET);
	exynos_blk_writel(exynos_pcie, 1, PCIE_PHY_GLOBAL_RESET);
}

static void exynos_pcie_deassert_phy_reset(struct pcie_port *pp)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	exynos_blk_writel(exynos_pcie, 0, PCIE_PHY_GLOBAL_RESET);
	exynos_elb_writel(exynos_pcie, 1, PCIE_PWR_RESET);
	exynos_blk_writel(exynos_pcie, 0, PCIE_PHY_COMMON_RESET);
	exynos_blk_writel(exynos_pcie, 0, PCIE_PHY_CMN_REG);
	exynos_blk_writel(exynos_pcie, 0, PCIE_PHY_TRSVREG_RESET);
	exynos_blk_writel(exynos_pcie, 0, PCIE_PHY_TRSV_RESET);
}

static void exynos_pcie_power_on_phy(struct pcie_port *pp)
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_COMMON_POWER);
	val &= ~PCIE_PHY_COMMON_PD_CMN;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_COMMON_POWER);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_TRSV0_POWER);
	val &= ~PCIE_PHY_TRSV0_PD_TSV;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_TRSV0_POWER);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_TRSV1_POWER);
	val &= ~PCIE_PHY_TRSV1_PD_TSV;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_TRSV1_POWER);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_TRSV2_POWER);
	val &= ~PCIE_PHY_TRSV2_PD_TSV;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_TRSV2_POWER);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_TRSV3_POWER);
	val &= ~PCIE_PHY_TRSV3_PD_TSV;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_TRSV3_POWER);
}

static void exynos_pcie_power_off_phy(struct pcie_port *pp)
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_COMMON_POWER);
	val |= PCIE_PHY_COMMON_PD_CMN;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_COMMON_POWER);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_TRSV0_POWER);
	val |= PCIE_PHY_TRSV0_PD_TSV;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_TRSV0_POWER);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_TRSV1_POWER);
	val |= PCIE_PHY_TRSV1_PD_TSV;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_TRSV1_POWER);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_TRSV2_POWER);
	val |= PCIE_PHY_TRSV2_PD_TSV;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_TRSV2_POWER);

	val = exynos_phy_readl(exynos_pcie, PCIE_PHY_TRSV3_POWER);
	val |= PCIE_PHY_TRSV3_PD_TSV;
	exynos_phy_writel(exynos_pcie, val, PCIE_PHY_TRSV3_POWER);
}

static void exynos_pcie_init_phy(struct pcie_port *pp)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	/* DCC feedback control off */
	exynos_phy_writel(exynos_pcie, 0x29, PCIE_PHY_DCC_FEEDBACK);

	/* set TX/RX impedance */
	exynos_phy_writel(exynos_pcie, 0xd5, PCIE_PHY_IMPEDANCE);

	/* set 50Mhz PHY clock */
	exynos_phy_writel(exynos_pcie, 0x14, PCIE_PHY_PLL_DIV_0);
	exynos_phy_writel(exynos_pcie, 0x12, PCIE_PHY_PLL_DIV_1);

	/* set TX Differential output for lane 0 */
	exynos_phy_writel(exynos_pcie, 0x7f, PCIE_PHY_TRSV0_DRV_LVL);

	/* set TX Pre-emphasis Level Control for lane 0 to minimum */
	exynos_phy_writel(exynos_pcie, 0x0, PCIE_PHY_TRSV0_EMP_LVL);

	/* set RX clock and data recovery bandwidth */
	exynos_phy_writel(exynos_pcie, 0xe7, PCIE_PHY_PLL_BIAS);
	exynos_phy_writel(exynos_pcie, 0x82, PCIE_PHY_TRSV0_RXCDR);
	exynos_phy_writel(exynos_pcie, 0x82, PCIE_PHY_TRSV1_RXCDR);
	exynos_phy_writel(exynos_pcie, 0x82, PCIE_PHY_TRSV2_RXCDR);
	exynos_phy_writel(exynos_pcie, 0x82, PCIE_PHY_TRSV3_RXCDR);

	/* change TX Pre-emphasis Level Control for lanes */
	exynos_phy_writel(exynos_pcie, 0x39, PCIE_PHY_TRSV0_EMP_LVL);
	exynos_phy_writel(exynos_pcie, 0x39, PCIE_PHY_TRSV1_EMP_LVL);
	exynos_phy_writel(exynos_pcie, 0x39, PCIE_PHY_TRSV2_EMP_LVL);
	exynos_phy_writel(exynos_pcie, 0x39, PCIE_PHY_TRSV3_EMP_LVL);

	/* set LVCC */
	exynos_phy_writel(exynos_pcie, 0x20, PCIE_PHY_TRSV0_LVCC);
	exynos_phy_writel(exynos_pcie, 0xa0, PCIE_PHY_TRSV1_LVCC);
	exynos_phy_writel(exynos_pcie, 0xa0, PCIE_PHY_TRSV2_LVCC);
	exynos_phy_writel(exynos_pcie, 0xa0, PCIE_PHY_TRSV3_LVCC);
}

static void exynos_pcie_assert_reset(struct pcie_port *pp)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	if (exynos_pcie->reset_gpio >= 0)
		devm_gpio_request_one(pp->dev, exynos_pcie->reset_gpio,
				GPIOF_OUT_INIT_HIGH, "RESET");
	return;
}

static int exynos_pcie_establish_link(struct pcie_port *pp)
{
	u32 val;
	int count = 0;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	if (dw_pcie_link_up(pp)) {
		dev_err(pp->dev, "Link already up\n");
		return 0;
	}

	/* assert reset signals */
	exynos_pcie_assert_core_reset(pp);
	exynos_pcie_assert_phy_reset(pp);

	/* de-assert phy reset */
	exynos_pcie_deassert_phy_reset(pp);

	/* power on phy */
	exynos_pcie_power_on_phy(pp);

	/* initialize phy */
	exynos_pcie_init_phy(pp);

	/* pulse for common reset */
	exynos_blk_writel(exynos_pcie, 1, PCIE_PHY_COMMON_RESET);
	udelay(500);
	exynos_blk_writel(exynos_pcie, 0, PCIE_PHY_COMMON_RESET);

	/* de-assert core reset */
	exynos_pcie_deassert_core_reset(pp);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

	/* setup root complex */
	dw_pcie_setup_rc(pp);

<<<<<<< HEAD
	if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie"))
		writel(readl(exynos_pcie->phy_pcs_base + 0xEC) & ~(0x1 << 3), exynos_pcie->phy_pcs_base + 0xEC);

	dev_info(dev, "D state: %x, %x\n",
		 exynos_elb_readl(exynos_pcie, PCIE_PM_DSTATE) & 0x7,
		 exynos_elb_readl(exynos_pcie, PCIE_ELBI_RDLH_LINKUP));

	/* assert LTSSM enable */
	exynos_elb_writel(exynos_pcie, PCIE_ELBI_LTSSM_ENABLE, PCIE_APP_LTSSM_ENABLE);
	count = 0;
	while (count < MAX_TIMEOUT) {
		val = exynos_elb_readl(exynos_pcie, PCIE_ELBI_RDLH_LINKUP) & 0x1f;
		if (val >= 0x0d && val <= 0x15)
			break;

		count++;

		udelay(10);
	}

	if (count >= MAX_TIMEOUT) {
		try_cnt++;
		dev_err(dev, "%s: Link is not up, try count: %d, %x\n", __func__,
			try_cnt, exynos_elb_readl(exynos_pcie, PCIE_ELBI_RDLH_LINKUP));
		if (try_cnt < 10) {
			gpio_set_value(exynos_pcie->perst_gpio, 0);
			/* LTSSM disable */
			exynos_elb_writel(exynos_pcie, PCIE_ELBI_LTSSM_DISABLE,
					  PCIE_APP_LTSSM_ENABLE);
			goto retry;
		} else {
			exynos_pcie_print_link_history(pp);
			BUG_ON(1);
			return -EPIPE;
		}
	} else {
		dev_info(dev, "%s: Link up:%x\n", __func__,
			 exynos_elb_readl(exynos_pcie, PCIE_ELBI_RDLH_LINKUP));

		if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
			val = exynos_elb_readl(exynos_pcie, PCIE_IRQ_PULSE);
			exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_PULSE);
			val = exynos_elb_readl(exynos_pcie, PCIE_IRQ_EN_PULSE);
			val |= IRQ_LINKDOWN_ENABLE;
			exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_EN_PULSE);
		}
	}

	return 0;
}

void exynos_pcie_work(struct work_struct *work)
{
	struct exynos_pcie *exynos_pcie = container_of(work, struct exynos_pcie, work.work);
	struct pcie_port *pp = &exynos_pcie->pp;
	struct device *dev = pp->dev;

	if (exynos_pcie->state == STATE_LINK_DOWN)
		return;

	exynos_pcie->linkdown_cnt++;
	dev_info(dev, "link down and recovery cnt: %d\n", exynos_pcie->linkdown_cnt);

#ifdef CONFIG_PCI_EXYNOS_TEST
	exynos_pcie_poweroff(exynos_pcie->ch_num);
	exynos_pcie_poweron(exynos_pcie->ch_num);
#else
	exynos_pcie_notify_callback(pp, EXYNOS_PCIE_EVENT_LINKDOWN);
#endif
}

static void exynos_pcie_assert_phy_reset(struct pcie_port *pp)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	u32 val;

	exynos_pcie_phy_config(exynos_pcie->phy_base, exynos_pcie->phy_pcs_base,
				exynos_pcie->block_base, exynos_pcie->elbi_base);
	exynos_pcie_phy_clock_enable(&exynos_pcie->pp, 1);

	/* Bus number enable */
	val = exynos_elb_readl(exynos_pcie, PCIE_SW_WAKE);
	val &= ~(0x1<<1);
	exynos_elb_writel(exynos_pcie, val, PCIE_SW_WAKE);
}

static void exynos_pcie_enable_interrupts(struct pcie_port *pp)
=======
	/* assert reset signal */
	exynos_pcie_assert_reset(pp);

	/* assert LTSSM enable */
	exynos_elb_writel(exynos_pcie, PCIE_ELBI_LTSSM_ENABLE,
			  PCIE_APP_LTSSM_ENABLE);

	/* check if the link is up or not */
	while (!dw_pcie_link_up(pp)) {
		mdelay(100);
		count++;
		if (count == 10) {
			while (exynos_phy_readl(exynos_pcie,
						PCIE_PHY_PLL_LOCKED) == 0) {
				val = exynos_blk_readl(exynos_pcie,
						       PCIE_PHY_PLL_LOCKED);
				dev_info(pp->dev, "PLL Locked: 0x%x\n", val);
			}
			/* power off phy */
			exynos_pcie_power_off_phy(pp);

			dev_err(pp->dev, "PCIe Link Fail\n");
			return -EINVAL;
		}
	}

	dev_info(pp->dev, "Link up\n");

	return 0;
}

static void exynos_pcie_clear_irq_pulse(struct pcie_port *pp)
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	val = exynos_elb_readl(exynos_pcie, PCIE_IRQ_PULSE);
	exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_PULSE);
	return;
}

static void exynos_pcie_enable_irq_pulse(struct pcie_port *pp)
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	/* enable INTX interrupt */
	val = IRQ_INTA_ASSERT | IRQ_INTB_ASSERT |
<<<<<<< HEAD
		IRQ_INTC_ASSERT | IRQ_INTD_ASSERT;
	exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_EN_PULSE);

	/* disable LEVEL interrupt */
	exynos_elb_writel(exynos_pcie, 0x0, PCIE_IRQ_EN_LEVEL);

	/* disable SPECIAL interrupt */
	exynos_elb_writel(exynos_pcie, 0x0, PCIE_IRQ_EN_SPECIAL);

=======
		IRQ_INTC_ASSERT | IRQ_INTD_ASSERT,
	exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_EN_PULSE);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	return;
}

static irqreturn_t exynos_pcie_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;
<<<<<<< HEAD
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

	/* handle PULSE interrupt */
	val = exynos_elb_readl(exynos_pcie, PCIE_IRQ_PULSE);
	exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_PULSE);

	if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
		if (val & IRQ_LINK_DOWN) {
			dev_info(pp->dev, "!!!PCIE LINK DOWN!!!\n");
			exynos_pcie_print_link_history(pp);
			exynos_pcie_dump_link_down_status(exynos_pcie->ch_num);
			queue_work(exynos_pcie->pcie_wq, &exynos_pcie->work.work);
		}
	}

	/* handle LEVEL interrupt */
	val = exynos_elb_readl(exynos_pcie, PCIE_IRQ_LEVEL);
#ifdef CONFIG_PCI_MSI
	if ((val | IRQ_MSI_CTRL) && exynos_pcie->use_msi)
		dw_handle_msi_irq(pp);
#endif
	exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_LEVEL);

	/* handle SPECIAL interrupt */
	val = exynos_elb_readl(exynos_pcie, PCIE_IRQ_SPECIAL);
	exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_SPECIAL);

	return IRQ_HANDLED;
}

#ifdef CONFIG_PCI_MSI
=======

	exynos_pcie_clear_irq_pulse(pp);
	return IRQ_HANDLED;
}

static irqreturn_t exynos_pcie_msi_irq_handler(int irq, void *arg)
{
	struct pcie_port *pp = arg;

	return dw_handle_msi_irq(pp);
}

>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
static void exynos_pcie_msi_init(struct pcie_port *pp)
{
	u32 val;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);

<<<<<<< HEAD
	if (!exynos_pcie->use_msi)
		return;

=======
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	dw_pcie_msi_init(pp);

	/* enable MSI interrupt */
	val = exynos_elb_readl(exynos_pcie, PCIE_IRQ_EN_LEVEL);
<<<<<<< HEAD
	val |= IRQ_MSI_CTRL;
	exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_EN_LEVEL);
	return;
}
#endif
=======
	val |= IRQ_MSI_ENABLE;
	exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_EN_LEVEL);
	return;
}

static void exynos_pcie_enable_interrupts(struct pcie_port *pp)
{
	exynos_pcie_enable_irq_pulse(pp);

	if (IS_ENABLED(CONFIG_PCI_MSI))
		exynos_pcie_msi_init(pp);

	return;
}

static inline void exynos_pcie_readl_rc(struct pcie_port *pp,
					void __iomem *dbi_base, u32 *val)
{
	exynos_pcie_sideband_dbi_r_mode(pp, true);
	*val = readl(dbi_base);
	exynos_pcie_sideband_dbi_r_mode(pp, false);
	return;
}

static inline void exynos_pcie_writel_rc(struct pcie_port *pp,
					u32 val, void __iomem *dbi_base)
{
	exynos_pcie_sideband_dbi_w_mode(pp, true);
	writel(val, dbi_base);
	exynos_pcie_sideband_dbi_w_mode(pp, false);
	return;
}
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

static int exynos_pcie_rd_own_conf(struct pcie_port *pp, int where, int size,
				u32 *val)
{
<<<<<<< HEAD
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	int ret = 0;

	if (!exynos_pcie->pcie_changed) {
		exynos_elb_writel(exynos_pcie, 0x1, PCIE_APP_REQ_EXIT_L1);
		exynos_elb_writel(exynos_pcie, 0x0, PCIE_APP_REQ_EXIT_L1_MODE);
	}
	ret = dw_pcie_cfg_read(exynos_pcie->rc_dbi_base + (where & ~0x3), where, size, val);
	if (!exynos_pcie->pcie_changed) {
		exynos_elb_writel(exynos_pcie, 0x1, PCIE_APP_REQ_EXIT_L1_MODE);
		exynos_elb_writel(exynos_pcie, 0x0, PCIE_APP_REQ_EXIT_L1);
	}

=======
	int ret;

	exynos_pcie_sideband_dbi_r_mode(pp, true);
	ret = dw_pcie_cfg_read(pp->dbi_base + (where & ~0x3), where, size, val);
	exynos_pcie_sideband_dbi_r_mode(pp, false);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	return ret;
}

static int exynos_pcie_wr_own_conf(struct pcie_port *pp, int where, int size,
				u32 val)
{
<<<<<<< HEAD
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	int ret = 0;

	if (!exynos_pcie->pcie_changed) {
		exynos_elb_writel(exynos_pcie, 0x1, PCIE_APP_REQ_EXIT_L1);
		exynos_elb_writel(exynos_pcie, 0x0, PCIE_APP_REQ_EXIT_L1_MODE);
	}
	ret = dw_pcie_cfg_write(exynos_pcie->rc_dbi_base + (where & ~0x3), where, size, val);
	if (!exynos_pcie->pcie_changed) {
		exynos_elb_writel(exynos_pcie, 0x1, PCIE_APP_REQ_EXIT_L1_MODE);
		exynos_elb_writel(exynos_pcie, 0x0, PCIE_APP_REQ_EXIT_L1);
	}

=======
	int ret;

	exynos_pcie_sideband_dbi_w_mode(pp, true);
	ret = dw_pcie_cfg_write(pp->dbi_base + (where & ~0x3),
			where, size, val);
	exynos_pcie_sideband_dbi_w_mode(pp, false);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	return ret;
}

static int exynos_pcie_link_up(struct pcie_port *pp)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
<<<<<<< HEAD
	u32 val;

	val = readl(exynos_pcie->elbi_base + PCIE_ELBI_RDLH_LINKUP) & 0x1f;
	if (val >= 0x0d && val <= 0x15)
=======
	u32 val = exynos_elb_readl(exynos_pcie, PCIE_ELBI_RDLH_LINKUP);

	if (val == PCIE_ELBI_LTSSM_ENABLE)
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
		return 1;

	return 0;
}

<<<<<<< HEAD
static struct pcie_host_ops exynos_pcie_host_ops = {
	.rd_own_conf = exynos_pcie_rd_own_conf,
	.wr_own_conf = exynos_pcie_wr_own_conf,
	.link_up = exynos_pcie_link_up,
=======
static void exynos_pcie_host_init(struct pcie_port *pp)
{
	exynos_pcie_establish_link(pp);
	exynos_pcie_enable_interrupts(pp);
}

static struct pcie_host_ops exynos_pcie_host_ops = {
	.readl_rc = exynos_pcie_readl_rc,
	.writel_rc = exynos_pcie_writel_rc,
	.rd_own_conf = exynos_pcie_rd_own_conf,
	.wr_own_conf = exynos_pcie_wr_own_conf,
	.link_up = exynos_pcie_link_up,
	.host_init = exynos_pcie_host_init,
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
};

static int __init add_pcie_port(struct pcie_port *pp,
				struct platform_device *pdev)
{
	int ret;

<<<<<<< HEAD
	pp->irq = platform_get_irq(pdev, 0);
=======
	pp->irq = platform_get_irq(pdev, 1);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	if (!pp->irq) {
		dev_err(&pdev->dev, "failed to get irq\n");
		return -ENODEV;
	}
	ret = devm_request_irq(&pdev->dev, pp->irq, exynos_pcie_irq_handler,
				IRQF_SHARED, "exynos-pcie", pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to request irq\n");
		return ret;
	}

<<<<<<< HEAD
	pp->root_bus_nr = 0;
	pp->ops = &exynos_pcie_host_ops;

	spin_lock_init(&pp->conf_lock);
=======
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 0);
		if (!pp->msi_irq) {
			dev_err(&pdev->dev, "failed to get msi irq\n");
			return -ENODEV;
		}

		ret = devm_request_irq(&pdev->dev, pp->msi_irq,
					exynos_pcie_msi_irq_handler,
					IRQF_SHARED | IRQF_NO_THREAD,
					"exynos-pcie", pp);
		if (ret) {
			dev_err(&pdev->dev, "failed to request msi irq\n");
			return ret;
		}
	}

	pp->root_bus_nr = -1;
	pp->ops = &exynos_pcie_host_ops;
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(&pdev->dev, "failed to initialize host\n");
		return ret;
	}

	return 0;
}

static int __init exynos_pcie_probe(struct platform_device *pdev)
{
	struct exynos_pcie *exynos_pcie;
	struct pcie_port *pp;
	struct device_node *np = pdev->dev.of_node;
	struct resource *elbi_base;
	struct resource *phy_base;
	struct resource *block_base;
<<<<<<< HEAD
	struct resource *rc_dbi_base;
	struct resource *phy_pcs_base;
	struct pinctrl *pinctrl_reset;
	int ch_num;
	int ret = 0;
	int mem_index = 0;

	if (create_pcie_sys_file(&pdev->dev))
		dev_err(&pdev->dev, "Failed to create pcie sys file\n");

	if (of_property_read_u32(np, "ch-num", &ch_num)) {
		dev_err(&pdev->dev, "Failed to parse the channel number\n");
		return -EINVAL;
	}

	exynos_pcie = &g_pcie[ch_num];
	pp = &exynos_pcie->pp;
	pp->dev = &pdev->dev;

	if (of_property_read_u32(np, "pcie-clk-num", &exynos_pcie->pcie_clk_num)) {
		dev_err(pp->dev, "Failed to parse the channel number\n");
		return -EINVAL;
	}

	if (of_property_read_u32(np, "phy-clk-num", &exynos_pcie->phy_clk_num)) {
		dev_err(pp->dev, "Failed to parse the channel number\n");
		return -EINVAL;
	}

	exynos_pcie->pcie_changed = of_property_read_bool(np, "pcie-changed");

	exynos_pcie->use_msi = of_property_read_bool(np, "use-msi");
	exynos_pcie->ch_num = ch_num;
	exynos_pcie->l1ss_enable = 1;
	exynos_pcie->state = STATE_LINK_DOWN;

	exynos_pcie->perst_gpio = of_get_gpio(np, 0);
#ifdef CONFIG_PM_DEVFREQ
	if (of_property_read_u32(np, "pcie-pm-qos-int", &exynos_pcie->int_min_lock))
		exynos_pcie->int_min_lock = 0;

	if (exynos_pcie->int_min_lock)
		pm_qos_add_request(&exynos_pcie_int_qos[ch_num],
				PM_QOS_DEVICE_THROUGHPUT, 0);
#endif

	if (exynos_pcie->perst_gpio < 0) {
		dev_err(&pdev->dev, "cannot get perst_gpio\n");
	} else {
		ret = devm_gpio_request_one(pp->dev, exynos_pcie->perst_gpio,
					    GPIOF_OUT_INIT_LOW,
					    dev_name(pp->dev));
		if (ret)
			goto probe_fail;
	}

	pinctrl_reset = devm_pinctrl_get_select(&pdev->dev, "clkreq_output");
	if (IS_ERR(pinctrl_reset)) {
		dev_err(&pdev->dev, "failed to set pcie clkerq pin to output high\n");
		goto probe_fail;
	}

#ifdef CONFIG_PCI_EXYNOS_TEST
	exynos_pcie->wlan_gpio = of_get_named_gpio(np, "pcie,wlan-gpio", 0);
	exynos_pcie->bt_gpio = of_get_named_gpio(np, "pcie,bt-gpio", 0);

	if (exynos_pcie->wlan_gpio < 0) {
		dev_err(&pdev->dev, "cannot get wlan_gpio\n");
	} else {
		ret = devm_gpio_request_one(pp->dev, exynos_pcie->wlan_gpio,
					    GPIOF_OUT_INIT_LOW,
					    dev_name(pp->dev));
		if (ret)
			goto probe_fail;
	}

	if (exynos_pcie->bt_gpio < 0) {
		dev_err(&pdev->dev, "cannot get bt_gpio\n");
	} else {
		ret = devm_gpio_request_one(pp->dev, exynos_pcie->bt_gpio,
					    GPIOF_OUT_INIT_LOW,
					    dev_name(pp->dev));
		if (ret)
			goto probe_fail;
	}
#endif

	exynos_pcie->linkdown_cnt = 0;

	ret = exynos_pcie_clock_get(pp);
	if (ret)
		goto probe_fail;

	elbi_base = platform_get_resource(pdev, IORESOURCE_MEM, mem_index++);
	exynos_pcie->elbi_base = devm_ioremap_resource(&pdev->dev, elbi_base);
	if (IS_ERR(exynos_pcie->elbi_base)) {
		ret = PTR_ERR(exynos_pcie->elbi_base);
		goto probe_fail;
	}

	phy_base = platform_get_resource(pdev, IORESOURCE_MEM, mem_index++);
	exynos_pcie->phy_base = devm_ioremap_resource(&pdev->dev, phy_base);
	if (IS_ERR(exynos_pcie->phy_base)) {
		ret = PTR_ERR(exynos_pcie->phy_base);
		goto probe_fail;
	}

	block_base = platform_get_resource(pdev, IORESOURCE_MEM, mem_index++);
	exynos_pcie->block_base = devm_ioremap_resource(&pdev->dev, block_base);
	if (IS_ERR(exynos_pcie->block_base)) {
		ret = PTR_ERR(exynos_pcie->block_base);
		goto probe_fail;
	}

	rc_dbi_base = platform_get_resource(pdev, IORESOURCE_MEM, mem_index++);
	exynos_pcie->rc_dbi_base = devm_ioremap_resource(&pdev->dev, rc_dbi_base);
	if (IS_ERR(exynos_pcie->rc_dbi_base)) {
		ret = PTR_ERR(exynos_pcie->rc_dbi_base);
		goto probe_fail;
	}

	phy_pcs_base = platform_get_resource(pdev, IORESOURCE_MEM, mem_index++);
	exynos_pcie->phy_pcs_base = devm_ioremap_resource(&pdev->dev, phy_pcs_base);
	if (IS_ERR(exynos_pcie->phy_pcs_base)) {
		ret = PTR_ERR(exynos_pcie->phy_pcs_base);
		goto probe_fail;
	}

	exynos_pcie->pmureg = syscon_regmap_lookup_by_phandle(np,
					"samsung,syscon-phandle");
	if (IS_ERR(exynos_pcie->pmureg)) {
		dev_err(&pdev->dev, "syscon regmap lookup failed.\n");
		return PTR_ERR(exynos_pcie->pmureg);
	}

	exynos_pcie_resumed_phydown(pp);

	ret = add_pcie_port(pp, pdev);
	if (ret)
		goto probe_fail;

#ifdef CONFIG_CPU_IDLE
	exynos_pcie->idle_ip_index = exynos_get_idle_ip_index(dev_name(&pdev->dev));
	exynos_update_ip_idle_status(exynos_pcie->idle_ip_index, 1);
	exynos_pcie->lpa_nb.notifier_call = exynos_pci_lpa_event;
	exynos_pcie->lpa_nb.next = NULL;
	exynos_pcie->lpa_nb.priority = 0;

	ret = exynos_pm_register_notifier(&exynos_pcie->lpa_nb);
	if (ret) {
		dev_err(&pdev->dev, "Failed to register lpa notifier\n");
		goto probe_fail;
	}
#endif

	exynos_pcie->pcie_wq = create_freezable_workqueue("pcie_wq");
	if (IS_ERR(exynos_pcie->pcie_wq)) {
		dev_err(pp->dev, "couldn't create workqueue\n");
		ret = EBUSY;
		goto probe_fail;
	}
	INIT_DELAYED_WORK(&exynos_pcie->work, exynos_pcie_work);

	platform_set_drvdata(pdev, exynos_pcie);

probe_fail:
	if (ret)
		dev_err(&pdev->dev, "%s: pcie probe failed\n", __func__);
	else
		dev_info(&pdev->dev, "%s: pcie probe success\n", __func__);

=======
	int ret;

	exynos_pcie = devm_kzalloc(&pdev->dev, sizeof(*exynos_pcie),
				GFP_KERNEL);
	if (!exynos_pcie)
		return -ENOMEM;

	pp = &exynos_pcie->pp;

	pp->dev = &pdev->dev;

	exynos_pcie->reset_gpio = of_get_named_gpio(np, "reset-gpio", 0);

	exynos_pcie->clk = devm_clk_get(&pdev->dev, "pcie");
	if (IS_ERR(exynos_pcie->clk)) {
		dev_err(&pdev->dev, "Failed to get pcie rc clock\n");
		return PTR_ERR(exynos_pcie->clk);
	}
	ret = clk_prepare_enable(exynos_pcie->clk);
	if (ret)
		return ret;

	exynos_pcie->bus_clk = devm_clk_get(&pdev->dev, "pcie_bus");
	if (IS_ERR(exynos_pcie->bus_clk)) {
		dev_err(&pdev->dev, "Failed to get pcie bus clock\n");
		ret = PTR_ERR(exynos_pcie->bus_clk);
		goto fail_clk;
	}
	ret = clk_prepare_enable(exynos_pcie->bus_clk);
	if (ret)
		goto fail_clk;

	elbi_base = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	exynos_pcie->elbi_base = devm_ioremap_resource(&pdev->dev, elbi_base);
	if (IS_ERR(exynos_pcie->elbi_base)) {
		ret = PTR_ERR(exynos_pcie->elbi_base);
		goto fail_bus_clk;
	}

	phy_base = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	exynos_pcie->phy_base = devm_ioremap_resource(&pdev->dev, phy_base);
	if (IS_ERR(exynos_pcie->phy_base)) {
		ret = PTR_ERR(exynos_pcie->phy_base);
		goto fail_bus_clk;
	}

	block_base = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	exynos_pcie->block_base = devm_ioremap_resource(&pdev->dev, block_base);
	if (IS_ERR(exynos_pcie->block_base)) {
		ret = PTR_ERR(exynos_pcie->block_base);
		goto fail_bus_clk;
	}

	ret = add_pcie_port(pp, pdev);
	if (ret < 0)
		goto fail_bus_clk;

	platform_set_drvdata(pdev, exynos_pcie);
	return 0;

fail_bus_clk:
	clk_disable_unprepare(exynos_pcie->bus_clk);
fail_clk:
	clk_disable_unprepare(exynos_pcie->clk);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	return ret;
}

static int __exit exynos_pcie_remove(struct platform_device *pdev)
{
	struct exynos_pcie *exynos_pcie = platform_get_drvdata(pdev);
<<<<<<< HEAD
	struct pcie_port *pp = &exynos_pcie->pp;
	u32 val;

#ifdef CONFIG_CPU_IDLE
	exynos_pm_unregister_notifier(&exynos_pcie->lpa_nb);
#endif
	if (exynos_pcie->state > STATE_LINK_DOWN) {
		if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
			val = exynos_phy_readl(exynos_pcie, 0x15*4);
			val |= 0xf << 3;
			exynos_phy_writel(exynos_pcie, val, 0x15*4);
			exynos_phy_writel(exynos_pcie, 0xff, 0x4E*4);
			exynos_phy_writel(exynos_pcie, 0x3f, 0x4F*4);
		}
		exynos_pcie_phy_clock_enable(pp, 0);
		regmap_update_bits(exynos_pcie->pmureg,
				   PCIE_PHY_CONTROL + exynos_pcie->ch_num * 4,
				   PCIE_PHY_CONTROL_MASK, 0);
		exynos_pcie_clock_enable(pp, 0);
#ifdef CONFIG_CPU_IDLE
		exynos_update_ip_idle_status(exynos_pcie->idle_ip_index, 1);
#endif
	}

	remove_pcie_sys_file(&pdev->dev);
=======

	clk_disable_unprepare(exynos_pcie->bus_clk);
	clk_disable_unprepare(exynos_pcie->clk);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

	return 0;
}

static const struct of_device_id exynos_pcie_of_match[] = {
<<<<<<< HEAD
	{ .compatible = "samsung,exynos8890-pcie", },
=======
	{ .compatible = "samsung,exynos5440-pcie", },
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
	{},
};
MODULE_DEVICE_TABLE(of, exynos_pcie_of_match);

<<<<<<< HEAD
static void exynos_pcie_resumed_phydown(struct pcie_port *pp)
{
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	u32 val;

	/* phy all power down on wifi off during suspend/resume */
	exynos_pcie_clock_enable(pp, 1);

	exynos_pcie_enable_interrupts(pp);
	regmap_update_bits(exynos_pcie->pmureg,
			   PCIE_PHY_CONTROL + exynos_pcie->ch_num * 4,
			   PCIE_PHY_CONTROL_MASK, 1);

	exynos_pcie_assert_phy_reset(pp);

	/* phy all power down */
	if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
		val = exynos_phy_readl(exynos_pcie, 0x15*4);
		val |= 0xf << 3;
		exynos_phy_writel(exynos_pcie, val, 0x15*4);
		exynos_phy_writel(exynos_pcie, 0xff, 0x4E*4);
		exynos_phy_writel(exynos_pcie, 0x3f, 0x4F*4);
	}

	exynos_pcie_phy_clock_enable(pp, 0);
	regmap_update_bits(exynos_pcie->pmureg,
			   PCIE_PHY_CONTROL + exynos_pcie->ch_num * 4,
			   PCIE_PHY_CONTROL_MASK, 0);
	exynos_pcie_clock_enable(pp, 0);
}

int exynos_pcie_poweron(int ch_num)
{
	struct pcie_port *pp = &g_pcie[ch_num].pp;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	struct pinctrl *pinctrl_reset;
	u32 val, vendor_id, device_id;

	dev_info(pp->dev, "%s, start of poweron, pcie state: %d\n", __func__,
		 exynos_pcie->state);

	if (exynos_pcie->state == STATE_LINK_DOWN) {
#ifdef CONFIG_CPU_IDLE
		exynos_update_ip_idle_status(exynos_pcie->idle_ip_index, 0);
#endif
#ifdef CONFIG_PM_DEVFREQ
		if (exynos_pcie->int_min_lock)
			pm_qos_update_request(&exynos_pcie_int_qos[ch_num],
						exynos_pcie->int_min_lock);
#endif
		pinctrl_reset = devm_pinctrl_get_select_default(pp->dev);
		if (IS_ERR(pinctrl_reset)) {
			dev_err(pp->dev, "faied to set pcie pin state\n");
			goto poweron_fail;
		}

		exynos_pcie_clock_enable(pp, 1);
		regmap_update_bits(exynos_pcie->pmureg,
				   PCIE_PHY_CONTROL + exynos_pcie->ch_num * 4,
				   PCIE_PHY_CONTROL_MASK, 1);

		/* phy all power down clear */
		if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
			val = exynos_phy_readl(exynos_pcie, 0x15*4);
			val &= ~(0xf << 3);
			exynos_phy_writel(exynos_pcie, val, 0x15*4);
			exynos_phy_writel(exynos_pcie, 0x0, 0x4E*4);
			exynos_phy_writel(exynos_pcie, 0x0, 0x4F*4);
		}

		exynos_pcie->state = STATE_LINK_UP_TRY;

		/* Enable history buffer */
		val = exynos_elb_readl(exynos_pcie, PCIE_STATE_HISTORY_CHECK);
		val |= HISTORY_BUFFER_ENABLE;
		exynos_elb_writel(exynos_pcie, val, PCIE_STATE_HISTORY_CHECK);

		if (exynos_pcie_establish_link(pp)) {
			dev_err(pp->dev, "pcie link up fail\n");
			goto poweron_fail;
		}
		exynos_pcie->state = STATE_LINK_UP;
		if (!exynos_pcie->probe_ok) {
			if (dw_pcie_scan(pp)) {
				dev_err(pp->dev, "pcie scan fail\n");
				goto poweron_fail;
			}

			exynos_pcie_rd_own_conf(pp, PCI_VENDOR_ID, 4, &val);
			vendor_id = val & ID_MASK;
			device_id = (val >> 16) & ID_MASK;

			exynos_pcie->pci_dev = pci_get_device(vendor_id, device_id, NULL);
			if (!exynos_pcie->pci_dev) {
				dev_err(pp->dev, "Failed to get pci device\n");
				goto poweron_fail;
			}

#ifdef CONFIG_PCI_MSI
			exynos_pcie_msi_init(pp);
#endif

			if (pci_save_state(exynos_pcie->pci_dev)) {
				dev_err(pp->dev, "Failed to save pcie state\n");
				goto poweron_fail;
			}
			exynos_pcie->pci_saved_configs =
				pci_store_saved_state(exynos_pcie->pci_dev);
			exynos_pcie->probe_ok = 1;
		} else if (exynos_pcie->probe_ok) {
			/* setup ATU for cfg/mem outbound */
			dw_pcie_prog_viewport_cfg0(pp, 0x1000000);
			dw_pcie_prog_viewport_mem_outbound(pp);

			/* L1.2 ASPM enable */
			dw_pcie_config_l1ss(pp);
#ifdef CONFIG_PCI_MSI
			exynos_pcie_msi_init(pp);
#endif

			if (pci_load_saved_state(exynos_pcie->pci_dev,
					     exynos_pcie->pci_saved_configs)) {
				dev_err(pp->dev, "Failed to load pcie state\n");
				goto poweron_fail;
			}
			pci_restore_state(exynos_pcie->pci_dev);
		}
	}

	dev_info(pp->dev, "%s, end of poweron, pcie state: %d\n", __func__,
		 exynos_pcie->state);

	return 0;

poweron_fail:
	exynos_pcie->state = STATE_LINK_UP;
	exynos_pcie_poweroff(exynos_pcie->ch_num);

	return -EPIPE;
}
EXPORT_SYMBOL(exynos_pcie_poweron);

void exynos_pcie_poweroff(int ch_num)
{
	struct pcie_port *pp = &g_pcie[ch_num].pp;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	struct pinctrl *pinctrl_reset;
	unsigned long flags;
	u32 val;

	dev_info(pp->dev, "%s, start of poweroff, pcie state: %d\n", __func__,
		 exynos_pcie->state);

	if (exynos_pcie->state == STATE_LINK_UP ||
	    exynos_pcie->state == STATE_LINK_DOWN_TRY) {
		exynos_pcie->state = STATE_LINK_DOWN_TRY;

		if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
			val = exynos_elb_readl(exynos_pcie, PCIE_IRQ_EN_PULSE);
			val &= ~IRQ_LINKDOWN_ENABLE;
			exynos_elb_writel(exynos_pcie, val, PCIE_IRQ_EN_PULSE);
		}

		spin_lock_irqsave(&pp->conf_lock, flags);
		exynos_pcie->state = STATE_LINK_DOWN;

		/* Disable history buffer */
		val = exynos_elb_readl(exynos_pcie, PCIE_STATE_HISTORY_CHECK);
		val &= ~HISTORY_BUFFER_ENABLE;
		exynos_elb_writel(exynos_pcie, val, PCIE_STATE_HISTORY_CHECK);

		gpio_set_value(exynos_pcie->perst_gpio, 0);
		/* LTSSM disable */
		writel(PCIE_ELBI_LTSSM_DISABLE, exynos_pcie->elbi_base + PCIE_APP_LTSSM_ENABLE);

		/* phy all power down */
		if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
			val = exynos_phy_readl(exynos_pcie, 0x15*4);
			val |= 0xf << 3;
			exynos_phy_writel(exynos_pcie, val, 0x15*4);
			exynos_phy_writel(exynos_pcie, 0xff, 0x4E*4);
			exynos_phy_writel(exynos_pcie, 0x3f, 0x4F*4);
		}

		spin_unlock_irqrestore(&pp->conf_lock, flags);

		exynos_pcie_phy_clock_enable(pp, 0);
		regmap_update_bits(exynos_pcie->pmureg,
				   PCIE_PHY_CONTROL + exynos_pcie->ch_num * 4,
				   PCIE_PHY_CONTROL_MASK, 0);
		exynos_pcie_clock_enable(pp, 0);

		pinctrl_reset = devm_pinctrl_get_select(pp->dev, "clkreq_output");
		if (IS_ERR(pinctrl_reset)) {
			dev_err(pp->dev, "failed to set pcie clkerq pin to output high\n");
			return;
		}
#ifdef CONFIG_PM_DEVFREQ
		if (exynos_pcie->int_min_lock)
			pm_qos_update_request(&exynos_pcie_int_qos[ch_num], 0);
#endif
#ifdef CONFIG_CPU_IDLE
		exynos_update_ip_idle_status(exynos_pcie->idle_ip_index, 1);
#endif
	}

	dev_info(pp->dev, "%s, end of poweroff, pcie state: %d\n",  __func__,
		 exynos_pcie->state);
}
EXPORT_SYMBOL(exynos_pcie_poweroff);

void exynos_pcie_send_pme_turn_off(struct exynos_pcie *exynos_pcie)
{
	struct pcie_port *pp = &exynos_pcie->pp;
	struct device *dev = pp->dev;
	int __maybe_unused count = 0;
	u32 __maybe_unused val;

	if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie")) {
		exynos_elb_writel(exynos_pcie, 0x1, PCIE_APP_REQ_EXIT_L1);
		val = exynos_elb_readl(exynos_pcie, PCIE_APP_REQ_EXIT_L1_MODE);
		val &= ~APP_REQ_EXIT_L1_MODE;
		exynos_elb_writel(exynos_pcie, val, PCIE_APP_REQ_EXIT_L1_MODE);
		exynos_elb_writel(exynos_pcie, 0x0, 0xa8);
		exynos_elb_writel(exynos_pcie, 0x1, 0xac);
		exynos_elb_writel(exynos_pcie, 0x13, 0xb0);
		exynos_elb_writel(exynos_pcie, 0x19, 0xd0);
		exynos_elb_writel(exynos_pcie, 0x1, 0xa8);
	}

	while (count < MAX_TIMEOUT) {
		if ((exynos_elb_readl(exynos_pcie, PCIE_IRQ_PULSE)
		    & IRQ_RADM_PM_TO_ACK)) {
			dev_err(dev, "ack message is ok\n");
			break;
		}

		udelay(10);
		count++;
	}

	if (count >= MAX_TIMEOUT)
		dev_err(dev, "cannot receive ack message from wifi\n");

	if (of_device_is_compatible(pp->dev->of_node, "samsung,exynos8890-pcie"))
		exynos_elb_writel(exynos_pcie, 0x0, PCIE_APP_REQ_EXIT_L1);

	do {
		val = exynos_elb_readl(exynos_pcie, PCIE_ELBI_RDLH_LINKUP);
		val = val & 0x1f;
		if (val == 0x15) {
			dev_err(dev, "received Enter_L23_READY DLLP packet\n");
			break;
		}
		udelay(10);
		count++;
	} while (count < MAX_TIMEOUT);

	if (count >= MAX_TIMEOUT)
		dev_err(dev, "cannot receive L23_READY DLLP packet\n");
}

int exynos_pcie_pm_suspend(int ch_num)
{
	struct pcie_port *pp = &g_pcie[ch_num].pp;
	struct exynos_pcie *exynos_pcie = to_exynos_pcie(pp);
	unsigned long flags;

	if (exynos_pcie->state == STATE_LINK_DOWN) {
		dev_info(pp->dev, "RC%d already off\n", exynos_pcie->ch_num);
		return 0;
	}

	spin_lock_irqsave(&pp->conf_lock, flags);
	exynos_pcie->state = STATE_LINK_DOWN_TRY;
	spin_unlock_irqrestore(&pp->conf_lock, flags);

	exynos_pcie_send_pme_turn_off(exynos_pcie);
	exynos_pcie_poweroff(ch_num);

	return 0;
}
EXPORT_SYMBOL(exynos_pcie_pm_suspend);

int exynos_pcie_pm_resume(int ch_num)
{
	exynos_pcie_poweron(ch_num);

	return 0;
}
EXPORT_SYMBOL(exynos_pcie_pm_resume);

#ifdef CONFIG_PM
static int exynos_pcie_suspend_noirq(struct device *dev)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	if (exynos_pcie->state == STATE_LINK_DOWN) {
		dev_info(dev, "RC%d already off\n", exynos_pcie->ch_num);
		return 0;
	}

	exynos_pcie_send_pme_turn_off(exynos_pcie);
	gpio_set_value(exynos_pcie->perst_gpio, 0);

	return 0;
}

static int exynos_pcie_resume_noirq(struct device *dev)
{
	struct exynos_pcie *exynos_pcie = dev_get_drvdata(dev);

	if (exynos_pcie->state == STATE_LINK_DOWN) {
		exynos_pcie_resumed_phydown(&exynos_pcie->pp);
		return 0;
	}

	exynos_pcie_enable_interrupts(&exynos_pcie->pp);

	regmap_update_bits(exynos_pcie->pmureg,
			   PCIE_PHY_CONTROL + exynos_pcie->ch_num * 4,
			   PCIE_PHY_CONTROL_MASK, 1);

	exynos_pcie_establish_link(&exynos_pcie->pp);

	/* setup ATU for cfg/mem outbound */
	dw_pcie_prog_viewport_cfg0(&exynos_pcie->pp, 0x1000000);
	dw_pcie_prog_viewport_mem_outbound(&exynos_pcie->pp);

	/* L1.2 ASPM enable */
	dw_pcie_config_l1ss(&exynos_pcie->pp);

	return 0;
}

#else
#define exynos_pcie_suspend_noirq	NULL
#define exynos_pcie_resume_noirq	NULL
#endif

static const struct dev_pm_ops exynos_pcie_dev_pm_ops = {
	.suspend_noirq	= exynos_pcie_suspend_noirq,
	.resume_noirq	= exynos_pcie_resume_noirq,
};

=======
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
static struct platform_driver exynos_pcie_driver = {
	.remove		= __exit_p(exynos_pcie_remove),
	.driver = {
		.name	= "exynos-pcie",
		.owner	= THIS_MODULE,
		.of_match_table = exynos_pcie_of_match,
<<<<<<< HEAD
		.pm	= &exynos_pcie_dev_pm_ops,
	},
};

#ifdef CONFIG_CPU_IDLE
static int exynos_pci_lpa_event(struct notifier_block *nb, unsigned long event, void *data)
{
	int ret = NOTIFY_DONE;
	struct exynos_pcie *exynos_pcie = container_of(nb,
				struct exynos_pcie, lpa_nb);

	switch (event) {
	case LPA_EXIT:
		if (exynos_pcie->state == STATE_LINK_DOWN)
			exynos_pcie_resumed_phydown(&exynos_pcie->pp);
		break;
	default:
		ret = NOTIFY_DONE;
	}

	return notifier_from_errno(ret);
}
#endif

=======
	},
};

>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012
/* Exynos PCIe driver does not allow module unload */

static int __init pcie_init(void)
{
	return platform_driver_probe(&exynos_pcie_driver, exynos_pcie_probe);
}
<<<<<<< HEAD
device_initcall(pcie_init);

int exynos_pcie_register_event(struct exynos_pcie_register_event *reg)
{
	int ret = 0;
	struct pcie_port *pp;
	struct exynos_pcie *exynos_pcie;
	if (!reg) {
		pr_err("PCIe: Event registration is NULL\n");
		return -ENODEV;
	}
	if (!reg->user) {
		pr_err("PCIe: User of event registration is NULL\n");
		return -ENODEV;
	}
	pp = PCIE_BUS_PRIV_DATA(((struct pci_dev *)reg->user));
	exynos_pcie = to_exynos_pcie(pp);
	if (pp) {
		exynos_pcie->event_reg = reg;
		dev_info(pp->dev,
				"Event 0x%x is registered for RC %d\n",
				reg->events, exynos_pcie->ch_num);
	} else {
		pr_err("PCIe: did not find RC for pci endpoint device\n");
		ret = -ENODEV;
	}
	return ret;
}
EXPORT_SYMBOL(exynos_pcie_register_event);

int exynos_pcie_deregister_event(struct exynos_pcie_register_event *reg)
{
	int ret = 0;
	struct pcie_port *pp;
	struct exynos_pcie *exynos_pcie;
	if (!reg) {
		pr_err("PCIe: Event deregistration is NULL\n");
		return -ENODEV;
	}
	if (!reg->user) {
		pr_err("PCIe: User of event deregistration is NULL\n");
		return -ENODEV;
	}
	pp = PCIE_BUS_PRIV_DATA(((struct pci_dev *)reg->user));
	exynos_pcie = to_exynos_pcie(pp);
	if (pp) {
		exynos_pcie->event_reg = NULL;
		dev_info(pp->dev, "Event is deregistered for RC %d\n",
				exynos_pcie->ch_num);
	} else {
		pr_err("PCIe: did not find RC for pci endpoint device\n");
		ret = -ENODEV;
	}
	return ret;
}
EXPORT_SYMBOL(exynos_pcie_deregister_event);

int exynos_pcie_gpio_debug(int ch_num, int option)
{
	void __iomem *sysreg;
	u32 addr, val;

	addr = (GPIO_DEBUG_SFR);

	sysreg = ioremap_nocache(addr, 4);
	if (!sysreg) {
		pr_err("Cannot get the sysreg address\n");
		return -EPIPE;
	}
	val = readl(sysreg);
	val &= ~FSYS1_MON_SEL_MASK;
	val |= ch_num + 2;
	val &= ~(PCIE_MON_SEL_MASK << ((ch_num + 2) * 8));
	val |= option << ((ch_num + 2) * 8);
	writel(val , sysreg);
	iounmap(sysreg);

	return 0;
}
EXPORT_SYMBOL(exynos_pcie_gpio_debug);
=======
subsys_initcall(pcie_init);
>>>>>>> 80ceebea74b0d231ae55ba1623fd83e1fbd8b012

MODULE_AUTHOR("Jingoo Han <jg1.han@samsung.com>");
MODULE_DESCRIPTION("Samsung PCIe host controller driver");
MODULE_LICENSE("GPL v2");
