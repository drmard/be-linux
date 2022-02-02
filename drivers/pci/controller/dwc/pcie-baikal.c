// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for Baikal SoC
 *
 * Copyright (C) 2019-2021 Baikal Electronics, JSC
 * Authors: Pavel Parkhomenko <pavel.parkhomenko@baikalelectronics.ru>
 *          Mikhail Ivanov <michail.ivanov@baikalelectronics.ru>
 *          Aleksandr Efimov <alexander.efimov@baikalelectronics.ru>
 */

#include <linux/acpi.h>
#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <linux/kernel.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/pci-ecam.h>
#include <linux/phy/phy.h>
#include <linux/regmap.h>
#include <linux/resource.h>

#include "pcie-designware.h"

struct baikal_pcie_rc {
	struct dw_pcie	 *pcie;
	unsigned	 bus_nr;
	struct regmap	 *lcru;
	struct gpio_desc *reset_gpio;
	bool		 reset_active_low;
	char		 reset_name[32];
	bool		 retrained;
};

#define to_baikal_pcie_rc(x)	dev_get_drvdata((x)->dev)

#define PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG	0x78
#define PCIE_CAP_CORR_ERR_REPORT_EN		BIT(0)
#define PCIE_CAP_NON_FATAL_ERR_REPORT_EN	BIT(1)
#define PCIE_CAP_FATAL_ERR_REPORT_EN		BIT(2)
#define PCIE_CAP_UNSUPPORT_REQ_REP_EN		BIT(3)

#define PCIE_LINK_CAPABILITIES_REG		0x7c

#define PCIE_LINK_CONTROL_LINK_STATUS_REG	0x80
#define PCIE_CAP_LINK_SPEED_MASK		0xf0000
#define PCIE_CAP_LINK_SPEED_SHIFT		16
#define PCIE_CAP_NEGO_LINK_WIDTH_MASK		0x3f00000
#define PCIE_CAP_NEGO_LINK_WIDTH_SHIFT		20
#define PCIE_CAP_LINK_TRAINING			BIT(27)

#define PCIE_ROOT_CONTROL_ROOT_CAPABILITIES_REG	0x8c
#define PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN		BIT(0)
#define PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN	BIT(1)
#define PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN	BIT(2)
#define PCIE_CAP_PME_INT_EN			BIT(3)

#define PCIE_LINK_CONTROL2_LINK_STATUS2_REG	0xa0
#define PCIE_CAP_TARGET_LINK_SPEED_MASK		0xf

#define PCIE_UNCORR_ERR_STATUS_REG		0x104
#define PCIE_CORR_ERR_STATUS_REG		0x110

#define PCIE_ROOT_ERR_CMD_REG			0x12c
#define PCIE_CORR_ERR_REPORTING_EN		BIT(0)
#define PCIE_NON_FATAL_ERR_REPORTING_EN		BIT(1)
#define PCIE_FATAL_ERR_REPORTING_EN		BIT(2)

#define PCIE_ROOT_ERR_STATUS_REG		0x130

#define PCIE_GEN2_CTRL_REG			0x80c
#define PCIE_DIRECT_SPEED_CHANGE		BIT(17)

#define PCIE_IATU_VIEWPORT_REG			0x900
#define PCIE_IATU_REGION_OUTBOUND		0
#define PCIE_IATU_REGION_CTRL_2_REG		0x908

#define BAIKAL_LCRU_PCIE_RESET_BASE		0x50000
#define BAIKAL_LCRU_PCIE_RESET(x)		((x * 0x20) + BAIKAL_LCRU_PCIE_RESET_BASE)
#define BAIKAL_PCIE_PHY_RST			BIT(0)
#define BAIKAL_PCIE_PIPE_RST			BIT(4) /* x4 controllers only */
#define BAIKAL_PCIE_PIPE0_RST			BIT(4) /* x8 controller only */
#define BAIKAL_PCIE_PIPE1_RST			BIT(5) /* x8 controller only */
#define BAIKAL_PCIE_CORE_RST			BIT(8)
#define BAIKAL_PCIE_PWR_RST			BIT(9)
#define BAIKAL_PCIE_STICKY_RST			BIT(10)
#define BAIKAL_PCIE_NONSTICKY_RST		BIT(11)
#define BAIKAL_PCIE_HOT_RST			BIT(12)
#define BAIKAL_PCIE_ADB_PWRDWN			BIT(13)

#define BAIKAL_LCRU_PCIE_STATUS_BASE		0x50004
#define BAIKAL_LCRU_PCIE_STATUS(x)		((x * 0x20) + BAIKAL_LCRU_PCIE_STATUS_BASE)
#define BAIKAL_PCIE_LTSSM_MASK			0x3f
#define BAIKAL_PCIE_LTSSM_STATE_L0		0x11

#define BAIKAL_LCRU_PCIE_GEN_CTL_BASE		0x50008
#define BAIKAL_LCRU_PCIE_GEN_CTL(x)		((x * 0x20) + BAIKAL_LCRU_PCIE_GEN_CTL_BASE)
#define BAIKAL_PCIE_LTSSM_ENABLE		BIT(1)
#define BAIKAL_PCIE_DBI2_MODE			BIT(2)
#define BAIKAL_PCIE_PHY_MGMT_ENABLE		BIT(3)

#define BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2		0x500f8
#define BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2_PCIE_MSI_TRANS_EN(x)	BIT(9 + (x))
#define BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2_PCIE_RCNUM_MASK(x)	((3) << (2 * (x)))

static int baikal_pcie_link_up(struct dw_pcie *pcie)
{
	struct baikal_pcie_rc *rc = to_baikal_pcie_rc(pcie);
	u32 reg;

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &reg);
	if (!(reg & BAIKAL_PCIE_LTSSM_ENABLE)) {
		return 0;
	}

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_STATUS(rc->bus_nr), &reg);
	return (reg & BAIKAL_PCIE_LTSSM_MASK) == BAIKAL_PCIE_LTSSM_STATE_L0;
}

static int baikal_pcie_host_init(struct pcie_port *pp)
{
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct baikal_pcie_rc *rc = to_baikal_pcie_rc(pcie);
	struct device *dev = pcie->dev;
	int err;
	int linkup;
	unsigned idx;
	u32 reg;

	/* Disable access to PHY registers and DBI2 mode */
	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &reg);
	reg &= ~(BAIKAL_PCIE_PHY_MGMT_ENABLE |
		 BAIKAL_PCIE_DBI2_MODE);
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	rc->retrained = false;
	linkup = baikal_pcie_link_up(pcie);

	/* If link is not established yet, reset the RC */
	if (!linkup) {
		/* Disable link training */
		regmap_read(rc->lcru,
				BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &reg);
		reg &= ~BAIKAL_PCIE_LTSSM_ENABLE;
		regmap_write(rc->lcru,
				BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

		/* Assert PERST pin */
		if (rc->reset_gpio != NULL) {
			unsigned long gpio_flags;

			if (rc->reset_active_low) {
				gpio_flags = GPIOF_ACTIVE_LOW |
					     GPIOF_OUT_INIT_LOW;
			} else {
				gpio_flags = GPIOF_OUT_INIT_HIGH;
			}

			err = devm_gpio_request_one(dev,
						   desc_to_gpio(rc->reset_gpio),
						   gpio_flags, rc->reset_name);
			if (err) {
				dev_err(dev, "request GPIO failed (%d)\n", err);
				return err;
			}
		}

		/* Reset the RC */
		regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), &reg);
		reg |= BAIKAL_PCIE_NONSTICKY_RST |
		       BAIKAL_PCIE_STICKY_RST	 |
		       BAIKAL_PCIE_PWR_RST	 |
		       BAIKAL_PCIE_CORE_RST	 |
		       BAIKAL_PCIE_PHY_RST;

		/* If the RC is PCIe x8, reset PIPE0 and PIPE1 */
		if (rc->bus_nr == 2) {
			reg |= BAIKAL_PCIE_PIPE0_RST |
			       BAIKAL_PCIE_PIPE1_RST;
		} else {
			reg |= BAIKAL_PCIE_PIPE_RST;
		}

		regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

		usleep_range(20000, 30000);

		if (rc->reset_gpio != NULL) {
			/* Deassert PERST pin */
			gpiod_set_value_cansleep(rc->reset_gpio, 0);
		}

		/* Deassert PHY reset */
		regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), &reg);
		reg &= ~BAIKAL_PCIE_PHY_RST;
		regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);

		/* Deassert all software controlled resets */
		regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), &reg);
		reg &= ~(BAIKAL_PCIE_ADB_PWRDWN	   |
			 BAIKAL_PCIE_HOT_RST	   |
			 BAIKAL_PCIE_NONSTICKY_RST |
			 BAIKAL_PCIE_STICKY_RST	   |
			 BAIKAL_PCIE_PWR_RST	   |
			 BAIKAL_PCIE_CORE_RST	   |
			 BAIKAL_PCIE_PHY_RST);

		if (rc->bus_nr == 2) {
			reg &= ~(BAIKAL_PCIE_PIPE0_RST |
				 BAIKAL_PCIE_PIPE1_RST);
		} else {
			reg &= ~BAIKAL_PCIE_PIPE_RST;
		}

		regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);
	}

	/* Deinitialise all iATU regions */
	for (idx = 0; idx < pcie->num_viewport; ++idx) {
		dw_pcie_writel_dbi(pcie, PCIE_IATU_VIEWPORT_REG,
					 PCIE_IATU_REGION_OUTBOUND | idx);
		dw_pcie_writel_dbi(pcie, PCIE_IATU_REGION_CTRL_2_REG, 0);
	}

	dw_pcie_setup_rc(pp);

	/* Set prog-if 01 [subtractive decode] */
	dw_pcie_dbi_ro_wr_en(pcie);
	reg = dw_pcie_readl_dbi(pcie, PCI_CLASS_REVISION);
	reg = (reg & 0xffff00ff) | (1 << 8);
	dw_pcie_writel_dbi(pcie, PCI_CLASS_REVISION, reg);
	dw_pcie_dbi_ro_wr_dis(pcie);

	/* Enable error reporting */
	reg = dw_pcie_readl_dbi(pcie, PCIE_ROOT_ERR_CMD_REG);
	reg |= PCIE_CORR_ERR_REPORTING_EN      |
	       PCIE_NON_FATAL_ERR_REPORTING_EN |
	       PCIE_FATAL_ERR_REPORTING_EN;
	dw_pcie_writel_dbi(pcie, PCIE_ROOT_ERR_CMD_REG, reg);

	reg = dw_pcie_readl_dbi(pcie, PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG);
	reg |= PCIE_CAP_CORR_ERR_REPORT_EN	|
	       PCIE_CAP_NON_FATAL_ERR_REPORT_EN	|
	       PCIE_CAP_FATAL_ERR_REPORT_EN	|
	       PCIE_CAP_UNSUPPORT_REQ_REP_EN;
	dw_pcie_writel_dbi(pcie, PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG, reg);

	reg = dw_pcie_readl_dbi(pcie, PCIE_ROOT_CONTROL_ROOT_CAPABILITIES_REG);
	reg |= PCIE_CAP_SYS_ERR_ON_CORR_ERR_EN	    |
	       PCIE_CAP_SYS_ERR_ON_NON_FATAL_ERR_EN |
	       PCIE_CAP_SYS_ERR_ON_FATAL_ERR_EN	    |
	       PCIE_CAP_PME_INT_EN;
	dw_pcie_writel_dbi(pcie, PCIE_ROOT_CONTROL_ROOT_CAPABILITIES_REG, reg);

	if (linkup) {
		dev_info(dev, "link is already up\n");
	} else {
		/* Use Gen1 mode for link establishing */
		reg = dw_pcie_readl_dbi(pcie,
					PCIE_LINK_CONTROL2_LINK_STATUS2_REG);
		reg &= ~PCIE_CAP_TARGET_LINK_SPEED_MASK;
		reg |= 1;
		dw_pcie_writel_dbi(pcie,
				   PCIE_LINK_CONTROL2_LINK_STATUS2_REG, reg);

		/*
		 * Clear DIRECT_SPEED_CHANGE bit. It has been set by
		 * dw_pcie_setup_rc(pp). This bit causes link retraining. But
		 * link retraining should be performed later by calling the
		 * baikal_pcie_link_speed_fixup().
		 */
		reg = dw_pcie_readl_dbi(pcie, PCIE_GEN2_CTRL_REG);
		reg &= ~PCIE_DIRECT_SPEED_CHANGE;
		dw_pcie_writel_dbi(pcie, PCIE_GEN2_CTRL_REG, reg);

		/* Establish link */
		regmap_read(rc->lcru,
				BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), &reg);
		reg |= BAIKAL_PCIE_LTSSM_ENABLE;
		regmap_write(rc->lcru,
				BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

		dw_pcie_wait_for_link(pcie);
	}

	regmap_read(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2, &reg);
	reg &= ~BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2_PCIE_RCNUM_MASK(rc->bus_nr);
	reg |= BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2_PCIE_MSI_TRANS_EN(rc->bus_nr);
	regmap_write(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2, reg);

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		dw_pcie_msi_init(pp);
	}

	return 0;
}

static const struct dw_pcie_host_ops baikal_pcie_host_ops = {
	.host_init = baikal_pcie_host_init
};

static void baikal_pcie_link_print_status(struct baikal_pcie_rc *rc)
{
	struct dw_pcie *pcie = rc->pcie;
	struct device *dev = pcie->dev;
	u32 reg;
	unsigned speed, width;

	if (!baikal_pcie_link_up(pcie)) {
		dev_info(dev, "link is down\n");
		return;
	}

	reg = dw_pcie_readl_dbi(pcie, PCIE_LINK_CONTROL_LINK_STATUS_REG);
	speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >> PCIE_CAP_LINK_SPEED_SHIFT;
	width = (reg & PCIE_CAP_NEGO_LINK_WIDTH_MASK) >>
		PCIE_CAP_NEGO_LINK_WIDTH_SHIFT;

	dev_info(dev, "link status is Gen%u%s, x%u\n", speed,
		 reg & PCIE_CAP_LINK_TRAINING ? " (training)" : "", width);
}

static unsigned baikal_pcie_link_is_training(struct baikal_pcie_rc *rc)
{
	struct dw_pcie *pcie = rc->pcie;
	return dw_pcie_readl_dbi(pcie, PCIE_LINK_CONTROL_LINK_STATUS_REG) &
	       PCIE_CAP_LINK_TRAINING;
}

static bool baikal_pcie_link_wait_training_done(struct baikal_pcie_rc *rc)
{
	struct dw_pcie *pcie = rc->pcie;
	struct device *dev = pcie->dev;
	unsigned long start_jiffies = jiffies;

	while (baikal_pcie_link_is_training(rc)) {
		if (time_after(jiffies, start_jiffies + HZ)) {
			dev_err(dev, "link training timeout occured\n");
			return false;
		}
		udelay(100);
	}
	return true;
}

static void baikal_pcie_link_retrain(struct baikal_pcie_rc *rc,
				     int target_speed)
{
	struct dw_pcie *pcie = rc->pcie;
	struct device *dev = pcie->dev;
	u32 reg;
	unsigned long start_jiffies;

	dev_info(dev, "retrain link to Gen%u\n", target_speed);

	/* In case link is already training wait for training to complete */
	baikal_pcie_link_wait_training_done(rc);

	/* Set desired speed */
	reg = dw_pcie_readl_dbi(pcie, PCIE_LINK_CONTROL2_LINK_STATUS2_REG);
	reg &= ~PCIE_CAP_TARGET_LINK_SPEED_MASK;
	reg |= target_speed;
	dw_pcie_writel_dbi(pcie, PCIE_LINK_CONTROL2_LINK_STATUS2_REG, reg);

	/* Deassert and assert DIRECT_SPEED_CHANGE bit */
	reg = dw_pcie_readl_dbi(pcie, PCIE_GEN2_CTRL_REG);
	reg &= ~PCIE_DIRECT_SPEED_CHANGE;
	dw_pcie_writel_dbi(pcie, PCIE_GEN2_CTRL_REG, reg);
	reg |= PCIE_DIRECT_SPEED_CHANGE;
	dw_pcie_writel_dbi(pcie, PCIE_GEN2_CTRL_REG, reg);

	/* Wait for link training begin */
	start_jiffies = jiffies;
	while (!baikal_pcie_link_is_training(rc)) {
		if (time_after(jiffies, start_jiffies + HZ)) {
			dev_err(dev, "link training has not started\n");
			/* Don't wait for training_done() if it hasn't started */
			return;
		}
		udelay(100);
	}

	/* Wait for link training end */
	if (!baikal_pcie_link_wait_training_done(rc)) {
		return;
	}

	if (!dw_pcie_wait_for_link(pcie)) {
		/* Wait if link has switched to configuration/recovery state */
		baikal_pcie_link_wait_training_done(rc);
		baikal_pcie_link_print_status(rc);
	}
}

static void baikal_pcie_link_speed_fixup(struct pci_dev *pdev)
{
	struct pcie_port *pp = pdev->bus->sysdata;
	struct dw_pcie *pcie = to_dw_pcie_from_pp(pp);
	struct baikal_pcie_rc *rc = to_baikal_pcie_rc(pcie);
	unsigned dev_lnkcap_speed;
	unsigned dev_lnkcap_width;
	unsigned rc_lnkcap_speed;
	unsigned rc_lnksta_speed;
	unsigned rc_target_speed;
	u32 reg;

	/* Skip Root Bridge */
	if (!pdev->bus->self) {
		return;
	}

	/* Skip any devices not directly connected to the RC */
	if (pdev->bus->self->bus->number != pp->root_bus_nr) {
		return;
	}

	/* Skip if the bus has already been retrained */
	if (rc->retrained) {
		return;
	}

	reg = dw_pcie_readl_dbi(pcie, PCIE_LINK_CAPABILITIES_REG);
	rc_lnkcap_speed = reg & PCI_EXP_LNKCAP_SLS;

	reg = dw_pcie_readl_dbi(pcie, PCIE_LINK_CONTROL_LINK_STATUS_REG);
	rc_lnksta_speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >>
			  PCIE_CAP_LINK_SPEED_SHIFT;

	pcie_capability_read_dword(pdev, PCI_EXP_LNKCAP, &reg);
	dev_lnkcap_speed = (reg & PCI_EXP_LNKCAP_SLS);
	dev_lnkcap_width = (reg & PCI_EXP_LNKCAP_MLW) >>
			   PCI_EXP_LNKSTA_NLW_SHIFT;

	baikal_pcie_link_print_status(rc);
	dev_info(&pdev->dev, "device link capability is Gen%u, x%u\n",
		 dev_lnkcap_speed, dev_lnkcap_width);

	/*
	 * Gen1->Gen3 is suitable way of retraining.
	 * Gen1->Gen2 is used when Gen3 could not be reached.
	 * Gen2->Gen3 causes system freezing sometimes.
	 */
	if (rc_lnkcap_speed < dev_lnkcap_speed) {
		rc_target_speed = rc_lnkcap_speed;
	} else {
		rc_target_speed = dev_lnkcap_speed;
	}

	while (rc_lnksta_speed < rc_target_speed) {
		/* Try to change link speed */
		baikal_pcie_link_retrain(rc, rc_target_speed);

		/* Check if the link is down after retrain */
		if (!baikal_pcie_link_up(pcie)) {
			/*
			 * Check if the link has already been down and
			 * the link is not re-established at Gen1.
			 */
			if (rc_lnksta_speed == 0 && rc_target_speed == 1) {
				/* Unable to re-establish the link */
				break;
			}

			rc_lnksta_speed = 0;
			if (rc_target_speed > 1) {
				/* Try to use lower speed */
				--rc_target_speed;
			}

			continue;
		}

		reg = dw_pcie_readl_dbi(pcie,
					PCIE_LINK_CONTROL_LINK_STATUS_REG);
		rc_lnksta_speed = (reg & PCIE_CAP_LINK_SPEED_MASK) >>
				  PCIE_CAP_LINK_SPEED_SHIFT;
		/* Check if the targeted speed has not been reached */
		if (rc_lnksta_speed < rc_target_speed && rc_target_speed > 1) {
			/* Try to use lower speed */
			--rc_target_speed;
		}
	}

	rc->retrained = true;
}

static void baikal_pcie_link_retrain_bus(const struct pci_bus *bus)
{
	struct pci_dev *dev;

	list_for_each_entry(dev, &bus->devices, bus_list) {
		baikal_pcie_link_speed_fixup(dev);
	}

	list_for_each_entry(dev, &bus->devices, bus_list) {
		if (dev->subordinate) {
			baikal_pcie_link_retrain_bus(dev->subordinate);
		}
	}
}

static irqreturn_t baikal_pcie_err_irq_handler(int irq, void *priv)
{
	struct baikal_pcie_rc *rc = priv;
	struct dw_pcie *pcie = rc->pcie;
	struct device *dev = pcie->dev;
	u32 corr_err_status;
	u32 dev_ctrl_dev_status;
	u32 root_err_status;
	u32 uncorr_err_status;

	uncorr_err_status   = dw_pcie_readl_dbi(pcie,
					 PCIE_UNCORR_ERR_STATUS_REG);
	corr_err_status	    = dw_pcie_readl_dbi(pcie,
					 PCIE_CORR_ERR_STATUS_REG);
	root_err_status     = dw_pcie_readl_dbi(pcie,
					 PCIE_ROOT_ERR_STATUS_REG);
	dev_ctrl_dev_status = dw_pcie_readl_dbi(pcie,
					 PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG);
	dev_err(dev,
		"dev_err:0x%x root_err:0x%x uncorr_err:0x%x corr_err:0x%x\n",
		(dev_ctrl_dev_status & 0xf0000) >> 16,
		root_err_status, uncorr_err_status, corr_err_status);

	dw_pcie_writel_dbi(pcie,
		    PCIE_UNCORR_ERR_STATUS_REG, uncorr_err_status);
	dw_pcie_writel_dbi(pcie,
		    PCIE_CORR_ERR_STATUS_REG, corr_err_status);
	dw_pcie_writel_dbi(pcie,
		    PCIE_ROOT_ERR_STATUS_REG, root_err_status);
	dw_pcie_writel_dbi(pcie,
		    PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG, dev_ctrl_dev_status);

	return IRQ_HANDLED;
}

static int baikal_pcie_add_pcie_port(struct baikal_pcie_rc *rc,
				     struct platform_device *pdev)
{
	struct dw_pcie *pcie = rc->pcie;
	struct pcie_port *pp = &pcie->pp;
	struct device *dev = &pdev->dev;
	int ret;

	pp->irq = platform_get_irq(pdev, 0);
	if (pp->irq < 0) {
		return pp->irq;
	}

	ret = devm_request_irq(dev, pp->irq, baikal_pcie_err_irq_handler,
			       IRQF_SHARED, "baikal-pcie-error-irq", pcie);

	if (ret) {
		dev_err(dev, "failed to request irq %d\n", pp->irq);
		return ret;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		pp->msi_irq = platform_get_irq(pdev, 1);
		if (pp->msi_irq < 0) {
			return pp->msi_irq;
		}
	}

	pp->ops = &baikal_pcie_host_ops;

	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}

	baikal_pcie_link_retrain_bus(pp->root_bus);
	return 0;
}

#ifdef CONFIG_PM_SLEEP
static int baikal_pcie_pm_resume(struct device *dev)
{
	struct baikal_pcie_rc *rc = dev_get_drvdata(dev);
	struct dw_pcie *pcie = rc->pcie;
	u32 reg;

	/* Set Memory Space Enable (MSE) bit */
	reg = dw_pcie_readl_dbi(pcie, PCI_COMMAND);
	reg |= PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pcie, PCI_COMMAND, reg);
	return 0;
}

static int baikal_pcie_pm_resume_noirq(struct device *dev)
{
	return 0;
}

static int baikal_pcie_pm_suspend(struct device *dev)
{
	struct baikal_pcie_rc *rc = dev_get_drvdata(dev);
	struct dw_pcie *pcie = rc->pcie;
	u32 reg;

	/* Clear Memory Space Enable (MSE) bit */
	reg = dw_pcie_readl_dbi(pcie, PCI_COMMAND);
	reg &= ~PCI_COMMAND_MEMORY;
	dw_pcie_writel_dbi(pcie, PCI_COMMAND, reg);
	return 0;
}

static int baikal_pcie_pm_suspend_noirq(struct device *dev)
{
	return 0;
}
#endif

static const struct dev_pm_ops baikal_pcie_pm_ops = {
	SET_SYSTEM_SLEEP_PM_OPS(baikal_pcie_pm_suspend,
				baikal_pcie_pm_resume)
	SET_NOIRQ_SYSTEM_SLEEP_PM_OPS(baikal_pcie_pm_suspend_noirq,
				      baikal_pcie_pm_resume_noirq)
};

static const struct of_device_id of_baikal_pcie_match[] = {
	{ .compatible = "baikal,bm1000-pcie" },

	/*
	 * TODO: it is pretty common to use "vendor,chip-*" prefixes
	 *	 to all SoC specific devices. So "pcie-m" is legacy.
	 */
	{ .compatible = "baikal,pcie-m" },
	{}
};
MODULE_DEVICE_TABLE(of, of_baikal_pcie_match);

static const struct dw_pcie_ops baikal_pcie_ops = {
	.link_up = baikal_pcie_link_up
};

static int baikal_pcie_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_pcie_rc *rc;
	struct dw_pcie *pcie;
	int ret;
	u32 idx[2];
	enum of_gpio_flags gpio_flags;
	int reset_gpio;
	struct resource *res;

	if (!of_match_device(of_baikal_pcie_match, dev)) {
		dev_err(dev, "device can't be handled by pcie-baikal\n");
		return -EINVAL;
	}

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		return -ENOMEM;
	}

	pcie->dev = dev;
	pcie->ops = &baikal_pcie_ops;

	rc = devm_kzalloc(dev, sizeof(*rc), GFP_KERNEL);
	if (!rc) {
		return -ENOMEM;
	}

	rc->pcie = pcie;
	rc->lcru = syscon_regmap_lookup_by_phandle(dev->of_node,
						   "baikal,pcie-lcru");
	if (IS_ERR(rc->lcru)) {
		dev_err(dev, "No LCRU phandle specified\n");
		rc->lcru = NULL;
		return -EINVAL;
	}

	if (of_property_read_u32_array(dev->of_node, "baikal,pcie-lcru", idx, 2)) {
		dev_err(dev, "failed to read LCRU\n");
		rc->lcru = NULL;
		return -EINVAL;
	}

	if (idx[1] > 2) {
		dev_err(dev, "incorrect pcie-lcru index\n");
		rc->lcru = NULL;
		return -EINVAL;
	}

	rc->bus_nr = idx[1];
	reset_gpio = of_get_named_gpio_flags(dev->of_node, "reset-gpios", 0,
					     &gpio_flags);
	if (gpio_is_valid(reset_gpio)) {
		rc->reset_gpio = gpio_to_desc(reset_gpio);
		rc->reset_active_low = !!(gpio_flags & OF_GPIO_ACTIVE_LOW);
		snprintf(rc->reset_name, sizeof(rc->reset_name), "pcie%u-reset",
			 rc->bus_nr);
	} else {
		rc->reset_gpio = NULL;
	}

	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_pm_disable;
	}

	platform_set_drvdata(pdev, rc);
	res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
	if (res) {
		devm_request_resource(dev, &iomem_resource, res);
		pcie->dbi_base = devm_ioremap_resource(dev, res);
		if (IS_ERR(pcie->dbi_base)) {
			dev_err(dev, "error with ioremap\n");
			ret = PTR_ERR(pcie->dbi_base);
			goto err_pm_put;
		}
	} else {
		dev_err(dev, "missing *dbi* reg space\n");
		ret = -EINVAL;
		goto err_pm_put;
	}

	ret = baikal_pcie_add_pcie_port(rc, pdev);
	if (ret < 0) {
		goto err_pm_put;
	}

	return 0;

err_pm_put:
	pm_runtime_put(dev);
err_pm_disable:
	pm_runtime_disable(dev);
	return ret;
}

static struct platform_driver baikal_pcie_driver = {
	.driver = {
		.name = "baikal-pcie",
		.of_match_table = of_baikal_pcie_match,
		.suppress_bind_attrs = true,
		.pm = &baikal_pcie_pm_ops
	},
	.probe = baikal_pcie_probe
};

module_platform_driver(baikal_pcie_driver);

#ifdef CONFIG_ACPI
static int baikal_pcie_get_res_acpi(struct acpi_device *adev, struct pcie_port *pp)
{
	struct device *dev = &adev->dev;
	struct resource_entry *entry;
	struct list_head list, *pos;
	int ret;
	unsigned long flags = IORESOURCE_MEM;

	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *) flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 3) {
		dev_err(dev, "invalid number of MEM resources present in _CRS (%i, need 3)\n", ret);
		return -EINVAL;
	}

	/* ECAM (CONFIG) */
	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->cfg0_size = resource_size(entry->res);
	pp->cfg1_size = pp->cfg0_size;
	pp->cfg0_base = entry->res->start;
	pp->cfg1_base = pp->cfg0_base;

	/* DBI */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	to_dw_pcie_from_pp(pp)->dbi_base = devm_ioremap_resource(dev, entry->res);
	if (IS_ERR(to_dw_pcie_from_pp(pp)->dbi_base)) {
		dev_err(dev, "error with dbi ioremap\n");
		ret = PTR_ERR(to_dw_pcie_from_pp(pp)->dbi_base);
		return ret;
	}

	/* 32bit non-prefetchable memory */
	pos = pos->next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->mem_base = entry->res->start;
	pp->mem_size = resource_size(entry->res);
	pp->mem_bus_addr = entry->res->start - entry->offset;

	acpi_dev_free_resource_list(&list);

	/* I/O */
	INIT_LIST_HEAD(&list);
	flags = IORESOURCE_IO;
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *) flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return ret;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of IO resources present in _CRS (%i, need 1)\n", ret);
		return -EINVAL;
	}

	pos = list.next;
	entry = list_entry(pos, struct resource_entry, node);
	pp->io_base = entry->res->start;
	pp->io_size = resource_size(entry->res);
	pp->io_bus_addr = entry->res->start - entry->offset;

	acpi_dev_free_resource_list(&list);
	return 0;
}

static int baikal_pcie_get_irq_acpi(struct acpi_device *adev, struct pcie_port *pp)
{
	struct device *dev = &adev->dev;
	struct resource res;
	int ret;

	memset(&res, 0, sizeof(res));

	ret = acpi_irq_get(adev->handle, 0, &res);
	if (ret) {
		dev_err(dev, "failed to get irq %d\n", 0);
		return ret;
	}

	if (res.flags & IORESOURCE_BITS) {
		struct irq_data *irqd;

		irqd = irq_get_irq_data(res.start);
		if (!irqd)
			return -ENXIO;
		irqd_set_trigger_type(irqd, res.flags & IORESOURCE_BITS);
	}

	pp->irq = res.start;

	ret = devm_request_irq(dev, pp->irq, baikal_pcie_err_irq_handler,
			       IRQF_SHARED, "baikal-pcie-error-irq", to_dw_pcie_from_pp(pp));
	if (ret) {
		dev_err(dev, "failed to request irq %d\n", pp->irq);
		return ret;
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		memset(&res, 0, sizeof(res));

		ret = acpi_irq_get(adev->handle, 1, &res);
		if (ret) {
			dev_err(dev, "failed to get irq %d\n", 1);
			return ret;
		}

		if (res.flags & IORESOURCE_BITS) {
			struct irq_data *irqd;

			irqd = irq_get_irq_data(res.start);
			if (!irqd)
				return -ENXIO;
			irqd_set_trigger_type(irqd, res.flags & IORESOURCE_BITS);
		}

		pp->msi_irq = res.start;
	}

	return 0;
}

static struct acpi_device *baikal_lcru;
static struct regmap *baikal_regmap;

static const struct regmap_config baikal_pcie_syscon_regmap_config = {
	.reg_bits = 32,
	.val_bits = 32,
	.reg_stride = 4
};

static struct regmap *baikal_get_regmap(struct acpi_device *adev)
{
	struct device *dev = &adev->dev;
	struct list_head list, *pos;
	struct resource *res;
	void __iomem *base;
	int ret;
	struct regmap *regmap = NULL;
	struct regmap_config config = baikal_pcie_syscon_regmap_config;
	unsigned long flags = IORESOURCE_MEM;

	INIT_LIST_HEAD(&list);
	ret = acpi_dev_get_resources(adev, &list,
				     acpi_dev_filter_resource_type_cb,
				     (void *) flags);
	if (ret < 0) {
		dev_err(dev, "failed to parse _CRS method, error code %d\n", ret);
		return NULL;
	}

	if (ret != 1) {
		dev_err(dev, "invalid number of MEM resources present in _CRS (%i, need 1)\n", ret);
		goto ret;
	}

	pos = list.next;
	res = list_entry(pos, struct resource_entry, node)->res;

	base = devm_ioremap(dev, res->start, resource_size(res));
	if (!base) {
		dev_err(dev, "error with ioremap\n");
		goto ret;
	}

	config.max_register = resource_size(res) - 4;

	regmap = devm_regmap_init_mmio(dev, base, &config);
	if (IS_ERR(regmap)) {
		dev_err(dev, "regmap init failed\n");
		devm_iounmap(dev, base);
		goto ret;
	}

	dev_dbg(dev, "regmap %pR registered\n", res);

	baikal_lcru = adev;
	baikal_regmap = regmap;

ret:
	acpi_dev_free_resource_list(&list);
	return regmap;
}

static struct regmap *baikal_pcie_get_lcru_acpi(struct acpi_device *adev, struct baikal_pcie_rc *rc)
{
	struct device *dev = &adev->dev;
	struct acpi_device *lcru;
	struct regmap *regmap = NULL;
	union acpi_object *package = NULL;
	union acpi_object *element = NULL;
	struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	acpi_status status = AE_OK;
	int ret;

	status = acpi_evaluate_object_typed(adev->handle, "LCRU", NULL, &buffer, ACPI_TYPE_PACKAGE);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to get LCRU data\n");
		return ERR_PTR(-ENODEV);
	}

	package = buffer.pointer;

	if (package->package.count != 2) {
		dev_err(dev, "invalid LCRU data\n");
		goto ret;
	}

	element = &(package->package.elements[0]);

	if (element->type != ACPI_TYPE_LOCAL_REFERENCE || !element->reference.handle) {
		dev_err(dev, "invalid LCRU reference\n");
		goto ret;
	}

	ret = acpi_bus_get_device(element->reference.handle, &lcru);
	if (ret) {
		dev_err(dev, "failed to process LCRU reference\n");
		goto ret;
	}

	element = &(package->package.elements[1]);

	if (element->type != ACPI_TYPE_INTEGER) {
		dev_err(dev, "failed to get LCRU index\n");
		goto ret;
	}

	rc->bus_nr = element->integer.value;
	if (baikal_regmap && lcru == baikal_lcru) {
		regmap = baikal_regmap;
	} else {
		regmap = baikal_get_regmap(lcru);
	}

ret:
	acpi_os_free(buffer.pointer);
	return regmap;
}

static void baikal_dw_msi_ack_irq(struct irq_data *d)
{
	irq_chip_ack_parent(d);
}

static void baikal_dw_msi_mask_irq(struct irq_data *d)
{
	pci_msi_mask_irq(d);
	irq_chip_mask_parent(d);
}

static void baikal_dw_msi_unmask_irq(struct irq_data *d)
{
	pci_msi_unmask_irq(d);
	irq_chip_unmask_parent(d);
}

static struct irq_chip baikal_dw_pcie_msi_irq_chip = {
	.name = "PCI-MSI",
	.irq_ack = baikal_dw_msi_ack_irq,
	.irq_mask = baikal_dw_msi_mask_irq,
	.irq_unmask = baikal_dw_msi_unmask_irq
};

static struct msi_domain_info baikal_dw_pcie_msi_domain_info = {
	.flags	= (MSI_FLAG_USE_DEF_DOM_OPS | MSI_FLAG_USE_DEF_CHIP_OPS |
		   MSI_FLAG_PCI_MSIX | MSI_FLAG_MULTI_PCI_MSI),
	.chip	= &baikal_dw_pcie_msi_irq_chip
};

static irqreturn_t baikal_dw_handle_msi_irq(struct pcie_port *pp)
{
	int i, pos, irq;
	unsigned long val;
	u32 status, num_ctrls;
	irqreturn_t ret = IRQ_NONE;

	num_ctrls = pp->num_vectors / MAX_MSI_IRQS_PER_CTRL;

	for (i = 0; i < num_ctrls; i++) {
		dw_pcie_read(to_dw_pcie_from_pp(pp)->dbi_base +
				PCIE_MSI_INTR0_STATUS +
				(i * MSI_REG_CTRL_BLOCK_SIZE),
			     4, &status);
		if (!status) {
			continue;
		}

		ret = IRQ_HANDLED;
		val = status;
		pos = 0;
		while ((pos = find_next_bit(&val, MAX_MSI_IRQS_PER_CTRL,
					    pos)) != MAX_MSI_IRQS_PER_CTRL) {
			irq = irq_find_mapping(pp->irq_domain,
					       (i * MAX_MSI_IRQS_PER_CTRL) +
					       pos);
			generic_handle_irq(irq);
			pos++;
		}
	}

	return ret;
}

static void baikal_dw_chained_msi_isr(struct irq_desc *desc)
{
	struct irq_chip *chip = irq_desc_get_chip(desc);
	struct pcie_port *pp;

	chained_irq_enter(chip, desc);

	pp = irq_desc_get_handler_data(desc);
	baikal_dw_handle_msi_irq(pp);

	chained_irq_exit(chip, desc);
}

static void baikal_dw_pci_setup_msi_msg(struct irq_data *d, struct msi_msg *msg)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	u64 msi_target;

	msi_target = (u64)pp->msi_data;

	msg->address_lo = lower_32_bits(msi_target);
	msg->address_hi = upper_32_bits(msi_target);

	msg->data = d->hwirq;

	dev_dbg(pci->dev, "msi#%d address_hi %#x address_lo %#x\n",
		(int)d->hwirq, msg->address_hi, msg->address_lo);
}

static int baikal_dw_pci_msi_set_affinity(struct irq_data *d,
					  const struct cpumask *mask,
					  bool force)
{
	return -EINVAL;
}

static void baikal_dw_pci_bottom_mask(struct irq_data *d)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	pp->irq_mask[ctrl] |= BIT(bit);
	dw_pcie_write(to_dw_pcie_from_pp(pp)->dbi_base + PCIE_MSI_INTR0_MASK + res,
		      4, pp->irq_mask[ctrl]);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void baikal_dw_pci_bottom_unmask(struct irq_data *d)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	pp->irq_mask[ctrl] &= ~BIT(bit);
	dw_pcie_write(to_dw_pcie_from_pp(pp)->dbi_base + PCIE_MSI_INTR0_MASK + res,
		      4, pp->irq_mask[ctrl]);

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static void baikal_dw_pci_bottom_ack(struct irq_data *d)
{
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned int res, bit, ctrl;

	ctrl = d->hwirq / MAX_MSI_IRQS_PER_CTRL;
	res = ctrl * MSI_REG_CTRL_BLOCK_SIZE;
	bit = d->hwirq % MAX_MSI_IRQS_PER_CTRL;

	dw_pcie_write(to_dw_pcie_from_pp(pp)->dbi_base + PCIE_MSI_INTR0_STATUS + res,
		      4, BIT(bit));
}

static struct irq_chip baikal_dw_pci_msi_bottom_irq_chip = {
	.name = "BAIKALPCI-MSI",
	.irq_ack = baikal_dw_pci_bottom_ack,
	.irq_compose_msi_msg = baikal_dw_pci_setup_msi_msg,
	.irq_set_affinity = baikal_dw_pci_msi_set_affinity,
	.irq_mask = baikal_dw_pci_bottom_mask,
	.irq_unmask = baikal_dw_pci_bottom_unmask
};

static int baikal_dw_pcie_irq_domain_alloc(struct irq_domain *domain,
					   unsigned int virq, unsigned int nr_irqs,
					   void *args)
{
	struct pcie_port *pp = domain->host_data;
	unsigned long flags;
	u32 i;
	int bit;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bit = bitmap_find_free_region(pp->msi_irq_in_use, pp->num_vectors,
				      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);

	if (bit < 0) {
		return -ENOSPC;
	}

	for (i = 0; i < nr_irqs; i++) {
		irq_domain_set_info(domain, virq + i, bit + i,
				    pp->msi_irq_chip,
				    pp, handle_edge_irq,
				    NULL, NULL);
	}

	return 0;
}

static void baikal_dw_pcie_irq_domain_free(struct irq_domain *domain,
					   unsigned int virq, unsigned int nr_irqs)
{
	struct irq_data *d = irq_domain_get_irq_data(domain, virq);
	struct pcie_port *pp = irq_data_get_irq_chip_data(d);
	unsigned long flags;

	raw_spin_lock_irqsave(&pp->lock, flags);

	bitmap_release_region(pp->msi_irq_in_use, d->hwirq,
			      order_base_2(nr_irqs));

	raw_spin_unlock_irqrestore(&pp->lock, flags);
}

static const struct irq_domain_ops baikal_dw_pcie_msi_domain_ops = {
	.alloc	= baikal_dw_pcie_irq_domain_alloc,
	.free	= baikal_dw_pcie_irq_domain_free
};

static int baikal_dw_pcie_allocate_domains(struct pcie_port *pp)
{
	struct dw_pcie *pci = to_dw_pcie_from_pp(pp);
	struct fwnode_handle *fwnode = pci->dev->fwnode;

	pp->irq_domain = irq_domain_create_linear(fwnode, pp->num_vectors,
						  &baikal_dw_pcie_msi_domain_ops, pp);
	if (!pp->irq_domain) {
		dev_err(pci->dev, "Failed to create IRQ domain\n");
		return -ENOMEM;
	}

	irq_domain_update_bus_token(pp->irq_domain, DOMAIN_BUS_NEXUS);

	pp->msi_domain = pci_msi_create_irq_domain(fwnode,
						   &baikal_dw_pcie_msi_domain_info,
						   pp->irq_domain);
	if (!pp->msi_domain) {
		dev_err(pci->dev, "Failed to create MSI domain\n");
		irq_domain_remove(pp->irq_domain);
		return -ENOMEM;
	}

	return 0;
}

static void baikal_dw_pcie_free_msi(struct pcie_port *pp)
{
	if (pp->msi_irq) {
		irq_set_chained_handler(pp->msi_irq, NULL);
		irq_set_handler_data(pp->msi_irq, NULL);
	}

	irq_domain_remove(pp->msi_domain);
	irq_domain_remove(pp->irq_domain);

	if (pp->msi_page) {
		__free_page(pp->msi_page);
	}
}

static int baikal_pcie_msi_init(struct pcie_port *pp)
{
	int ret;

	if (pci_msi_enabled()) {
		pp->num_vectors = MSI_DEF_NUM_VECTORS;

		pp->msi_irq_chip = &baikal_dw_pci_msi_bottom_irq_chip;

		ret = baikal_dw_pcie_allocate_domains(pp);
		if (ret) {
			return ret;
		}

		if (pp->msi_irq) {
			irq_set_chained_handler_and_data(pp->msi_irq,
							 baikal_dw_chained_msi_isr,
							 pp);
		}
	}

	return 0;
}

static int baikal_pcie_init(struct pci_config_window *cfg)
{
	struct device *dev = cfg->parent;
	struct acpi_device *adev = to_acpi_device(dev);
	struct baikal_pcie_rc *rc;
	struct dw_pcie *pcie;
	struct pcie_port *pp;
	acpi_status status = AE_OK;
	u64 num_viewport;
	int ret;

	pcie = devm_kzalloc(dev, sizeof(*pcie), GFP_KERNEL);
	if (!pcie) {
		return -ENOMEM;
	}

	pcie->dev = dev;
	pcie->ops = &baikal_pcie_ops;

	rc = devm_kzalloc(dev, sizeof(*rc), GFP_KERNEL);
	if (!rc) {
		return -ENOMEM;
	}

	rc->pcie = pcie;
	cfg->priv = rc;
	pp = &pcie->pp;
	dev_set_drvdata(dev, rc);

	rc->lcru = baikal_pcie_get_lcru_acpi(adev, rc);

	if (IS_ERR_OR_NULL(rc->lcru)) {
		dev_err(dev, "No LCRU specified\n");
		return -EINVAL;
	}

	if (rc->bus_nr > 2) {
		dev_err(dev, "incorrect LCRU index\n");
		return -EINVAL;
        }

	/* TODO: gpio */

	ret = baikal_pcie_get_res_acpi(adev, pp);
	if (ret) {
		dev_err(dev, "failed to get resource info\n");
		return ret;
	}

	ret = baikal_pcie_get_irq_acpi(adev, pp);
	if (ret) {
		dev_err(dev, "failed to get irq info\n");
		return ret;
	}

	pp->ops = &baikal_pcie_host_ops;
	pp->root_bus_nr = cfg->busr.start;
	pp->busn = &cfg->busr;

	raw_spin_lock_init(&pp->lock);
	pp->va_cfg0_base = devm_pci_remap_cfgspace(dev, pp->cfg0_base, pp->cfg0_size);
	if (!pp->va_cfg0_base) {
		dev_err(dev, "error with ioremap\n");
		return -ENOMEM;
	}
	pp->va_cfg1_base = pp->va_cfg0_base;

	status = acpi_evaluate_integer(adev->handle, "NUMV", NULL, &num_viewport);
	if (ACPI_FAILURE(status)) {
		dev_err(dev, "failed to get num-viewport\n");
		return -EINVAL;
	}
	pcie->num_viewport = num_viewport;

	ret = baikal_pcie_msi_init(pp);
	if (ret) {
		dev_err(dev, "failed to init MSI\n");
		return ret;
	}

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "failed to enable DMA\n");
		return ret;
	}

	ret = baikal_pcie_host_init(pp);

	if (ret && pci_msi_enabled()) {
		baikal_dw_pcie_free_msi(pp);
	}

	return ret;
}

static void __iomem *baikal_pci_ecam_map_bus(struct pci_bus *bus, unsigned int devfn,
				     int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct baikal_pcie_rc *priv = cfg->priv;
	u64 cpu_addr = priv->pcie->pp.cfg0_base;
	unsigned int devfn_shift = cfg->ops->bus_shift - 8;
	unsigned int busn = bus->number;
	void __iomem *base;
	int type;
	u32 retries, val;

	if (bus->parent->number == priv->pcie->pp.root_bus_nr) {
		type = PCIE_ATU_TYPE_CFG0;
	} else {
		type = PCIE_ATU_TYPE_CFG1;
	}

	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_VIEWPORT, PCIE_ATU_REGION_OUTBOUND | PCIE_ATU_REGION_INDEX1);
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_LOWER_BASE, lower_32_bits(cpu_addr));
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_UPPER_BASE, upper_32_bits(cpu_addr));
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_LIMIT, lower_32_bits(cpu_addr + resource_size(&cfg->res) - 1));
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_LOWER_TARGET, 0);
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_UPPER_TARGET, 0);
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_CR1, type);
	dw_pcie_writel_dbi(priv->pcie, PCIE_ATU_CR2, PCIE_ATU_ENABLE | (0x1 << 28));

	/*
	 * Make sure ATU enable takes effect before any subsequent config
	 * and I/O accesses.
	 */
	for (retries = 0; retries < LINK_WAIT_MAX_IATU_RETRIES; retries++) {
		val = dw_pcie_readl_dbi(priv->pcie, PCIE_ATU_CR2);
		if (val & PCIE_ATU_ENABLE) {
			break;
		}

		mdelay(LINK_WAIT_IATU);
	}

	if (!(val & PCIE_ATU_ENABLE)) {
		dev_err(priv->pcie->dev, "Outbound iATU is not being enabled\n");
	}

	if (busn < cfg->busr.start || busn > cfg->busr.end) {
		return NULL;
	}

	busn -= cfg->busr.start;
	base = cfg->win + (busn << cfg->ops->bus_shift);
	return base + (devfn << devfn_shift) + where;
}

static void __iomem *baikal_pcie_map_bus(struct pci_bus *bus, unsigned int devfn,
					 int where)
{
	struct pci_config_window *cfg = bus->sysdata;
	struct baikal_pcie_rc *priv = cfg->priv;

	if (priv->pcie->pp.root_bus == NULL) {
		priv->pcie->pp.root_bus = bus;
		baikal_pcie_link_retrain_bus(priv->pcie->pp.root_bus);
	}

	if (bus->number != cfg->busr.start && !baikal_pcie_link_up(priv->pcie)) {
		return NULL;
	}

	if (bus->number == cfg->busr.start) {
		/*
		 * The DW PCIe core doesn't filter out transactions to other
		 * devices/functions on the root bus num, so we do this here.
		 */
		if (PCI_SLOT(devfn) > 0) {
			return NULL;
		} else {
			return priv->pcie->dbi_base + where;
		}
	}

	return baikal_pci_ecam_map_bus(bus, devfn, where);
}

struct pci_ecam_ops baikal_pcie_ecam_ops = {
	.bus_shift	= 20,
	.init		= baikal_pcie_init,
	.pci_ops	= {
		.map_bus	= baikal_pcie_map_bus,
		.read		= pci_generic_config_read,
		.write		= pci_generic_config_write
	}
};
#endif

MODULE_DESCRIPTION("Baikal PCIe host controller driver");
MODULE_LICENSE("GPL v2");
