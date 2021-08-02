// SPDX-License-Identifier: GPL-2.0
/*
 * PCIe RC driver for Baikal SoC
 *
 * Copyright (C) 2019-2021 Baikal Electronics, JSC
 * Authors: Pavel Parkhomenko <pavel.parkhomenko@baikalelectronics.ru>
 *          Mikhail Ivanov <michail.ivanov@baikalelectronics.ru>
 */

#include <linux/err.h>
#include <linux/interrupt.h>
#include <linux/irqchip/arm-gic-v3.h>
#include <linux/kernel.h>
#include <linux/mfd/baikal/lcru-pcie.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/pci.h>
#include <linux/phy/phy.h>
#include <linux/resource.h>

#include "pcie-designware.h"
//#include <linux/mod_devicetable.h>

struct baikal_pcie_rc {
    struct pcie_port  pp;
	struct dw_pcie	 *pcie;
	unsigned	      bus_nr;
	struct regmap	 *lcru;
	struct gpio_desc *reset_gpio;
	bool		      reset_active_low;
	char		      reset_name[32];
	bool		      retrained;
};

#define to_baikal_pcie_rc(x)	dev_get_drvdata((x)->dev)

#define BE_PCIE_MSI_RCNUM_MASK(x)		(0x3 << (2 * (x)))

#define PCIE_DEVICE_CONTROL_DEVICE_STATUS_REG	0x78
#define PCIE_CAP_CORR_ERR_REPORT_EN		BIT(0)
#define PCIE_CAP_NON_FATAL_ERR_REPORT_EN	BIT(1)
#define PCIE_CAP_FATAL_ERR_REPORT_EN		BIT(2)
#define PCIE_CAP_UNSUPPORT_REQ_REP_EN		BIT(3)

#define PCIE_LINK_CAPABILITIES_REG		0x7c
#define PCIE_CAP_MAX_LINK_WIDTH_MASK		0x3f0
#define PCIE_CAP_MAX_LINK_WIDTH_SHIFT		4

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

#define PCIE_MSI_CTRL_ADDR_LO_REG		0x820
#define PCIE_MSI_CTRL_ADDR_HI_REG		0x824

#define PCIE_IATU_VIEWPORT_REG			0x900
#define PCIE_IATU_REGION_INBOUND		BIT(31)
#define PCIE_IATU_REGION_OUTBOUND		0
#define PCIE_IATU_REGION_CTRL_2_REG		0x908

int be_debug = 0;

static int baikal_pcie_link_up(struct dw_pcie *pcie){
	struct baikal_pcie_rc *rc = to_baikal_pcie_rc(pcie);
	u32 reg;
	reg = baikal_pcie_lcru_readl(rc->lcru,BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	if (!(reg & BAIKAL_PCIE_LTSSM_ENABLE)) {
		return 0;
	}
	reg = baikal_pcie_lcru_readl(rc->lcru,BAIKAL_LCRU_PCIE_STATUS(rc->bus_nr));

	return (reg & BAIKAL_PCIE_SMLH_LINKUP) &&
	       (reg & BAIKAL_PCIE_RDLH_LINKUP);
}

int dw_pcie_cfg_write(void __iomem *addr, int size, u32 val) {
	if ((uintptr_t)addr & (size - 1))
		return PCIBIOS_BAD_REGISTER_NUMBER;
	if (size == 4)
		writel(val, addr);
	else if (size == 2)
		writew(val, addr);
	else if (size == 1)
		writeb(val, addr);
	else
		return PCIBIOS_BAD_REGISTER_NUMBER;
	return PCIBIOS_SUCCESSFUL;
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
	reg = baikal_pcie_lcru_readl(rc->lcru,
				     BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));

	reg &= ~(BAIKAL_PCIE_PHY_MGMT_ENABLE |
		 BAIKAL_PCIE_DBI2_MODE);

	baikal_pcie_lcru_writel(rc->lcru,BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);
	rc->retrained = false;
	linkup = baikal_pcie_link_up(pcie);

	/* If link is not established yet, reset the RC */
	if (!linkup) {
		/* Disable link training */
		reg = baikal_pcie_lcru_readl(rc->lcru,
					  BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));

		reg &= ~BAIKAL_PCIE_LTSSM_ENABLE;
		baikal_pcie_lcru_writel(rc->lcru,
					BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr),
					reg);

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
		reg = baikal_pcie_lcru_readl(rc->lcru,
					    BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));

		reg |= BAIKAL_PCIE_NONSTICKY_RST |
		       BAIKAL_PCIE_STICKY_RST	 |
		       BAIKAL_PCIE_PWR_RST	 |
		       BAIKAL_PCIE_CORE_RST	 |
		       BAIKAL_PCIE_PHY_RESET;

		/* If the RC is PCIe x8, reset PIPE0 and PIPE1 */
		if (rc->bus_nr == 2) {
			reg |= BAIKAL_PCIE_PIPE0_RESET |
			       BAIKAL_PCIE_PIPE1_RESET;
		} else {
			reg |= BAIKAL_PCIE_PIPE_RESET;
		}

		baikal_pcie_lcru_writel(rc->lcru,
					BAIKAL_LCRU_PCIE_RESET(rc->bus_nr),
					reg);

		usleep_range(20000, 30000);

		if (rc->reset_gpio != NULL) {
			/* Deassert PERST pin */
			gpiod_set_value_cansleep(rc->reset_gpio, 0);
		}

		/* Deassert PHY reset */
		reg = baikal_pcie_lcru_readl(rc->lcru,
					    BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));

		reg &= ~BAIKAL_PCIE_PHY_RESET;
		baikal_pcie_lcru_writel(rc->lcru,
					BAIKAL_LCRU_PCIE_RESET(rc->bus_nr),
					reg);

		/* Deassert all software controlled resets */
		reg = baikal_pcie_lcru_readl(rc->lcru,
					    BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));

		reg &= ~(BAIKAL_PCIE_ADB_PWRDWN	   |
			 BAIKAL_PCIE_HOT_RESET	   |
			 BAIKAL_PCIE_NONSTICKY_RST |
			 BAIKAL_PCIE_STICKY_RST	   |
			 BAIKAL_PCIE_PWR_RST	   |
			 BAIKAL_PCIE_CORE_RST	   |
			 BAIKAL_PCIE_PHY_RESET);

		if (rc->bus_nr == 2) {
			reg &= ~(BAIKAL_PCIE_PIPE0_RESET |
				 BAIKAL_PCIE_PIPE1_RESET);
		} else {
			reg &= ~BAIKAL_PCIE_PIPE_RESET;
		}

		baikal_pcie_lcru_writel(rc->lcru,
					BAIKAL_LCRU_PCIE_RESET(rc->bus_nr),
					reg);
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
		reg = baikal_pcie_lcru_readl(rc->lcru,
					  BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));

		reg |= BAIKAL_PCIE_LTSSM_ENABLE;
		baikal_pcie_lcru_writel(rc->lcru,
					BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr),
					reg);

		dw_pcie_wait_for_link(pcie);
	}

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		dw_pcie_msi_init(pp);
	}

	return 0;
}

static int baikal_pcie_msi_host_init(struct pcie_port *pp //, struct msi_controller *chip)
)
{

	struct device *dev = pp->dev;
	struct device_node *np = dev->of_node;
	struct device_node *msi_node;
	/*
	 * The MSI domain is set by the generic of_msi_configure().  This
	 * .msi_host_init() function keeps us from doing the default MSI
	 * domain setup in dw_pcie_host_init() and also enforces the
	 * requirement that "msi-parent" exists.
	 */

    printk (KERN_INFO "PCIE:BAIKAL - start 0f baikal_pcie_msi_host_init()\n");


	msi_node = of_parse_phandle(np, "msi-parent", 0);
	if (!msi_node) {
		dev_err(dev, "failed to find msi-parent\n");
        printk (KERN_INFO "%s PCIE:BAIKAL - failed to find msi-parent\n",__func__);
		return -EINVAL;
	}

        printk (KERN_INFO "%s PCIE:BAIKAL - end of baikal_pcie_msi_host_init() - success...\n",__func__);
	return 0;
}

static const struct dw_pcie_host_ops baikal_pcie_host_ops = {
    //.link_up = baikal_pcie_link_up,
	.host_init = baikal_pcie_host_init,
        .msi_host_init = baikal_pcie_msi_host_init,
};

static void baikal_pcie_link_print_status(struct baikal_pcie_rc *rc)
{
	struct dw_pcie *pcie = rc->pcie;
	struct device *dev = pcie->dev;
	u32 reg;
	unsigned speed, width;

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

static int baikal_pcie_get_msi(struct baikal_pcie_rc *rc,
			struct device_node *msi_node, u64 *msi_addr)
{
	struct pcie_port *pp = &rc->pp;
	struct device *dev = pp->dev;
	int ret;
	struct resource res;
	/*
	 * Check if 'msi-parent' points to ARM GICv3 ITS, which is the only
	 * supported MSI controller.
	 */
	if (!of_device_is_compatible(msi_node, "arm,gic-v3-its")) {
		dev_err(dev, "unable to find compatible MSI controller\n");
        printk(KERN_INFO "%s: PCIE:BAIKAL - error of baikal_pcie_get_msi() \n",__func__);
		return -ENODEV;
	}
	/* derive GITS_TRANSLATER address from GICv3 */
	ret = of_address_to_resource(msi_node, 0, &res);
	if (ret < 0) {
		dev_err(dev, "unable to obtain MSI controller resources\n");
		return ret;
	}
	*msi_addr = res.start + GITS_TRANSLATER;
	return 0;
}


static int baikal_pcie_msi_steer(struct baikal_pcie_rc *rc,struct device_node *msi_node)
{
	struct pcie_port *pp = &rc->pp;
	struct device *dev = pp->dev;
	int ret;
	u64 msi_addr;
	ret = baikal_pcie_get_msi(rc, msi_node, &msi_addr);
	if (ret < 0) {
		dev_err(dev, "MSI steering failed\n");
		return ret;
	}

	// Program the msi_data
	dw_pcie_cfg_write(pp->dbi_base + PCIE_MSI_ADDR_LO, 4,
			lower_32_bits(msi_addr));
	dw_pcie_cfg_write(pp->dbi_base + PCIE_MSI_ADDR_HI, 4,
			upper_32_bits(msi_addr));
	return 0;
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

static int baikal_pcie_msi_enable(struct baikal_pcie_rc *rc)
{
	struct pcie_port *pp = &rc->pp;
	struct device *dev = pp->dev;
	struct device_node *msi_node;
	int ret;

	/*
	 * The "msi-parent" phandle needs to exist
	 * for us to obtain the MSI node.
	 */
    if (dev != NULL)
	  msi_node = of_parse_phandle(dev->of_node, "msi-parent", 0);
    else
      return -ENODEV;

	if (!msi_node) {
        printk(KERN_INFO "%s: PCIE:BAIKAL - cannot get msi_node \n",__func__);
		return -ENODEV;
    }

	ret = baikal_pcie_msi_steer(rc, msi_node);
	if (ret)
		goto out_put_node;

out_put_node:
	of_node_put(msi_node);
	return ret;
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

static int __init baikal_pcie_add_pcie_port(struct baikal_pcie_rc *rc, struct platform_device *pdev) {
    int irq, ret;
    struct resource *res;
    //struct dw_pcie *pcie = rc->pcie;
    struct pcie_port *pp = &rc->pp;
    pp->dev = &pdev->dev;
    res = platform_get_resource_byname(pdev, IORESOURCE_MEM, "dbi");
    if (res) {
		pp->dbi_base = devm_ioremap(pp->dev, res->start, resource_size(res));
		if (!pp->dbi_base) {
			dev_err(pp->dev, "error with ioremap\n");
			return -ENOMEM;
		}
	} else {
        printk(KERN_INFO "%s PCIE:BAIKAL - missing dbi_base in pcie_port\n",__func__);
		dev_err(pp->dev, "missing *dbi* reg space\n");
		return -EINVAL;
	}

	//pp->irq 
    irq = platform_get_irq(pdev, 0);
	if (irq < 0) {
		dev_err(pp->dev, "missing IRQ resource: %d\n", irq);
        printk(KERN_INFO "PCIE:BAIKAL -%s Error:missing IRQ resource\n",__func__);
		return irq;
	}
    
	ret = request_irq(irq, baikal_pcie_err_irq_handler, IRQF_SHARED, 
      "baikal-pcie-error-irq", rc);
	if (ret < 0) {
		dev_err(pp->dev, "failed to request error IRQ %d\n",irq);
		printk (KERN_INFO "PCIE:BAIKAL -%s Error:for request IRQ\n",__func__);
		return ret;
	}
	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		ret = baikal_pcie_msi_enable(rc);
		if (ret) {
		  dev_err(pp->dev, "failed to initialize MSI\n");
          printk(KERN_INFO "%s: PCIE:BAIKAL - failed to init MSI \n",__func__);
          return ret;
		}
	}  

    /*
	ret = devm_request_irq (dev,pp->irq,baikal_pcie_err_irq_handler,
      IRQF_SHARED,"baikal-pcie-error-irq",pcie) ;
    if(ret) {
          dev_err(dev,"failed to request irq %d\n",pp->irq);
          return ret;
    }
    if (IS_ENABLED(CONFIG_PCI_MSI)) {
          pp->msi_irq = platform_get_irq(pdev,1);
          if (pp->msi_irq < 0) {
            printk (KERN_INFO "PCIE:BAIKAL -%s Error:get incorrect MSI IRQ for Baikal device\n",__func__);
            return  pp->msi_irq;
          }
    }   */

	pp->root_bus_nr = -1;
	pp->ops = &baikal_pcie_host_ops;
	ret = dw_pcie_host_init(pp);
	if (ret) {
          dev_err(pp->dev, "Failed to initialize host\n");
          printk (KERN_INFO "PCIE:BAIKAL -%s Error:Failed to initialize host\n",__func__);
          return ret;
	}

    //baikal_pcie_link_retrain_bus (pp->root_bus);
	return 0;
}

/*  
static int baikal_pcie_add_pcie_port(struct baikal_pcie_rc *rc,
				     struct platform_device *pdev){
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
	pp->root_bus_nr = -1;
	ret = dw_pcie_host_init(pp);
	if (ret) {
		dev_err(dev, "Failed to initialize host\n");
		return ret;
	}
	baikal_pcie_link_retrain_bus(pp->root_bus);
	return 0;
}*/

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

static void baikal_pcie_cease_link(struct baikal_pcie_rc *rc)
{
	u32 reg;
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg &= ~BAIKAL_PCIE_LTSSM_ENABLE;

	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);
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

static int baikal_pcie_hw_init_m(struct baikal_pcie_rc *rc)
{
	//unsigned int timeout = 10;
	u32 reg;
    if (rc->lcru == NULL){
    printk (KERN_INFO "PCIE:BAIKAL - rc->lcru == NULL ; abort !!! \n");
    return -ENODEV;
    }
	// TODO add PHY configuration if needed
	/* Deassert PHY reset */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));
	reg &= ~BAIKAL_PCIE_PHY_RESET;
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);
	// TODO timeout?

	/* Enable access to the PHY registers */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr));
	reg |= (BAIKAL_PCIE_PHY_MGMT_ENABLE | BAIKAL_PCIE_DBI2_MODE);
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_GEN_CTL(rc->bus_nr), reg);

	// TODO timeout?
	/* Clear all software controlled resets of the controller */
	reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr));
	reg &= 
      ~(BAIKAL_PCIE_ADB_PWRDWN | BAIKAL_PCIE_HOT_RESET |
        BAIKAL_PCIE_NONSTICKY_RST |	BAIKAL_PCIE_STICKY_RST |
        BAIKAL_PCIE_PWR_RST | BAIKAL_PCIE_CORE_RST | BAIKAL_PCIE_PIPE_RESET);
	baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_RESET(rc->bus_nr), reg);
	// TODO timeout?

	if (IS_ENABLED(CONFIG_PCI_MSI)) {
		/* Set up the MSI translation mechanism: */

		/* First, set MSI_AWUSER to 0 */
		reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL0);
		reg &= ~BAIKAL_PCIE_MSI_AWUSER_MASK;
		reg |= (0 << BAIKAL_PCIE_MSI_AWUSER_SHIFT);
		baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL0, reg);

		// TODO timeout?

		/* Second, enable MSI, the RC number for all RC is 0*/
		reg = baikal_pcie_lcru_readl(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2);
		reg |= BAIKAL_PCIE_MSI_TRANS_EN(rc->bus_nr);
		//reg &= ~BAIKAL_PCIE_MSI_RCNUM_MASK(rc->bus_nr);
        reg &= ~BE_PCIE_MSI_RCNUM_MASK(rc->bus_nr);
		//reg |= BAIKAL_PCIE_MSI_RCNUM(rc->bus_nr);
		baikal_pcie_lcru_writel(rc->lcru, BAIKAL_LCRU_PCIE_MSI_TRANS_CTL2, reg);

	}
    printk (KERN_INFO "%s: PCIE:BAIKAL - end of baikal_pcie_hw_init_m() - success..... \n",
      __func__);
	return 0;
}

static const struct of_device_id of_baikal_pcie_match[] = {
	{ .compatible = "baikal,pcie-m"  //,
      //.data = baikal_pcie_hw_init_m,
    },
	{}
};
MODULE_DEVICE_TABLE(of, of_baikal_pcie_match);

static const struct dw_pcie_ops baikal_pcie_ops = {
	.link_up = baikal_pcie_link_up
};

static int baikal_pcie_probe(struct platform_device *pdev)
{

    struct device *dev = &pdev->dev;
    if (dev == 0)
    printk (KERN_INFO "PCIE:BAIKAL dev == NULL \n");

    //struct resource *res ;
    struct baikal_pcie_rc *rc;
    struct dw_pcie *pcie;
    const struct of_device_id *of_id;
    int err;
    int (*hw_init_fn)(struct baikal_pcie_rc *);
	u32 idx[2];
	enum of_gpio_flags gpio_flags;
	int reset_gpio;
    
    
    if (dev == 0 || np == 0)  {
        printk (KERN_INFO "%s:  dev == NULL or np == NULL \n",__func__);
        return -EINVAL;
    } else  {
        printk (KERN_INFO "%s::   PCIE:BAIKAL dev is not NULL \n",__func__);
    }
    dev_dbg(dev, "pci be_debug %x\n", be_debug);


	if (!of_match_device(of_baikal_pcie_match, dev)) {
		dev_err(dev, "device can't be handled by pcie-baikal\n");

        printk(KERN_INFO "PCIE:BAIKAL  of_match_device() returnrd '0' ; return - %d\n",-EINVAL);
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
	rc->lcru = syscon_regmap_lookup_by_phandle(dev->of_node,"baikal,pcie-lcru");
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
    rc->bus_nr = idx[1];
    /*
	if (idx[1] > 2) {
		dev_err(dev, "incorrect pcie-lcru index\n");
		rc->lcru = NULL;
		return -EINVAL;
	}*/
/****
const struct of_device_id *of_match_device(const struct of_device_id *matches,
					   const struct device *dev) {
	if ((!matches) || (!dev->of_node))
		return NULL;
	return of_match_node(matches, dev->of_node);
}
*/
    if (dev == 0 || dev->of_node == 0) {
      printk (KERN_INFO "PCIE:BAIKAL dev of pcie:baikal is not Valid \n");
      return -22 ;
    }
    of_id = of_match_device (of_baikal_pcie_match, dev);
	if (!of_id || !of_id->data) {
		printk(KERN_INFO "**%s: Baikal pcie device can't be handled by pcie-baikal driver\n",__func__);
		return -EINVAL;
	} else {
        printk (KERN_INFO "%s PCIE:BAIKAL - success OF of_match_device() \n",__func__) ;
    }

    hw_init_fn = of_id->data;
    pm_runtime_enable(dev);

    err = pm_runtime_get_sync(dev);
    if (err < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_pm_disable;
    }
    baikal_pcie_cease_link(/*pcie*/rc);
    /* LINK DISABLED */

	reset_gpio = of_get_named_gpio_flags(dev->of_node, "reset-gpios", 0, &gpio_flags);
	if (reset_gpio != -EPROBE_DEFER && gpio_is_valid(reset_gpio)) {
      unsigned long gpio_flags_be;
      snprintf(rc->reset_name,32,"pcie%d-reset",rc->bus_nr);
      if (gpio_flags & OF_GPIO_ACTIVE_LOW) {
        gpio_flags_be= GPIOF_ACTIVE_LOW | GPIOF_OUT_INIT_LOW; 
      } else {
        gpio_flags_be = GPIOF_OUT_INIT_HIGH;
      }
      err = devm_gpio_request_one (dev,reset_gpio,gpio_flags_be,rc->reset_name);
      if (err) {
        dev_err(dev, "request GPIO failed (%d)\n", err);
        goto err_pm_disable;
      }
      rc->reset_gpio = gpio_to_desc(reset_gpio);
      //rc->reset_active_low = !!(gpio_flags & OF_GPIO_ACTIVE_LOW);
      udelay(100);
      gpiod_set_value_cansleep(rc->reset_gpio, 0);
      //snprintf(rc->reset_name, sizeof(rc->reset_name), "pcie%u-reset",rc->bus_nr);
	} else {
	  rc->reset_gpio = NULL;
	}

    err = hw_init_fn(rc);
    if (err) {
      printk(KERN_INFO "**%s: Baikal PCIe link down\n",__func__);
      err = 0;
      goto err_pm_put;
    }

    // PHY INITIALIZED
    err = baikal_pcie_add_pcie_port (rc,pdev);
    if (err < 0) {
      printk (KERN_INFO "**%s: PCIE:BAIKAL - cannot add pcie port\n",__func__);
      goto err_pm_put;
    }
    /*
	pm_runtime_enable(dev);
	ret = pm_runtime_get_sync(dev);
	if (ret < 0) {
		dev_err(dev, "pm_runtime_get_sync failed\n");
		goto err_pm_disable;
	}    */
	platform_set_drvdata(pdev, rc);

    /*
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
	}*/
    printk (KERN_INFO "**%s: PCIE:BAIKAL - baikal_pcie_probe() - OK\n",__func__);
	return 0;

err_pm_put:
	pm_runtime_put(dev);
err_pm_disable:
	pm_runtime_disable(dev);
	return err;
}

module_param(be_debug, int, 0644);

static struct platform_driver baikal_pcie_driver = {
	.driver = {
		.name = "baikal-pcie",
		.of_match_table = of_baikal_pcie_match,
		.suppress_bind_attrs = true,
		.pm = &baikal_pcie_pm_ops
	},
	.probe = baikal_pcie_probe
};
//module_platform_driver(baikal_pcie_driver);
builtin_platform_driver(baikal_pcie_driver);

MODULE_DESCRIPTION("Baikal PCIe host controller driver");
MODULE_LICENSE("GPL v2");
