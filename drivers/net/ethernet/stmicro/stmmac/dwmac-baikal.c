// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Baikal Electronics DWMAC specific glue layer
 *
 * Copyright (C) 2015,2016,2021 Baikal Electronics JSC
 * Authors: Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>
 *          Alexey Sheplyakov <asheplyakov@altlinux.org>
 */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/clk-provider.h>
#include <linux/iopoll.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>

#include "stmmac.h"
#include "stmmac_platform.h"
#include "common.h"
#include "dwmac_dma.h"
#include "dwmac1000_dma.h"

#define MAC_GPIO	0x00e0		/* GPIO register */
#define MAC_GPIO_GPO	(1 << 8)	/* Output port */

struct baikal_gmac {
	struct device	*dev;
	struct clk	*tx2_clk;
};

static int baikal_gmac_dma_reset(void __iomem *ioaddr)
{
	int err;
	u32 value;

	/* DMA SW reset */
	value = readl(ioaddr + DMA_BUS_MODE);
	value |= DMA_BUS_MODE_SFT_RESET;
	writel(value, ioaddr + DMA_BUS_MODE);

	udelay(100);

	/* Clear PHY reset */
	value = readl(ioaddr + MAC_GPIO);
	value |= MAC_GPIO_GPO;
	writel(value, ioaddr + MAC_GPIO);

	err = readl_poll_timeout(ioaddr + DMA_BUS_MODE, value,
				 !(value & DMA_BUS_MODE_SFT_RESET),
				 10000, 1000000);
	if (err) {
		return -EBUSY;
	}

	return 0;
}

static const struct stmmac_dma_ops baikal_gmac_dma_ops = {
	.reset = baikal_gmac_dma_reset,
	.init = dwmac1000_dma_init,
	.init_rx_chan = dwmac1000_dma_init_rx,
	.init_tx_chan = dwmac1000_dma_init_tx,
	.axi = dwmac1000_dma_axi,
	.dump_regs = dwmac1000_dump_dma_regs,
	.dma_rx_mode = dwmac1000_dma_operation_mode_rx,
	.dma_tx_mode = dwmac1000_dma_operation_mode_tx,
	.enable_dma_transmission = dwmac_enable_dma_transmission,
	.enable_dma_irq = dwmac_enable_dma_irq,
	.disable_dma_irq = dwmac_disable_dma_irq,
	.start_tx = dwmac_dma_start_tx,
	.stop_tx = dwmac_dma_stop_tx,
	.start_rx = dwmac_dma_start_rx,
	.stop_rx = dwmac_dma_stop_rx,
	.dma_interrupt = dwmac_dma_interrupt,
	.get_hw_feature = dwmac1000_get_hw_feature,
	.rx_watchdog = dwmac1000_rx_watchdog
};

static struct mac_device_info *baikal_gmac_setup(void *ppriv)
{
	struct mac_device_info *mac, *old_mac;
	struct stmmac_priv *priv = ppriv;
	int ret;
	u32 value;

	mac = devm_kzalloc(priv->device, sizeof(*mac), GFP_KERNEL);
	if (!mac) {
		return NULL;
	}

	/* Clear PHY reset */
	value = readl(priv->ioaddr + MAC_GPIO);
	value |= MAC_GPIO_GPO;
	writel(value, priv->ioaddr + MAC_GPIO);

	mac->dma = &baikal_gmac_dma_ops;
	old_mac = priv->hw;
	priv->hw = mac;
	ret = dwmac1000_setup(priv);
	priv->hw = old_mac;
	if (ret) {
		dev_err(priv->device, "dwmac1000_setup: error %d", ret);
		return NULL;
	}

	return mac;
}

static void baikal_gmac_fix_mac_speed(void *priv, unsigned int speed)
{
	struct baikal_gmac *gmac = priv;
	unsigned long tx2_clk_freq = 0;

	switch (speed) {
	case SPEED_1000:
		tx2_clk_freq = 250000000;
		break;
	case SPEED_100:
		tx2_clk_freq = 50000000;
		break;
	case SPEED_10:
		tx2_clk_freq = 5000000;
		break;
	}

	if (gmac->tx2_clk != NULL && tx2_clk_freq) {
		clk_set_rate(gmac->tx2_clk, tx2_clk_freq);
	}
}

#ifdef CONFIG_ACPI
static struct plat_stmmacenet_data *baikal_stmmac_probe_config(struct device *dev,
								const char **mac)
{
	struct plat_stmmacenet_data *plat_dat;
	u8 nvmem_mac[ETH_ALEN];
	int ret;

	plat_dat = devm_kzalloc(dev, sizeof(*plat_dat), GFP_KERNEL);
	if (!plat_dat) {
		return ERR_PTR(-ENOMEM);
	}

	ret = nvmem_get_mac_address(dev, &nvmem_mac);
	if (ret) {
		if (ret == -EPROBE_DEFER) {
			return ERR_PTR(ret);
		}

		*mac = NULL;
	} else {
		*mac = devm_kmemdup(dev, nvmem_mac, ETH_ALEN, GFP_KERNEL);
	}

	plat_dat->phy_interface = device_get_phy_mode(dev);
	if (plat_dat->phy_interface < 0) {
		return NULL;
	}

	plat_dat->interface = plat_dat->phy_interface;

	if (device_property_read_u32(dev, "max-speed", &plat_dat->max_speed)) {
		plat_dat->max_speed = -1;
	}

	plat_dat->bus_id = ACPI_COMPANION(dev)->pnp.instance_no;

	ret = device_property_read_u32(dev, "reg", &plat_dat->phy_addr);
	if (ret) {
		dev_err(dev, "couldn't get reg property\n");
		return ERR_PTR(ret);
	}

	if (plat_dat->phy_addr >= PHY_MAX_ADDR) {
		dev_err(dev, "PHY address %i is too large\n",
				plat_dat->phy_addr);
		return ERR_PTR(-EINVAL);
	}

	plat_dat->mdio_bus_data = devm_kzalloc(dev,
						sizeof(*plat_dat->mdio_bus_data),
						GFP_KERNEL);
	if (!plat_dat->mdio_bus_data) {
		return ERR_PTR(-ENOMEM);
	}

	plat_dat->mdio_bus_data->needs_reset = true;
	plat_dat->maxmtu = JUMBO_LEN;
	plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;
	plat_dat->unicast_filter_entries = 1;
	plat_dat->bugged_jumbo = 1; /* TODO: is it really required? */

	plat_dat->dma_cfg = devm_kzalloc(dev,
					sizeof(*plat_dat->dma_cfg),
					GFP_KERNEL);
	if (!plat_dat->dma_cfg) {
		return ERR_PTR(-ENOMEM);
	}

	plat_dat->dma_cfg->pbl = DEFAULT_DMA_PBL;
	device_property_read_u32(dev, "snps,txpbl", &plat_dat->dma_cfg->txpbl);
	device_property_read_u32(dev, "snps,rxpbl", &plat_dat->dma_cfg->rxpbl);
	plat_dat->dma_cfg->fixed_burst = device_property_read_bool(dev,
							"snps,fixed-burst");

	plat_dat->axi = devm_kzalloc(dev, sizeof(*plat_dat->axi), GFP_KERNEL);
	if (!plat_dat->axi) {
		return ERR_PTR(-ENOMEM);
	}

	device_property_read_u32_array(dev, "snps,blen",
					plat_dat->axi->axi_blen, AXI_BLEN);

	plat_dat->rx_queues_to_use = 1;
	plat_dat->tx_queues_to_use = 1;
	plat_dat->rx_queues_cfg[0].mode_to_use = MTL_QUEUE_DCB;
	plat_dat->tx_queues_cfg[0].mode_to_use = MTL_QUEUE_DCB;

	if (device_property_read_u32(dev, "stmmac-clk", &plat_dat->clk_ptp_rate)) {
		plat_dat->clk_ptp_rate = 50000000;
	}

	plat_dat->stmmac_clk = devm_clk_get(dev, STMMAC_RESOURCE_NAME);
	if (IS_ERR(plat_dat->stmmac_clk)) {
		plat_dat->stmmac_clk = clk_register_fixed_rate(NULL,
							dev_name(dev), NULL, 0,
							plat_dat->clk_ptp_rate);
		if (IS_ERR(plat_dat->stmmac_clk)) {
			plat_dat->stmmac_clk = NULL;
			return ERR_CAST(plat_dat->stmmac_clk);
		}
	}

	clk_prepare_enable(plat_dat->stmmac_clk);

	plat_dat->mdio_bus_data->phy_mask = ~0;

	if (device_get_child_node_count(dev) != 1) {
		clk_disable_unprepare(plat_dat->stmmac_clk);
		clk_unregister_fixed_rate(plat_dat->stmmac_clk);
		return ERR_PTR(-EINVAL);
	}

	return plat_dat;
}

static int baikal_add_mdio_phy(struct device *dev)
{
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(dev));
	struct fwnode_handle *fwnode = device_get_next_child_node(dev, NULL);
	struct phy_device *phy;
	int ret;

	phy = get_phy_device(priv->mii, priv->plat->phy_addr, 0);
	if (IS_ERR(phy)) {
		return PTR_ERR(phy);
	}

	phy->irq = priv->mii->irq[priv->plat->phy_addr];
	phy->mdio.dev.fwnode = fwnode;

	ret = phy_device_register(phy);
	if (ret) {
		phy_device_free(phy);
		return ret;
	}

	return 0;
}
#else
static struct plat_stmmacenet_data *baikal_stmmac_probe_config(struct device *dev,
								const char **mac)
{
	return NULL;
}

static int baikal_add_mdio_phy(struct device *dev)
{
	return 0;
}
#endif

static int baikal_gmac_probe(struct platform_device *pdev)
{
	struct plat_stmmacenet_data *plat_dat;
	struct stmmac_resources stmmac_res;
	struct baikal_gmac *gmac;
	struct device_node *dn = NULL;
	int ret;

	if (acpi_disabled) {
		ret = stmmac_get_platform_resources(pdev, &stmmac_res);
		if (ret) {
			return ret;
		}
	} else {
		memset(&stmmac_res, 0, sizeof(stmmac_res));
		stmmac_res.irq = platform_get_irq(pdev, 0);
		if (stmmac_res.irq < 0) {
			return stmmac_res.irq;
		}

		stmmac_res.wol_irq = stmmac_res.irq;
		stmmac_res.addr = devm_platform_ioremap_resource(pdev, 0);
		if (IS_ERR(stmmac_res.addr)) {
			return PTR_ERR(stmmac_res.addr);
		}
	}

	ret = dma_set_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret) {
		dev_warn(&pdev->dev, "no suitable DMA available\n");
		return ret;
	}

	if (pdev->dev.of_node) {
		plat_dat = stmmac_probe_config_dt(pdev, &stmmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "dt configuration failed\n");
			return PTR_ERR(plat_dat);
		}
	} else if (!acpi_disabled) {
		plat_dat = baikal_stmmac_probe_config(&pdev->dev,
							&stmmac_res.mac);
		if (IS_ERR(plat_dat)) {
			dev_err(&pdev->dev, "acpi configuration failed\n");
			return PTR_ERR(plat_dat);
		}

		dn = kzalloc(sizeof(struct device_node), GFP_KERNEL);
		if (!dn) {
			ret = -ENOMEM;
			goto err_remove_config_dt;
		}

		plat_dat->phy_node = dn;
	} else {
		plat_dat = dev_get_platdata(&pdev->dev);
		if (!plat_dat) {
			dev_err(&pdev->dev, "no platform data provided\n");
			return -EINVAL;
		}

		/* Set default value for multicast hash bins */
		plat_dat->multicast_filter_bins = HASH_TABLE_SIZE;

		/* Set default value for unicast filter entries */
		plat_dat->unicast_filter_entries = 1;
	}

	gmac = devm_kzalloc(&pdev->dev, sizeof(*gmac), GFP_KERNEL);
	if (!gmac) {
		ret = -ENOMEM;
		goto err_remove_config_dt;
	}

	gmac->dev = &pdev->dev;
	gmac->tx2_clk = devm_clk_get(gmac->dev, "tx2_clk");
	if (IS_ERR(gmac->tx2_clk)) {
		dev_warn(&pdev->dev, "couldn't get TX2 clock\n");
		gmac->tx2_clk = NULL;
	}

	plat_dat->fix_mac_speed = baikal_gmac_fix_mac_speed;
	plat_dat->bsp_priv = gmac;
	plat_dat->has_gmac = 1;
	plat_dat->bugged_jumbo = 1; /* TODO: is it really required? */
	plat_dat->setup = baikal_gmac_setup;

	ret = stmmac_dvr_probe(&pdev->dev, plat_dat, &stmmac_res);
	if (ret) {
		goto err_remove_config_dt;
	}

	if (!acpi_disabled) {
		ret = baikal_add_mdio_phy(&pdev->dev);
		if (ret) {
			goto err_remove_config_dt;
		}
	}

	kfree(dn);
	return 0;

err_remove_config_dt:
	stmmac_remove_config_dt(pdev, plat_dat);

	if (!acpi_disabled) {
		clk_disable_unprepare(plat_dat->stmmac_clk);
		clk_unregister_fixed_rate(plat_dat->stmmac_clk);
	}

	kfree(dn);
	return ret;
}

static int baikal_gmac_remove(struct platform_device *pdev)
{
	struct stmmac_priv *priv = netdev_priv(dev_get_drvdata(&pdev->dev));

	if (!acpi_disabled) {
		clk_disable_unprepare(priv->plat->stmmac_clk);
		clk_unregister_fixed_rate(priv->plat->stmmac_clk);
		priv->plat->stmmac_clk = NULL;
	}

	return stmmac_pltfr_remove(pdev);
}

static const struct of_device_id baikal_gmac_dwmac_match[] = {
	{ .compatible = "baikal,bm1000-gmac" },
	{ .compatible = "baikal,bs1000-gmac" },

	/*
	 * TODO: "be,dwmac" is legacy. Correct prefix is "baikal":
	 * https://www.kernel.org/doc/Documentation/devicetree/bindings/vendor-prefixes.yaml
	 * Use "baikal,b*-gmac" for future development.
	 */
	{ .compatible = "be,dwmac" },
	{ }
};
MODULE_DEVICE_TABLE(of, baikal_gmac_dwmac_match);

static struct platform_driver baikal_gmac_dwmac_driver = {
	.probe	= baikal_gmac_probe,
	.remove	= baikal_gmac_remove,
	.driver	= {
		.name = "baikal-gmac-dwmac",
		.pm = &stmmac_pltfr_pm_ops,
		.of_match_table = of_match_ptr(baikal_gmac_dwmac_match)
	}
};
module_platform_driver(baikal_gmac_dwmac_driver);

MODULE_DESCRIPTION("Baikal DWMAC specific glue driver");
MODULE_LICENSE("GPL v2");
