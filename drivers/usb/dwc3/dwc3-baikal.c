/**
 * dwc3-baikal.c - Baikal Electronics SoCs Specific Glue layer
 *
 * Copyright (C) 2015 Baikal Electronics JSC - http://www.baikalelectronics.ru
 *
 * Author: Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License version 2  of
 * the License as published by the Free Software Foundation.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 */

#include <linux/acpi.h>
#include <linux/clk.h>
#include <linux/irq.h>
#include <linux/clk-provider.h>
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/extcon.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/dma-mapping.h>
#include <linux/usb/usb_phy_generic.h>
#include <linux/io.h>
#include <linux/iopoll.h>
#include <linux/reset.h>
#include <linux/usb/of.h>
#include <linux/of_platform.h>

#include "core.h"

#define QSCRATCH_HS_PHY_CTRL			0x10
#define UTMI_OTG_VBUS_VALID			BIT(20)
#define SW_SESSVLD_SEL				BIT(28)

#define QSCRATCH_SS_PHY_CTRL			0x30
#define LANE0_PWR_PRESENT			BIT(24)

#define QSCRATCH_GENERAL_CFG			0x08
#define PIPE_UTMI_CLK_SEL			BIT(0)
#define PIPE3_PHYSTATUS_SW			BIT(3)
#define PIPE_UTMI_CLK_DIS			BIT(8)

#define PWR_EVNT_IRQ_STAT_REG			0x58
#define PWR_EVNT_LPM_IN_L2_MASK			BIT(4)
#define PWR_EVNT_LPM_OUT_L2_MASK		BIT(5)

#define SDM845_QSCRATCH_BASE_OFFSET		0xf8800
#define SDM845_QSCRATCH_SIZE			0x400
#define SDM845_DWC3_CORE_SIZE			0xcd00


#define  USB3_BASE_ADDRESS               0x002C500000

// This register uses to configure the USB3 PHY and PIPE interface
#define  GUSB3PIPECTL0                          0xc2c0          //  0000 0001 0011 1110 0000 0000 0000 0010  0x013e0002 
#define  GUSB3PIPECTL0                          0xc2c4


struct dwc3_acpi_pdata {
	u32			qscratch_base_offset;
	u32			qscratch_base_size;
	u32			dwc3_core_base_size;
};

struct dwc3_baikal {
	struct device	*dev;
	void __iomem    *qscratch_base ;
	struct platform_device	*dwc3;

	int			num_clocks;
	struct clk		**clks;

	const struct dwc3_acpi_pdata  *acpi_pdata ;
	enum usb_dr_mode  mode;
	bool		is_suspended ;
	bool		pm_suspended ;
};

static inline void
dwc3_be_setbits(void __iomem *base, u32 offset, u32 val) {
	u32 reg;
	reg = readl(base + offset);
	reg |= val;
	writel(reg, base + offset);
	/* ensure that above write is through */
	readl(base + offset);
}

static inline void be_get_ctrl_regs (void __iomem *b, u32 offset)  {
	u32 reg ;
	offset = 0x0 ;
	reg = readl(b + offset) ;

	printk (KERN_INFO "%s    offset:0x%8x    reg: 0x%8x \n",__func__,offset,reg);  // usb3 base

	offset = GUSB3PIPECTL0 ;
	reg = readl (b + offset);
	printk (KERN_INFO "%s    offset:0x%8x    reg: 0x%8x \n",__func__,offset,reg);  // ctrl 0

	offset = GUSB3PIPECTL1 ;
	reg = readl (b + offset);
	printk (KERN_INFO "%s    offset:0x%8x    reg: 0x%8x \n",__func__,offset,reg);  // ctrl 1
} 




static inline void
dwc3_be_clrbits(void __iomem *base, u32 offset, u32 val) {
	u32 reg;
	reg = readl(base + offset);
	reg &= ~val;
	writel(reg, base + offset);
	/* ensure that above write is through */
	readl(base + offset);
}
static void
dwc3_be_vbus_override_enable(struct dwc3_baikal *be, bool enable) {
	if (enable) {
		dwc3_be_setbits(/*qcom*/be->qscratch_base, QSCRATCH_SS_PHY_CTRL, LANE0_PWR_PRESENT);
		dwc3_be_setbits(/*qcom*/be->qscratch_base, QSCRATCH_HS_PHY_CTRL, UTMI_OTG_VBUS_VALID | SW_SESSVLD_SEL);
	} else {
		dwc3_be_clrbits(/*qcom*/be->qscratch_base, QSCRATCH_SS_PHY_CTRL, LANE0_PWR_PRESENT);
		dwc3_be_clrbits(/*qcom*/be->qscratch_base, QSCRATCH_HS_PHY_CTRL, UTMI_OTG_VBUS_VALID | SW_SESSVLD_SEL);
	}
}
static int dwc3_be_of_register_core /*dwc3_*/ (struct platform_device *pdev) {
	int ret;
	struct dwc3_baikal *be = platform_get_drvdata(pdev);
	struct device_node *np = pdev->dev.of_node , *dwc3_np;
	struct device   *dev = &pdev->dev ;

	dwc3_np = of_get_child_by_name (np, "dwc3");
	if (!dwc3_np)  {
		printk (KERN_INFO "%s: cannot get child node for BE usb node \n",__func__);
		return -ENODEV ;
	}
	ret = of_platform_populate (np,NULL,NULL,dev) ;
	if (ret) {
		printk (KERN_INFO "%s: failed registering dwc3 core , error - %d \n",__func__,ret);
		return  ret;
	}
	be->dwc3 =
		of_find_device_by_node (dwc3_np) ;
	if(!be->dwc3) {
		printk (KERN_INFO "%s: cant get dwc3 platform device \n",__func__);
		return -ENODEV ;
	}
	return 0;
}   

 
//    ret = dwc3_be_clk_init (dwc, of_clk_get_parent_count(np)) ;
static int dwc3_be_clk_init (struct dwc3_baikal   *dwc, int count)
{
	struct device          *dev = dwc->dev ;
	struct device_node     *np  = dev->of_node;
	int     i;
	if (!np || !count)
		return 0;
	if (count < 0)  
		return count ;
	dwc->num_clocks = count;
	dwc->clks = devm_kcalloc (dev, dwc->num_clocks, sizeof(struct clk *), GFP_KERNEL);
	if (!dwc->clks)  
		return -ENOMEM ; 

	for (i = 0; i < dwc->num_clocks; i++) {
		struct clk    *clk;
		int   ret ;
		clk = of_clk_get (np, i);
		if (IS_ERR(clk)) {
			while (--i >= 0)
				clk_put(dwc->clks[i]);
			return PTR_ERR(clk);
		}
		ret = clk_prepare_enable (clk);
		if (ret < 0) {
			while (--i >= 0) {
				clk_disable_unprepare(dwc->clks[i]);
				clk_put(dwc->clks[i]);
			}
			clk_put (clk);
			return ret;
		}
		dwc->clks[i] = clk;
	}
	return 0;
}

static int
be_dwc3_probe(struct platform_device *pdev)
{
	struct device		*dev = &pdev->dev;
	struct device_node	*node = pdev->dev.of_node;
	struct dwc3_baikal	*dwc;
	struct resource         *res, *parent_res = NULL ;
	int			ret;
	int    numbe ;

	dwc = devm_kzalloc(dev, sizeof(*dwc), GFP_KERNEL);
	if (!dwc)
		return -ENOMEM;

	ret = dma_coerce_mask_and_coherent(dev, DMA_BIT_MASK(64));
	if (ret) {
		dev_err(dev, "DMA mask error %d\n", ret);
		return ret;
	}
	platform_set_drvdata(pdev, dwc);
	dwc->dev = &pdev->dev ;


        //dwc3_be_setbits
	be_get_ctrl_regs (dwc->qscratch_base,0x0);         //  be->qscratch_base


	/***
	dwc->clk = devm_clk_get(dwc->dev, "usb");
	if (IS_ERR(dwc->clk)) {
		printk (KERN_INFO "%s  no interface clk specified \n",__func__);
		return -EINVAL;
	}
	ret = clk_prepare_enable(dwc->clk);
	if (ret < 0) {
		printk (KERN_INFO  "%s  unable to enable usb clock \n",__func__) ;
		return ret;
	}     ***/
	numbe = of_clk_get_parent_count (node);
	if (numbe >= 0)  {
	ret = dwc3_be_clk_init (dwc, /*of_clk_get_parent_count(node)*/numbe);
	if (ret) {
		printk (KERN_INFO "%s     cant init BE clk's   \n",__func__);
		return  ret;
	}
	} else {
            printk (KERN_INFO "%s   incorrect number of clk's \n", __func__) ;
	}

	/*********
	ret = dwc3_qcom_clk_init(qcom, of_clk_get_parent_count(np));
	if (ret) {
		dev_err(dev, "failed to get clocks\n");
		goto reset_assert;
	}  *******/



	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (node) {
		parent_res = res;
	} else {
		parent_res = kmemdup(res, sizeof(struct resource), GFP_KERNEL);
		if (!parent_res)
			return -ENOMEM;
		parent_res->start = res->start + /*qcom*/dwc->acpi_pdata->qscratch_base_offset;
		parent_res->end = parent_res->start + /*qcom*/dwc->acpi_pdata->qscratch_base_size;
	}

	if (node) {
		//ret = of_platform_populate(node, NULL, NULL, dev);
		ret = dwc3_be_of_register_core (pdev) ;  // should be implement
		if (ret) {
			dev_err(&pdev->dev, "failed to register dwc3 core\n");
			goto __error;
		}
	} else {
		dev_err(dev, "no device node, failed to add dwc3 core\n");
		ret = -ENODEV;
		goto __error;
	}

	dwc->mode = usb_get_dr_mode (&dwc->dwc3->dev) ;

	if (dwc->mode == USB_DR_MODE_PERIPHERAL)
		dwc3_be_vbus_override_enable(dwc, true);

	return 0;

__error:
	//clk_disable_unprepare(dwc->clk);
	return ret;
}

static int be_dwc3_remove_core(struct device *dev, void *c)
{
	struct platform_device *pdev = to_platform_device(dev);

	platform_device_unregister(pdev);
	return 0;
}

static int be_dwc3_remove(struct platform_device *pdev)
{
	int  i;
	struct dwc3_baikal *dwc = platform_get_drvdata(pdev);
	device_for_each_child(&pdev->dev, NULL, be_dwc3_remove_core);

	// disable clk's
	for (i = dwc->num_clocks - 1; i >= 0; i--)  {
		clk_disable_unprepare(dwc->clks[i]);
		clk_put(dwc->clks[i]);
	}

	platform_set_drvdata(pdev, NULL);

	return 0;
}

static const struct of_device_id be_dwc3_of_match[] = {
	{ .compatible = "be,baikal-dwc3", },
	{},
};

MODULE_DEVICE_TABLE(of, be_dwc3_of_match);

static const struct of_device_id usb_extcon_dt_match[] = {
  { . compatible = "linux,extcon-usb-gpio", },
  { },
};

static struct platform_driver be_dwc3_driver = {
	.probe		= be_dwc3_probe,
	.remove		= be_dwc3_remove,
	.driver		= {
		.name	= "baikal-dwc3",
		.of_match_table	= be_dwc3_of_match,
	},
	.id_table = usb_extcon_platform_dt_match,
};

module_platform_driver(be_dwc3_driver);

MODULE_ALIAS("platform:baikal-dwc3");
MODULE_AUTHOR("Dmitry Dunaev <dmitry.dunaev@baikalelectronics.ru>");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("DesignWare USB3 Baikal SoCs Glue Layer");
