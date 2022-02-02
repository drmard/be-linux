// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2021 Baikal Electronics JSC
 * Author: Ekaterina Skachko <ekaterina.skachko@baikalelectronics.ru>
 */

#include <asm/setup.h>
#include <linux/acpi.h>
#include <linux/arm-smccc.h>
#include <linux/clk-provider.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/spinlock.h>

#define BAIKAL_SMC_CLK			(0x82000000 + 0x400)
#define BAIKAL_SMC_CLK_ROUND		(BAIKAL_SMC_CLK + 0)
#define BAIKAL_SMC_CLK_SET		(BAIKAL_SMC_CLK + 1)
#define BAIKAL_SMC_CLK_GET		(BAIKAL_SMC_CLK + 2)
#define BAIKAL_SMC_CLK_ENABLE		(BAIKAL_SMC_CLK + 3)
#define BAIKAL_SMC_CLK_DISABLE		(BAIKAL_SMC_CLK + 4)
#define BAIKAL_SMC_CLK_IS_ENABLED	(BAIKAL_SMC_CLK + 5)

struct baikal_clk {
	struct clk_hw hw;
	uint32_t base;
};
#define to_baikal_clk(_hw) container_of(_hw, struct baikal_clk, hw)

//--------------------
// SMC
//--------------------
static int baikal_clk_enable(struct clk_hw *hw)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_ENABLE, clk->base, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static void baikal_clk_disable(struct clk_hw *hw)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_DISABLE, clk->base, 0, 0, 0, 0, 0, 0, &res);
}

static int baikal_clk_is_enabled(struct clk_hw *hw)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_IS_ENABLED, clk->base, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static unsigned long baikal_clk_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_GET, clk->base, 0, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static int baikal_clk_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_SET, clk->base, rate, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static long baikal_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	struct baikal_clk *clk = to_baikal_clk(hw);
	struct arm_smccc_res res;
	arm_smccc_smc(BAIKAL_SMC_CLK_ROUND, clk->base, rate, 0, 0, 0, 0, 0, &res);
	return res.a0;
}

static const struct clk_ops baikal_clk_ops = {
	.enable      = baikal_clk_enable,
	.disable     = baikal_clk_disable,
	.is_enabled  = baikal_clk_is_enabled,
	.recalc_rate = baikal_clk_recalc_rate,
	.set_rate    = baikal_clk_set_rate,
	.round_rate  = baikal_clk_round_rate
};

static int baikal_clk_probe(struct platform_device *pdev)
{
	struct device_node *node = pdev->dev.of_node;
	struct clk_init_data init;
	struct baikal_clk *cmu;
	struct clk_onecell_data *clk_data;
	const char *clk_name;
	struct property *prop;
	const __be32 *p;
	int clk_index;
	int clk_index_max;
	int clk_index_cnt;
	int clk_name_cnt;
	int clk_cnt;
	int i;
	uint32_t base;
	struct clk *clk;
	int ret;
	int multi;

	// base
	ret = of_property_read_u32(node, "reg", &base);
	if (IS_ERR(ret)) {
		base = 0;
	}

	// cnt
	clk_index_cnt = of_property_count_u32_elems(node, "clock-indices");
	clk_name_cnt = of_property_count_strings (node, "clock-output-names");
	clk_cnt = clk_index_cnt > clk_name_cnt ? clk_index_cnt : clk_name_cnt;
	if (IS_ERR(clk_cnt)) {
		clk_cnt = 1;
	}
	multi = clk_cnt > 1;

	if (multi) {
		clk_index_max = clk_cnt - 1;
		of_property_for_each_u32(node, "clock-indices", prop, p, clk_index) {
			if (clk_index_max < clk_index) {
				clk_index_max = clk_index;
			}
		}
		clk_data = kzalloc(sizeof(*clk_data), GFP_KERNEL);
		clk_data->clks = kcalloc(clk_index_max + 1, sizeof(struct clk*), GFP_KERNEL);
		clk_data->clk_num = clk_index_max + 1;
	}

	for (i = 0; i < clk_cnt; i++) {

		ret = of_property_read_u32_index (node, "clock-indices", i, &clk_index);
		if (IS_ERR(ret)) {
			clk_index = i;
		}
		ret = of_property_read_string_index (node, "clock-output-names", i, &clk_name);
		if (IS_ERR(ret)) {
			if (multi)
				init.name = kasprintf(GFP_KERNEL, "%s.%d", node->name, clk_index);
			else
				init.name = kasprintf(GFP_KERNEL, "%s",    node->name);
		} else {
				init.name = kasprintf(GFP_KERNEL, "%s.%s", node->name, clk_name);
		}

		init.ops = &baikal_clk_ops;
		init.flags = CLK_IGNORE_UNUSED;
		init.parent_names = NULL;
		init.num_parents = 0;

		cmu = kmalloc(sizeof(*cmu), GFP_KERNEL);
		cmu->base = base + 0x10 * clk_index;
		cmu->hw.init = &init;

		clk = clk_register(NULL, &cmu->hw);
		if (!IS_ERR(clk)) {
			clk_register_clkdev(clk, init.name, NULL);
			if (multi) {
				clk_data->clks[clk_index] = clk;
			}
		}
	}

	// add
	if (multi) {
		ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get, clk_data);
	} else {
		ret = of_clk_add_provider(pdev->dev.of_node, of_clk_src_simple_get, clk);
	}
	return ret;
}

static int baikal_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);
	return 0;
}

static const struct of_device_id baikal_clk_of_match[] = {
	{ .compatible = "baikal,bs1000-cmu" },
	{ /* sentinel */ }
};

static struct platform_driver cmu_driver = {
	.probe	= baikal_clk_probe,
	.remove	= baikal_clk_remove,
	.driver	= {
		.name = "bs1000-cmu",
		.of_match_table = baikal_clk_of_match
	}
};
module_platform_driver(cmu_driver);

MODULE_DESCRIPTION("Baikal BE-S1000 clock driver");
MODULE_AUTHOR("Ekaterina Skachko <ekaterina.skachko@baikalelectronics.ru>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bs1000-cmu");
