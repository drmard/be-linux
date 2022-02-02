// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2015,2016,2020,2021 Baikal Electronics JSC
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

#define CMU_PLL_SET_RATE		0
#define CMU_PLL_GET_RATE		1
#define CMU_PLL_ENABLE			2
#define CMU_PLL_DISABLE			3
#define CMU_PLL_ROUND_RATE		4
#define CMU_PLL_IS_ENABLED		5
#define CMU_CLK_CH_SET_RATE		6
#define CMU_CLK_CH_GET_RATE		7
#define CMU_CLK_CH_ENABLE		8
#define CMU_CLK_CH_DISABLE		9
#define CMU_CLK_CH_ROUND_RATE		10
#define CMU_CLK_CH_IS_ENABLED		11

struct baikal_clk_cmu {
	struct clk_hw	hw;
	uint32_t	cmu_id;
	unsigned int	parent;
	const char	*name;
	spinlock_t	*lock;
	void __iomem	*reg;
	unsigned int	latency; /* ns */
	unsigned int	min, max, step;
	unsigned int	clk_ch_num;
	uint32_t	is_clk_ch;
};

#define to_baikal_cmu(_hw) container_of(_hw, struct baikal_clk_cmu, hw)

/* Pointer to the place on handling SMC CMU calls in monitor */
#define BAIKAL_SMC_LCRU_ID	0x82000000

static int baikal_clk_enable(struct clk_hw *hw)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_ENABLE;
	} else {
		cmd = CMU_PLL_ENABLE;
	}

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, 0,
			pclk->parent, 0, 0, 0, &res);

	pr_debug("%s(%s, %s@0x%x): %s\n",
			__func__,
			pclk->name,
			pclk->is_clk_ch ? "clkch" : "pll",
			pclk->cmu_id,
			res.a0 ? "error" : "ok");

	return res.a0;
}

static void baikal_clk_disable(struct clk_hw *hw)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_DISABLE;
	} else {
		cmd = CMU_PLL_DISABLE;
	}

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, 0,
			pclk->parent, 0, 0, 0, &res);

	pr_debug("%s(%s, %s@0x%x): %s\n",
			__func__,
			pclk->name,
			pclk->is_clk_ch ? "clkch" : "pll",
			pclk->cmu_id,
			res.a0 ? "error" : "ok");
}

static int baikal_clk_is_enabled(struct clk_hw *hw)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_IS_ENABLED;
	} else {
		cmd = CMU_PLL_IS_ENABLED;
	}

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, 0,
			pclk->parent, 0, 0, 0, &res);

	pr_debug("%s(%s, %s@0x%x): %s\n",
			__func__,
			pclk->name,
			pclk->is_clk_ch ? "clkch" : "pll",
			pclk->cmu_id,
			res.a0 ? "true" : "false");

	return res.a0;
}

static unsigned long baikal_clk_recalc_rate(struct clk_hw *hw,
						unsigned long parent_rate)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;
	unsigned long parent;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_GET_RATE;
		parent = pclk->parent;
	} else {
		cmd = CMU_PLL_GET_RATE;
		parent= parent_rate;
	}

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, 0,
			parent, 0, 0, 0, &res);

	pr_debug("%s(%s, %s@0x%x): %ld Hz\n",
			__func__,
			pclk->name,
			pclk->is_clk_ch ? "clkch" : "pll",
			pclk->cmu_id,
			res.a0);

	/* Return actual freq */
	return res.a0;
}

static int baikal_clk_set_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long parent_rate)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	uint32_t cmd;
	unsigned long parent;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_SET_RATE;
		parent = pclk->parent;
	} else {
		cmd = CMU_PLL_SET_RATE;
		parent = parent_rate;
	}

	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, rate,
			parent, 0, 0, 0, &res);

	pr_debug("%s(%s, %s@0x%x, %ld Hz): %s\n",
			__func__,
			pclk->name,
			pclk->is_clk_ch ? "clkch" : "pll",
			pclk->cmu_id,
			rate,
			res.a0 ? "error" : "ok");

	return res.a0;
}

static long baikal_clk_round_rate(struct clk_hw *hw, unsigned long rate,
					unsigned long *prate)
{
	struct arm_smccc_res res;
	struct baikal_clk_cmu *pclk = to_baikal_cmu(hw);
	unsigned long parent;
	uint32_t cmd;

	if (pclk->is_clk_ch) {
		cmd = CMU_CLK_CH_ROUND_RATE;
		parent = pclk->parent;
	} else {
		cmd = CMU_PLL_ROUND_RATE;
		parent = *prate;
	}

	/* If clock valid */
	arm_smccc_smc(BAIKAL_SMC_LCRU_ID, pclk->cmu_id, cmd, rate,
			parent, 0, 0, 0, &res);

	pr_debug("%s(%s, %s@0x%x): %ld Hz\n",
			__func__,
			pclk->name,
			pclk->is_clk_ch ? "clkch" : "pll",
			pclk->cmu_id,
			res.a0);

	/* Return actual freq */
	return res.a0;
}

const struct clk_ops be_clk_pll_ops = {
	.enable = baikal_clk_enable,
	.disable = baikal_clk_disable,
	.is_enabled = baikal_clk_is_enabled,
	.recalc_rate = baikal_clk_recalc_rate,
	.set_rate = baikal_clk_set_rate,
	.round_rate = baikal_clk_round_rate
};

static int baikal_clk_probe(struct platform_device *pdev)
{
	struct clk_init_data init;
	struct clk_init_data *init_ch;
	struct baikal_clk_cmu *cmu;
	struct baikal_clk_cmu **cmu_ch;
	struct device_node *node = pdev->dev.of_node;

	struct clk *clk;
	struct clk_onecell_data *clk_ch;

	int number, i = 0;
	u32 rc, index;
	struct property *prop;
	const __be32 *p;
	const char *clk_ch_name;
	const char *parent_name;

	cmu = kmalloc(sizeof(struct baikal_clk_cmu *), GFP_KERNEL);
	if (!cmu) {
		pr_err("%s: could not allocate CMU clk\n", __func__);
		kfree(cmu);
		return -ENOMEM;
	}

	of_property_read_string(node, "clock-output-names", &cmu->name);
	of_property_read_u32(node, "clock-frequency", &cmu->parent);
	of_property_read_u32(node, "cmu-id", &cmu->cmu_id);

	parent_name = of_clk_get_parent_name(node, 0);

	/* Setup clock init structure */
	init.parent_names = &parent_name;
	init.num_parents = 1;
	init.name = cmu->name;
	init.ops = &be_clk_pll_ops;
	init.flags = CLK_IGNORE_UNUSED;

	cmu->hw.init = &init;
	cmu->is_clk_ch = 0;

	/* Register the clock */
	pr_debug("%s: add %s, parent %s\n", __func__, cmu->name, parent_name ? parent_name : "null");
	clk = clk_register(NULL, &cmu->hw);

	if (IS_ERR(clk)) {
		pr_err("%s: could not register clk %s\n", __func__, cmu->name);
		return -ENOMEM;
	}

	/* Register the clock for lookup */
	rc = clk_register_clkdev(clk, cmu->name, NULL);
	if (rc != 0) {
		pr_err("%s: could not register lookup clk %s\n",
			__func__, cmu->name);
	}

	/* FIXME We probably SHOULDN'T enable it here */
	clk_prepare_enable(clk);

	number = of_property_count_u32_elems(node, "clock-indices");

	if (number > 0) {
		clk_ch = kmalloc(sizeof(struct clk_onecell_data), GFP_KERNEL);
 		if (!clk_ch) {
			pr_err("%s: could not allocate CMU clk channel\n", __func__);
			return -ENOMEM;
 		}

		/* Get the last index to find out max number of children*/
		of_property_for_each_u32(node, "clock-indices", prop, p, index) {
			;
		}

		clk_ch->clks = kcalloc(index + 1, sizeof(struct clk *), GFP_KERNEL);
		clk_ch->clk_num = index + 1;
		cmu_ch = kcalloc((index + 1), sizeof(struct baikal_clk_cmu *), GFP_KERNEL);
		if (!cmu_ch) {
			kfree(clk_ch);
			return -ENOMEM;
		}
		init_ch = kcalloc((number + 1), sizeof(struct clk_init_data), GFP_KERNEL);
		if (!init_ch) {
			pr_err("%s: could not allocate CMU init structure \n", __func__);
			kfree(cmu_ch);
			kfree(clk_ch);
			return -ENOMEM;
		}

		of_property_for_each_u32(node, "clock-indices", prop, p, index) {
			of_property_read_string_index(node, "clock-names",
							i, &clk_ch_name);
			pr_debug("%s: clkch <%s>, index %d, i %d\n", __func__, clk_ch_name, index, i);
			init_ch[i].parent_names = &cmu->name;
			init_ch[i].num_parents = 1;
			init_ch[i].name = clk_ch_name;
			init_ch[i].ops = &be_clk_pll_ops;
			init_ch[i].flags = CLK_IGNORE_UNUSED;

			cmu_ch[index] = kmalloc(sizeof(struct baikal_clk_cmu), GFP_KERNEL);
			cmu_ch[index]->name = clk_ch_name;
			cmu_ch[index]->cmu_id = index;
			cmu_ch[index]->parent = cmu->cmu_id;
			cmu_ch[index]->is_clk_ch = 1;
			cmu_ch[index]->hw.init = &init_ch[i];
			clk_ch->clks[index] = clk_register(NULL, &cmu_ch[index]->hw);

			if (IS_ERR(clk_ch->clks[index])) {
				pr_err("%s: could not register clk %s\n", __func__, clk_ch_name);
			}

			/* Register the clock for lookup */
			rc = clk_register_clkdev(clk_ch->clks[index], clk_ch_name, NULL);
			if (rc != 0) {
				pr_err("%s: could not register lookup clk %s\n",
					__func__, clk_ch_name);
			}

			/* FIXME We probably SHOULDN'T enable it here */
			clk_prepare_enable(clk_ch->clks[index]);
			i++;
		}

		return of_clk_add_provider(pdev->dev.of_node, of_clk_src_onecell_get, clk_ch);
	}

	return of_clk_add_provider(pdev->dev.of_node, of_clk_src_simple_get, clk);
}

static int baikal_clk_remove(struct platform_device *pdev)
{
	of_clk_del_provider(pdev->dev.of_node);

	return 0;
}

static const struct of_device_id baikal_clk_of_match[] = {
	{ .compatible = "baikal,cmu" },
	{ /* sentinel */ }
};

static struct platform_driver clk_avlsp_cmu0_driver = {
	.probe	= baikal_clk_probe,
	.remove	= baikal_clk_remove,
	.driver	= {
		.name = "baikal-avlsp-cmu0",
		.of_match_table = baikal_clk_of_match
	}
};

static struct platform_driver clk_avlsp_cmu1_driver = {
	.probe	= baikal_clk_probe,
	.remove	= baikal_clk_remove,
	.driver	= {
		.name = "baikal-avlsp-cmu1",
		.of_match_table = baikal_clk_of_match
	}
};

static struct platform_driver clk_ca57_cmu_driver = {
	.probe	= baikal_clk_probe,
	.remove	= baikal_clk_remove,
	.driver	= {
		.name = "baikal-ca57_cmu",
		.of_match_table = baikal_clk_of_match
	}
};

static struct platform_driver clk_mali_cmu_driver = {
	.probe	= baikal_clk_probe,
	.remove	= baikal_clk_remove,
	.driver	= {
		.name = "baikal-mali-cmu",
		.of_match_table = baikal_clk_of_match
	}
};

static struct platform_driver clk_xgbe_cmu0_driver = {
	.probe	= baikal_clk_probe,
	.remove	= baikal_clk_remove,
	.driver	= {
		.name = "baikal-xgbe-cmu0",
		.of_match_table = baikal_clk_of_match
	}
};

static struct platform_driver clk_xgbe_cmu1_driver = {
	.probe	= baikal_clk_probe,
	.remove	= baikal_clk_remove,
	.driver	= {
		.name = "baikal-xgbe-cmu1",
		.of_match_table = baikal_clk_of_match
	}
};

#ifdef CONFIG_ACPI

const char *baikal_acpi_clk_osc25_str[] = { "baikal_osc25" };
const char *baikal_acpi_clk_osc27_str[] = { "baikal_osc27" };

static struct clk *baikal_acpi_clk_osc25;
static struct clk *baikal_acpi_clk_osc27;

struct baikal_acpi_clk_data {
	struct clk *cmu_clk;
	struct clk_lookup *cmu_clk_l;
	struct clk **clk;
	struct clk_lookup **clk_l;
	unsigned int clk_num;
};

static int baikal_acpi_clk_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct acpi_device *ref_dev, *adev = to_acpi_device_node(pdev->dev.fwnode);
	struct clk_init_data init, *init_ch;
	struct baikal_clk_cmu *cmu, *cmu_ch;
	struct baikal_acpi_clk_data *clk_data = NULL;
        union acpi_object *package, *element;
        acpi_status status;
        struct acpi_buffer buffer = { ACPI_ALLOCATE_BUFFER, NULL };
	int osc27, size, i, index, ret = 0;
	char *str, *str2;

	cmu = devm_kzalloc(dev, sizeof(*cmu), GFP_KERNEL);
	if (!cmu)
		return -ENOMEM;

        status = acpi_evaluate_object_typed(adev->handle, "PROP", NULL, &buffer, ACPI_TYPE_PACKAGE);
        if (ACPI_FAILURE(status)) {
                dev_err(dev, "failed to get PROP data\n");
                return -ENODEV;
        }

        package = buffer.pointer;
        if (package->package.count != 4) {
                dev_err(dev, "invalid PROP data\n");
		ret = -EINVAL;
		goto ret;
        }

	element = &(package->package.elements[0]);
        if (element->type != ACPI_TYPE_INTEGER) {
                dev_err(dev, "failed to get CMU id\n");
		ret = -EINVAL;
		goto ret;
        }
        cmu->cmu_id = element->integer.value;

	element = &(package->package.elements[1]);
        if (element->type != ACPI_TYPE_STRING) {
                dev_err(dev, "failed to get CMU clock name\n");
		ret = -EINVAL;
		goto ret;
        }
	str = devm_kzalloc(dev, element->string.length + 1, GFP_KERNEL);
	if (!str) {
		ret = -ENOMEM;
		goto ret;
	}
	memcpy(str, element->string.pointer, element->string.length);
	cmu->name = str;

	element = &(package->package.elements[2]);
        if (element->type != ACPI_TYPE_INTEGER) {
                dev_err(dev, "failed to get CMU frequency\n");
		ret = -EINVAL;
		goto ret;
        }
        cmu->parent = element->integer.value;

	element = &(package->package.elements[3]);
        if (element->type != ACPI_TYPE_INTEGER) {
                dev_err(dev, "failed to get CMU osc type\n");
		ret = -EINVAL;
		goto ret;
        }
        osc27 = element->integer.value ? 1 : 0;

        acpi_os_free(buffer.pointer);
	buffer.length = ACPI_ALLOCATE_BUFFER;
	buffer.pointer = NULL;

	init.parent_names = osc27 ? baikal_acpi_clk_osc27_str : baikal_acpi_clk_osc25_str;
	init.num_parents = 1;
	init.name = cmu->name;
	init.ops = &be_clk_pll_ops;
	init.flags = CLK_IGNORE_UNUSED;

	cmu->hw.init = &init;
	cmu->is_clk_ch = 0;

	clk_data = devm_kzalloc(dev, sizeof(*clk_data), GFP_KERNEL);
	if (!clk_data)
		return -ENOMEM;

	clk_data->cmu_clk = clk_register(NULL, &cmu->hw);
	if (IS_ERR(clk_data->cmu_clk)) {
		dev_err(dev, "failed to register CMU clock\n");
		return PTR_ERR(clk_data->cmu_clk);
	}
	clk_data->cmu_clk_l = clkdev_create(clk_data->cmu_clk, cmu->name, NULL);
	if (!clk_data->cmu_clk_l) {
		dev_err(dev, "failed to register CMU clock lookup\n");
		clk_unregister(clk_data->cmu_clk);
		return -ENOMEM;
	}
	clk_prepare_enable(clk_data->cmu_clk);

	platform_set_drvdata(pdev, clk_data);

        status = acpi_evaluate_object_typed(adev->handle, "CLKS", NULL, &buffer, ACPI_TYPE_PACKAGE);
        if (ACPI_FAILURE(status)) {
		buffer.pointer = NULL;
		goto ret;
	}

        package = buffer.pointer;
        if (!package->package.count || package->package.count % 4) {
                dev_err(dev, "invalid CLKS data\n");
		ret = -EINVAL;
		goto ret;
        }
	clk_data->clk_num = package->package.count >> 2;

	clk_data->clk = devm_kzalloc(dev, clk_data->clk_num * sizeof(struct clk *), GFP_KERNEL);
	if (!clk_data->clk) {
		ret = -ENOMEM;
		goto ret;
	}

	clk_data->clk_l = devm_kzalloc(dev, clk_data->clk_num * sizeof(struct clk_lookup *), GFP_KERNEL);
	if (!clk_data->clk_l) {
		ret = -ENOMEM;
		goto ret;
	}

	init_ch = devm_kzalloc(dev, clk_data->clk_num * sizeof(struct clk_init_data), GFP_KERNEL);
	if (!init_ch) {
		ret = -ENOMEM;
		goto ret;
	}

	cmu_ch = devm_kzalloc(dev, clk_data->clk_num * sizeof(struct baikal_clk_cmu), GFP_KERNEL);
	if (!cmu_ch) {
		ret = -ENOMEM;
		goto ret;
	}

	for (i = 0; i < clk_data->clk_num; ++i) {
		ref_dev = NULL;
		size = 0;

		element = &(package->package.elements[4 * i]);
		if (element->type == ACPI_TYPE_LOCAL_REFERENCE && element->reference.handle) {
			if (acpi_bus_get_device(element->reference.handle, &ref_dev))
				ref_dev = NULL;
		}

		element = &(package->package.elements[4 * i + 1]);
		if (element->type == ACPI_TYPE_STRING) {
			if (ref_dev)
				size = strlen(dev_name(&ref_dev->dev)) + 1;

			str = devm_kzalloc(dev, size + element->string.length + 1, GFP_KERNEL);
			if (str) {
				if (ref_dev) {
					memcpy(str, dev_name(&ref_dev->dev), size - 1);
					str[size - 1] = '_';
					memcpy(str + size, element->string.pointer, element->string.length);
				} else
					memcpy(str, element->string.pointer, element->string.length);
			}
	        } else {
			dev_err(dev, "failed to process clock device name #%i\n", i);
			continue;
		}

		element = &(package->package.elements[4 * i + 2]);
		if (element->type == ACPI_TYPE_INTEGER) {
			index = element->integer.value;
		} else {
			dev_err(dev, "failed to process clock device id #%i\n", i);
			continue;
		}

		element = &(package->package.elements[4 * i + 3]);
		if (element->type == ACPI_TYPE_STRING) {
			str2 = devm_kzalloc(dev, element->string.length + 1, GFP_KERNEL);
			if (str2)
				memcpy(str2, element->string.pointer, element->string.length);
	        } else
			str2 = NULL;

		init_ch[i].parent_names = &cmu->name;
		init_ch[i].num_parents = 1;
		init_ch[i].name = str;
		init_ch[i].ops = &be_clk_pll_ops;
		init_ch[i].flags = CLK_IGNORE_UNUSED;

		cmu_ch[i].name = str;
		cmu_ch[i].cmu_id = i;
		cmu_ch[i].parent = cmu->cmu_id;
		cmu_ch[i].is_clk_ch = 1;
		cmu_ch[i].hw.init = &init_ch[i];

		clk_data->clk[i] = clk_register(ref_dev ? &ref_dev->dev : NULL, &cmu_ch[i].hw);
		if (IS_ERR(clk_data->clk[i])) {
			dev_err(dev, "failed to register CMU channel clock #%i\n", i);
			clk_data->clk[i] = NULL;
			continue;
		}

		if (ref_dev)
			clk_data->clk_l[i] = clkdev_create(clk_data->clk[i], str2, "%s", dev_name(&ref_dev->dev));
		else
			clk_data->clk_l[i] = clkdev_create(clk_data->clk[i], str2, NULL);
		if (!clk_data->clk_l[i]) {
			dev_err(dev, "failed to register CMU channel clock lookup #%i\n", i);
			clk_unregister(clk_data->clk[i]);
			clk_data->clk[i] = NULL;
			continue;
		}
	}

	clk_data = NULL;

ret:
	if (buffer.pointer)
		acpi_os_free(buffer.pointer);
	if (clk_data) {
		clk_disable_unprepare(clk_data->cmu_clk);
		clkdev_drop(clk_data->cmu_clk_l);
		clk_unregister(clk_data->cmu_clk);
	}
	return ret;
}

static int baikal_acpi_clk_remove(struct platform_device *pdev)
{
	struct baikal_acpi_clk_data *clk_data = platform_get_drvdata(pdev);
	int i;

	if (clk_data) {
		clk_disable_unprepare(clk_data->cmu_clk);
		clkdev_drop(clk_data->cmu_clk_l);
		clk_unregister(clk_data->cmu_clk);

		for (i = 0; i < clk_data->clk_num; ++i) {
			if (clk_data->clk_l[i])
				clkdev_drop(clk_data->clk_l[i]);
			if (clk_data->clk[i])
				clk_unregister(clk_data->clk[i]);
		}
	}

	return 0;
}

static const struct acpi_device_id baikal_acpi_clk_device_ids[] = {
	{ "BKLE0001" },
	{ }
};

static struct platform_driver baikal_acpi_clk_driver = {
	.probe		= baikal_acpi_clk_probe,
	.remove		= baikal_acpi_clk_remove,
	.driver		= {
		.name	= "baikal_acpi_clk",
		.acpi_match_table = ACPI_PTR(baikal_acpi_clk_device_ids)
	}
};

static int __init baikal_acpi_clk_driver_init(void)
{
	struct clk_lookup *baikal_acpi_clk_lookup_osc25;
	struct clk_lookup *baikal_acpi_clk_lookup_osc27;

	baikal_acpi_clk_osc25 = clk_register_fixed_rate(NULL, baikal_acpi_clk_osc25_str[0], NULL, 0, 25000000);
	if (IS_ERR(baikal_acpi_clk_osc25)) {
		pr_err("%s: failed to register osc25 clock\n", __func__);
		return PTR_ERR(baikal_acpi_clk_osc25);
	}

	baikal_acpi_clk_osc27 = clk_register_fixed_rate(NULL, baikal_acpi_clk_osc27_str[0], NULL, 0, 27000000);
	if (IS_ERR(baikal_acpi_clk_osc27)) {
		clk_unregister_fixed_rate(baikal_acpi_clk_osc25);
		pr_err("%s: failed to register osc27 clock\n", __func__);
		return PTR_ERR(baikal_acpi_clk_osc27);
	}

	baikal_acpi_clk_lookup_osc25 = clkdev_create(baikal_acpi_clk_osc25, NULL, "%s", baikal_acpi_clk_osc25_str[0]);
	if (!baikal_acpi_clk_lookup_osc25) {
		clk_unregister_fixed_rate(baikal_acpi_clk_osc27);
		clk_unregister_fixed_rate(baikal_acpi_clk_osc25);
		pr_err("%s: failed to register osc25 clock lookup\n", __func__);
		return -ENOMEM;
	}

	baikal_acpi_clk_lookup_osc27 = clkdev_create(baikal_acpi_clk_osc27, NULL, "%s", baikal_acpi_clk_osc27_str[0]);
	if (!baikal_acpi_clk_lookup_osc27) {
		clkdev_drop(baikal_acpi_clk_lookup_osc25);
		clk_unregister_fixed_rate(baikal_acpi_clk_osc27);
		clk_unregister_fixed_rate(baikal_acpi_clk_osc25);
		pr_err("%s: failed to register osc27 clock lookup\n", __func__);
		return -ENOMEM;
	}

	clk_prepare_enable(baikal_acpi_clk_osc25);
	clk_prepare_enable(baikal_acpi_clk_osc27);

	return platform_driver_register(&baikal_acpi_clk_driver);
}

device_initcall(baikal_acpi_clk_driver_init);

#endif

module_platform_driver(clk_avlsp_cmu0_driver);
module_platform_driver(clk_avlsp_cmu1_driver);
module_platform_driver(clk_ca57_cmu_driver);
module_platform_driver(clk_mali_cmu_driver);
module_platform_driver(clk_xgbe_cmu0_driver);
module_platform_driver(clk_xgbe_cmu1_driver);

MODULE_DESCRIPTION("Baikal BE-M1000 clock driver");
MODULE_AUTHOR("Ekaterina Skachko <ekaterina.skachko@baikalelectronics.ru>");
MODULE_LICENSE("GPL v2");
MODULE_ALIAS("platform:bm1000-cmu");
