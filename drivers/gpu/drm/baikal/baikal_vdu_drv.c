/*
 * Copyright (C) 2019-2020 Baikal Electronics JSC
 *
 * Author: Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>
 *
 * This driver is based on ARM PL111 DRM driver
 *
 * Parts of this file were based on sources as follows:
 *
 * Copyright (c) 2006-2008 Intel Corporation
 * Copyright (c) 2007 Dave Airlie <airlied@linux.ie>
 * Copyright (C) 2011 Texas Instruments
 * (C) COPYRIGHT 2012-2013 ARM Limited. All rights reserved.
 *
 * This program is free software and is provided to you under the terms of the
 * GNU General Public License version 2 as published by the Free Software
 * Foundation, and any use by you of this program is subject to the terms of
 * such GNU licence.
 *
 */

#include <linux/arm-smccc.h>
#include <linux/irq.h>
#include <linux/clk.h>
#include <linux/version.h>
#include <linux/shmem_fs.h>
#include <linux/dma-buf.h>
#include <linux/module.h>
#include <linux/slab.h>
#include <linux/workqueue.h>
#include <linux/fb.h>

#include <drm/drmP.h>
#include <drm/drm_atomic_helper.h>
#include <drm/drm_connector.h>
#include <drm/drm_crtc_helper.h>
#include <drm/drm_gem_cma_helper.h>
#include <drm/drm_fb_cma_helper.h>
#include <drm/drm_fb_helper.h>

#include "baikal_vdu_drm.h"
#include "baikal_vdu_regs.h"

#define DRIVER_NAME                 "baikal-vdu"
#define DRIVER_DESC                 "DRM module for Baikal VDU"
#define DRIVER_DATE                 "20200131"

#define BAIKAL_SMC_SCP_LOG_DISABLE  0x82000200

int mode_fixup = 0;

static void baikal_vdu_fb_output_poll_changed(struct drm_device *dev)
{
	struct baikal_vdu_private *priv = dev->dev_private;
	drm_fbdev_cma_hotplug_event(priv->fbdev);
}

static struct drm_mode_config_funcs mode_config_funcs = {
	.fb_create = drm_fb_cma_create,
	.output_poll_changed = baikal_vdu_fb_output_poll_changed,
	.atomic_check = drm_atomic_helper_check,
	.atomic_commit = drm_atomic_helper_commit,
};

static int baikal_vdu_remove_efifb(struct drm_device *dev)
{
	int err;
	struct apertures_struct *a;
	a = alloc_apertures(1);
	if (!a) {
		err = -ENOMEM;
		dev_warn(dev->dev, "failed to allocate apertures\n");
		goto out;
	}
	a->ranges[0].base = 0;
	a->ranges[0].size = ~0;
	err = drm_fb_helper_remove_conflicting_framebuffers(a, "baikal-vdudrmfb", false);
	if (err) {
		dev_warn(dev->dev, "failed to remove firmware framebuffer\n");
	}
	kfree(a);
out:
	return err;
}

static int vdu_modeset_init(struct drm_device *dev)
{
	struct drm_mode_config *mode_config;
	struct baikal_vdu_private *priv = dev->dev_private;
	struct arm_smccc_res res;
	int ret = 0;

	if (priv == NULL)
		return -EINVAL;

	drm_mode_config_init(dev);
	mode_config = &dev->mode_config;
	mode_config->funcs = &mode_config_funcs;
	mode_config->min_width = 1;
	mode_config->max_width = 4096;
	mode_config->min_height = 1;
	mode_config->max_height = 4096;

	ret = baikal_vdu_primary_plane_init(dev);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to init primary plane\n");
		goto out_config;
	}

	ret = baikal_vdu_crtc_create(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to create crtc\n");
		goto out_config;
	}

	ret = get_panel_or_bridge(dev->dev, &priv->connector.panel, &priv->bridge);
	if (ret == -EPROBE_DEFER) {
		dev_info(dev->dev, "Bridge probe deferred\n");
		goto out_config;
	}

	ret = baikal_vdu_encoder_init(dev);
	if (ret) {
		dev_err(dev->dev, "Failed to create DRM encoder\n");
		goto out_config;
	}

	if (priv->bridge) {
		priv->bridge->encoder = &priv->encoder;
		priv->encoder.bridge = priv->bridge;
		ret = drm_bridge_attach(priv->encoder.dev, priv->bridge);
		if (ret) {
			dev_err(dev->dev, "Failed to attach DRM bridge %d\n", ret);
			goto out_config;
		}
	} else if (priv->connector.panel) {
		ret = baikal_vdu_connector_create(dev);
		if (ret) {
			dev_err(dev->dev, "Failed to create DRM connector\n");
			goto out_config;
		}
		ret = drm_mode_connector_attach_encoder(&priv->connector.connector,
						&priv->encoder);
		if (ret != 0) {
			dev_err(dev->dev, "Failed to attach encoder\n");
			goto out_config;
		}
	} else
		ret = -EINVAL;

	if (ret) {
		dev_err(dev->dev, "No bridge or panel attached!\n");
		goto out_config;
	}

	priv->clk = clk_get(dev->dev, "pclk");
	if (IS_ERR(priv->clk)) {
		dev_err(dev->dev, "fatal: unable to get pclk, err %ld\n", PTR_ERR(priv->clk));
		ret = PTR_ERR(priv->clk);
		goto out_config;
	}

	priv->mode_fixup = mode_fixup;

	baikal_vdu_remove_efifb(dev);

	ret = drm_vblank_init(dev, 1);
	if (ret != 0) {
		dev_err(dev->dev, "Failed to init vblank\n");
		goto out_clk;
	}

	arm_smccc_smc(BAIKAL_SMC_SCP_LOG_DISABLE, 0, 0, 0, 0, 0, 0, 0, &res);
	INIT_DEFERRABLE_WORK(&priv->update_work,
			     baikal_vdu_update_work);

	drm_mode_config_reset(dev);

	priv->fbdev = drm_fbdev_cma_init(dev, 32,
			dev->mode_config.num_crtc,
			dev->mode_config.num_connector);

	drm_kms_helper_poll_init(dev);

	goto finish;

out_clk:
	clk_put(priv->clk);
out_config:
	drm_mode_config_cleanup(dev);
finish:
	return ret;
}

static const struct file_operations drm_fops = {
	.owner = THIS_MODULE,
	.open = drm_open,
	.release = drm_release,
	.unlocked_ioctl = drm_ioctl,
	.mmap = drm_gem_cma_mmap,
	.poll = drm_poll,
	.read = drm_read,
};

static void vdu_lastclose(struct drm_device *dev)
{
	struct baikal_vdu_private *priv = dev->dev_private;

	drm_fbdev_cma_restore_mode(priv->fbdev);
}

static struct drm_driver vdu_drm_driver = {
	.driver_features = DRIVER_HAVE_IRQ | DRIVER_GEM |
			DRIVER_MODESET | DRIVER_PRIME | DRIVER_ATOMIC,
	.lastclose = vdu_lastclose,
	.irq_handler = baikal_vdu_irq,
	.ioctls = NULL,
	.fops = &drm_fops,
	.name = DRIVER_NAME,
	.desc = DRIVER_DESC,
	.date = DRIVER_DATE,
	.major = 1,
	.minor = 0,
	.patchlevel = 0,
	.dumb_create = baikal_vdu_dumb_create,
	.dumb_destroy = drm_gem_dumb_destroy,
	.dumb_map_offset = drm_gem_cma_dumb_map_offset,
	.gem_free_object = drm_gem_cma_free_object,
	.gem_vm_ops = &drm_gem_cma_vm_ops,

	.enable_vblank = baikal_vdu_enable_vblank,
	.disable_vblank = baikal_vdu_disable_vblank,

	.get_vblank_counter = drm_vblank_no_hw_counter,

	.prime_handle_to_fd = drm_gem_prime_handle_to_fd,
	.prime_fd_to_handle = drm_gem_prime_fd_to_handle,
	.gem_prime_import = drm_gem_prime_import,
	.gem_prime_import_sg_table = drm_gem_cma_prime_import_sg_table,
	.gem_prime_export = drm_gem_prime_export,
	.gem_prime_get_sg_table	= drm_gem_cma_prime_get_sg_table,
	.gem_prime_mmap = drm_gem_cma_prime_mmap,
	.gem_prime_vmap = drm_gem_cma_prime_vmap,

#if defined(CONFIG_DEBUG_FS)
	.debugfs_init = baikal_vdu_debugfs_init,
#endif
};

static int baikal_vdu_drm_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct baikal_vdu_private *priv;
	struct drm_device *drm;
	struct resource *mem;
	int irq;
	int ret;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	drm = drm_dev_alloc(&vdu_drm_driver, dev);
	if (IS_ERR(drm))
		return PTR_ERR(drm);
	platform_set_drvdata(pdev, drm);
	priv->drm = drm;
	drm->dev_private = priv;

	if (!(mem = platform_get_resource(pdev, IORESOURCE_MEM, 0))) {
		dev_err(dev, "%s no MMIO resource specified\n", __func__);
		return -EINVAL;
	}

	priv->regs = devm_ioremap_resource(dev, mem);
	if (IS_ERR(priv->regs)) {
		dev_err(dev, "%s MMIO allocation failed\n", __func__);
		return PTR_ERR(priv->regs);
	}

	/* turn off interrupts before requesting the irq */
	writel(0, priv->regs + IMR);

	if (!(irq = platform_get_irq(pdev, 0))) {
		dev_err(dev, "%s no IRQ resource specified\n", __func__);
		return -EINVAL;
	}

	spin_lock_init(&priv->lock);

	ret = drm_irq_install(drm, irq);
	if (ret != 0) {
		dev_err(dev, "%s IRQ %d allocation failed\n", __func__, irq);
		return ret;
	}

	ret = vdu_modeset_init(drm);
	if (ret != 0) {
		dev_err(dev, "Failed to init modeset\n");
		goto dev_unref;
	}

	ret = drm_dev_register(drm, 0);
	if (ret < 0)
		goto dev_unref;

	return 0;

dev_unref:
	drm_irq_uninstall(drm);
	drm->dev_private = NULL;
	drm_dev_unref(drm);
	return ret;
}

static int baikal_vdu_drm_remove(struct platform_device *pdev)
{
	struct drm_device *drm = platform_get_drvdata(pdev);
	struct baikal_vdu_private *priv = drm->dev_private;

	drm_dev_unregister(drm);
	if (priv->fbdev)
		drm_fbdev_cma_fini(priv->fbdev);
	drm_mode_config_cleanup(drm);
	drm_irq_uninstall(drm);
	drm->dev_private = NULL;
	drm_dev_unref(drm);

	return 0;
}

static const struct of_device_id baikal_vdu_of_match[] = {
    { .compatible = "baikal,vdu" },
    { },
};

static struct platform_driver baikal_vdu_platform_driver = {
    .probe  = baikal_vdu_drm_probe,
    .remove = baikal_vdu_drm_remove,
    .driver = {
        .name   = DRIVER_NAME,
        .of_match_table = baikal_vdu_of_match,
    },
};

module_param(mode_fixup, int, 0644);

module_platform_driver(baikal_vdu_platform_driver);

MODULE_AUTHOR("Pavel Parkhomenko <Pavel.Parkhomenko@baikalelectronics.ru>");
MODULE_DESCRIPTION("Baikal Electronics BE-M1000 Video Display Unit (VDU) DRM Driver");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:" DRIVER_NAME);
