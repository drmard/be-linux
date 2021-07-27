// SPDX-License-Identifier: GPL-2.0
/*
 * From setup-res.c, by:
 *	Dave Rusling (david.rusling@reo.mts.dec.com)
 *	David Mosberger (davidm@cs.arizona.edu)
 *	David Miller (davem@redhat.com)
 *	Ivan Kokshaysky (ink@jurassic.park.msu.ru)
 */
#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/pci.h>
#include <linux/pci_hotplug.h>
#include <linux/errno.h>
#include <linux/ioport.h>
#include <linux/proc_fs.h>
#include <linux/slab.h>

#include "pci.h"

extern bool pci_bridge_d3_disable_be;
extern bool pci_bridge_d3_force_be;
extern struct pci_platform_pm_ops *pci_platform_pm_be;

void pci_add_resource_offset(struct list_head *resources, struct resource *res,
			     resource_size_t offset)
{
	struct resource_entry *entry;

	entry = resource_list_create_entry(res, 0);
	if (!entry) {
		pr_err("PCI: can't add host bridge window %pR\n", res);
		return;
	}

	entry->offset = offset;
	resource_list_add_tail(entry, resources);
}
EXPORT_SYMBOL(pci_add_resource_offset);

void pci_add_resource(struct list_head *resources, struct resource *res)
{
	pci_add_resource_offset(resources, res, 0);
}
EXPORT_SYMBOL(pci_add_resource);

void pci_free_resource_list(struct list_head *resources)
{
	resource_list_free(resources);
}
EXPORT_SYMBOL(pci_free_resource_list);

void pci_bus_add_resource(struct pci_bus *bus, struct resource *res,
			  unsigned int flags)
{
	struct pci_bus_resource *bus_res;

	bus_res = kzalloc(sizeof(struct pci_bus_resource), GFP_KERNEL);
	if (!bus_res) {
		dev_err(&bus->dev, "can't add %pR resource\n", res);
		return;
	}

	bus_res->res = res;
	bus_res->flags = flags;
	list_add_tail(&bus_res->list, &bus->resources);
}

struct resource *pci_bus_resource_n(const struct pci_bus *bus, int n)
{
	struct pci_bus_resource *bus_res;

	if (n < PCI_BRIDGE_RESOURCE_NUM)
		return bus->resource[n];

	n -= PCI_BRIDGE_RESOURCE_NUM;
	list_for_each_entry(bus_res, &bus->resources, list) {
		if (n-- == 0)
			return bus_res->res;
	}
	return NULL;
}
EXPORT_SYMBOL_GPL(pci_bus_resource_n);

void pci_bus_remove_resources(struct pci_bus *bus)
{
	int i;
	struct pci_bus_resource *bus_res, *tmp;

	for (i = 0; i < PCI_BRIDGE_RESOURCE_NUM; i++)
		bus->resource[i] = NULL;

	list_for_each_entry_safe(bus_res, tmp, &bus->resources, list) {
		list_del(&bus_res->list);
		kfree(bus_res);
	}
}

int devm_request_pci_bus_resources(struct device *dev,
				   struct list_head *resources)
{
	struct resource_entry *win;
	struct resource *parent, *res;
	int err;

	resource_list_for_each_entry(win, resources) {
		res = win->res;
		switch (resource_type(res)) {
		case IORESOURCE_IO:
			parent = &ioport_resource;
			break;
		case IORESOURCE_MEM:
			parent = &iomem_resource;
			break;
		default:
			continue;
		}

		err = devm_request_resource(dev, parent, res);
		if (err)
			return err;
	}

	return 0;
}
EXPORT_SYMBOL_GPL(devm_request_pci_bus_resources);

static struct pci_bus_region pci_32_bit = {0, 0xffffffffULL};
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
static struct pci_bus_region pci_64_bit = {0,
				(pci_bus_addr_t) 0xffffffffffffffffULL};
static struct pci_bus_region pci_high = {(pci_bus_addr_t) 0x100000000ULL,
				(pci_bus_addr_t) 0xffffffffffffffffULL};
#endif
#define pci_bus_for_each_resource_be(bus, res, i)				\
	for (i = 0;							\
	    (res = pci_bus_resource_n(bus, i)) || i < PCI_BRIDGE_RESOURCE_NUM; \
	     i++)

    int __must_check pci_bus_alloc_resource(struct pci_bus *bus,
			struct resource *res, resource_size_t size,
			resource_size_t align, resource_size_t min,
			unsigned long type_mask,
			resource_size_t (*alignf)(void *,
						  const struct resource *,
						  resource_size_t,
						  resource_size_t),
			void *alignf_data);
static bool region_contains_be(struct pci_bus_region *region1,
			    struct pci_bus_region *region2)
{
	return region1->start <= region2->start && region1->end >= region2->end;
}

void pcibios_resource_to_bus_be(struct pci_bus *bus, struct pci_bus_region *region,
			     struct resource *res)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(bus);
	struct resource_entry *window;
	resource_size_t offset = 0;

	resource_list_for_each_entry(window, &bridge->windows) {
		if (resource_contains(window->res, res)) {
			offset = window->offset;
			break;
		}
	}

	region->start = res->start - offset;
	region->end = res->end - offset;
}

static inline bool platform_pci_bridge_d3_be(struct pci_dev *dev)
{
	if (pci_platform_pm_be && pci_platform_pm_be->bridge_d3)
		return pci_platform_pm_be->bridge_d3(dev);
	return false;
}

void pcibios_bus_to_resource_be(struct pci_bus *bus, struct resource *res,
			     struct pci_bus_region *region)
{
	struct pci_host_bridge *bridge = pci_find_host_bridge(bus);
	struct resource_entry *window;
	resource_size_t offset = 0;

	resource_list_for_each_entry(window, &bridge->windows) {
		struct pci_bus_region bus_region;

		if (resource_type(res) != resource_type(window->res))
			continue;

		bus_region.start = window->res->start - window->offset;
		bus_region.end = window->res->end - window->offset;

		if (region_contains_be(&bus_region, region)) {
			offset = window->offset;
			break;
		}
	}

	res->start = region->start + offset;
	res->end = region->end + offset;
}

bool pci_bridge_d3_possible_be(struct pci_dev *bridge)
{
	if (!pci_is_pcie(bridge))
		return false;

	switch (pci_pcie_type(bridge)) {
	case PCI_EXP_TYPE_ROOT_PORT:
	case PCI_EXP_TYPE_UPSTREAM:
	case PCI_EXP_TYPE_DOWNSTREAM:
		if (pci_bridge_d3_disable_be)
			return false;

		/*
		 * Hotplug ports handled by firmware in System Management Mode
		 * may not be put into D3 by the OS (Thunderbolt on non-Macs).
		 */
		if (bridge->is_hotplug_bridge && !pciehp_is_native(bridge))
			return false;

		//if (pci_bridge_d3_force)
		if (pci_bridge_d3_force_be)
			return true;

		/* Even the oldest 2010 Thunderbolt controller supports D3. */
		if (bridge->is_thunderbolt)
			return true;

		/* Platform might know better if the bridge supports D3 */
		if (platform_pci_bridge_d3_be(bridge))
			return true;

		/*
		 * Hotplug ports handled natively by the OS were not validated
		 * by vendors for runtime D3 at least until 2018 because there
		 * was no OS support.
		 */
		if (bridge->is_hotplug_bridge)
			return false;

		if (dmi_check_system(bridge_d3_blacklist))
			return false;

		/*
		 * It should be safe to put PCIe ports from 2015 or newer
		 * to D3.
		 */
		if (dmi_get_bios_year() >= 2015)
			return true;
		break;
	}

	return false;
}

void pci_bridge_d3_update_be(struct pci_dev *dev)
{
	bool remove = !device_is_registered(&dev->dev);
	struct pci_dev *bridge;
	bool d3cold_ok = true;
	bridge = pci_upstream_bridge(dev);
	if (!bridge || !pci_bridge_d3_possible_be(bridge))
		return;
	if (remove && bridge->bridge_d3)
		return;

	if (!remove)
		pci_dev_check_d3cold(dev, &d3cold_ok);

	if (d3cold_ok && !bridge->bridge_d3)
		pci_walk_bus(bridge->subordinate, pci_dev_check_d3cold,
			     &d3cold_ok);

	if (bridge->bridge_d3 != d3cold_ok) {
		bridge->bridge_d3 = d3cold_ok;
		pci_bridge_d3_update_be(bridge);
	}
}

/*
 * @res contains CPU addresses.  Clip it so the corresponding bus addresses
 * on @bus are entirely within @region.  This is used to control the bus
 * addresses of resources we allocate, e.g., we may need a resource that
 * can be mapped by a 32-bit BAR.
 */
static void pci_clip_resource_to_region(struct pci_bus *bus,
					struct resource *res,
					struct pci_bus_region *region)
{
	struct pci_bus_region r;

	pcibios_resource_to_bus_be(bus, &r, res);
	if (r.start < region->start)
		r.start = region->start;
	if (r.end > region->end)
		r.end = region->end;

	if (r.end < r.start)
		res->end = res->start - 1;
	else
		pcibios_bus_to_resource_be(bus, res, &r);
}

static int pci_bus_alloc_from_region(struct pci_bus *bus, struct resource *res,
		resource_size_t size, resource_size_t align,
		resource_size_t min, unsigned long type_mask,
		resource_size_t (*alignf)(void *,
					  const struct resource *,
					  resource_size_t,
					  resource_size_t),
		void *alignf_data,
		struct pci_bus_region *region)
{
	int i, ret;
	struct resource *r, avail;
	resource_size_t max;

	type_mask |= IORESOURCE_TYPE_BITS;

	pci_bus_for_each_resource_be(bus, r, i) {
		resource_size_t min_used = min;

		if (!r)
			continue;

		/* type_mask must match */
		if ((res->flags ^ r->flags) & type_mask)
			continue;

		/* We cannot allocate a non-prefetching resource
		   from a pre-fetching area */
		if ((r->flags & IORESOURCE_PREFETCH) &&
		    !(res->flags & IORESOURCE_PREFETCH))
			continue;

		avail = *r;
		pci_clip_resource_to_region(bus, &avail, region);

		/*
		 * "min" is typically PCIBIOS_MIN_IO or PCIBIOS_MIN_MEM to
		 * protect badly documented motherboard resources, but if
		 * this is an already-configured bridge window, its start
		 * overrides "min".
		 */
		if (avail.start)
			min_used = avail.start;

		max = avail.end;

		/* Ok, try it out.. */
		ret = allocate_resource(r, res, size, min_used, max,
					align, alignf, alignf_data);
		if (ret == 0)
			return 0;
	}
	return -ENOMEM;
}

/**
 * pci_bus_alloc_resource - allocate a resource from a parent bus
 * @bus: PCI bus
 * @res: resource to allocate
 * @size: size of resource to allocate
 * @align: alignment of resource to allocate
 * @min: minimum /proc/iomem address to allocate
 * @type_mask: IORESOURCE_* type flags
 * @alignf: resource alignment function
 * @alignf_data: data argument for resource alignment function
 *
 * Given the PCI bus a device resides on, the size, minimum address,
 * alignment and type, try to find an acceptable resource allocation
 * for a specific device resource.
 */
int pci_bus_alloc_resource(struct pci_bus *bus, struct resource *res,
		resource_size_t size, resource_size_t align,
		resource_size_t min, unsigned long type_mask,
		resource_size_t (*alignf)(void *,
					  const struct resource *,
					  resource_size_t,
					  resource_size_t),
		void *alignf_data)
{
#ifdef CONFIG_ARCH_DMA_ADDR_T_64BIT
	int rc;

	if (res->flags & IORESOURCE_MEM_64) {
		rc = pci_bus_alloc_from_region(bus, res, size, align, min,
					       type_mask, alignf, alignf_data,
					       &pci_high);
		if (rc == 0)
			return 0;

		return pci_bus_alloc_from_region(bus, res, size, align, min,
						 type_mask, alignf, alignf_data,
						 &pci_64_bit);
	}
#endif

	return pci_bus_alloc_from_region(bus, res, size, align, min,
					 type_mask, alignf, alignf_data,
					 &pci_32_bit);
}
EXPORT_SYMBOL(pci_bus_alloc_resource);

/*
 * The @idx resource of @dev should be a PCI-PCI bridge window.  If this
 * resource fits inside a window of an upstream bridge, do nothing.  If it
 * overlaps an upstream window but extends outside it, clip the resource so
 * it fits completely inside.
 */
bool pci_bus_clip_resource(struct pci_dev *dev, int idx)
{
	struct pci_bus *bus = dev->bus;
	struct resource *res = &dev->resource[idx];
	struct resource orig_res = *res;
	struct resource *r;
	int i;

	pci_bus_for_each_resource_be(bus, r, i) {
		resource_size_t start, end;

		if (!r)
			continue;

		if (resource_type(res) != resource_type(r))
			continue;

		start = max(r->start, res->start);
		end = min(r->end, res->end);

		if (start > end)
			continue;	/* no overlap */

		if (res->start == start && res->end == end)
			return false;	/* no change */

		res->start = start;
		res->end = end;
		res->flags &= ~IORESOURCE_UNSET;
		orig_res.flags &= ~IORESOURCE_UNSET;
		pci_info(dev, "%pR clipped to %pR\n", &orig_res, res);

		return true;
	}

	return false;
}

void __weak pcibios_resource_survey_bus(struct pci_bus *bus) { }

void __weak pcibios_bus_add_device(struct pci_dev *pdev) { }

/**
 * pci_bus_add_device - start driver for a single device
 * @dev: device to add
 *
 * This adds add sysfs entries and start device drivers
 */
void pci_bus_add_device(struct pci_dev *dev)
{
	int retval;

	/*
	 * Can not put in pci_device_add yet because resources
	 * are not assigned yet for some devices.
	 */
	pcibios_bus_add_device(dev);
	pci_fixup_device(pci_fixup_final, dev);
	pci_create_sysfs_dev_files(dev);
	pci_proc_attach_device(dev);
	pci_bridge_d3_update_be(dev);

	dev->match_driver = true;
	retval = device_attach(&dev->dev);
	if (retval < 0 && retval != -EPROBE_DEFER)
		pci_warn(dev, "device attach failed (%d)\n", retval);

	pci_dev_assign_added(dev, true);
}
EXPORT_SYMBOL_GPL(pci_bus_add_device);

/**
 * pci_bus_add_devices - start driver for PCI devices
 * @bus: bus to check for new devices
 *
 * Start driver for PCI devices and add some sysfs entries.
 */
void pci_bus_add_devices(const struct pci_bus *bus)
{
	struct pci_dev *dev;
	struct pci_bus *child;

	list_for_each_entry(dev, &bus->devices, bus_list) {
		/* Skip already-added devices */
		if (pci_dev_is_added(dev))
			continue;
		pci_bus_add_device(dev);
	}

	list_for_each_entry(dev, &bus->devices, bus_list) {
		/* Skip if device attach failed */
		if (!pci_dev_is_added(dev))
			continue;
		child = dev->subordinate;
		if (child)
			pci_bus_add_devices(child);
	}
}
EXPORT_SYMBOL(pci_bus_add_devices);

/** pci_walk_bus - walk devices on/under bus, calling callback.
 *  @top      bus whose devices should be walked
 *  @cb       callback to be called for each device found
 *  @userdata arbitrary pointer to be passed to callback.
 *
 *  Walk the given bus, including any bridged devices
 *  on buses under this bus.  Call the provided callback
 *  on each device found.
 *
 *  We check the return of @cb each time. If it returns anything
 *  other than 0, we break out.
 *
 */
void pci_walk_bus(struct pci_bus *top, int (*cb)(struct pci_dev *, void *),
		  void *userdata)
{
	struct pci_dev *dev;
	struct pci_bus *bus;
	struct list_head *next;
	int retval;

	bus = top;
	down_read(&pci_bus_sem);
	next = top->devices.next;
	for (;;) {
		if (next == &bus->devices) {
			/* end of this bus, go up or finish */
			if (bus == top)
				break;
			next = bus->self->bus_list.next;
			bus = bus->self->bus;
			continue;
		}
		dev = list_entry(next, struct pci_dev, bus_list);
		if (dev->subordinate) {
			/* this is a pci-pci bridge, do its devices next */
			next = dev->subordinate->devices.next;
			bus = dev->subordinate;
		} else
			next = dev->bus_list.next;

		retval = cb(dev, userdata);
		if (retval)
			break;
	}
	up_read(&pci_bus_sem);
}
EXPORT_SYMBOL_GPL(pci_walk_bus);

struct pci_bus *pci_bus_get(struct pci_bus *bus)
{
	if (bus)
		get_device(&bus->dev);
	return bus;
}

void pci_bus_put(struct pci_bus *bus)
{
	if (bus)
		put_device(&bus->dev);
}
