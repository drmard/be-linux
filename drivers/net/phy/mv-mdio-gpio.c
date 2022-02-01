// SPDX-License-Identifier: GPL-2.0+
/*
 * RTT UNA1.0 MDIO bus controller driver
 *
 * Copyright (C) 2021-2022 RTT
 */
#include <linux/kernel.h>
#include <linux/phy.h>
#include <linux/module.h>
#include <linux/types.h>
#include <linux/delay.h>
#include <linux/clk.h>
#include <linux/platform_device.h>
#include <linux/gpio.h>
#include <linux/platform_data/mdio-gpio.h>

#include <linux/init.h>
#include <linux/of.h>
#include <linux/interrupt.h>
#include <linux/mutex.h>
#include <linux/of_platform.h>
#include <linux/of_mdio.h>
#include <linux/mdio-bitbang.h>

#include "mv-mdio-gpio.h"

#define MDIO_READ         2
#define MDIO_WRITE        1
#define MDIO_C45          (1<<15)

#define MDIO_C45_ADDR    (MDIO_C45 | 0)
#define MDIO_C45_READ    (MDIO_C45 | 3)
#define MDIO_C45_WRITE   (MDIO_C45 | 1)

#define MDIO_SETUP_TIME  10
#define MDIO_HOLD_TIME   10

#define GPIO01           17
#define GPIO02           27
#define GPIO03           28
#define GPIO04           29
#define GPIO05           30
#define GPIO06           31

#define GPIO_CLK         GPIO03
#define GPIO_DATA        GPIO04
#define GPIO_RESET       GPIO05
#define GPIO_INTERRUPT   GPIO06

#define MDIO_GPIO_MDO    2
#define MDIO_GPIO_MDIO   GPIO_DATA
#define MDIO_GPIO_MDC    GPIO_CLK
#define MDIO_GPIO_RESET  GPIO_RESET
#define MDIO_GPIO_INTR   GPIO_INTERRUPT

#define MDIO_DELAY          250
#define MDIO_READ_DELAY     400

#define PHY_ADDRESS_LENGTH    5
#define REG_ADDRESS_LENGTH    5

struct mdio_gpio_info {
  struct mdiobb_ctrl ctrl;
  struct gpio_desc *mdc, *mdio, *mdo;
};

struct mv_mdio_data {
  struct mii_bus *bus;
  struct mdio_gpio_info *bb;
  u64 register_base;
  struct gpio_desc *reset;
  struct gpio_desc *intr;
  struct clk *clk;
  u32 clk_freq;
  // PHY reset duration
  int msec_reset;
  //bool skip_scan;
  void *priv;
};

static struct mdiobb_ops mv_bb_ops = 
{
  .owner = THIS_MODULE,
  .set_mdc = mv_set_mdc,
  .set_mdio_dir = mv_set_mdio_dir,
  .set_mdio_data = mv_set_mdio_data,
  .get_mdio_data = mv_get_mdio_data,
  .owner = THIS_MODULE,
};

static struct mdiobb_ctrl mv_mdiobb_ctrl = {
  .ops = &mv_bb_ops,
};

static void *
mv_mdio_gpio_get_data(struct platform_device *pdev
)
{
  struct mdio_gpio_info *bb;
  struct mv_mdio_data *pdata;
  struct mdiobb_ctrl *ctrl;
  struct device *dev = &pdev->dev;
  struct device_node *dn = pdev->dev.of_node;
  if (!dn) {
    printk(KERN_INFO "%s cannot get device node from platform device\n",__func__);
    return NULL;
  }

  pdata = devm_kzalloc(dev, sizeof(*pdata), GFP_KERNEL);
  if (!pdata) {
    printk(KERN_INFO "%s - cannot allocate memory\n",__func__);
    return pdata;
  }

  bb = devm_kzalloc(dev, sizeof(*bb), GFP_KERNEL);
  if(unlikely(!bb)) {
    printk(KERN_INFO "[%s] - cannot allocate memory\n",__func__);
    return NULL;
  }

  ctrl = &mv_mdiobb_ctrl;
  bb->ctrl = *ctrl;
  pdata->bb = bb;

  pdata->bb->mdc = gpiod_get(dev, "mv,mdc", GPIOD_OUT_LOW);
  if (IS_ERR(pdata->bb->mdc)){
    printk(KERN_INFO "%s - failed to get gpio mdc: %ld\n",
    __func__,PTR_ERR(pdata->bb->mdc));
    return /*PTR_ERR(pdata->bb->mdc)*/NULL;  
  }

  pdata->bb->mdio = gpiod_get(dev, "mv,mdio", GPIOD_OUT_LOW);
  if (IS_ERR(pdata->bb->mdio)) {
    printk(KERN_INFO "%s - failed to get gpio mdio: %ld\n",
    __func__, PTR_ERR(pdata->bb->mdio));
    return /*PTR_ERR(pdata->bb->mdio)*/NULL;
  }

  pdata->bb->mdo = gpiod_get(dev, "mv,mdo", GPIOD_OUT_LOW);
  if (IS_ERR(pdata->bb->mdo)) {
    printk(KERN_INFO "%s - cannot get gpio mdo\n",__func__) ;
  }

  pdata->reset = gpiod_get(dev, "mv,reset", GPIOD_OUT_LOW);
  if (IS_ERR(pdata->reset)) {
    printk (KERN_INFO "%s - failed to get gpio reset: %ld\n",
    __func__, PTR_ERR(pdata->reset));
  }

  pdata->intr = gpiod_get(dev, "mv,intr", GPIOD_IN);
  if (IS_ERR(pdata->intr)) {
    printk (KERN_INFO "%s - failed to get gpio intr: %ld\n",
    __func__,PTR_ERR(pdata->intr));
  }

  // printk (KERN_INFO "%s - returned address of priv. data: %lu\n",__func__, (void *)pdata);
  return pdata;
}

static void mv_set_mdc(struct mdiobb_ctrl *ctrl, int what)
{
  struct mdio_gpio_info *bb =
    container_of(ctrl, struct mdio_gpio_info, ctrl);
  if (bb)
    gpiod_set_value_cansleep(bb->mdc, what);
}

static void mv_set_mdio_dir(struct mdiobb_ctrl *ctrl, int dir)
{
  struct mdio_gpio_info *bb =
    container_of(ctrl, struct mdio_gpio_info, ctrl);
  if (bb && bb->mdo) {
    // Separate output pin. Always set its value to high
    // when changing direction. If direction is input,
    // assume the pin serves as pull-up. If direction is
    // output, the default value is high.
    gpiod_set_value_cansleep(bb->mdo, 1);
    return;
  }
  if (bb) {
    if (dir)
    gpiod_direction_output(bb->mdio, 1);
    else
    gpiod_direction_input(bb->mdio);
  }
}

static void mv_set_mdio_data(struct mdiobb_ctrl *ctrl, int val)
{
  struct mdio_gpio_info *bb =
    container_of(ctrl, struct mdio_gpio_info, ctrl);
  if (bb && bb->mdo) {
    gpiod_set_value_cansleep(bb->mdo, val);
  } else { 
    gpiod_set_value_cansleep(bb->mdio, val);
  }
}

static int mv_get_mdio_data(struct mdiobb_ctrl *ctrl)
{
  struct mdio_gpio_info *bb =
    container_of(ctrl, struct mdio_gpio_info, ctrl);

  return gpiod_get_value_cansleep(bb->mdio);
}

// MDIO must already be configured as output
static void mv_mdiobb_send_bit(struct mdiobb_ctrl *ctrl, int val)
{
  const struct mdiobb_ops *ops = ctrl->ops;
  ops->set_mdio_data(ctrl, val);
  ndelay(MDIO_DELAY);

  ops->set_mdc(ctrl, 1);
  ndelay(MDIO_DELAY);
  ops->set_mdc(ctrl, 0);
}

// MDIO must be configured as input.
static int mv_mdiobb_get_bit(struct mdiobb_ctrl *ctrl)
{
  const struct mdiobb_ops *ops = ctrl->ops;

  ndelay(MDIO_DELAY);
  ops->set_mdc(ctrl, 1);
  ndelay(MDIO_READ_DELAY);
  ops->set_mdc(ctrl, 0);

  return ops->get_mdio_data(ctrl);
}

// MDIO (DATA pin) must be configured as input.
static u16 mv_mdiobb_get_num(struct mdiobb_ctrl *ctrl, int bits)
{
  int i;
  u16 ret = 0;
  for (i = bits - 1; i >= 0; i--) {
    ret <<= 1;
    ret |= mv_mdiobb_get_bit(ctrl);
  }

  return ret;
}

static void mv_mdiobb_cmd(struct mdiobb_ctrl *ctrl, int op, u8 phy, u8 reg)
{
  int i;
  const struct mdiobb_ops *ops = ctrl->ops;
  ops->set_mdio_dir(ctrl, 1);

  // Send a 32 bit preamble ('1's) with an extra '1' bit for good
  // measure.  The IEEE spec says this is a PHY optional
  // requirement.  The AMD 79C874 requires one after power up and
  // one after a MII communications error.  This means that we are
  // doing more preambles than we need, but it is safer and will be
  // much more robust.
  for (i = 0; i < 32; i++) {
    mv_mdiobb_send_bit(ctrl, 1);
  }

  // send the start bit (01) and the read opcode (10) or write (01).
  // Clause 45 operation uses 00 for the start and 11, 10 for
  // read/write
  mv_mdiobb_send_bit(ctrl, 0);
  if (op & MDIO_C45) {
    mv_mdiobb_send_bit(ctrl, 0);
  } else {
    mv_mdiobb_send_bit(ctrl, 1);
  }
  mv_mdiobb_send_bit(ctrl, (op >> 1) & 1);
  mv_mdiobb_send_bit(ctrl, (op >> 0) & 1);
  mv_mdiobb_send_num(ctrl, phy, 5);
  mv_mdiobb_send_num(ctrl, reg, 5);
}

// In clause 45 mode all commands are prefixed by MDIO_ADDR to specify the
// lower 16 bits of the 21 bit address. This transfer is done identically to a
// MDIO_WRITE except for a different code. To enable clause 45 mode or
// MII_ADDR_C45 into the address. Theoretically clause 45 and normal devices
// can exist on the same bus. Normal devices should ignore the MDIO_ADDR
// phase.
static int mv_mdiobb_cmd_addr(struct mdiobb_ctrl *ctrl, int phy, u32 addr)
{
  unsigned int dev_addr = (addr >> 16) & 0x1F;
  unsigned int reg = addr & 0xFFFF;
  mv_mdiobb_cmd(ctrl, MDIO_C45_ADDR, phy, dev_addr);
  // send the turnaround (10)
  mv_mdiobb_send_bit(ctrl, 1);
  mv_mdiobb_send_bit(ctrl, 0);
  mv_mdiobb_send_num(ctrl, reg, 16);
  ctrl->ops->set_mdio_dir(ctrl, 0);
  mv_mdiobb_get_bit(ctrl);

  return dev_addr;
}

// MDIO (DATA pin) must be here configured as output
static void mv_mdiobb_send_num(
    struct mdiobb_ctrl *ctrl, u16 val, int bits)
{
  int i;
  for (i = bits - 1; i >= 0; i--) {
    mv_mdiobb_send_bit(ctrl, (val >> i) & 1);
  }
}

// If there is an error during turnover, flush all bits
// and return 0xffff
int mv_mdiobb_read (struct mii_bus *bus, int phy, int reg)
{
  int ret, i;
  struct mv_mdio_data *priv = bus->priv;
  struct mdiobb_ctrl *ctrl = &priv->bb->ctrl;
  if (!ctrl) {
    ctrl = &mv_mdiobb_ctrl;
  }

  if (reg & MII_ADDR_C45) {
    reg = mv_mdiobb_cmd_addr(ctrl, phy, reg);
    mv_mdiobb_cmd(ctrl, MDIO_C45_READ, phy, reg);
  } else {
    mv_mdiobb_cmd(ctrl, MDIO_READ, phy, reg);
  }

  ctrl->ops->set_mdio_dir(ctrl, 0);
  // check the turnaround bit: the PHY should be driving it to zero, if this
  // PHY is listed in phy_ignore_ta_mask as having broken TA, skip that
  if (mv_mdiobb_get_bit(ctrl) != 0 &&
      !(bus->phy_ignore_ta_mask & (1 << phy))) {
    // PHY didn't drive TA low -- flush any bits it
    // may be trying to send.
    for (i = 0; i < 32; i++) {
      mv_mdiobb_get_bit(ctrl);
    }
    return 0xffff;
  }
  ret = mv_mdiobb_get_num(ctrl, 16);
  mv_mdiobb_get_bit(ctrl);

  return ret;
}
EXPORT_SYMBOL(mv_mdiobb_read);

int mv_mdiobb_write (struct mii_bus *bus, int phy, int reg, u16 val) {
  struct mv_mdio_data *data = bus->priv;
  struct mdiobb_ctrl *ctrl = &data->bb->ctrl;

  if (reg & MII_ADDR_C45) {
    reg = mv_mdiobb_cmd_addr(ctrl, phy, reg);
    mv_mdiobb_cmd(ctrl, MDIO_C45_WRITE, phy, reg);
  } else {
    mv_mdiobb_cmd(ctrl, MDIO_WRITE, phy, reg);
  }
  // send the turnaround (10)
  mv_mdiobb_send_bit(ctrl, 1);
  mv_mdiobb_send_bit(ctrl, 0);

  mv_mdiobb_send_num(ctrl, val, 16);
  ctrl->ops->set_mdio_dir(ctrl, 0);
  mv_mdiobb_get_bit(ctrl);

  return 0;
}
EXPORT_SYMBOL(mv_mdiobb_write);

int mv_mdiobb_reset(struct mii_bus *bus)
{
  struct mv_mdio_data *priv = bus->priv;
  if (!priv) {
    printk (
    KERN_INFO "%s - cannot get private data\n",__func__);
    return -ENOMEM;
  }
  //struct mv_mdio_gpio_info *data = &priv->bus_data;
  if ( priv->reset )
  {
    mutex_lock(&bus->mdio_lock);
    gpiod_set_value_cansleep(priv->reset, 1);
    msleep(priv->msec_reset);      //   xxxx
    gpiod_set_value_cansleep(priv->reset, 0);
    mutex_unlock(&bus->mdio_lock);
  }

  return 0;
}
EXPORT_SYMBOL(mv_mdiobb_reset);

struct mii_bus *mv_alloc_mdio_bitbang(
    struct mdiobb_ctrl *ctrl)
{
  struct mii_bus *bus = mdiobus_alloc();
  if (!bus) {
    printk(KERN_INFO "%s - \'mdiobus_alloc\' failed \n",__func__);
    return NULL;
  }
  __module_get(ctrl->ops->owner);

  bus->read = mv_mdiobb_read;
  bus->write = mv_mdiobb_write;
  bus->reset = mv_mdiobb_reset;

  bus->state = MDIOBUS_ALLOCATED;
  return bus;
}
EXPORT_SYMBOL(mv_alloc_mdio_bitbang);

void mv_free_mdio_bitbang(struct mii_bus *bus) 
{
  if (!bus)
    return;

  if (bus->state == MDIOBUS_REGISTERED) {
    mdiobus_unregister(bus);
    //dev_set_drvdata(dev, NULL);
    if (bus->priv != NULL)
      kfree(bus->priv);
    bus->priv = NULL;
  }

  mdiobus_free(bus);
}
EXPORT_SYMBOL(mv_free_mdio_bitbang);

// If we will use clk device on this mdio bus
// we should implement corresponding init function
static void mv_mdio_init_clk(struct mv_mdio_data *data) {
  if (!data)
    return;
}

static struct mii_bus *
mv_mdio_gpio_bus_init(struct platform_device *pdev,
    struct mv_mdio_data *md, int bus_id)
{
  //struct mdio_gpio_info * in;                                                     //  xxxxxx
  struct device *dev = &pdev->dev;
  struct device_node *np = dev->of_node;
  if (unlikely(!np))
  {
    printk (KERN_INFO "%s - cannot get device node\n",__func__);
    return NULL ;
  }
  //md->bb->ctrl = &mv_mdiobb_ctrl;

  md->bus = mv_alloc_mdio_bitbang(&md->bb->ctrl);
  if (unlikely(!(md->bus))) {
    printk(KERN_INFO "%s - cannot allocate memory for mdio bus\n",
    __func__);
    return NULL;
  }

  md->bus->state = MDIOBUS_ALLOCATED;
  md->bus->name = "UNA MDIO GPIO bus";
  md->bus->parent = &pdev->dev;
  md->bus->priv = md;

  // set bus ID
  if (bus_id > -1)
    snprintf(md->bus->id, MII_BUS_ID_SIZE, "mv-mdio-gpio-%x", bus_id);
  else
    strncpy(md->bus->id, "mv-mdio-gpio", MII_BUS_ID_SIZE);
  // *** DEBUG ***
  printk (KERN_INFO "%s - DEBUG: bus ID - %s\n",__func__,md->bus->id);

  // PHY reset duration
  of_property_read_u32(np, "phy-reset-duration", &md->msec_reset);

  // A sane reset duration should not be longer than 1s
  if (md->msec_reset > 1000 || md->msec_reset <= 0) {
    md->msec_reset = 100;
  }

  dev_set_drvdata(dev, md);

  return md->bus;
}

static int
mv_mdio_gpio_probe(struct platform_device *pdev)
{
  int ret = -1,
  _debug = 0,
  bus_id = -1,
  register_failed = 0,
  addr ;
  struct phy_device *phy;
  resource_size_t mdio_phys, regsize;
  struct device_node *dn;
  struct resource *res;
  struct mv_mdio_data *data;

  dn = pdev->dev.of_node;
  if (!dn) {
    printk(KERN_INFO "%s - can't get device node\n",__func__);
    return ret;
  }

  data = (struct mv_mdio_data *) mv_mdio_gpio_get_data(pdev);
  if (!data) {
    printk(KERN_INFO "%s - cannot get private data\n",__func__);
    return -ENOMEM;
  }

  // to dt aliases should be added some as una-mdio1:   'una-mdio1 = &mdio1'
  bus_id = of_alias_get_id(dn, "una-mdio");
  if (bus_id < 0) {
    printk(KERN_INFO "%s - cannot get alias ID\n",__func__);
    bus_id = pdev->id;

    // this value should be removed after debug
    _debug++;
  }

  printk(KERN_INFO "%s - extracted bus_id : %d\n",__func__,bus_id);

  data->bus = mv_mdio_gpio_bus_init(pdev, data, bus_id);
  if (!(data->bus)) {
    printk(KERN_INFO "%s - error: cannot init mdio bus\n",__func__);
    return -ENODEV;
  }

  res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  if (!res) {
    printk(KERN_INFO "%s - not found resource memory\n", __func__);
    return -EINVAL;
  }
  mdio_phys = res->start;
  regsize = resource_size(res);
  data->register_base = (u64) devm_ioremap(&pdev->dev, mdio_phys, regsize);
  if(!data->register_base) {
    printk(KERN_INFO "%s - err:cannot pass \'dev_ioremap\'\n", __func__);
  }
  //can be impl. as davinci_mdio_init_clk(data);
  mv_mdio_init_clk(data);

  ret = of_mdiobus_register(data->bus, dn);
  if (ret) {
    printk(KERN_INFO "%s - error: cannot register MDIO bus %s\n",
    __func__, data->bus->name);

    register_failed++;
    goto err0;
  }

  data->bus->state = MDIOBUS_REGISTERED;

  // scan and dump the bus
  for (addr = 0; addr < PHY_MAX_ADDR; addr++)
  {
    phy = mdiobus_get_phy(data->bus, addr);
    if (phy) {
      dev_info(&pdev->dev, "phy[%d]: device %s, driver %s\n",
        phy->mdio.addr,phydev_name(phy),phy->drv ? phy->drv->name : "unknown");
    }
  }

err0:
  if (register_failed)
    mdiobus_free(data->bus);

  printk (KERN_INFO "%s - mdio bus probe : returned %d\n", __func__,ret);
  return ret;
}

#define DRIVER_NAME     "mv-mdio-gpio"

static int
mv_mdio_gpio_remove(struct platform_device *pdev)
{
  mdiobus_unregister(platform_get_drvdata(pdev));
  dev_set_drvdata(&pdev->dev, NULL);
  //kfree(bus->priv);
  //bus->priv = NULL;
  return 0;
}

// If we need 'suspend' implementation we can
// impl. it as 'unimac_mdio_suspend'
static int __maybe_unused mv_mdio_gpio_suspend(struct device *dev)
{
  // should be impl. as 'unimac_mdio_suspend'
  return 0;
}

static struct of_device_id mv_mdio_gpio_of_match[] = {
  { .compatible = "mv,mv-mdio-gpio",  },
  { .compatible = "una,mv-mdio-gpio", },
  {},
};

static struct platform_driver mv_mdio_gpio_driver = {
  .probe = mv_mdio_gpio_probe,
  .remove = mv_mdio_gpio_remove,
  .driver = {
    .name = DRIVER_NAME,
    .of_match_table = mv_mdio_gpio_of_match,
  },
};

module_platform_driver(mv_mdio_gpio_driver);
MODULE_ALIAS("platform:mv-mdio-gpio");
MODULE_AUTHOR("RTT");
MODULE_LICENSE("GPL v2");
MODULE_DESCRIPTION("Generic driver for UNA/Marvell MDIO bus emulation using GPIO");

