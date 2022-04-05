// SPDX-License-Identifier: GPL-2.0+
/*
 * drivers/net/phy/mv88x5113.c
 *
 * Driver for Marvell multi-speed ethernet transceiver 88x5113.
 *
 * Supports: 1000Base-X, 10GBase-R, 10GBase-W and more modes.
 * Driver is adapted for use on RTT UNA boards.
 *
 */

// ----
// ----
// Attention :
// It is Draft Version of Driver for DEBUG on board UNA1.0
// ----
// ----

#include <linux/module.h>
#include <linux/delay.h>
#include <linux/phy.h>
#include <linux/marvell_phy.h>
#include <linux/hwmon.h>
#include <linux/gpio.h>
#include <linux/mdio.h>
#include <linux/mdio-bitbang.h>
#include <linux/marvell_phy.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/of_gpio.h>
#include <linux/of_mdio.h>

#include "mv88x5113.h"
#include "mv-mdio-gpio.h"

#define MV_MODE_LINE_10GBKR         0x4103
#define MV_MODE_LINE_10GBCR         0x4003
#define MV_MODE_LINE_10GBSR         0x4003
#define MV_MODE_HOST_10GBKX4        0x4023
#define MV_MODE_HOST_10GBKR         0x4103

//MODULE_DESCRIPTION("Marvell Alaska 88X5113 Ethernet PHY driver");
MODULE_DESCRIPTION("RTT mv88x5113 PHY driver");
MODULE_AUTHOR("Sergej Khrumalo <s.khrumalo@rusteletech.ru>");
MODULE_LICENSE("GPL");

struct mode { 
  unsigned int mode_num;
  char mode_name[16];
};

static struct mode line_modes[] = {
  { MV_MODE_LINE_10GBKR, "KR" },
  { MV_MODE_LINE_10GBSR, "SR" },
};
static struct mode host_modes[] = {
  { MV_MODE_HOST_10GBKR, "KR" },
  { MV_MODE_HOST_10GBKX4, "KX4" },
};

struct mv5113_chip {
  void (*init_supported_interfaces)(unsigned long *mask);
  //int (*get_mactype)(struct phy_device *phydev);
  int (*init_interface)(struct phy_device *phydev);
#ifdef CONFIG_HWMON
  int (*hwmon_read_temp_reg)(struct phy_device *phydev);
#endif
};

struct mv5113_priv {
  struct mii_bus *bus;
  struct device_node *phy_node;
  struct device_node *mii_dn;

  DECLARE_BITMAP(supported_interfaces, PHY_INTERFACE_MODE_MAX);
  bool rate_match;
  phy_interface_t /*mv5113_interface*/interface;
  int irq;
  int irq_active_low;
  int line_mode;
  int host_mode;
  int addr;
  void *priv;
};
/*
static const struct sfp_upstream_ops mv5113_sfp_ops = {
  .attach = phy_sfp_attach,
  .detach = phy_sfp_detach,
  .module_insert = mv5113_sfp_insert,
};*/

static const struct mv5113_chip *to_mv5113_chip(
    struct phy_device *phydev)
{
  return phydev->drv->driver_data;
}

static void mv5113_init_supported_interfaces(unsigned long *mask)
{
  //__set_bit(PHY_INTERFACE_MODE_XAUI, mask);
  //__set_bit(PHY_INTERFACE_MODE_10GKR, mask);
  //__set_bit(PHY_INTERFACE_MODE_XGMII), mask);
}

static int mv5113_init_interface(struct phy_device *phydev)
{
  struct mv5113_priv *priv = dev_get_drvdata(&phydev->mdio.dev);
  if (!priv) {
    printk (KERN_INFO "%s - cannot get private driver data\n",__func__);
    return -EINVAL;
  }

  priv->interface = PHY_INTERFACE_MODE_10GKR;

  if (phydev->interface == priv->interface)
    priv->rate_match = true;
  else
    priv->rate_match = false;

  return 0;
}

/*
static int mv5113_sfp_insert(
    void *upstream, const struct sfp_eeprom_id *id)
{
  phy_interface_t iface;

  struct phy_device *phydev = (struct phy_device *)upstream;
  __ETHTOOL_DECLARE_LINK_MODE_MASK(support) = { 0, };

  sfp_parse_support(phydev->sfp_bus, id, support);

  // need to check that 'sfp_select_interface' returned proper value of
  // interface
  iface = sfp_select_interface(phydev->sfp_bus, support);
  if (iface != PHY_INTERFACE_MODE_10GKR)
  {
    printk (KERN_INFO "%s:  incompatible SFP module inserted\n",
    __func__);

    return -EINVAL;
  }

  return 0;
}*/

int una_mdio_read (struct mii_bus *bus, int phy, int reg) {
  return mv_mdiobb_read (bus, phy, reg);
}

int una_mdio_write (struct mii_bus *bus, int phy, int reg, u16 val) {
  return mv_mdiobb_write (bus, phy, reg, val);
}

int una_mdio_reset (struct mii_bus *bus) {
  return mv_mdiobb_reset(bus);
}

static struct device_node *get_phydevice_node(
    struct device *dev)
{
  struct device_node *phy_node =
    of_parse_phandle(dev->of_node, "mv-phy-handle", 0);
  if (!phy_node) {
    printk(KERN_INFO "%s - cannot parse phy device mode \n",__func__);
  }

  return phy_node;
}
EXPORT_SYMBOL_GPL(get_phydevice_node);

struct phy_device *_parse_mv_phydevice (struct device_node *node) {
  int ret;
  struct phy_device *phydev;
  if (!node)
    return NULL;

  phydev = of_phy_find_device(node);
  if (!phydev) {
    printk(KERN_INFO "%s - failed \n",__func__);
    //err->error = -ENODEV;
    //strcpy(err->description,__func__);
    //goto err1;
    return NULL;
  }
  of_node_put(node);
  ret = phy_init_hw(phydev);
  if (ret != 0) {
    printk(KERN_INFO "%s - \'phy_init_hw\' failed\n",__func__);
    return NULL;
  }
  return phydev;
}

struct phy_device *parse_mv_phydevice (struct device_node *node) {
  return _parse_mv_phydevice(node);
}
EXPORT_SYMBOL_GPL(parse_mv_phydevice);

// This method sets mdio bitbang bus for target phydev
int set_mdio_bus (struct phy_device *phydev)
{
  struct mii_bus *bus;
  struct mv5113_priv *priv;

  bus = phydev->mdio.bus;
  if (!bus || !bus->read || !bus->write)
  {
    priv = dev_get_drvdata(&phydev->mdio.dev);
    if (!priv) {
      return -1;
    }
    phydev->mdio.bus = priv->bus;
  }
  printk (KERN_INFO"%s - returned 0 \n",__func__);
  return 0;  
}
EXPORT_SYMBOL_GPL(set_mdio_bus);

static int mv5113_soft_reset(struct phy_device *phydev) {
  int ret, val = -1;
  int cnt = 50;

  val = phy_read_mmd(phydev, MV88X5113_PORT_REG, MV88X5113_PORT_RESET);
  if (val < 0) {
    printk (KERN_INFO "%s - cannot read data from register\n",__func__);
    ret = set_mdio_bus(phydev);
    if (ret != 0) {
      printk(KERN_INFO "%s - error during pass \'set_mdio_bus\' \n",__func__);
      return ret;
    }
    val = phy_read_mmd(phydev, MV88X5113_PORT_REG, MV88X5113_PORT_RESET);
    if (val < 0)
      return val;
  }
  val |= MV88X5113_SW_RST_ALL;
  ret = phy_write_mmd(phydev, MV88X5113_PORT_REG, MV88X5113_PORT_RESET, val);
  if (ret) {
      printk (KERN_INFO "%s - cannot perform soft reset\n",__func__);
      return ret;
  }
  /*
  for (cnt = 0; cnt < 50; cnt++) {
    usleep_range(MV88X5113_RESET_DELAY_US, MV88X5113_RESET_DELAY_US + 50);
    ret = phy_read_mmd(phydev, MV88X5113_PORT_REG, MV88X5113_PORT_RESET);
    if (!(ret & MV88X5113_SW_RST_ALL))
      break;
  } */
  do {
    usleep_range(MV88X5113_RESET_DELAY_US, MV88X5113_RESET_DELAY_US + 100);
    ret = phy_read_mmd(phydev, MV88X5113_PORT_REG, MV88X5113_PORT_RESET);
  } while ((ret & MV88X5113_SW_RST_ALL) || cnt--);

  return 0;   
}

int mv_soft_reset(struct phy_device *phydev) {
  return mv5113_soft_reset(phydev);
}
EXPORT_SYMBOL_GPL(mv_soft_reset);

struct mii_bus *mv88X5113_mdio_bus_find (struct phy_device *phydev)
{
  //int ret;
  struct device_node *dn;
  struct mii_bus *bus;
  //struct phy_device *pd;
  if (phydev)
    dn = phydev->mdio.dev.parent->of_node;

  if (!phydev || !dn) {
    // Try to find in DT some una/mv mdio bus node
    dn = of_find_compatible_node(NULL, NULL, "mv,mv-mdio-gpio");
    if (!dn)   
      return NULL;
  }
  bus = of_mdio_find_bus(dn);
  if (!bus) {
    of_node_put(dn);

    printk (KERN_INFO "%s - cannot find mdio bus on target device node\n",__func__);
    return NULL;
  }
    
  bus->priv = dn;
  return bus;
}
EXPORT_SYMBOL_GPL(mv88X5113_mdio_bus_find);
/*
static int mv5113_power_down(struct phy_device *phydev) {
  int ret, val;
  struct mii_bus *bus;
  struct mv5113_priv *priv = dev_get_drvdata(&phydev->mdio.dev);
  if (!priv) {
    printk(KERN_INFO "%s - err: cannot get private data for mv88x5113 \n", __func__) ;
    return -ENODEV;
  }

  if (priv->addr < 0) {
    printk (KERN_INFO "%s - err: incorrect phy address in private data\n", __func__);
    return 1;
  }
  bus = priv->bus;
  ret = phy_set_bits_mmd(phydev, MDIO_MMD_VEND2, MV_V2_PORT_CTRL, MV_V2_PORT_CTRL_PWRDOWN);
  if (ret) {
    //printk(KERN_INFO "%s - cannot pass \'mv5113_power_down\' \n",__func__);
    return 1;
  }

  // need use MV88X5113_PORT_PWD_RESET here 
  val = bus->read(bus, priv->addr, MV88X5113_PORT_PWD_RESET);
  val |= MV_V2_PORT_CTRL_PWRDOWN;
  bus->write (bus, priv->addr, MV_V2_PORT_CTRL, val);

  return 0;
}
*/

/*
static int mv5113_port_power_up(struct phy_device *phydev)
{
  int ret;
  struct mii_bus *bus;
  struct mv5113_priv *priv = dev_get_drvdata(&phydev->mdio.dev);
  if (!priv || phydev->drv->phy_id != MARVELL_PHY_ID_88X5113) {
    printk(
    KERN_INFO "%s - err: phy ID is not match\n",__func__);
    return -ENODEV;
  }
  bus = priv->bus;

  ret = phy_write_mmd(phydev, MV88X5113_HOST_SIDE, MV88X5113_PCS_10G_CNTL, 0x2040);
  if (ret)
    ret = bus->write(bus, priv->addr, MV88X5113_PCS_10G_CNTL, 0x2040);
  if (ret != 0) {  
    printk(
    KERN_INFO "%s-err:cannot write value to reg \'MV88X5113_PCS_10G_CNTL\' err - %d\n",__func__, ret);
    return ret;
  }
  return 0;
}*/

static int mv5113_suspend(struct phy_device *phydev)
{
  //return mv5113_power_down(phydev);
  return 0;
}

static int mv5113_hwmon_config(struct phy_device *phydev, bool enable)
{
  int ret;
  //u16 val;
  printk (KERN_INFO "%s  ---  start \n", __func__);


  if (phydev->phy_id != MARVELL_PHY_ID_88X5113)  {
    ret = 2;  // not such device
    goto err1;
  }

  ret = phy_read_mmd(phydev, MV88X5113_VEND_REG, MV88X5113_MISC_INTR_REG);     //0xF41F
  if (ret == 0) { 

  }

  if (!(ret & MV88X5113_SET_HIGH_TEMP_INTR) && enable) {
    ret |= MV88X5113_SET_HIGH_TEMP_INTR;
    ret = phy_write_mmd(phydev, MV88X5113_VEND_REG, MV88X5113_MISC_INTR_REG, ret);
  }
  if (ret) {
    printk(KERN_INFO "%s - cannot enable high temp. interrupt\n",__func__);
    return ret;
  }


err1:
  printk (KERN_INFO "%s - err: ret == %d \n",__func__,ret);
  return ret;
}

static int mv_hwmon_config(
    struct phy_device *phydev, bool enable)
{
  return mv5113_hwmon_config(phydev, enable);
}
EXPORT_SYMBOL_GPL(mv_hwmon_config);

static int mv5113_resume(struct phy_device *phydev) 
{ /*
  int ret;
  ret = mv5113_power_up(phydev);
  if (ret) {
    return ret;
  }
  return mv5113_hwmon_config(phydev, true);  */  
  return 0; 
}
/*
static int mv5113_get_features(struct phy_device *phydev)
{
  // All supported linkmodes are set at probe
  return 0;
}   */

static int mv5113_enable_aneg(
    struct phy_device *phydev, MV88X5113_OP_CONFIG conf)
{
  int ret = -1;

  if (conf >= MV88X5113_OP_CONFIG_NUM)
    return -1;

  if (conf == MV88X5113_P10LN) {
    ret = phy_set_bits_mmd(phydev, MV88X5113_AUTONEG, MV88X5113_AUTONEG_CNTL_LINE,
      MV88X5113_SET_AUTONEG_ON_CTNL_LINE);
  }
  if (conf == MV88X5113_P10KN) {
    // On BE internal PHY aneg disabled by default
    // so we'll not set aneg mode on host side

    // need set autoNegPause (how - see mxd API)

    //  ret = phy_set_bits_mmd(
    //    phydev, MV88X5113_AUTONEG, MV88X5113_AUTONEG_CNTL_HOST,
    //    MV88X5113_SET_AUTONEG_ON_CTNL_HOST);
  }

  return ret;
}

static int mv5113_aneg_complete(struct phy_device *phydev)
{
  int val = 0;
  struct mv5113_priv *priv = dev_get_drvdata(&phydev->mdio.dev);
  struct mii_bus *bus = phydev->mdio.bus;
  if (!bus)
    return 1;

  val = phy_read_mmd(phydev, MV88X5113_AUTONEG, MV88X5113_AUTONEG_STATUS_LINE);
  if (val <= 0) {
    if (bus && bus->read && bus->write)
      val = bus->read (bus, priv->addr, MV88X5113_AUTONEG_STATUS_LINE);
  }

  if (val & MV88X5113_AUTONEG_COMPLETE)
    return 1;

  return 0;
}

static int mv_config_aneg(struct phy_device *phydev) {
  int ret = 0;

  if (!phydev)
    return -1;

  if (phydev->autoneg == AUTONEG_DISABLE) {
    // Enable aneg on line side interface
    ret = mv5113_enable_aneg(phydev, MV88X5113_P10LN);
    if(ret < 0) {
      printk (KERN_INFO "%s - err: \'mv5113_enable_aneg\' returned %d\n",__func__,ret);
      return ret;
    }
  }

  return ret;
}
EXPORT_SYMBOL_GPL(mv_config_aneg);

static int get_link_status_ext (struct phy_device *phydev)
{
  int val = -1;
  struct mv5113_priv *priv;
  printk (KERN_INFO "%s   -start \n",__func__);
  priv = dev_get_drvdata(&phydev->mdio.dev);
  if (!priv) {
    printk (KERN_INFO"%s - cannot get priv. data\n",__func__);
    return 1;
  }

  if(priv->interface == PHY_INTERFACE_MODE_10GKR) {
    val = phydev_read(phydev, 0x3, 0x2001);
  } 

  if (val > 0) return 1;
  else         return 0;
} 
/*
static int get_link_status(
    struct phy_device *phydev, MV88X5113_OP_CONFIG conf)
{
  int val = -1;
  struct mii_bus *bus;
  struct mv5113_priv *priv;

  switch (conf) {
    case MV88X5113_P10LN:
    case MV88X5113_P10KN:
      // try via mmd
      val = phy_read_mmd(phydev, MV88X5113_LINE_SIDE, MV88X5113_PCS_10G_STATUS);
      if (val & MV88X5113_PCS_LINK_UP)
        return 1;

      if (val < 0) {
        // now we will try via mdio gpio bitbanged bus    
        priv = dev_get_drvdata(&phydev->mdio.dev);
        if (priv) {
          bus = priv->bus;
          if (bus) {
            val = bus->read(bus, priv->addr, MV88X5113_PCS_10G_STATUS) & MV88X5113_PCS_LINK_UP;
            if (val)   return 1;
            else       return 0;
          }
        }
      }
      break;

    default:
      break;
  }
}  */

int mv_get_link_status(struct phy_device *phydev)
{
  return get_link_status_ext(phydev);
}
// EXTERNAL_SYMBOL_GPL(mv_get_link_status);

int mv_read_status(struct phy_device *phydev)
{
  int val = -1;
  val = get_link_status_ext(phydev);
  printk(KERN_INFO "%s - link status: %d\n",__func__,val);
  // val2 = get_link_status (phydev, MV88X5113_P10KN);
  // printk(KERN_INFO "%s - link status on host side: %d\n",__func__,val2);
  if (val)
    return 1;

  return 0;
}
//EXTERNAL_SYMBOL_GPL(mv_read_status);  // we can pass all code to AMD    driver instead THIS definition !!!

// manage temp. status
static int mv5113_hwmon_read_temp_reg(struct phy_device *phydev)
{
  // we need not get temp. param
  // we should control of high temp. interrupt
  // during passing suspend and resume actions

  // ret = phy_read_mmd(phydev, MV88X5113_CHIP_REG, MV_V2_TEMP);

  return 0;
}

static int mv5113_match_phy_device(struct phy_device *phydev)
{
  int ret;
  unsigned int phy_id;
  /*
  if (!(phydev->c45_ids | 0x0000)) {
    printk(KERN_INFO "%s - invalid priv. data\n",__func__);
    return -1;
  }   */
  printk (KERN_INFO "%s    MARVELL_PHY_ID_MASK == 0x%08x\n", __func__, MARVELL_PHY_ID_MASK);
  phy_id = phydev->c45_ids.device_ids[MDIO_MMD_PCS] & MARVELL_PHY_ID_MASK;
  printk(KERN_INFO "%s: matching PHY ID: phy_id - %ux\n", __func__, phy_id);

  printk (KERN_INFO "%s    MARVELL_PHY_ID_88X5113 == 0x%08x\n", __func__, MARVELL_PHY_ID_88X5113);

  ret = (phy_id == MARVELL_PHY_ID_88X5113);
  printk (KERN_INFO "%s - returned %d\n", __func__, ret);

  if (ret)   return ret;// for debug

  return 1;
}

int phydev_read(struct phy_device *phydev, int dev, u32 reg)
{
  if (phydev->mdio.bus && phydev->mdio.addr >= 0)
    return phy_read(phydev, reg);

  return phy_read_mmd(phydev, dev, reg);  
}

int phydev_write(struct phy_device *phydev, int dev, u32 reg, u16 val) 
{
  if (phydev->mdio.bus && phydev->mdio.addr >= 0)
    return phy_write(phydev, reg, val);

  return phy_write_mmd(phydev, dev, reg, val);
}

static int mv5113_config_init(struct phy_device *phydev)
{
  int err,ret;
  //struct mv5113_priv *priv = dev_get_drvdata(&phydev->mdio.dev);
  const struct mv5113_chip *chip;

  printk (KERN_INFO  "%s ---  \n",__func__);
  err = 0;
  chip = to_mv5113_chip(phydev);
  // write mode and speed for interface on line side
  ret = phydev_write(phydev, MV88X5113_LINE_SIDE, MV88X5113_MODE_REGISTER, 0x4003);
  if (ret) {
    printk(KERN_INFO"%s - cannot write mode to mv88x5113's registers\n",__func__);
    return 1;
  }
  // the same on host side
  ret = phy_write_mmd(
    phydev, MV88X5113_HOST_SIDE, MV88X5113_MODE_REGISTER, 0x4103);
  if (ret) {
    printk(KERN_INFO "%s - cannot write mode to mv88x5113's registers\n",__func__);
    return 1;
  }

  phydev->speed = SPEED_10000;
  phydev->duplex = DUPLEX_FULL;

  // call the soft reset after setting mode and speed
  mv5113_soft_reset(phydev);

  phydev->pause = 0;
  phydev->asym_pause = 0;
  phydev->interface = PHY_INTERFACE_MODE_10GKR;
  phydev->duplex = DUPLEX_FULL;
  phydev->speed = SPEED_10000;

  err = chip->init_interface(phydev);
  if (err) {
    printk(KERN_INFO "%s: cannot pass \'chip->init_interface\'\n",__func__);
    return err;
  }

  ret = mv5113_hwmon_config(phydev, true);
  if (ret != 0) {
    printk(KERN_INFO "%s - cannot enable temp interrupt\n",__func__);
    err++;
  }

  return err;
}

static const struct mv5113_chip mv5113_type =
{
  .init_supported_interfaces = mv5113_init_supported_interfaces,
  .init_interface = mv5113_init_interface,
#ifdef CONFIG_HWMON
  .hwmon_read_temp_reg = mv5113_hwmon_read_temp_reg,
#endif
};

static void *mv88x5113_of_get_data(struct phy_device *phydev)
{
  int ret, i;
  struct device_node *dn;
  struct mii_bus *mii_bus;
  struct mv5113_priv *pdata;
  enum of_gpio_flags flags;
  char mode[64];
  const char *pm = mode;
  int address = -1;
  struct device* dev;

  pdata = devm_kzalloc(&phydev->mdio.dev, sizeof(*pdata), GFP_KERNEL);
  if (unlikely(!pdata))
  {
    printk (
    KERN_INFO "%s - cannot allocate memory for struct mv88x5113_data \n",__func__);
    return pdata;
  }
  dev = &phydev->mdio.dev ;
  dn = phydev->mdio.dev.of_node;
  if (!dn) {
    dn = get_phydevice_node(dev);
  }
  if (dn) {
    pdata->phy_node = dn;
  } else {
    printk(KERN_INFO "%s - device node is not valid\n",__func__);
    return (void *)NULL;
  }

  // try to find phydev's mdio bus
  mii_bus = mv88X5113_mdio_bus_find(phydev);
  if(mii_bus) {
    pdata->bus = mii_bus;
    get_device(&mii_bus->dev);       //   get_device(&priv->master_mii_bus->dev);

    if (mii_bus->priv) {
      pdata->mii_dn = mii_bus->priv;
    }
  }

  ret = of_get_named_gpio_flags(dn, "mv-irq-pin", 0, &flags);
  if (ret >= 0) {
    pdata->irq = ret;
    pdata->irq_active_low = flags & OF_GPIO_ACTIVE_LOW;
    printk (KERN_INFO "%s - irq gpio pin = %d  \n",__func__,ret) ;
  }

  // try to get phy address
  of_property_read_u32(dn, "reg", &address);
  if (address >= 0)
    pdata->addr = address;

  pdata->line_mode = -1;
  pdata->host_mode = -1;

  ret = of_property_read_string(dn, "una,mv-line-mode", &pm);
  if (!ret) {
    for(i = 0; i < sizeof(line_modes)/sizeof(struct mode); ++i) {
      if(strcasecmp(line_modes[i].mode_name, pm) == 0) {
        pdata->line_mode = line_modes[i].mode_num;
        break;
      }
    }
  }
  ret = of_property_read_string(dn, "una,mv-host-mode", &pm);
  if (!ret) {
    for(i = 0; i < sizeof(host_modes) / sizeof(struct mode); ++i) {
      if(strcasecmp(host_modes[i].mode_name, pm) == 0) {
        pdata->host_mode = host_modes[i].mode_num;
        break;
      }
    }
  }

  return pdata;
}
EXPORT_SYMBOL_GPL(mv88x5113_of_get_data);

static int mv5113_hwmon_probe (struct phy_device *phydev)
{
  int val = -1;
  int ret = -1;
  printk (KERN_INFO "%s  -----start\n", __func__);

  if (phydev == NULL)  {
    printk  (KERN_INFO  "%s   phydev == NULL \n",__func__) ; return -1 ;

  }
  printk (KERN_INFO  "%s  phydev - is not NULL \n",__func__);

  val = phy_read_mmd(phydev, MV88X5113_VEND_REG, MV88X5113_MISC_INTR_REG);     //0xF41F
  if (val != 0)   {
    printk  (KERN_INFO  "%s   cannot read value from MV88X5113_VEND_REG  \n",__func__);
    return   val  ;
  }

  ret = mv5113_hwmon_config (phydev, true);
  printk (KERN_INFO"%s: val == %d returned ret == %d\n",__func__, val, ret) ;


  return ret;
}

// Device probed after reset or after power up 
static int mv5113_probe(struct phy_device *phydev)
{
  int ret;
  struct mv5113_priv *priv;
  //printk (KERN_INFO "%s   --- start \n",__func__);
  if (phydev->mdio.dev.of_node) {
    printk (KERN_INFO"%s - \'phydev->mdio.dev.of_node\' is NOT NULL\n", __func__);
    priv = mv88x5113_of_get_data(phydev);
    if (priv)
    printk (KERN_INFO "%s - priv is not NULL\n",__func__);
  }
  if (priv != NULL) { 
    printk (KERN_INFO"%s - phydev get data : Ok\n",__func__);
  }

  printk (KERN_INFO "%s    before phydev->interface : %2x\n", __func__, phydev->interface);
  if (phydev->interface != PHY_INTERFACE_MODE_10GKR)
    phydev->interface = PHY_INTERFACE_MODE_10GKR;

  printk (KERN_INFO "%s    after phydev->interface : %2x\n", __func__, phydev->interface);

  // Marvell10g.c (drivers\net\phy):	dev_set_drvdata(&phydev->mdio.dev, priv);
  dev_set_drvdata(&phydev->mdio.dev, priv);

  // save private data
  phydev->priv = priv;

  ret = mv5113_hwmon_probe (phydev);
  if (ret != 0)
    goto err0;

err0:
  printk (KERN_INFO "%s - phydev \'%s\' probed at 0x%02x: returned value - %d\n",__func__,
    (phydev->drv->name != 0) ? phydev->drv->name : "name-NONE", phydev->mdio.addr, ret);

  return ret;
}

static int mv_chip_probe(struct phy_device *phy)
{
  return mv5113_probe(phy);
}
EXPORT_SYMBOL_GPL(mv_chip_probe);

static struct phy_driver rtt_10g_drivers[] = {
  {
  .phy_id = MARVELL_PHY_ID_88X5113,
  .phy_id_mask = 0xfffffff0,
  .match_phy_device  = mv5113_match_phy_device,
  .name  = "mv88x5113",
  .driver_data = &mv5113_type,
  //.get_feature = mv5113_get_features,
  .config_init = mv5113_config_init,
  .probe = mv5113_probe,
  .suspend = mv5113_suspend,
  .resume = mv5113_resume,
  .config_aneg = mv_config_aneg,
  .aneg_done = mv5113_aneg_complete,
  .read_status = mv_read_status,
  .soft_reset = mv5113_soft_reset,
  },
};

static const struct of_device_id mv88x5113_of_match[] = {
  {
    .compatible = "marvell,mv88x5113",
    .compatible = "una,mv88x5113",
  },
};

module_phy_driver(rtt_10g_drivers);

static struct mdio_device_id __maybe_unused rtt_10g_tbl[] = {
  { MARVELL_PHY_ID_88X5113, 0xfffffff0 },
  { },
};

MODULE_DEVICE_TABLE(mdio, /*mv5113_tbl*/rtt_10g_tbl);
