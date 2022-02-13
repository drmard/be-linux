#ifndef __XGBE_RTT_H__
#define __XGBE_RTT_H__

#include "xgbe.h"

#define MV88X5113_RESET_DELAY_US    500
#define MV88X5113_PORT_RESET        0xF003
#define MV88X5113_PORT_REG          31
#define MV88X5113_SW_RST_ALL        (1UL << 15) | (1UL << 7)  
#define MV88X5113_LINE_SIDE         3
#define MV88X5113_AUTONEG           7
#define MV88X5113_AUTONEG_CNTL_LINE 0x0000
#define MV88X5113_SET_AUTONEG_ON       BIT(12)
#define MV88X5113_10GBASE_PCS_STATUS   BIT(2)  

struct mv5113_priv {
  struct mii_bus *bus;
  struct device_node *phy_node;
  struct device_node *mii_dn;
  struct phy_device *phydevice;

  DECLARE_BITMAP(supported_interfaces, PHY_INTERFACE_MODE_MAX);
  bool rate_match;
  phy_interface_t interface;
  int irq;
  int irq_active_low;
  int line_mode;
  int host_mode;
  int addr;
  void *priv;
};

struct mv_error {
  int error;
  char description[64];
};

static struct mii_bus *mv_mdio_bus_find_ex(struct xgbe_prv_data *);
struct mv_error *una_phy_config_init(struct xgbe_prv_data *);

static inline 
struct mii_bus *mv5113_mdio_bus_find_ex (struct phy_device *phydev)
{
  struct device_node *dn;
  struct mii_bus *bus;
  if (phydev)
    dn = phydev->mdio.dev.parent->of_node;
  if (!phydev || !dn) {
    // try to find some una/mv mdio bus node
    dn = of_find_compatible_node(NULL, NULL, "mv,mv-mdio-gpio");
    if (!dn) {
      printk (KERN_INFO "%s - cannot find compat node for mdio gpio bus\n",__func__) ; 
      return NULL;
    }
  }
  bus = of_mdio_find_bus(dn);
  if (!bus) {
    of_node_put(dn);
    printk (KERN_INFO "%s - cannot find mdio bus\n",__func__);
    return NULL;
  } 
  bus->priv = dn;
  return bus;
}

static inline
struct mii_bus *mv_mdio_bus_find_ex(
    struct xgbe_prv_data *pdata)
{
  struct phy_device *phydev = pdata->mv_phydev;
  // we can have phydev is equal NULL
  // we should find mii_bus as the parent dev
  // for phydev
  return mv5113_mdio_bus_find_ex(phydev);
}

static inline
int phydev_read(struct phy_device *phydev, int dev, u32 reg)
{
  if (phydev->mdio.bus && phydev->mdio.addr >= 0)
    return phy_read(phydev, reg);

  return phy_read_mmd(phydev, dev, reg);  
}

static inline
int phydev_write(struct phy_device *phydev, int dev, u32 reg, u16 val) 
{
  if (phydev->mdio.bus && phydev->mdio.addr >= 0)
    return phy_write(phydev, reg, val);

  return phy_write_mmd(phydev, dev, reg, val);
}

static inline int
mv_link_status_ext(struct phy_device *phydev)
{
  int val = -1; 
  //int ret = 0;
  struct mv5113_priv *priv;
  struct mv_error *err;
  err = kzalloc(sizeof(*err), GFP_KERNEL);
  if(!err)
    goto err2; 
  err->error = 0;

  priv = dev_get_drvdata(&phydev->mdio.dev);
  if (!priv) {
    printk (KERN_INFO"%s - cannot get private data\n",__func__);
    err->error = -1;
    goto err2;
  }
  if(priv->interface == PHY_INTERFACE_MODE_10GKR) {
    val = phydev_read(phydev, MV88X5113_LINE_SIDE, 0x2001);
    if (val & MV88X5113_10GBASE_PCS_STATUS)
      err->error = 1;
    else
      err->error = 0;
  } else {
    printk (  
    KERN_INFO "%s : we have incorrect priv->interface value in the PHY config\n",__func__);
    err->error = -1;
  }

err2:
  if (!err)
    return -1;
  if (err)
    kfree(err);

  return err->error;
}

static int mv5113_enable_aneg(struct phy_device *phydev)
{
  int ret = -1,val;
  ret = phy_set_bits_mmd(phydev, MV88X5113_AUTONEG, MV88X5113_AUTONEG_CNTL_LINE,
    MV88X5113_SET_AUTONEG_ON);
  if (ret < 0) {
    val = phydev_read(phydev, MV88X5113_AUTONEG, MV88X5113_AUTONEG_CNTL_LINE); 
    if (val < 0)
      return val;
    val = val | MV88X5113_SET_AUTONEG_ON;
    ret = phydev_write(phydev, MV88X5113_AUTONEG, MV88X5113_AUTONEG_CNTL_LINE, val);
    printk (KERN_INFO"%s - \'phydev_write\' passed...   returned value: %d\n",__func__,ret);
  }

  return ret;
}

static inline int 
mv_config_aneg(struct phy_device *phydev) {
  int ret = -1;
  if (phydev) {
  if (phydev->autoneg == AUTONEG_DISABLE) {
    // Enable aneg on line side interface
    ret = mv5113_enable_aneg(phydev);
    if(ret < 0) {
      printk (KERN_INFO "%s - err: \'mv5113_enable_aneg\' returned %d\n",__func__,ret);
      return ret;
    }
  }
  }

  printk (KERN_INFO "%s - returned %d\n",__func__,ret);
  return ret;
}

static inline int mv_soft_reset(struct phy_device *phydev) {
  int ret, val = -1;
  int cnt = 50;
  val = phy_read_mmd(phydev, MV88X5113_PORT_REG, MV88X5113_PORT_RESET);
  if (val < 0) {
    val = phydev_read(phydev, 31, MV88X5113_PORT_RESET);
    if (val < 0)
      return val;
  }
  // we should make reset now
  val |= (1UL << 15) | (1UL << 7);
  ret = phydev_write(phydev, 31, 0xF003, val);
  if (ret) {
    printk (KERN_INFO "%s - cannot perform soft reset\n",__func__);
    return ret;
  }
  do {
    usleep_range(MV88X5113_RESET_DELAY_US, MV88X5113_RESET_DELAY_US + 100);
    ret = phy_read_mmd(phydev, MV88X5113_PORT_REG, MV88X5113_PORT_RESET);
  } while ((ret & (MV88X5113_SW_RST_ALL)) || cnt--);

  return 0;   
}

#endif
