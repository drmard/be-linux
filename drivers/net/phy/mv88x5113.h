#ifndef __MARVELL_88X5113_H
#define __MARVELL_88X5113_H

#define MV88X5113_RESULT_OK               0  // Operation succeeded
#define MV88X5113_RESULT_FAIL             1  // Operation failed
#define MV88X5113_RESULT_PENDING          2  // Pending  
#define MV88X5113_RESULT_AUTONEG_FAIL    -1

#define MV88X5113_NUMBER_OF_PORTS         1
#define MARVELL_PHY_ID_88X5113            0x002b0b45
  
enum {
	MV_V2_5113_PORT_CTRL_MACTYPE_RXAUI			= 0x0,
	MV_V2_5113_PORT_CTRL_MACTYPE_XAUI_RATE_MATCH		= 0x1,
	MV_V2_5113_PORT_CTRL_MACTYPE_RXAUI_NO_SGMII_AN		= 0x1,
	MV_V2_5113_PORT_CTRL_MACTYPE_RXAUI_RATE_MATCH		= 0x2,
	MV_V2_5113_PORT_CTRL_MACTYPE_XAUI		        	= 0x3,
	MV_V2_5113_PORT_CTRL_MACTYPE_10GBASER			= 0x4,
	MV_V2_5113_PORT_CTRL_MACTYPE_10GBASER_NO_SGMII_AN	= 0x5,
	MV_V2_5113_PORT_CTRL_MACTYPE_10GBASER_RATE_MATCH	= 0x6,
	MV_V2_5113_PORT_CTRL_MACTYPE_USXGMII			= 0x7,
	MV_V2_5113_PORT_CTRL_MACTYPE_10GKR_RATE_MATCH	= 0x8,
};

/* opModeTable needs to match up with MV88X5113_OP_CONFIG */
typedef enum
{
    MV88X5113_CONFIG_UNKNOWN,
    MV88X5113_P1X,        /* 1GB mode, 1 lane, non-grouped  = 1*/
    MV88X5113_P1S,        /* 1GB mode, 1 lane, non-grouped */
    MV88X5113_P1P,        /* 1GB mode, 1 lane, non-grouped */  
    MV88X5113_R1,         /* 1GB mode, 1 lane, non-grouped */ 

    MV88X5113_P2_5X,      /* 2.5GB mode, 1 lane, non-grouped  = 5 */
    MV88X5113_R2_5,       /* 2.5GB mode, 1 lane, non-grouped */

    MV88X5113_P5L,        /* 5GB mode, 1 lane, non-grouped   = 7 */ 
    MV88X5113_P5K,        /* 5GB mode, 1 lane, non-grouped   */ 
    MV88X5113_R5,         /* 5GB mode, 1 lane, non-grouped   */

    MV88X5113_P10LN,      /* 10GB mode, 1 lane, non-grouped  = 10 */
    MV88X5113_P10KN,      /* 10GB mode, 1 lane, non-grouped  */
    MV88X5113_P10KF,      /* 10GB mode, 1 lane, non-grouped  */
    MV88X5113_P10LF,      /* 10GB mode, 1 lane, non-grouped  */
    MV88X5113_R10L,       /* 10GB mode, 1 lane, non-grouped  */
    MV88X5113_R10K,       /* 10GB mode, 1 lane, non-grouped  */

    MV88X5113_P25LN,      /* 25GB mode, 1 lane = 16 */
    MV88X5113_P25LF,      /* 25GB mode, 1 lane */
    MV88X5113_P25LR,      /* 25GB mode, 1 lane */
    MV88X5113_P25CN,      /* 25GB mode, 1 lane */
    MV88X5113_P25CF,      /* 25GB mode, 1 lane */
    MV88X5113_P25CR,      /* 25GB mode, 1 lane */
    MV88X5113_P25KN,      /* 25GB mode, 1 lane */
    MV88X5113_P25KF,      /* 25GB mode, 1 lane */
    MV88X5113_P25KR,      /* 25GB mode, 1 lane */

    MV88X5113_P25BN,      /* 25GB mode, 1 lane = 25 */
    MV88X5113_P25BF,      /* 25GB mode, 1 lane */
    MV88X5113_P25BR,      /* 25GB mode, 1 lane */
    MV88X5113_P25JN,      /* 25GB mode, 1 lane */
    MV88X5113_P25JF,      /* 25GB mode, 1 lane */
    MV88X5113_P25JR,      /* 25GB mode, 1 lane */

    MV88X5113_P25BCN,     /* 25GB mode, 1 lane = 31 */ /* Line side only */
    MV88X5113_P25BCF,     /* 25GB mode, 1 lane */ /* Line side only */
    MV88X5113_P25BCR,     /* 25GB mode, 1 lane */ /* Line side only */
    MV88X5113_P25JKN,     /* 25GB mode, 1 lane */ /* Line side only */
    MV88X5113_P25JKF,     /* 25GB mode, 1 lane */ /* Line side only */
    MV88X5113_P25JKR,     /* 25GB mode, 1 lane */ /* Line side only */ 
    MV88X5113_R25L,       /* 25GB mode, 1 lane */
    MV88X5113_R25C,       /* 25GB mode, 1 lane */
    MV88X5113_R25K,       /* 25GB mode, 1 lane */

    MV88X5113_P29P09LN,   /* 29.09GB mode, 4 lanes, grouped */ /* Host side only */

    MV88X5113_P40LN,      /* 40GB mode, 4 lanes, grouped = 41 */
    MV88X5113_P40CN,      /* 40GB mode, 4 lanes, grouped  */
    MV88X5113_P40CF,      /* 40GB mode, 4 lanes, grouped  */
    MV88X5113_P40KN,      /* 40GB mode, 4 lanes, grouped  */
    MV88X5113_P40KF,      /* 40GB mode, 4 lanes, grouped  */
    MV88X5113_P40LF,      /* 40GB mode, 4 lanes, grouped  */
    MV88X5113_R40L,       /* 40GB mode,  4 lanes, grouped */
    MV88X5113_R40C,       /* 40GB mode,  4 lanes, grouped */
    MV88X5113_R40K,       /* 40GB mode,  4 lanes, grouped */

    MV88X5113_P50LN,      /* 50GB mode, 4 lanes, grouped  = 50 */
    MV88X5113_P50CN,      /* 50GB mode, 4 lanes, grouped  */ /* work-in-progress */
    MV88X5113_P50CF,      /* 50GB mode, 4 lanes, grouped  */ /* work-in-progress */
    MV88X5113_P50KN,      /* 50GB mode, 4 lanes, grouped  */ /* work-in-progress */
    MV88X5113_P50KF,      /* 50GB mode, 4 lanes, grouped  */ /* work-in-progress */
    MV88X5113_P50LF,      /* 50GB mode, 4 lanes, grouped  */
    MV88X5113_P50MN,      /* 50GB mode, 2 lanes, grouped  */
    MV88X5113_P50MF,      /* 50GB mode, 2 lanes, grouped  */
    MV88X5113_P50MR,      /* 50GB mode, 2 lanes, grouped  */
    MV88X5113_P50BN,      /* 50GB mode, 2 lanes, grouped  */
    MV88X5113_P50BF,      /* 50GB mode, 2 lanes, grouped  */
    MV88X5113_P50BR,      /* 50GB mode, 2 lanes, grouped  */
    MV88X5113_P50JN,      /* 50GB mode, 2 lanes, grouped  */
    MV88X5113_P50JF,      /* 50GB mode, 2 lanes, grouped  */
    MV88X5113_P50JR,      /* 50GB mode, 2 lanes, grouped  */

    MV88X5113_P100LN,     /* 100GB mode, 4 lanes, grouped = 65 */
    MV88X5113_P100LR,     /* 100GB mode, 4 lanes, grouped */
    MV88X5113_P100CR,     /* 100GB mode, 4 lanes, grouped */
    MV88X5113_P100KR,     /* 100GB mode, 4 lanes, grouped */
    MV88X5113_P100KN,     /* 100GB mode, 4 lanes, grouped */ /* work-in-progress */
    MV88X5113_R100L,      /* 100GB mode, 4 lanes, grouped */
    MV88X5113_R100C,      /* 100GB mode, 4 lanes, grouped */
    MV88X5113_R100K,      /* 100GB mode, 4 lanes, grouped */

    MV88X5113_OP_CONFIG_NUM

} MV88X5113_OP_CONFIG;

typedef enum
{
    MV88X5113_MODE_UNKNOWN,

    MV88X5113_MODE_P1X_P1P,      /* mode selection = 1 */
    MV88X5113_MODE_P1X_P1X,

    MV88X5113_MODE_P10LN_P10LF,
    MV88X5113_MODE_P10LN_P10LN,
    MV88X5113_MODE_P10LN_P10KF,
    MV88X5113_MODE_P10LN_P10KN,      // our mode

    MV88X5113_MODE_P10KN_P10LN,
    MV88X5113_MODE_P10KN_P10KN,
    MV88X5113_MODE_P10KN_P10KF,
    MV88X5113_MODE_P10KF_P10KF,

    MV88X5113_MODE_NUM,
    MV88X5113_MODE_USER_DEFINED = 0xFFFF
}   MV88X5113_OP_MODE;

/* 802.3AP Auto-Negotiation Advertisement Register 1 on MMD Device 7 */
#define MV88X5113_AUTONEG_ADV1_HOST          0x1010

#define MV88X5113_AUTONEG_CNTL_LINE          0x0000
/* Auto-Negotiation On Line Side Status Register */
#define MV88X5113_AUTONEG_STATUS_LINE        0x0001

#define MV88X5113_AUTONEG_CNTL_HOST          0x1000
/* Auto-Negotiation On Host Side Status Register */
#define MV88X5113_AUTONEG_STATUS_HOST        0x1001

#define MV88X5113_AUTONEG_COMPLETE           BIT(5)  //AutoNegotiation Complete on Dev 7
#define MV88X5113_PCS_LINK_UP                BIT(2)
#define MV88X5113_SET_AUTONEG_ON_CTNL_LINE   BIT(12)
#define MV88X5113_SET_AUTONEG_ON_CTNL_HOST   BIT(12)
#define MV88X5113_SET_LANE_SOFT_RESET        BIT(15)

#define MV88X5113_SET_HIGH_TEMP_INTR         BIT(15)

#define MV88X5113_SW_RST_LINE    (1UL << 15)
#define MV88X5113_SW_RST_HOST    (1UL << 7)
#define MV88X5113_SW_RST_ALL     (MV88X5113_SW_RST_HOST | MV88X5113_SW_RST_LINE)

#define MV88X5113_IO_ERROR        0xFFFF /* Error reading or writing MDIO register */

#define MV88X5113_LINE_PMA        1
#define MV88X5113_LINE_SIDE       3
#define MV88X5113_HOST_SIDE       4
#define MV88X5113_BOTH_SIDE       5
#define MV88X5113_AUTONEG         7
#define MV88X5113_PORT_REG       31
#define MV88X5113_CHIP_REG       31
#define MV88X5113_VEND_REG       31
#define MV88X5113_MAX_MDIO_NUM   31

#define MXD_PMA_10G_CNTL                 0x4000
#define MV88X1553_PCS_LANE_OFFSET_SHIFT  0x200

#define  MV88X5113_PCS_10G_CNTL          0x2000 //MXD_PCS_25G_CTNL
//#define  MV88X5113_PCS_10G_CNTL(laneX)   MV88X5113_PCS_10G_CNTL + (laneX * MV88X1553_PCS_LANE_OFFSET_SHIFT)

#define  MV88X5113_PCS_10G_STATUS        0x2001 //MXD_PCS_25G_STATUS in mxd driver
#define  MV88X5113_PCS_10G_PCS_STATUS2   0x2008
#define  MXD_PCS_10G_INTR_EN             0xA000
#define  MXD_PCS_10G_INTR_STAT           0xA001
#define  MXD_PCS_10G_RT_STAT             0xA002

#define  MV88X5113_MODE_REGISTER         0xF000 

#define  MV88X5113_PORT_RESET            0xF003
#define  MV88X5113_PORT_PCS_CNTL         0xF010
#define  MV88X5113_MISC_INTR_REG         0xF41F   // we can use this reg to raise high temp Interrupt

#define MV88X5113_RESET_DELAY_US         500

static void *mv88x5113_of_get_data(struct phy_device *);
static struct phy_device *parse_mv_phydevice(struct device_node *);
static struct device_node *get_phydevice_node(struct device *);
int set_mdio_bus(struct phy_device *);
int mv_get_link_status(struct phy_device *); 
int mv_read_status(struct phy_device *);
static int mv_hwmon_config(struct phy_device *, bool);
static int mv_chip_probe(struct phy_device *);
int mv_soft_reset(struct phy_device *);
static int mv_config_aneg(struct phy_device *);
struct mii_bus *mv88X5113_mdio_bus_find (struct phy_device *);
int phydev_read(struct phy_device *, int, u32);
int phydev_write(struct phy_device *, int, u32, u16); 

#endif /* __MARVELL_88X5113_H      */
