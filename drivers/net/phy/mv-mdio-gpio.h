/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_MV_MDIO_GPIO_H
#define __LINUX_MV_MDIO_GPIO_H

static void mv_mdiobb_send_num(struct mdiobb_ctrl *ctrl, u16 val, int bits);
static int mv_mdiobb_cmd_addr(struct mdiobb_ctrl *ctrl, int phy, u32 addr);

int mv_mdiobb_read (struct mii_bus *, int, int);
int mv_mdiobb_write (struct mii_bus *, int, int, u16);
int mv_mdiobb_reset(struct mii_bus *);

#endif
