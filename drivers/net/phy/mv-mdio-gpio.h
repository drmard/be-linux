/* SPDX-License-Identifier: GPL-2.0 */
#ifndef __LINUX_MV_MDIO_GPIO_H
#define __LINUX_MV_MDIO_GPIO_H

static void mv_set_mdc(struct mdiobb_ctrl *, int);
static void mv_set_mdio_dir(struct mdiobb_ctrl *, int);
static void mv_set_mdio_data(struct mdiobb_ctrl *, int);
static int mv_get_mdio_data(struct mdiobb_ctrl *);
static void mv_mdiobb_send_num(struct mdiobb_ctrl *ctrl, u16 val, int bits);
static int mv_mdiobb_cmd_addr(struct mdiobb_ctrl *ctrl, int phy, u32 addr);

#endif
