/************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*************************************************************************/

#ifndef _MCB_FLASH_H_
#define _MCB_FLASH_H_

#define VERSION_STR         ("v1.0\n")

//#define NUTTX_BIN "/data/misc/mcb/nuttx.bin"
#define NUTTX_BIN "./nuttx.bin"

#define QRC_DEV  "/dev/qrc"

/* IOCTL commands */
#define QRC_IOC_MAGIC   'q'
/* Clear read fifo */
#define QRC_FIFO_CLEAR  _IO(QRC_IOC_MAGIC, 1)
/* Reboot QRC controller */
#define QRC_REBOOT      _IO(QRC_IOC_MAGIC, 2)
/* QRC boot from memory */

#define CMD_FLASH   '1'
#define CMD_BOOT    '2'

int qrc_clean_rx_buff(int fd);
void usage();
int print_help(void *p);
int print_version(void *p);
int do_reboot(void *p);
int do_flash(void *p, int* p_count);

#endif //_LIBQRC_H_
