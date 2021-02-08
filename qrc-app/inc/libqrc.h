/*
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef _LIBQRC_H_
#define _LIBQRC_H_

/* IOCTL commands */
#define QRC_IOC_MAGIC   'q'

/* Clear read fifo */
#define QRC_FIFO_CLEAR  _IO(QRC_IOC_MAGIC, 1)
/* Reboot QRC controller */
#define QRC_REBOOT      _IO(QRC_IOC_MAGIC, 2)
/* QRC boot from memory */
#define QRC_BOOT_TO_MEM _IO(QRC_IOC_MAGIC, 3)
/* QRC boot from flash */
#define QRC_BOOT_TO_FLASH       _IO(QRC_IOC_MAGIC, 4)

#define QRC_DEV  "/dev/qrc"


#endif //_LIBQRC_H_
