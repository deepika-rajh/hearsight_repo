/************************************************************************
 *Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All rights reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *************************************************************************/

#ifndef _MCB_FLASH_H_
#define _MCB_FLASH_H_

#include <stddef.h>

#ifdef QRC_USER_DRIVER
#define QRC_GPIOCHIP ("/dev/gpiochip0")
#define QRC_RESETGPIO 168
#define QRC_MAX_READ_SIZE 1024
#define QRC_DEV "/dev/ttyHS1"
#else
/* IOCTL commands */
#define QRC_IOC_MAGIC 'q'
/* Clear read fifo */
#define QRC_FIFO_CLEAR _IO(QRC_IOC_MAGIC, 1)
/* Reboot QRC controller */
#define QRC_REBOOT _IO(QRC_IOC_MAGIC, 2)
#define QRC_DEV "/dev/qrc"
#endif

void usage();
int print_help(void * p);
int print_version(void * p);
int do_reboot(void * p);
int do_flash(void * p, int * p_count);

int mcb_qrc_open(char * dev_path);
void mcb_qrc_close(int fd);
int mcb_qrc_clean_rx_buff(int fd);
size_t mcb_qrc_read(int fd, char * data, size_t data_len);
int mcb_qrc_write(int fd, char * data, size_t data_len);
int mcb_reboot(int fd);

#endif  //_LIBQRC_H_
