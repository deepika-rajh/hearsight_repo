/************************************************************************
 *Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All rights reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *************************************************************************/

#ifndef _MCB_FLASH_H_
#define _MCB_FLASH_H_

#include <stddef.h>

#ifdef QRC_USER_DRIVER

#ifdef QRC_RB3
#define QRC_THREAD_NUM (2)
#define QRC_IOC_MAGIC 'q'
#define QRC_FIONREAD _IO(QRC_IOC_MAGIC, 5)
#define QRC_RESET_MCB _IO(QRC_IOC_MAGIC, 2)
#define QRC_DEV ("/dev/ttyHS2")
#define QRC_BOOT_APP '2'
#define QRC_GPIOCHIP ("/dev/gpiochip4")
#define QRC_RESETGPIO 147
#define QRC_MAX_READ_SIZE 1024
#endif

#ifdef QRC_RB8
#define QRC_THREAD_NUM (2)
#define QRC_IOC_MAGIC 'q'
#define QRC_FIONREAD _IO(QRC_IOC_MAGIC, 5)
#define QRC_RESET_MCB _IO(QRC_IOC_MAGIC, 2)
#define QRC_DEV ("/dev/ttyHS2")
#define QRC_BOOT_APP '2'
#define QRC_GPIOCHIP ("/dev/gpiochip4")
#define QRC_RESETGPIO 129
#define QRC_MAX_READ_SIZE 1024
#endif

#ifdef QRC_RB5GEN2
#define QRC_GPIOCHIP ("/dev/gpiochip0")
#define QRC_RESETGPIO 168
#define QRC_MAX_READ_SIZE 1024
#define QRC_DEV "/dev/ttyHS1"
#endif

#endif

#ifndef QRC_DEV
#define QRC_GPIOCHIP ("/dev/gpiochip0")
#define QRC_RESETGPIO 168
#define QRC_MAX_READ_SIZE 1024
#define QRC_DEV "/dev/ttyHS1"

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
