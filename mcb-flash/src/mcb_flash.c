/************************************************************************
 * Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
 * SPDX-License-Identifier: BSD-3-Clause-Clear
 *************************************************************************/

#include "mcb_flash.h"

#include <assert.h>
#include <fcntl.h>
#include <stdbool.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <sys/ioctl.h>
#include <sys/types.h>
#include <unistd.h>

#include "qti_qrc_udriver.h"
#include "xmodem.h"

#define VERSION_STR ("v1.0\n")

#define NUTTX_BIN "/data/misc/mcb/nuttx.bin"

#define CMD_FLASH '1'
#define CMD_BOOT '2'

/************************qrc functions **********************************/

int mcb_qrc_open(char * dev_path)
{
#ifdef QRC_USER_DRIVER
  return qrc_udriver_open(dev_path);
#else
  return open(dev_path, O_RDWR);
#endif
}

void mcb_qrc_close(int fd)
{
#ifdef QRC_USER_DRIVER
  qrc_udriver_close(fd);
#else
  close(fd);
#endif
}

int mcb_qrc_clean_rx_buff(int fd)
{
  if (fd < 0) {
    printf("Dev QRC Error!\n");
    return -1;
  }

#ifdef QRC_USER_DRIVER
  if (qrc_udriver_tcflsh(fd) < 0)
#else
  if (ioctl(fd, QRC_FIFO_CLEAR) < 0)
#endif
  {
    printf("qrc clean read buffer fail\n");
    return -1;
  }

  return 0;
}

int mcb_qrc_write(int fd, char * data, size_t data_len)
{
  int res = -1;

  if (fd < 0) {
    printf("Dev QRC Error!\n");
    return -1;
  }

  if (data == NULL) {
    printf("Error: QRC data pointer invalid!\n");
    return -1;
  }

#ifdef QRC_USER_DRIVER
  res = qrc_udriver_write(fd, data, data_len);
#else
  res = write(fd, data, data_len);
#endif

  return res;
}

size_t mcb_qrc_read(int fd, char * data, size_t data_len)
{
  int res = 0;

  if (fd < 0) {
    printf("Dev QRC Error!\n");
    return 0;
  }

  if (data == NULL) {
    printf("Error: QRC data pointer invalid!\n");
    return 0;
  }

#ifdef QRC_USER_DRIVER
  res = qrc_udriver_read(fd, data, data_len);
#else
  res = read(fd, data, data_len);
#endif

  return res;
}

int mcb_reboot(int fd)
{
  if (fd < 0) {
    printf("Dev QRC Error!\n");
    return -1;
  }

#ifdef QRC_USER_DRIVER
  if (qrc_mcb_reset(QRC_GPIOCHIP, QRC_RESETGPIO) < 0)
#else
  if (ioctl(fd, QRC_REBOOT) < 0)
#endif
  {
    printf("mcb reboot fail\n");
    return -1;
  }

  return 0;
}

/*****************************flash function *********************************/

/* check if MCB in bootloader mode with char 'a', try 3 times */
static bool mcb_check_bootloader_mode(int fd)
{
  int try = 10;
  bool ret = false;
  char rec_data[2];
  ssize_t rec_bytes;

  while (try--) {
    sleep(3);
    rec_bytes = mcb_qrc_read(fd, rec_data, sizeof(rec_data));
    if (rec_bytes == 2) {
      if (rec_data[0] == X_a && rec_data[1] == X_a) {
        return true;
      }
    }
    if (try < 6) {
      mcb_reboot(fd);
    }
  }

  printf("\ncheck bootloader mode failed\n");
  printf("\nplease check MCB bootloader \n");
  return false;
}

static bool clean_qrc_buff(int fd)
{
  int try = 10;
  bool ret = false;
  char rec_data[20001];
  ssize_t rec_bytes;

  while (try--) {
    rec_bytes = mcb_qrc_read(fd, rec_data, 20000);
    if (rec_bytes < 0) {
      return false;
    }
    if (rec_bytes < 20000) {
      return true;
    }
    if (try < 6) {
      mcb_reboot(fd);
    }
  }

  printf("\n clean qrc failed\n");
  return false;
}

static bool bootloader_flash(int fd, char * bin_arr, size_t bin_size, int * p_count)
{
  char cmd;
  int retry = 20;
  char rec_data;

  // send '1' cmd
  cmd = CMD_FLASH;
  mcb_qrc_write(fd, &cmd, sizeof(cmd));
  printf("Waiting for Xmodem handshake...");

  while (retry--) {
    usleep(50000);
    if (mcb_qrc_read(fd, &rec_data, 1)) {
      printf("receive handshake =%d\n", rec_data);
      if (rec_data == X_C) {
        printf("Xmodem protocol begin\n");
        return xmodem_send(fd, bin_arr, bin_size, p_count);
      }
    }
  }

  return false;
}

/************************** public control interface ************************************/
void usage()
{
  printf("mcbflash  -  MCB flash tool.\n");
  printf("   --help:\n");
  printf("         prints this help.\n");
  printf("   --version:\n");
  printf("         prints the program version string.\n");
  printf("   --reboot:\n");
  printf("         reboot MCB hardware.\n");
  printf("   --flash:\n");
  printf("         flash the mcb with nuttx-image.\n");
  printf("         please make sure the FW in /opt/qcom/qirf-sdk/data/misc/mcb/nuttx.bin\n");
  printf("\n");
}

int print_help(void * p)
{
  usage();
  return 0;
}

int print_version(void * p)
{
  printf("Version: %s\n", VERSION_STR);
  return 0;
}
/*****************************action function***********************************/
int do_reboot(void * p)
{
  int fd = mcb_qrc_open(QRC_DEV);

  if (fd < 0) {
    printf("Open Dev QRC Error!\n");
    mcb_qrc_close(fd);
    return -1;
  }
  mcb_reboot(fd);
  mcb_qrc_close(fd);

  return 0;
}

int do_flash(void * p, int * p_count)
{
  int fd = *(int *)p;
  FILE * bin_file;
  int bin_size = 0;
  char * bin_arr;
  size_t read_num;
  int ret = -1;
  char cmd;

  if (fd < 0) {
    printf("Open Dev QRC Error!\n");
    return -1;
  }

  // reboot mcb
  mcb_reboot(fd);

  // clean qrc
  mcb_qrc_clean_rx_buff(fd);
  clean_qrc_buff(fd);
  // check nuttx.bin file
  bin_file = fopen(NUTTX_BIN, "r");  // open the file
  if (bin_file == NULL) {
    printf("%s file does not exist\n", NUTTX_BIN);
    return -1;
  }
  // read nutttx.bin file.
  if (fseek(bin_file, (long long)(0), SEEK_END)) {
    printf("fseek() function failed!\n");
    goto err_return;
  }
  bin_size = ftell(bin_file);
  printf(" nuttx.bin size=%d\n", bin_size);
  bin_arr = (char *)malloc(bin_size * sizeof(char));

  if (fseek(bin_file, (long long)(0), SEEK_SET)) {
    printf("fseek() function failed!\n");
    goto err_return;
  }

  read_num = fread(bin_arr, sizeof(char), bin_size, bin_file);

  if (read_num == bin_size) {
    printf(" read nuttx.bin successed\n");

    // check if attached bootloader.
    if (false == mcb_check_bootloader_mode(fd))
      goto err_return;
    printf(" mcb bootloader detected \n");
    // xmodem send data (128 bytes mode)

    if (!bootloader_flash(fd, bin_arr, bin_size, p_count)) {
      printf(" failed flash  nuttx.bin \n");
      goto err_return;
    }
    printf(" flash nuttx.bin successed !\n");
    ret = 0;
  } else {
    printf(" read nuttx.bin failed num = %ld\n", read_num);
    goto err_return;
  }

  printf(" Reboot MCB  !\n");
  usleep(300000);
  mcb_reboot(fd);
  printf(" Reboot MCB done !\n");
err_return:
  free(bin_arr);
  fclose(bin_file);
  return ret;
}
