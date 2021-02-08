/*
 * Copyright (c) 2021 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>

#include "libqrc.h"

#define VERSION_STR         ("v1.0\n")

int main(int argc, char *argv[])
{
   unsigned int i;
   int fd = 0;
   int cmd;

   fd = open(QRC_DEV, O_RDWR);
   if (fd < 0)
   {
      printf("Open Dev QRC Error!\n");
      return -1;
   }

   printf("<--- Call QRC IOCTL REBOOT  --->\n");
   cmd = QRC_REBOOT;
   if (ioctl(fd, cmd) < 0)
   {
      printf("Call cmd QRC_REBOOT fail\n");
      return -1;
   }
   sleep(30);

printf("<--- Call QRC IOCTL QRC_BOOT_TO_MEM  --->\n");
   cmd = QRC_BOOT_TO_MEM;
   if (ioctl(fd, cmd) < 0)
   {
      printf("Call cmd QRC_BOOT_TO_MEM fail\n");
      return -1;
   }
   sleep(30);
printf("<--- Call QRC IOCTL QRC_BOOT_TO_FLASH  --->\n");
   cmd = QRC_BOOT_TO_FLASH;
   if (ioctl(fd, cmd) < 0)
   {
      printf("Call cmd QRC_BOOT_TO_FLASH fail\n");
      return -1;
   }
   close(fd);
   return 0;
}
