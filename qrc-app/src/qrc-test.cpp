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

typedef struct {
   char *option_string;
   unsigned int opt_str_len;
   int  (*action_func)(void *param);
   int  is_param_needed;
} opt_func_map;

static void usage()
{
   printf("qrctest  -  check qrc driver ioctl .\n");
   printf("   --help:\n");
   printf("         prints this help.\n");
   printf("   --version:\n");
   printf("         prints the program version string.\n");
   printf("   --reset:\n");
   printf("         reset qrc hardware.\n");
   printf("   --boot0 <pin-value 0|1>:\n");
   printf("         set the qrc boot0 pin as <pin-value>.\n");
   printf("\n");
}

static int print_help(void *p)
{
   usage();
   return 0;
}

static int print_version(void *p)
{
   printf("Version: %s\n", VERSION_STR);
   return 0;
}

static int do_reset(void *p)
{
   int cmd = QRC_REBOOT;
   int fd = open(QRC_DEV, O_RDWR);

   if (fd < 0)
   {
      printf("Open Dev QRC Error!\n");
      return -1;
   }

   if (ioctl(fd, cmd) < 0)
   {
      printf("qrc reboot fail\n");
      close(fd);
      return -1;
   }

   close(fd);
   return 0;
}

static int do_boot0(void *p)
{
   char *opt = (char *)p;
   unsigned int value = atoi(opt);
   int cmd;
   int fd = open(QRC_DEV, O_RDWR);

   if (fd < 0)
   {
      printf("Open Dev QRC Error!\n");
      return -1;
   }

   if (value)
   {
      cmd = QRC_BOOT_TO_MEM;
      if (ioctl(fd, cmd) < 0)
      {
         printf("qrc ioctl fail\n");
         return -1;
      }
   }
   else
   {
      cmd = QRC_BOOT_TO_FLASH;
      if (ioctl(fd, cmd) < 0)
      {
         printf("qrc ioctl fail\n");
         return -1;
      }
   }

   usage();
   return 0;
}

opt_func_map opt_func_list[] = {
   {"--help",           (sizeof("--help") - 1), &print_help,       0},
   {"--version",        (sizeof("--version") - 1), &print_version,    0},
   {"--reset",     (sizeof("--reset") - 1), &do_reset,    0},
   {"--boot0",    (sizeof("--boot0") - 1), &do_boot0,   1},
};

int main(int argc, char *argv[])
{
   unsigned int i;

   if((argc == 1) || (argc >= 4)) {
      usage();
      return 0;
   }

   for(i = 0; i < (sizeof(opt_func_list)/sizeof(opt_func_map)); i++) {
      if (!(strncmp(argv[1], opt_func_list[i].option_string,
                             opt_func_list[i].opt_str_len))) {
         if (opt_func_list[i].is_param_needed == 1) {
            if (argv[2]) {
               opt_func_list[i].action_func(argv[2]);
            } else {
               usage();
            }
         } else {
            opt_func_list[i].action_func(NULL);
         }
         return 0;
      }
   }

   usage();
   return 0;
}
