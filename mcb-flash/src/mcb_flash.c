/************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <sys/types.h>
#include <sys/ioctl.h>
#include <unistd.h>
#include <fcntl.h>
#include <stdbool.h>

#include "mcb_flash.h"
#include "xmodem.h"

#define VERSION_STR         ("v1.0\n")

/************************flash functions **********************************/



int qrc_clean_rx_buff(int fd)
{
   int cmd = QRC_FIFO_CLEAR;

   if (fd < 0)
   {
      printf("Dev QRC Error!\n");
      return -1;
   }

   if (ioctl(fd, cmd) < 0)
   {
      printf("qrc clean read buffer fail\n");
      return -1;
   }

   return 0;
}


static int mcb_reboot(int fd)
{
   int cmd = QRC_REBOOT;

   if (fd < 0)
   {
      printf("Dev QRC Error!\n");
      return -1;
   }

   if (ioctl(fd, cmd) < 0)
   {
      printf("mcb reboot fail\n");
      return -1;
   }

   return 0;
}

/* check if MCB in bootloader mode with char 'a', try 3 times */
static bool mcb_check_bootloader_mode(int fd)
{
   int try = 5;
   bool ret =false;
   char rec_data[2];
   ssize_t rec_bytes;

   //read 字符。
   while (try --)
   {
      sleep(2);
      rec_bytes = read(fd, rec_data, sizeof(rec_data));
      if (rec_bytes == 2) {
         if (rec_data[0]==X_a && rec_data[1]==X_a) {
            return true;
         }
      }
   }
   
   printf("\ncheck bootloader mode failed\n");
   return false;
}

static bool bootloader_flash(int fd, char *bin_arr, size_t bin_size,int * p_count)
{
   char cmd;
   int retry =5;
   char rec_data;
   //send '1' cmd
   cmd = CMD_FLASH;
   write(fd,&cmd,sizeof(cmd));
   printf("Waiting for Xmodem handshake...");

   while(retry--) {
			usleep(50000);
			if (read(fd,&rec_data,1)) {
				printf("receive handshake =%d\n",rec_data);
				if (rec_data == X_C) {
               printf("Xmodem protocol begin\n");
               return xmodem_send(fd, bin_arr ,bin_size,p_count);
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
   printf("   --flash <nuttx-image>:\n");
   printf("         flash the mcb with nuttx-image.\n");
   printf("\n");
}

int print_help(void *p)
{
   usage();
   return 0;
}

int print_version(void *p)
{
   printf("Version: %s\n", VERSION_STR);
   return 0;
}

int do_reboot(void *p)
{
   int cmd = QRC_REBOOT;
   int fd = open(QRC_DEV, O_RDWR);

   if (fd < 0)
   {
      printf("Open Dev QRC Error!\n");
      return -1;
   }
   mcb_reboot(fd);
   close(fd);
   return 0;
}


int do_flash(void *p, int* p_count)
{
   int fd = open(QRC_DEV, O_RDWR);
   FILE *bin_file;
   int bin_size = 0;
   char *bin_arr;
   size_t read_num;
   int ret = -1;

   if (fd < 0)
   {
      printf("Open Dev QRC Error!\n");
      close(fd);
      return -1;
   }

   //reboot mcb
   mcb_reboot(fd);
   //clean  serial buffer. (need todo)

   //clean qrc
   qrc_clean_rx_buff(fd);

   //check nuttx.bin file
   bin_file = fopen(NUTTX_BIN, "r");  //open the file
   if (bin_file == NULL) {
      printf("%s file does not exist\n", NUTTX_BIN);
      return -1;
   }
   //read nutttx.bin file.
   if (fseek(bin_file, (long long)(0), SEEK_END)) {
      printf("fseek() function failed!\n");
      goto err_return;
   }
   bin_size = ftell(bin_file);
   printf(" nuttx.bin size=%d\n",bin_size);
   bin_arr =  (char*)malloc(bin_size * sizeof(char));

   if (fseek(bin_file, (long long)(0), SEEK_SET)) {
      printf("fseek() function failed!\n");
      goto err_return;
   }

   read_num =fread(bin_arr, sizeof(char), bin_size, bin_file);

   if (read_num == bin_size) {
      printf(" read nuttx.bin successed\n");

      //check if attached bootloader.
      if (false == mcb_check_bootloader_mode(fd))
         goto err_return;
      printf(" mcb bootloader detected \n");
      //xmodem send data (128 bytes mode)

      if (!bootloader_flash(fd, bin_arr, bin_size, p_count)) {
          printf(" failed flash  nuttx.bin \n");
          goto err_return;
      }
      printf(" flash nuttx.bin successed !\n");
   }
   else
      printf(" read nuttx.bin failed num = %ld\n",read_num);

err_return:
   free(bin_arr);
   fclose(bin_file);
   close(fd);
   return ret;
}
