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
#include <pthread.h>
#include <time.h>
#include <sys/types.h>
#include <signal.h>
#include <errno.h>

#include "mcb_flash.h"
#include "xmodem.h"


/* pthread solve timeout issue */
static  int pthread_count=0;
static  pthread_t flash_id;

#define  MCB_FLASH_RETRY   (3)

static void* thread_callback_flash(void* arg){
   int* p=(int*)arg;
   do_flash(p, &pthread_count);
}

static int do_flash_pthread(void *p)
{
   int retry = MCB_FLASH_RETRY;
   int chunk_count = 0;
   int kill_rc;


   if (pthread_create(&flash_id,NULL,thread_callback_flash,p)) {
      printf("create flash thread error\n");
      return -1;
   }
   printf("created flash thread\n");

   // check if count added.

while ( retry > 0) {
   chunk_count = pthread_count;
   if (chunk_count < 200)
     sleep(19);//check every 19s
   else
     sleep(2);

   kill_rc= pthread_kill(flash_id, 0);//send check signal.
   // check flash status
   if (kill_rc == ESRCH) {
      printf("thread already quit\n\n");
      return 0;
   }
   else if (kill_rc == EINVAL)
        printf("signal invalid\n\n");

   if (pthread_count > chunk_count) {
      // flash thread is alive
      chunk_count = pthread_count;
   }
   else {
      /* recreate flash thread */
      printf(" recreate flash thread \n\n");
      // exit present thread.
      pthread_cancel(flash_id);
      pthread_count = 0;
      if (pthread_create(&flash_id,NULL,thread_callback_flash,p)) {
         printf("create flash thread error\n");
         return -1;
      }
      printf(" recreate flash thread done \n\n");
   }

   printf("\n\n main thread count  = %d\n",chunk_count);

  }


}


typedef struct {
   char *option_string;
   unsigned int opt_str_len;
   int  (*action_func)(void *param);
   int  is_param_needed;
} opt_func_map;

opt_func_map opt_func_list[] = {
   {"--help",           (sizeof("--help") - 1), &print_help,       0},
   {"--version",        (sizeof("--version") - 1), &print_version,    0},
   {"--reboot",     (sizeof("--reboot") - 1), &do_reboot,    0},
   {"--flash",    (sizeof("--flash") - 1), &do_flash_pthread,   0},
};



int main(int argc, char *argv[])
{
   unsigned int i;
   int count=0;


   if(argc ==1 ||argc >= 3) {
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