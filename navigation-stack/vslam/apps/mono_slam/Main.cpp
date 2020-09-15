/*****************************************************************************
@copyright
Copyright (c) 2020 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

#include <stdlib.h>
#include <iostream>
#include <string>
#include <unistd.h>
#include <stdio.h>
#include <mcheck.h>

#include "VSLAMSystem.h"

#define ENABLE_MTRACE 0

bool debugLevel = 0;
static char *helpMsg =
      "mv_vwslam \n"
      "Usage: mv_vwslam [-options]\n"
      "-c : set configuration files path, default path is /data/misc/vwslam/ \n"
      "-o : set output files path, default path is /data/vwslam/ \n"
      "-d : set vslam debug level: enable debug info(1), disable debug info(0) \n"
      "-v : get vslam app version \n"
      "-h : print help msg\n";

int main( int argc, char** argv )
{
   int opt;

#if ENABLE_MTRACE
   setenv("MALLOC_TRACE", "mem.log", 1);
   mtrace();
#endif

   std::string root = std::string( "/data/misc/vwslam/" );
   std::string output = std::string( "/data/vwslam/" );

   if( argc < 2 )
   {
      printf( "%s run with default setting.\n", argv[0] );
   }
   else
   {
      while((opt = getopt(argc, argv, "c:o:i:d:vh")) != -1)
      {
        switch(opt) {
         case 'c':
            root = std::string(optarg);
          break;

         case 'o':
            output = std::string(optarg);
          break;

         case 'd':
            debugLevel = atoi(optarg);
            printf("VSLAM debug level is %d\n", debugLevel);
          break;

         case 'v':
            printf( "%s version: %s \n", argv[0], VSLAM_APP_VERSION);
            return 0;

        case 'h':
        default:
            printf("%s", helpMsg);
            return 1;
         }
      }
   }

   char tmp = *(output.end() - 1);
   if( tmp != '/' && tmp != '\\' )
   {
      output = output + '/';
   }

   tmp = *(root.end() - 1);
   if( tmp != '/' && tmp != '\\' )
   {
      root = root + '/';
   }

#ifdef ARM_BASED
   //add log for ARM platform to check boot time
   system("echo vSLAM Start Initialization > /dev/kmsg");
#endif

   //start VSLAM system
   std::shared_ptr<VSLAMSystem> sys = VSLAMSystem::Initialize(root, output, false, false);
   sys->Run();

   //wait to quit
   sys->Spin();
   //stop VSLAM
   sys->Quit();
   sys->deinit();
   sys = nullptr;
   printf("vslam application exits\n");
   fflush(stdout);

#if ENABLE_MTRACE
   muntrace();
#endif

   return 0;
} 
