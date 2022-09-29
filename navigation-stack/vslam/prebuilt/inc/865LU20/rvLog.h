/*****************************************************************************
@copyright
Copyright (c) 2020-2022 Qualcomm Technologies, Inc.
All Rights Reserved.
Confidential and Proprietary - Qualcomm Technologies, Inc.
*******************************************************************************/

/***************************************************************************//**
@brief
 Robot Vion Demo,
 Logging implementation for RV lib.

@section Overview
 This section includes the definition of logging function. This
 is one of the main classes
*******************************************************************************/
#ifndef RV_DEBUG_LOG_H
#define RV_DEBUG_LOG_H

#ifndef _WIN32
#include <syslog.h>
#endif
#include <cstdio>
#include <cstdarg>
#include <time.h>

enum AppLoglevel {
   RV_LOG_ERROR = 0,
   RV_LOG_KPI = 1,
   RV_LOG_INFO = 2,
   RV_LOG_DEBUG = 3,
   RV_LOG_MAX,
};

#ifndef _RV_LOG_LEVEL_
#define _RV_LOG_LEVEL_ 0
#endif

extern bool RV_STDERR_LOGGING;
extern FILE* RV_CONSOLE_STREAM;
#if defined _WIN32
#include <Windows.h>

//void __declspec(dllexport)   RVDebugLog( enum AppLoglevel level, const char* format, ... )
//{
//    if (_RV_LOG_LEVEL_ < (int)level)
//        return;
//    va_list args;
//    va_start(args, format);
//    vfprintf(DG_CONSOLE_STREAM, format, args);
//    fflush(DG_CONSOLE_STREAM);
//    va_end(args);
//
//    if (level <= RV_LOG_ERROR && RV_CONSOLE_STREAM != stderr)
//    {
//        va_list nargs;
//        va_start(nargs, format);
//        vfprintf(stderr, format, nargs);
//        fflush(stderr);
//        va_end(nargs);
//    }
//
//}

#define RV_TS( h, m, s, ms) //do { \
   SYSTEMTIME lt; \
   GetLocalTime( &lt ); \
   h = lt.wHour; \
   m = lt.wMinute; \
   s = lt.wSecond; \
   ms = lt.wMilliseconds; \
   __pragma(warning(suppress:4127)) \
} while( 0 )

#define RV_ERR(fmt, ...) //do { \
   int h,m,s,ms; \
   RV_TS( h, m, s, ms); \
   RVDebugLog(RV_LOG_ERROR, "%d:%d:%d.%d %s:%d ERROR: " fmt "\n", h,m,s,ms, __FILE__, __LINE__, __VA_ARGS__ ); \
   if (RV_CONSOLE_STREAM != stderr) \
   { \
      FILE* temp = RV_CONSOLE_STREAM; \
      RV_CONSOLE_STREAM = stderr; \
      RVDebugLog(RV_LOG_ERROR, "%d:%d:%d.%d %s:%d ERROR: " fmt "\n", h,m,s,ms, __FILE__, __LINE__, __VA_ARGS__ ); \
      RV_CONSOLE_STREAM = temp; \
   } \
   __pragma(warning(suppress:4127)) \
}while(0)


#define RV_KPI(fmt, ...)  //do { \
   int h,m,s,ms; \
   RV_TS( h, m, s, ms); \
   RVDebugLog(RV_LOG_KPI, "%d:%d:%d.%d %s:%d KPI: " fmt "\n", h,m,s,ms, __FILE__, __LINE__, __VA_ARGS__ ); \
   __pragma(warning(suppress:4127)) \
} while(0)

#define RV_INFO(fmt, ...) //do {\
   int h,m,s,ms; \
   RV_TS( h, m, s, ms); \
   RVDebugLog(RV_LOG_INFO, "%d:%d:%d.%d %s:%d INFO: " fmt "\n", h,m,s,ms, __FILE__, __LINE__, __VA_ARGS__ ); \
   __pragma(warning(suppress:4127)) \
} while(0)

#define RV_DBG(fmt, ...) //do {\
   int h,m,s,ms; \
   RV_TS( h, m, s, ms); \
   RVDebugLog(RV_LOG_DEBUG, "%d:%d:%d.%d %s:%d DEBUG: " fmt "\n", h,m,s,ms, __FILE__, __LINE__, __VA_ARGS__ ); \
   __pragma(warning(suppress:4127)) \
} while(0)

#else// _LINUX
#define    RV_TS(sec, usec) do { \
   struct timespec ts; \
   clock_gettime(CLOCK_REALTIME, &ts); \
   sec = (   unsigned int)ts.tv_sec%100000;\
   usec = (   unsigned int)ts.tv_nsec/1000;\
} while (0)

#define RV_ERR(fmt, ...) do { \
   if (_RV_LOG_LEVEL_ >= RV_LOG_ERROR) { \
      unsigned int sec, usec;\
      RV_TS(sec,usec);\
      syslog(LOG_ALERT, "(%d.%06d) %s:%d ERROR: " fmt, sec, usec, __FILE__, __LINE__, ##__VA_ARGS__ ); \
      if (RV_STDERR_LOGGING) { \
         fprintf(stderr, "(%d.%06d) %s:%d ERROR: " fmt "\n", sec, usec, __FILE__, __LINE__, ##__VA_ARGS__ ); \
      } \
   }\
} while (0)

#define RV_KPI(fmt, ...) do { \
   if (_RV_LOG_LEVEL_ >= RV_LOG_KPI) { \
      unsigned int sec, usec;\
      RV_TS(sec,usec);\
      syslog(LOG_ALERT, "(%d.%06d) KPI: " fmt, sec, usec, ##__VA_ARGS__ ); \
      if (RV_STDERR_LOGGING) { \
         fprintf(stderr, "(%d.%06d) KPI: " fmt "\n", sec, usec, ##__VA_ARGS__ ); \
      } \
   }\
} while (0)

#define RV_INFO(fmt, ...) do { \
   if (_RV_LOG_LEVEL_ >= RV_LOG_INFO) { \
      unsigned int sec, usec;\
      RV_TS(sec,usec);\
      syslog(LOG_ALERT, "(%d.%06d) %s:%d INFO: " fmt, sec, usec, __FILE__, __LINE__, ##__VA_ARGS__ ); \
      if (RV_STDERR_LOGGING) { \
         fprintf(stderr, "(%d.%06d) %s:%d INFO: " fmt "\n", sec, usec, __FILE__, __LINE__, ##__VA_ARGS__ ); \
      } \
   }\
} while (0)

#define RV_DBG(fmt, ...) do { \
   if (_RV_LOG_LEVEL_ >= RV_LOG_DEBUG) { \
      unsigned int sec, usec;\
      RV_TS(sec,usec);\
      syslog(LOG_ALERT, "(%d.%06d) %s:%d DEBUG: " fmt, sec, usec, __FILE__, __LINE__, ##__VA_ARGS__ ); \
      if (RV_STDERR_LOGGING) { \
         fprintf(stderr, "(%d.%06d) %s:%d DEBUG: " fmt "\n", sec, usec, __FILE__, __LINE__, ##__VA_ARGS__ ); \
      } \
   }\
} while (0)

#endif
#endif
