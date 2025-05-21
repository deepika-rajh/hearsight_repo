# Makefile.am - Automake script for sensor service

AM_CPPFLAGS = -Wall                             \
              -Wno-unused-parameter             \
              -Wno-unused-variable              \
              -fexceptions                      \
              -DSNS_TARGET_LE_PLATFORM          \
              -Iinc                             \
              $(JSONCPP_CFLAGS) $(PROTOBUF_CFLAGS)       \
              $(SENSINGHUB_CFLAGS) $(SENSORSUTILS_CFLAGS) \
              $(SENSORSAPI_CFLAGS) $(QMIFRAMEWORK_CFLAGS)

requiredlibs = -lcdsprpc                           \
               -lbsd                               \
               $(SENSORSUTILS_LIBS)                \
               $(SENSORSAPI_LIBS)                  \
               $(SENSINGHUB_LIBS)                  \
               $(SENSORSUTILS_LIBS)                  \
               $(JSONCPP_LIBS) $(PROTOBUF_LIBS)

bin_PROGRAMS = sensor_service
sensor_service_SOURCES = src/sensor_service.cpp
sensor_service_CC = $(CC)
sensor_service_CPPFLAGS = $(AM_CPPFLAGS)
sensor_service_LDFLAGS =  $(requiredlibs)
