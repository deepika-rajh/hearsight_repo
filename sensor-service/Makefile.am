# Makefile.am - Automake script for sensor service

AM_CPPFLAGS = -Wall                             \
              -Wno-unused-parameter             \
              -Wno-unused-variable              \
              -fexceptions                      \
              -DSNS_TARGET_LE_PLATFORM          \
              -Iinc

requiredlibs = -lcdsprpc                                                \
               -lbsd                                                    \
               -lsensorslog                                             \
               @SENSORSEE_LIBS@                                         \
               -ljsoncpp

cpp_sources = src/sensor_service.cpp
bin_PROGRAMS =  sensor_service
sensor_service_SOURCES = $(cpp_sources)
sensor_service_CC = @CC@
sensor_service_CPPFLAGS = $(AM_CPPFLAGS) @SENSORSEE_LIBS@
sensor_service_LDFLAGS = -lprotobuf


if USE_GLIB
sensor_service_CPPFLAGS += -DUSE_GLIB @GLIB_CFLAGS@
sensor_service_LDFLAGS += @GLIB_LIBS@
endif
sensor_service_LDADD = $(requiredlibs)
