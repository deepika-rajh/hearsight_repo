# Makefile.am - Automake script for sensor service

AM_CPPFLAGS = -Wall                             \
              -Wno-unused-parameter             \
              -Wno-unused-variable              \
              -fexceptions                      \
              -DSNS_TARGET_LE_PLATFORM          \
              -Iinc

requiredlibs = -lcdsprpc                                                \
               -lbsd                                                    \
               @SENSORSUTILS_LIBS@                                      \
               @SENSORSAPI_LIBS@                                        \
               @SENSINGHUB_LIBS@                                        \
               -ljsoncpp

cpp_sources = src/sensor_service.cpp
bin_PROGRAMS =  sensor_service
sensor_service_SOURCES = $(cpp_sources)
sensor_service_CC = @CC@
sensor_service_CPPFLAGS = $(AM_CPPFLAGS)
sensor_service_LDFLAGS = -lprotobuf

sensor_service_LDADD = $(requiredlibs)
