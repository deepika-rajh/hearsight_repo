AM_CPPFLAGS  = $(GLIB_CFLAGS) -include glib.h -Dstrlcpy=g_strlcpy

# Build qrc test bin
root_sbindir           := "/sbin"
root_sbin_PROGRAMS     := qrctest


qrctest_SOURCES  := src/qrc-test.cpp

qrctest_CPPFLAGS := -I$(top_srcdir)/inc
qrctest_CPPFLAGS += -std=c++11 -Wall -Wno-unused-parameter

