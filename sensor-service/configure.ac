# -*- Autoconf -*-
# configure.ac -- Autoconf script for sensors
#
# Process this file with autoconf to produce a configure script
# Requires autoconf tool later than 2.61
AC_PREREQ([2.69])
AC_INIT([sensors-service], 1.0.0)
AM_INIT_AUTOMAKE([foreign subdir-objects])
AM_MAINTAINER_MODE

AC_CONFIG_HEADERS([config.h])
AC_CONFIG_MACRO_DIR([m4])

# Checks for programs.
AC_PROG_CXX
AC_PROG_CC
AM_PROG_CC_C_O
AC_PROG_CXX
AC_PROG_LIBTOOL
AC_PROG_AWK
AC_PROG_CPP
AC_PROG_INSTALL
AC_PROG_LN_S
AC_PROG_MAKE_SET
PKG_PROG_PKG_CONFIG

AC_CHECK_LIB([bsd], [main])

PKG_CHECK_MODULES([PROTOBUF], [protobuf])
AC_SUBST([PROTOBUF_CFLAGS])
AC_SUBST([PROTOBUF_LIBS])
AC_SUBST(PROTOBUF_VERSION)

PKG_CHECK_MODULES([JSONCPP], [jsoncpp >= 1.9])
AC_SUBST(JSONCPP_CFLAGS)
AC_SUBST(JSONCPP_LIBS)

PKG_CHECK_MODULES([SENSINGHUB], [sensing-hub])
AC_SUBST([SENSINGHUB_CFLAGS])
AC_SUBST([SENSINGHUB_LIBS])
 
PKG_CHECK_MODULES([SENSORSUTILS], [sensors-utils])
AC_SUBST([SENSORSUTILS_CFLAGS])
AC_SUBST([SENSORSUTILS_LIBS])
 
PKG_CHECK_MODULES([SENSORSAPI], [sensors-api])
AC_SUBST([SENSORSAPI_CFLAGS])
AC_SUBST([SENSORSAPI_LIBS])

AC_ARG_ENABLE(sns-direct-channel,
        AS_HELP_STRING([--enable-sns-direct-channel], [enables sensors direct channel]),
        [sns_direct_channel_support="yes"],
        [sns_direct_channel_support="no"])

if test "x${sns_direct_channel_support}" = "xyes"; then
      CFLAGS="${CFLAGS} -DSNS_SUPPORT_DIRECT_CHANNEL"
        CPPFLAGS="${CPPFLAGS} -DSNS_SUPPORT_DIRECT_CHANNEL"
fi

AC_SUBST([CFLAGS])
AC_SUBST([CPPFLAGS])
AC_SUBST([CC])

AC_CONFIG_FILES([
        Makefile
        ])
AC_OUTPUT
