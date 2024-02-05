#! /bin/sh
###############################################################################
#
# This script is used for starting sensor-service daemon
#
# Copyright (c) 2024 Qualcomm Technologies, Inc.
# All Rights Reserved.
# Confidential and Proprietary - Qualcomm Technologies, Inc.
#
###############################################################################
set -e

case "$1" in
  start)
        echo "Starting sensor-service daemon: "
        start-stop-daemon -S -b -x /sbin/sensor_service
        echo "done"
        ;;
  stop)
        echo "Stopping sensor-service daemon: "
        start-stop-daemon -K -n /sbin/sensor_service
        ;;
  restart)
        $0 stop
        $0 start
        ;;
  *)
        echo "Usage sensor-service.sh { start | stop | restart}"
        exit 1
        ;;
esac

exit 0
