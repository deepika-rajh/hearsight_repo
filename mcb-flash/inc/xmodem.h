/************************************************************************
 *Copyright (c) Qualcomm Technologies, Inc. and/or its subsidiaries.
 * All rights reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 *************************************************************************/

#ifndef XMODEM_H
#define XMODEM_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

/* XMODEM protocol constants */
/* Bytes defined by the protocol. */
#define X_SOH ((uint8_t)0x01u)
#define X_STX ((uint8_t)0x02u)
#define X_EOT ((uint8_t)0x04u)
#define X_ACK ((uint8_t)0x06u)
#define X_NAK ((uint8_t)0x15u)
#define X_CAN ((uint8_t)0x18u)
#define X_C ((uint8_t)0x43u)
#define X_a ((uint8_t)0x61u)

/* Status report for the functions. */
enum
{
  XMODEM_OK = 0x00u,
  XMODEM_ERROR_CRC = 0x01u,
  XMODEM_ERROR_NUMBER = 0x02u,
  XMODEM_ERROR_NUM_MIN = 0x10u,
  XMODEM_ERROR_QRC = 0x04u,
  XMODEM_ERROR_FLASH = 0x08u,
  XMODEM_ERROR = 0xFFu,
};

bool xmodem_send(int fd, char * bin_arr, size_t bin_size, int * p_count);

#ifdef __cplusplus
}
#endif

#endif
