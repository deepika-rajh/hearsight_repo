/*
 * Copyright (c) 2020 Qualcomm Technologies, Inc.
 * All Rights Reserved.
 * Confidential and Proprietary - Qualcomm Technologies, Inc.
 */

#ifndef SNS_CLIENT_EXAMPLE_CPP_H
#define SNS_CLIENT_EXAMPLE_CPP_H

#ifdef __cplusplus
extern "C" {
#endif /*__cplusplus*/

typedef int (*get_raw_data_func_t)(float, float, float);
extern int register_data_callback(get_raw_data_func_t data_func);
extern int start_sns_connection(void);
extern int stop_sns_connection(void);

#ifdef __cplusplus	
}
#endif /*__cplusplus*/

#endif 