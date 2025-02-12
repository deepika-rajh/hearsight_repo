/************************************************************************
* Copyright (c) 2023 Qualcomm Innovation Center, Inc. All rights reserved.
* SPDX-License-Identifier: BSD-3-Clause-Clear
*************************************************************************/

#include <string.h>

#include "xmodem.h"
#include "mcb_flash.h"


#define XMODEM_PACKET_TIMEOUT 1000

#define XMODEM_MAX_ERRORS 5


#define X_PACKET_NUMBER_SIZE  ((uint16_t)2u)
#define X_PACKET_128_SIZE     ((uint16_t)128u)
#define X_PACKET_CRC_SIZE     ((uint16_t)2u)

#define PADDING		(0xFF)

static uint16_t xmodem_calc_crc(uint8_t *data, uint16_t length)
{
    uint16_t crc = 0u;
    while (length)
    {
        length--;
        crc = crc ^ ((uint16_t)*data++ << 8u);
        for (uint8_t i = 0u; i < 8u; i++)
        {
            if (crc & 0x8000u)
            {
                crc = (crc << 1u) ^ 0x1021u;
            }
            else
            {
                crc = crc << 1u;
            }
        }
    }
    return crc;
}


bool xmodem_send(int fd, char *bin_arr ,size_t bin_size,int * p_count)
{
	uint32_t bin_index = 0;
	size_t bin_len = bin_size;
	uint32_t padding_num =0;
	char try;
	char rec_ack;
	uint8_t  end_eot = X_EOT;

	uint16_t crc;
	uint16_t chunk =1;
	char  frame[1+X_PACKET_NUMBER_SIZE+X_PACKET_128_SIZE+X_PACKET_CRC_SIZE] = {0};

	frame[0] = X_SOH;

	while(bin_len > 0) {
		//<128 bytes
		frame[1] = chunk & 0xff;
		frame[2] = ~frame[1];

		if (bin_len < 128) {

			memcpy(&frame[3],&bin_arr[bin_index],bin_len);
			padding_num = X_PACKET_128_SIZE - bin_len;
			memset(&frame[3+bin_len], PADDING, padding_num);
			bin_len = 0;
			bin_index = bin_index + bin_len;

		} else {
		//> 128 bytes
			memcpy(&frame[3],&bin_arr[bin_index],X_PACKET_128_SIZE);
			bin_len = bin_len - X_PACKET_128_SIZE;
			bin_index = bin_index + X_PACKET_128_SIZE;
		}

		crc=xmodem_calc_crc(&frame[3], X_PACKET_128_SIZE);
		frame[1+X_PACKET_NUMBER_SIZE+X_PACKET_128_SIZE] = (crc >> 8) & 0xff;
		frame[1+X_PACKET_NUMBER_SIZE+X_PACKET_128_SIZE + 1] = crc & 0xff;

		write(fd,frame,sizeof(frame));
		printf("send frame %d \n",chunk);

		qrc_clean_rx_buff(fd);

		//receive ack
		try = XMODEM_MAX_ERRORS;
		while(try--) {
			usleep(20000);
			if (read(fd,&rec_ack,1)) {
				//printf("receive ack =%d\n",rec_ack);
				if (rec_ack == X_ACK)
					break;
				else if (rec_ack == X_NAK) {
					write(fd,frame,sizeof(frame));
					printf("send frame %d again\n",chunk);
				}
				else if (rec_ack == X_CAN) {
					printf("bootloader error\n");
					return false;
				}

			}
		}
		if (try == 0xff) {
			printf("bootloader send frame error\n");
			return false;
		}

    chunk = chunk +1;
	*p_count = chunk;
	}
	//send EOF.
	write(fd,&end_eot,sizeof(uint8_t));
	usleep(50000);
	//check if recevice  ACK
	if (rec_ack == X_ACK)
		write(fd,&end_eot,sizeof(uint8_t));

	return true;
}
