/*
 * sd_card.h
 *
 *  Created on: Apr 4, 2022
 *      Author: Jakub Szczęsny
 */

#include <stdint.h>

#ifndef INC_SD_CARD_H_
#define INC_SD_CARD_H_


typedef enum {
    STATUS_NULL, // ?
    STATUS_OK,
    STATUS_ERROR
} ret_status;

void myprintf(const char *fmt, ...);

ret_status sd_card_init(void);
ret_status sd_card_close(void);

ret_status sd_card_scan_file(char* path, char* buffor_dirs, uint8_t* size_dirs);
ret_status sd_card_read_data(uint8_t * data);

#endif /* INC_SD_CARD_H_ */
