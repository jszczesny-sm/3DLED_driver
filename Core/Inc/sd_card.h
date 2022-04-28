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

extern char fresult_name[20][22];

struct layers_struct {
    uint8_t isSet;
    uint8_t count;
    uint8_t values[32];
};

void myprintf(const char *fmt, ...);

ret_status sd_card_init(void);
ret_status sd_card_close(void);

ret_status sd_card_scan_file(char *path, char *buffor_dirs, uint8_t *number_of_dirs);
ret_status sd_card_read_data(char *path, uint8_t *data, struct layers_struct *layers, uint8_t *number_of_images, uint8_t *speed_of_animation);

#endif /* INC_SD_CARD_H_ */
