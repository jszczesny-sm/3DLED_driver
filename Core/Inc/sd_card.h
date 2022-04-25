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

//char fresult_name[20][22] = {
//    { "FR_OK" }, /* (0) Succeeded */
//    { "FR_DISK_ERR" }, /* (1) A hard error occurred in the low level disk I/O layer */
//    { "FR_INT_ERR" }, /* (2) Assertion failed */
//    { "FR_NOT_READY" }, /* (3) The physical drive cannot work */
//    { "FR_NO_FILE" }, /* (4) Could not find the file */
//    { "FR_NO_PATH" }, /* (5) Could not find the path */
//    { "FR_INVALID_NAME" }, /* (6) The path name format is invalid */
//    { "FR_DENIED" }, /* (7) Access denied due to prohibited access or directory full */
//    { "FR_EXIST" }, /* (8) Access denied due to prohibited access */
//    { "FR_INVALID_OBJECT" }, /* (9) The file/directory object is invalid */
//    { "FR_WRITE_PROTECTED" }, /* (10) The physical drive is write protected */
//    { "FR_INVALID_DRIVE" }, /* (11) The logical drive number is invalid */
//    { "FR_NOT_ENABLED" }, /* (12) The volume has no work area */
//    { "FR_NO_FILESYSTEM" }, /* (13) There is no valid FAT volume */
//    { "FR_MKFS_ABORTED" }, /* (14) The f_mkfs() aborted due to any problem */
//    { "FR_TIMEOUT" }, /* (15) Could not get a grant to access the volume within defined period */
//    { "FR_LOCKED" }, /* (16) The operation is rejected according to the file sharing policy */
//    { "FR_NOT_ENOUGH_CORE" }, /* (17) LFN working buffer could not be allocated */
//    { "FR_TOO_MANY_OPEN_FILES" }, /* (18) Number of open files > _FS_LOCK */
//    { "FR_INVALID_PARAMETER" } /* (19) Given parameter is invalid */
//};

struct layers_struct {
    uint8_t isSet;
    uint8_t count;
    uint8_t values[32];
};

void myprintf(const char *fmt, ...);

ret_status sd_card_init(void);
ret_status sd_card_close(void);

ret_status sd_card_scan_file(char *path, char *buffor_dirs, uint8_t *number_of_dirs);
ret_status sd_card_read_data(char *path, uint8_t *data, struct layers_struct *layers, uint8_t *number_of_images);

#endif /* INC_SD_CARD_H_ */
