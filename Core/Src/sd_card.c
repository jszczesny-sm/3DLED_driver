/*
 * sd_card.c
 *
 *  Created on: Apr 4, 2022
 *      Author: KubaWinPC
 */

#include <stdarg.h>
#include <string.h>

#include <sd_card.h>
#include <stdio.h>
#include "stb_image.h"
#include "fatfs.h"

extern SPI_HandleTypeDef hspi1;
extern UART_HandleTypeDef huart2;

void myprintf(const char *fmt, ...) {
    static char buffer[256];
    va_list args;
    va_start(args, fmt);
    vsnprintf(buffer, sizeof(buffer), fmt, args);
    va_end(args);

    int len = strlen(buffer);
    HAL_UART_Transmit(&huart2, (uint8_t*) buffer, len, -1);

}

//some variables for FatFs
FATFS FatFs; 	//Fatfs handle
FIL fil; 		//File handle
FRESULT fres; //Result after operations

ret_status sd_card_init(void) {
    myprintf("\r\n~ SD card demo by kiwih ~\r\n\r\n");

    HAL_Delay(1000); //a short delay is important to let the SD card settle

    fres = f_mount(&FatFs, "", 1); //1=mount now
    if (fres != FR_OK) {
        myprintf("f_mount error (%i)\r\n", fres);
        return STATUS_ERROR;
    }
    DWORD free_clusters, free_sectors, total_sectors;

    FATFS *getFreeFs;

    fres = f_getfree("", &free_clusters, &getFreeFs);
    if (fres != FR_OK) {
        myprintf("f_getfree error (%i)\r\n", fres);

        return STATUS_ERROR;
    }

    //Formula comes from ChaN's documentation
    total_sectors = (getFreeFs->n_fatent - 2) * getFreeFs->csize;
    free_sectors = free_clusters * getFreeFs->csize;

    myprintf(
            "SD card stats:\r\n%10lu KiB total drive space.\r\n%10lu KiB available.\r\n",
            total_sectors / 2, free_sectors / 2);

    return STATUS_OK;
}

ret_status sd_card_scan_file(char *path, char* buffor_dirs, uint8_t* number_of_dirs) {

    FRESULT res;
    DIR dir;
    UINT i;
    static FILINFO fno;
    res = f_opendir(&dir, path); /* Open the directory */
    if (res == FR_OK) {
        while (1) {
            res = f_readdir(&dir, &fno); /* Read a directory item */
            if (res != FR_OK || fno.fname[0] == 0)
                break; /* Break on error or end of dir */
            if (fno.fattrib & AM_DIR && !(fno.fattrib & AM_HID)
                    && !(fno.fattrib & AM_SYS)) { /* It is a directory */
                i = strlen(path);
                sprintf(&path[i], "/%s", fno.fname);
                sprintf((char*)(buffor_dirs + (*number_of_dirs)*16), "%s", path);
                (*number_of_dirs)++;
                res = sd_card_scan_file(path, NULL, NULL); /* Enter the directory */
                if (res != STATUS_OK)
                    myprintf("Error!!! Scanning dir error: %d\n", res);
                path[i] = 0;
            } else { /* It is a file. */
                myprintf("%s/%s\r\n", path, fno.fname);
            }
        }
        f_closedir(&dir);
    }

    if (res != FR_OK){
        myprintf("Error!!! Scanning dir error: %d\n", res);
        return STATUS_ERROR;
    }
    else
        return STATUS_OK;
}

ret_status sd_card_close(void) {
    f_close(&fil);
    f_mount(NULL, "", 0);
    myprintf("\r\nSD card is UNMOUNTED\r\n");
    return STATUS_ERROR;
}

ret_status sd_card_read_data(char *path, uint8_t *data, struct layers_struct *layers, uint8_t *number_of_images) {


    FIL file;
    UINT br;
    char name[20] = { 0 };

    // #### READING CONGFIGURATION
    sprintf((char*)name, "%s//%s", (char*)path, "CONFIG~1.txt");
    myprintf("start reading file: %s\r\n", name);
    fres = f_open(&file, name, FA_READ);
    if (fres != FR_OK) {
        myprintf("f_open error (%i)\r\n", fres);
        return STATUS_ERROR;
    }

    UINT num = 0;

    uint8_t isComment = 0;
    char ch;
    TCHAR buffor[32] = {0};
    while(!f_eof(&file)){
        fres = f_read(&file, &ch, 1, &br);
        if (fres != FR_OK) {
            myprintf("f_read error (%i)\r\n", fres);
            return STATUS_ERROR;
        }

        if('\n' == ch){
            isComment = 0;
            continue;
        }
        if(isComment) continue;
        if('#' == ch){
            isComment = 1;
            continue;
        }

        // Check number of images in animation
        if('N' == ch){
            uint8_t length_of_line = 0;
            uint8_t position_of_char = 0;
            char c = 0;
            while(';' != c){
                length_of_line++;

                if('=' == c) position_of_char = length_of_line;

                fres = f_read(&file, &c, 1, &br);
                if (fres != FR_OK) {
                    myprintf("f_read error (%i)\r\n", fres);
                    return STATUS_ERROR;
                }
            }
            f_lseek(&file, f_tell(&file)-length_of_line);
            fres = f_read(&file, &buffor, length_of_line, &br);
            if (fres != FR_OK) {
                myprintf("f_read error (%i)\r\n", fres);
                return STATUS_ERROR;
            }
            char buffor_number_of_image[3] = {0};
            buffor_number_of_image[2] = '\0';
            if('=' != buffor[15]){
                myprintf("Wrong format of NUMBER_OF_IMAGES in configuration.txt");
                return STATUS_ERROR;
            }
            strncpy(buffor_number_of_image, &buffor[16], length_of_line-position_of_char);
            *number_of_images = (uint8_t)atoi(buffor_number_of_image);
            continue;
        }


        // Check what image should show on the property layer
        if('L' == ch){
            uint8_t length = 0;
            char c = 0;
            while(';' != c){
                length++;
                fres = f_read(&file, &c, 1, &br);
                if (fres != FR_OK) {
                    myprintf("f_read error (%i)\r\n", fres);
                    return STATUS_ERROR;
                }
            }
            f_lseek(&file, f_tell(&file)-length);
            fres = f_read(&file, &buffor, length, &br);
            if (fres != FR_OK) {
                myprintf("f_read error (%i)\r\n", fres);
                return STATUS_ERROR;
            }
            int i = 7;
            int count = 0;
            while(length > i){
                layers[num].values[count] = atoi(&buffor[i]);
                i+=2;
                count++;
            }
            layers[num].count = count;
            num++;
        }

    }
    f_close(&file);

    // #### READING FILES
//    myprintf("Reading files with animation\r\n");
    for (uint8_t image = 0; image < *number_of_images; image++) {
        char name[20] = { 0 };
        sprintf((char*)name, "%s//%d.txt", (char*)path, image);

		fres = f_open(&file, name, FA_READ);
		if (fres != FR_OK) {
			myprintf("f_open error (%i)\r\n", fres);
			return STATUS_ERROR;
		}

        UINT num = 0;
        int i = 0;
        int j = 0;
        while(i < 256){
            num = 1;
            TCHAR c[2] = {0};
            TCHAR string[4] = {0};

            fres = f_read(&file, c, 1, &br);
            if (fres != FR_OK) {
                myprintf("f_read error (%i)\r\n", fres);
                return STATUS_ERROR;
            }
            while(',' != c[0]){
                num++;
                fres = f_read(&file, c, 1, &br);
                if (fres != FR_OK) {
                    myprintf("f_read error (%i)\r\n", fres);
                    return STATUS_ERROR;
                }
            }
            f_lseek(&file, f_tell(&file)-num);
            fres = f_read(&file, string, num-1, &br);
            if (fres != FR_OK) {
                myprintf("f_read error (%i)\r\n", fres);
                return STATUS_ERROR;
            }
            f_lseek(&file, f_tell(&file)+1);
            *(data + image*256*3 + i*3 + j++) = atoi(string);
            if(j>2){
                j = 0;
                i++;
            }
        }
        f_close(&file);
        myprintf("Reading file %s is completed successfully\n", name);
    }
    return STATUS_OK;
}
