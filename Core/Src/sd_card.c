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
//	BYTE readBuf[30];

//	// f_gets <--------------
//	//Now let's try to open file "test.txt"
//	fres = f_open(&fil, "test.txt", FA_READ);
//	if (fres != FR_OK) {
//		myprintf("f_open error (%i)\r\n", fres);
//		return NOT_OK;
//	}
//	myprintf("I was able to open 'test.txt' for reading!\r\n");
//
//	//Read 30 bytes from "test.txt" on the SD card
//
//
//	//We can either use f_read OR f_gets to get data out of files
//	//f_gets is a wrapper on f_read that does some string formatting for us
//	TCHAR *rres = f_gets((TCHAR*) readBuf, 30, &fil);
//	if (rres != 0) {
//		myprintf("Read string from 'test.txt' contents: %s\r\n", readBuf);
//	} else {
//		myprintf("f_gets error (%i)\r\n", fres);
//	}
//
//	//Be a tidy kiwi - don't forget to close your file!
//	f_close(&fil);

    //Now let's try and write a file "write.txt"
//	fres = f_open(&fil, "write.txt",
//			FA_WRITE | FA_OPEN_ALWAYS | FA_CREATE_ALWAYS);
//	if (fres == FR_OK) {
//		myprintf("I was able to open 'write.txt' for writing\r\n");
//	} else {
//		myprintf("f_open error (%i)\r\n", fres);
//	}
//
//	//Copy in a string
//	strncpy((char*) readBuf, "a new file is made!", 19);
//	UINT bytesWrote;
//	fres = f_write(&fil, readBuf, 19, &bytesWrote);
//	if (fres == FR_OK) {
//		myprintf("Wrote %i bytes to 'write.txt'!\r\n", bytesWrote);
//	} else {
//		myprintf("f_write error (%i)\r\n", fres);
//	}
//

    return STATUS_OK;
}

ret_status sd_card_scan_file(char *path, char* buffor_dirs, uint8_t* size_dirs) {

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
                sprintf((char*)(buffor_dirs + (*size_dirs)*16), "%s", path);
                (*size_dirs)++;
                res = sd_card_scan_file(path, NULL, NULL); /* Enter the directory */
                if (res != FR_OK)
                    break;
                path[i] = 0;
            } else { /* It is a file. */
                myprintf("%s/%s\r\n", path, fno.fname);
            }
        }
        f_closedir(&dir);
    }
    else {
        myprintf("Error!!! Scanning dir error: %d\n", res);
        return STATUS_ERROR;
    }
    return STATUS_OK;
}

ret_status sd_card_close(void) {
    f_close(&fil);
    f_mount(NULL, "", 0);
    myprintf("\r\nSD card is UNMOUNTED\r\n");
    return STATUS_ERROR;
}

ret_status sd_card_read_data(char *path, uint8_t *data, struct layers_struct *layers) {


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
    TCHAR buffer[32] = {0};
    while(!f_eof(&file)){
        fres = f_read(&file, &ch, 1, &br);
        if (fres != FR_OK) {
            myprintf("f_read error (%i)\r\n", fres);
            return STATUS_ERROR;
        }

        if('\n' == ch){
            isComment = 0;
            printf("end of line\n");
        }
        if(isComment) continue;
        if('#' == ch){
            isComment = 1;
            continue;
        }

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
            fres = f_read(&file, &buffer, length, &br);
            if (fres != FR_OK) {
                myprintf("f_read error (%i)\r\n", fres);
                return STATUS_ERROR;
            }
            int i = 7;
            int count = 0;
            while(length > i){
                layers[num].values[count] = atoi(&buffer[i]);
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
    for (uint8_t image = 0; image < 10; image++) {
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
