#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// struct layers_struct
// {
//     int count;
//     int values[32];
// };

int main(void)
{
    FILE *file;
    if((file = fopen("./walking_anim/0.txt", "r")) == NULL){
        printf("Error! opening file\n");
    }

    // WORKING
    int array[2][256][3] = {0};
    int i = 0;
    for (i = 0; i < 256; i++){
        fscanf(file, "%d,", &array[0][i][0]);
        fscanf(file, "%d,", &array[0][i][1]);
        fscanf(file, "%d,", &array[0][i][2]);
    }
    fclose(file);

    if((file = fopen("./walking_anim/1.txt", "r")) == NULL){
        printf("Error! opening file\n");
    }

    // WORKING
    for (i = 0; i < 256; i++){
        fscanf(file, "%d,", &array[1][i][0]);
        fscanf(file, "%d,", &array[1][i][1]);
        fscanf(file, "%d,", &array[1][i][2]);
    }
    fclose(file);


    // int num = 0;
    // unsigned int isComment = 0;
    // char ch;
    // char buffor[255];
    // // struct layers_struct layers[5];
    // int number_of_animation;
    // while(!feof(file)){
    //     ch = fgetc(file);
        
    //     if('\n' == ch){
    //         isComment = 0;
    //         printf("end of line\n");
    //     }
    //     if(isComment) continue;
    //     if('#' == ch){
    //         isComment = 1;
    //         continue;
    //     }
    //        if('N' == ch){   
    //         // int file_index = ftell(file);
    //         int length = 0;
    //         int pos_of_char = 0;
    //         char c = 0;
    //         while(';' != c){
    //             length++;
    //             if('=' == c) pos_of_char = length;
    //             c = fgetc(file);
    //         }
    //         fseek(file, -length, SEEK_CUR);         
    //         fread(&buffor, 1, length, file);
           
    //         char buffor_number_of_image[16];
    //         if('=' != buffor[15]){
    //             printf("Wrong format of NUMBER_OF_IMAGES in configuration.txt 16:%s", &buffor[15]);
    //             return -1;
    //         }
    //         printf("len-pos=%d\n", length-pos_of_char);
    //         strncpy(buffor_number_of_image, &buffor[16], length-pos_of_char);
    //         printf("buffor_num=%s", buffor_number_of_image);
    //         number_of_animation = atoi(buffor_number_of_image);
    //     }
    // }

    //     printf("%d\n", number_of_animation);
    //     if('L' == ch){   
    //         int file_index = ftell(file);
    //         int length = 0;
    //         while(';' != fgetc(file)) length++;
    //         fseek(file, file_index, SEEK_SET);         
    //         fread(&buffer, 1, length, file);
    //         printf("length=%d\n", length);
    //         int i = 7;
    //         int count = 0;
    //         while(length > i){
    //             layers[num].values[count] = atoi(&buffer[i]);
    //             printf("value[i=%d length=%d] = %d\n", i, length, layers[num].values[count]);
    //             i+=2;
    //             count++;
    //         } 
    //         layers[num].count = count;
    //         num++;
    //     }

    // }

    // printf("end of file\n");num++;

    // for (size_t i = 0; i < 5; i++)
    // {
    //     printf("LAYER_%d:\n", i);
    //     for (size_t j = 0; j < layers[i].count; j++)
    //     {
    //         printf("%d\n", layers[i].values[j]);
    //     }
        
    // }
    



    //WORKING
    // uint8_t array[1][256][3] = {0};
    // char string[4];
    // int num = 0;
    // int i = 0;
    // int j = 0;
    // while(!feof(file) && i < 256){
    //     num = 1;
    //     while(',' != fgetc(file)) num++;
    //     fseek(file, -num, SEEK_CUR);
    //     // fread(string, 1, num, file);
    //     fgets(string, num, file);
    //      if( ferror( file ) != 0 )
    //          printf( "Blad zapisu danych do pliku.\n" );
    //     fseek(file, 1, SEEK_CUR);
    //     array[0][i][j++] = atoi(string);
    //     if(j>2){
    //         j = 0;
    //         i++;
    //     }
    //     printf("Value of n = %s\n", string);
    // }


    

    // uint8_t array[1][256][3] = {0};
    // int i = 0;
    // int j = 0;
    // int curr = 0;
    // rewind(file);
    // while(!feof(file)){
    //     char string[4] = {0};
    //     // fgets(string, 5, file);
    //     fread(string, 1, 4, file);
    //     printf("string = %s \n", string);
        
    //     char *token = strtok(string, ",");
    //     curr += strlen(token)+1;
    //     fseek(file, curr, SEEK_SET);
        
    //     array[1][i][j] = atoi(token);
    //     printf("array[1][%d][%d] = %d \n", i, j, array[1][i][j]);
    //     j++;
    //     if(2 < j){
    //         j = 0;
    //         i++;  
    //     }   
    // }
    


    // printf("Read data:\n");
    // for (size_t x = 0; x < 256; x++)
    // {
    //     printf("{%d, %d, %d}, ", array[0][x][0], array[0][x][1], array[0][x][2]);
    //     if(x != 0 && (x+1)%16==0)
    //         printf("\n");
    // }
    printf("Read data:\n");
    // for (size_t x = 0; x < 256; x++)
    // {
    //     if( 200 < array[0][x][0]) printf("%c", (char)219);
    //     else printf("%c", (char)250);
    //     if(x != 0 && (x+1)%16==0)
    //         printf("\n");
    // }

    uint8_t odd = 1U;
    uint8_t index = 0;
    uint8_t counter = 0U;
    // #define abs(x) (x<0)?(x*(-1)):(x)
    for (uint16_t i = 0U; i < 256; i++) {
        if ( (i % 16U) == 0U ) {
            odd ^= 1U;
            counter++;
        }
        if (!odd) {
            index = 16*(16-i%16) - counter;
        } else {
            index = 256-(16*(16-((i+1)%16))+counter);
        }
        if( 200 < array[1][index][0]) /*printf("%c", (char)219);*/printf("%d ", index);
        else /*printf("%c", (char)250);*/printf("%d ", index);
        if(i != 0 && (i+1)%16==0)
            printf("\n");
    }

    // getchar();
    return 0;
}