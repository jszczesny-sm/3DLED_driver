#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdint.h>

int main(void)
{
    FILE *file;
    if((file = fopen("./walking_anim/0.txt", "r")) == NULL){
        printf("Error! opening file\n");
    }

    // Working read data from file
    // int array[1][256][3] = {0};
    // int i = 0;
    // for (i = 0; i < 256; i++){
    //     fscanf(file, "%d,", &array[0][i][0]);
    //     fscanf(file, "%d,", &array[0][i][1]);
    //     fscanf(file, "%d,", &array[0][i][2]);
    // }

    // while(!feof(file)){
    //     num++;
    //     fgetc(file);
    // // }
    // fseek(file, 0, SEEK_SET);
    // // char* tmp = strrchr(argv[1], '\\')

    // printf("n = %d\n", num);
    uint8_t array[1][256][3] = {0};
    char string[4];
    int num = 0;
    int i = 0;
    int j = 0;
    while(!feof(file) && i < 256){
        num = 1;
        while(',' != fgetc(file)) num++;
        fseek(file, -num, SEEK_CUR);
        // fread(string, 1, num, file);
        fgets(string, num, file);
         if( ferror( file ) != 0 )
             printf( "Blad zapisu danych do pliku.\n" );
        fseek(file, 1, SEEK_CUR);
        array[0][i][j++] = atoi(string);
        if(j>2){
            j = 0;
            i++;
        }
        printf("Value of n = %s\n", string);
    }


    

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
    fclose(file);


    printf("Read data:\n");
    for (size_t x = 0; x < 256; x++)
    {
        printf("{%d, %d, %d}, ", array[0][x][0], array[0][x][1], array[0][x][2]);
        if(x != 0 && (x+1)%16==0)
            printf("\n");
    }
    

    getchar();
    return 0;
}