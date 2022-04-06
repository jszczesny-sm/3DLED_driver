#include <stdio.h>
#include <string.h>
#include <unistd.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

int main(int argc, char **argv)
{
    printf("argc = %d\n", argc);
    if(argc<2)
    {
        printf("Missing file path...\n");
        getchar();
        return 0;
    }
    mkdir("output");
    
    for (int i = 1; i < argc; i++)
    {
        printf("Loaded file:\n");
        char* tmp = strrchr(argv[i], '\\');
        char tmp2[40];
        strncpy(tmp2, &tmp[1], (size_t)(strlen(tmp)-5));
        tmp2[strlen(tmp)-5] = '\0';
        printf("%s\n",  tmp2);

        printf("Convert FILE %s to TXT format\r\n",  tmp2);
        FILE *file;
        char file_name[40];
        sprintf(file_name, "./output/%s.txt", tmp2);
        printf("%s\n", file_name);
        file = fopen(file_name, "w+");

        int x,y,n;
        unsigned char *data = stbi_load(argv[i], &x, &y, &n, 0);
        int i = 0;
        while(i<x*y*n)
        {
            fprintf(file, "%d,",data[i]);
            i++;
        }

        stbi_image_free(data);
        fclose(file);
    }
    printf("CONVERTING SUCESFULL\r\n");
    getchar();
    return 0;
}