#include <stdio.h>
#include <string.h>
#define STB_IMAGE_IMPLEMENTATION
#include "stb_image.h"

int main(int argc, char **argv)
{
    printf("argc = %d\n", argc);
  
    for (int i = 0; i < argc; i++)
    {
         printf("argv[%d] = %s\n", i, argv[i]);
    }
    if(argc<2)
    {
        printf("Missing file path...\n");
        getchar();
        return 0;
    }
    
    // char* tmp = strrchr(argv[2], '\\');

    // printf("strstr = %s", tmp);

    printf("Convert FILE %s to TXT format\r\n",  argv[1]);

    FILE *file;
    file = fopen("./output/output.txt", "w+");

    int x,y,n;
    unsigned char *data = stbi_load(argv[1], &x, &y, &n, 0);

    for (size_t i = 0; (int)i < x*y*n; i+=3)
    {
        if(i>0 && i%48 == 0)
            fprintf(file, "\n");
        fprintf(file, "{%d,%d,%d} ", data[i], data[i+1], data[i+2]);
    }

    stbi_image_free(data);
    fclose(file);
    printf("CONVERTING SUCESFULL\r\n");

    getchar();
    return 0;
}