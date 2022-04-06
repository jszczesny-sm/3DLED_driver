#include <stdio.h>

int main(void)
{
    char tmp2[] = "dupa";
    char file_name[40];
    sprintf(file_name, "./%s.txt", tmp2);
    printf("%s\n", file_name);
    getchar();
    return 0;
}