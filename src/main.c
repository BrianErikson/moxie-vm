#include <stdio.h>
#include <sys/param.h>
#include <stdlib.h>
#include <string.h>

#define ASM_BUF_SIZE 1024

const char *asmFile = "../asm/init.8basm";

int main(int argc, char *argv[]) {
    char filePath[PATH_MAX];
    memset(filePath, 0x0, PATH_MAX);
    void *result = realpath("../asm/test.masm", filePath);
    if (result == NULL) {
        puts("Cannot get absolute path for test.masm");
        return -1;
    }

    FILE *file = fopen(filePath, "rb"); // open file for reading

    unsigned int readSize = 0;
    unsigned short buf[ASM_BUF_SIZE];
    memset(buf, 0x0, ASM_BUF_SIZE);

    while((readSize = (unsigned int) fread(buf, sizeof(unsigned short),
                                           ASM_BUF_SIZE, file)) > 0) {
        puts(buf);
    }

    fclose(file);

    return 0;
}
