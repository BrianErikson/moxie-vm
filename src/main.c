#include <stdio.h>
#include <sys/param.h>
#include <stdlib.h>
#include <string.h>
#include "mox/processor.h"

#define ASM_BUF_SIZE 1024

const char *asmFile = "../asm/init.8basm";

int main(int argc, char *argv[]) {
    char filePath[PATH_MAX];
    memset(filePath, 0x0, PATH_MAX);
    void *result = realpath(argv[1], filePath);
    if (result == NULL) {
        printf("Cannot get absolute path for %s", argv[1]);
        return -1;
    }

    FILE *file = fopen(filePath, "rb"); // open file for reading

    unsigned int readSize = 0;
    unsigned short buf[ASM_BUF_SIZE];
    memset(buf, 0x0, ASM_BUF_SIZE);
    
    mox_init();

    while((readSize = (unsigned int) fread(buf, sizeof(unsigned short),
                                           ASM_BUF_SIZE, file)) > 0) {
         mox_load(buf, readSize);
    }

    fclose(file);

    mox_free();
    return 0;
}
