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
        for (unsigned int i = 0; i < readSize; i++) {
            unsigned short instr = buf[0];

            if ((instr & I_ADD) == I_ADD) {

            }
            else if ((instr & I_AND) == I_AND) {

            }
            else if ((instr & I_ASHL) == I_ASHL) {

            }
            else if ((instr & I_ASHR) == I_ASHR) {

            }
            else if ((instr & I_BEQ) == I_BEQ) {

            }
            else if ((instr & I_BGE) == I_BGE) {

            }
            else if ((instr & I_BGEU) == I_BGEU) {

            }
            else if ((instr & I_BGT) == I_BGT) {

            }
            else if ((instr & I_BGTU) == I_BGTU) {

            }
            else if ((instr & I_BLE) == I_BLE) {

            }
            else if ((instr & I_BLEU) == I_BLEU) {

            }
            else if ((instr & I_BLT) == I_BLT) {

            }
            else if ((instr & I_BLTU) == I_BLTU) {

            }
            else if ((instr & I_BNE) == I_BNE) {

            }
            else if ((instr & I_BRK) == I_BRK) {

            }
            else if ((instr & I_CMP) == I_CMP) {

            }
            else if ((instr & I_DEC) == I_DEC) {

            }
            else if ((instr & I_DIV) == I_DIV) {

            }
            else if ((instr & I_GSR) == I_GSR) {

            }
            else if ((instr & I_INC) == I_INC) {

            }
            else if ((instr & I_JMP) == I_JMP) {

            }
            else if ((instr & I_JMPA) == I_JMPA) {

            }
            else if ((instr & I_JSR) == I_JSR) {

            }
            else if ((instr & I_JSRA) == I_JSRA) {

            }
            else if ((instr & I_LDAB) == I_LDAB) {

            }
            else if ((instr & I_LDAL) == I_LDAL) {

            }
            else if ((instr & I_LDAS) == I_LDAS) {

            }
            else if ((instr & I_LDB) == I_LDB) {

            }
            else if ((instr & I_LDIB) == I_LDIB) {

            }
            else if ((instr & I_LDIL) == I_LDIL) {

            }
            else if ((instr & I_LDIS) == I_LDIS) {

            }
            else if ((instr & I_LDL) == I_LDL) {

            }
            else if ((instr & I_LDOB) == I_LDOB) {

            }
            else if ((instr & I_LDOL) == I_LDOL) {

            }
            else if ((instr & I_LDOS) == I_LDOS) {

            }
            else if ((instr & I_LDS) == I_LDS) {

            }
            else if ((instr & I_LSHR) == I_LSHR) {

            }
            else if ((instr & I_MOD) == I_MOD) {

            }
            else if ((instr & I_MOV) == I_MOV) {

            }
            else if ((instr & I_MUL) == I_MUL) {

            }
            else if ((instr & I_MULX) == I_MULX) {

            }
            else if ((instr & I_NEG) == I_NEG) {

            }
            else if ((instr & I_NOP) == I_NOP) {

            }
            else if ((instr & I_NOT) == I_NOT) {

            }
            else if ((instr & I_OR) == I_OR) {

            }
            else if ((instr & I_POP) == I_POP) {

            }
            else if ((instr & I_PUSH) == I_PUSH) {

            }
            else if ((instr & I_RET) == I_RET) {

            }
            else if ((instr & I_SEXB) == I_SEXB) {

            }
            else if ((instr & I_SEXS) == I_SEXS) {

            }
            else if ((instr & I_SSR) == I_SSR) {

            }
            else if ((instr & I_STAB) == I_STAB) {

            }
            else if ((instr & I_STAL) == I_STAL) {

            }
            else if ((instr & I_STAS) == I_STAS) {

            }
            else if ((instr & I_STB) == I_STB) {

            }
            else if ((instr & I_STL) == I_STL) {

            }
            else if ((instr & I_STOB) == I_STOB) {

            }
            else if ((instr & I_STOL) == I_STOL) {

            }
            else if ((instr & I_STOS) == I_STOS) {

            }
            else if ((instr & I_STS) == I_STS) {

            }
            else if ((instr & I_SUB) == I_SUB) {

            }
            else if ((instr & I_SWI) == I_SWI) {

            }
            else if ((instr & I_UDIV) == I_UDIV) {

            }
            else if ((instr & I_UMOD) == I_UMOD) {

            }
            else if ((instr & I_UMULX) == I_UMULX) {

            }
            else if ((instr & I_XOR) == I_XOR) {

            }
            else if ((instr & I_ZEXB) == I_ZEXB) {

            }
            else if ((instr & I_ZEXS) == I_ZEXS) {

            }
            else {
                printf("Unrecognized command: %x", buf[i]);
            }

        }

    }

    fclose(file);

    return 0;
}
