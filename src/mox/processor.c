#include <stdio.h>
#include <malloc.h>
#include <string.h>
#include <errno.h>
#include "processor.h"
#include "opcodes.h"
#include "paging.h"


Registers *registers;
InstructionSet *instructionSet;

/**
 * Must be followed by mox_free() when done
 * @return
 */
unsigned char mox_init() {
    registers = malloc(sizeof(Registers));
    instructionSet = malloc(sizeof(instructionSet));

    memset(registers, 0x0, sizeof(Registers));
    memset(instructionSet, 0x0, sizeof(InstructionSet));

    return 0;
}

void mox_free(){
    free(registers);
    free(instructionSet->instructions);
    free(instructionSet);
}

/**
 * Load instructions to the CPU. NOTE: instructions are copied
 * @param instructions
 * @param size
 * @return 0 on success, MOX_ERR code on ret > 0
 */
unsigned char mox_load(unsigned short *instructions, unsigned int size) {
    unsigned short *_instructions = malloc(sizeof(unsigned short) * size);
    memcpy(_instructions, instructions, sizeof(unsigned short) * size);
    instructionSet->instructions = _instructions;
    instructionSet->size = size;
    // TODO: Ensure valid instruction set?
    return 0;
}

unsigned char mox_exec() {int retval = -1;
    for (int i = 0; i < instructionSet->size; i++) {
        unsigned short instr = instructionSet->instructions[i];
        if ((instr & I_ADD) == I_ADD) {
            retval = mox_add((unsigned char)(instr & I_ADD_A),
                             (unsigned char)(instr & I_ADD_B));
        }
        else if ((instr & I_AND) == I_AND) {
            retval = mox_and((unsigned char)(instr & I_AND_A),
                             (unsigned char)(instr & I_AND_B));
        }
        else if ((instr & I_ASHL) == I_ASHL) {
            retval = mox_ashl((unsigned char)(instr & I_ASHL_A),
                              (unsigned char)(instr & I_ASHL_B));
        }
        else if ((instr & I_ASHR) == I_ASHR) {
            retval = mox_ashr((unsigned char)(instr & I_ASHR_A),
                              (unsigned char)(instr & I_ASHR_B));
        }
        else if ((instr & I_BEQ) == I_BEQ) {
            retval = mox_beq((unsigned short)(instr & I_BEQ_V));
        }
        else if ((instr & I_BGE) == I_BGE) {
            retval = mox_bge((unsigned short)(instr & I_BGE_V));
        }
        else if ((instr & I_BGEU) == I_BGEU) {
            retval = mox_bgeu((unsigned short)(instr & I_BGEU_V));
        }
        else if ((instr & I_BGT) == I_BGT) {
            retval = mox_bgt((unsigned short)(instr & I_BGT_V));
        }
        else if ((instr & I_BGTU) == I_BGTU) {
            retval = mox_bgtu((unsigned short)(instr & I_BGTU_V));
        }
        else if ((instr & I_BLE) == I_BLE) {
            retval = mox_ble((unsigned short)(instr & I_BLE_V));
        }
        else if ((instr & I_BLEU) == I_BLEU) {
            retval = mox_bleu((unsigned short)(instr & I_BLEU_V));
        }
        else if ((instr & I_BLT) == I_BLT) {
            retval = mox_blt((unsigned short)(instr & I_BLT_V));
        }
        else if ((instr & I_BLTU) == I_BLTU) {
            retval = mox_bltu((unsigned short)(instr & I_BLTU_V));
        }
        else if ((instr & I_BNE) == I_BNE) {
            retval = mox_bne((unsigned short)(instr & I_BNE_V));
        }
        else if ((instr & I_BRK) == I_BRK) {
            retval = mox_brk();
        }
        else if ((instr & I_CMP) == I_CMP) {
            retval = mox_cmp((unsigned char)(instr & I_CMP_A),
                             (unsigned char)(instr & I_CMP_B));
        }
        else if ((instr & I_DEC) == I_DEC) {
            retval = mox_dec((unsigned char)(instr & I_DEC_A),
                             (unsigned short)(instr & I_DEC_I));
        }
        else if ((instr & I_DIV) == I_DIV) {
            retval = mox_div((unsigned char)(instr & I_DIV_A),
                             (unsigned char)(instr & I_DIV_B));
        }
        else if ((instr & I_GSR) == I_GSR) {
            retval = mox_gsr((unsigned char)(instr & I_GSR_A),
                             (unsigned char)(instr & I_GSR_S));
        }
        else if ((instr & I_INC) == I_INC) {
            retval = mox_inc((unsigned char)(instr & I_INC_A),
                             (unsigned short)(instr & I_INC_I));
        }
        else if ((instr & I_JMP) == I_JMP) {
            retval = mox_jmp((unsigned char)(instr & I_JMP_A));
        }
        else if ((instr & I_JMPA) == I_JMPA) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_jmpa(addr);
        }
        else if ((instr & I_JSR) == I_JSR) {
            retval = mox_jsr((unsigned char)(instr & I_JSR_A));
        }
        else if ((instr & I_JSRA) == I_JSRA) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_jsra(addr);
        }
        else if ((instr & I_LDAB) == I_LDAB) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_ldab((unsigned char)(instr & I_LDAB_A), addr);
        }
        else if ((instr & I_LDAL) == I_LDAL) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_ldal((unsigned char)(instr & I_LDAL_A), addr);
        }
        else if ((instr & I_LDAS) == I_LDAS) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_ldas((unsigned char)(instr & I_LDAS_A), addr);
        }
        else if ((instr & I_LDB) == I_LDB) {
            retval = mox_ldb((unsigned char)(instr & I_LDB_A),
                             (unsigned char)(instr & I_LDB_B));
        }
        else if ((instr & I_LDIB) == I_LDIB) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_ldib((unsigned char)(instr & I_LDIB_A), addr);
        }
        else if ((instr & I_LDIL) == I_LDIL) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_ldil((unsigned char)(instr & I_LDIL_A), addr);
        }
        else if ((instr & I_LDIS) == I_LDIS) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_ldis((unsigned char)(instr & I_LDIS_A), addr);
        }
        else if ((instr & I_LDL) == I_LDL) {
            retval = mox_ldl((unsigned char)(instr & I_LDL_A),
                             (unsigned char)(instr & I_LDL_B));
        }
        else if ((instr & I_LDOB) == I_LDOB) {
            unsigned short addr = instructionSet->instructions[++i];
            retval = mox_ldob((unsigned char)(instr & I_LDOB_A),
                              (unsigned char)(instr & I_LDOB_B), addr);
        }
        else if ((instr & I_LDOL) == I_LDOL) {
            unsigned short addr = instructionSet->instructions[++i];
            retval = mox_ldol((unsigned char)(instr & I_LDOL_A),
                              (unsigned char)(instr & I_LDOL_B), addr);
        }
        else if ((instr & I_LDOS) == I_LDOS) {
            unsigned short addr = instructionSet->instructions[++i];
            retval = mox_ldos((unsigned char)(instr & I_LDOS_A),
                              (unsigned char)(instr & I_LDOS_B), addr);
        }
        else if ((instr & I_LDS) == I_LDS) {
            retval = mox_lds((unsigned char)(instr & I_LDS_A),
                             (unsigned char)(instr & I_LDS_B));
        }
        else if ((instr & I_LSHR) == I_LSHR) {
            retval = mox_lshr((unsigned char)(instr & I_LSHR_A),
                              (unsigned char)(instr & I_LSHR_B));
        }
        else if ((instr & I_MOD) == I_MOD) {
            retval = mox_mod((unsigned char)(instr & I_MOD_A),
                             (unsigned char)(instr & I_MOD_B));
        }
        else if ((instr & I_MOV) == I_MOV) {
            retval = mox_mov((unsigned char)(instr & I_MOV_A),
                             (unsigned char)(instr & I_MOV_B));
        }
        else if ((instr & I_MUL) == I_MUL) {
            retval = mox_mul((unsigned char)(instr & I_MUL_A),
                             (unsigned char)(instr & I_MUL_B));
        }
        else if ((instr & I_MULX) == I_MULX) {
            retval = mox_mulx((unsigned char)(instr & I_MULX_A),
                              (unsigned char)(instr & I_MULX_B));
        }
        else if ((instr & I_NEG) == I_NEG) {
            retval = mox_neg((unsigned char)(instr & I_NEG_A),
                             (unsigned char)(instr & I_NEG_B));
        }
        else if ((instr & I_NOP) == I_NOP) {
            retval = mox_nop();
        }
        else if ((instr & I_NOT) == I_NOT) {
            retval = mox_not((unsigned char)(instr & I_NOT_A),
                             (unsigned char)(instr & I_NOT_B));
        }
        else if ((instr & I_OR) == I_OR) {
            retval = mox_or((unsigned char)(instr & I_OR_A),
                            (unsigned char)(instr & I_OR_B));
        }
        else if ((instr & I_POP) == I_POP) {
            retval = mox_pop((unsigned char)(instr & I_POP_A),
                             (unsigned char)(instr & I_POP_B));
        }
        else if ((instr & I_PUSH) == I_PUSH) {
            retval = mox_push((unsigned char)(instr & I_PUSH_A),
                              (unsigned char)(instr & I_PUSH_B));
        }
        else if ((instr & I_RET) == I_RET) {
            retval = mox_ret();
        }
        else if ((instr & I_SEXB) == I_SEXB) {
            retval = mox_sexb((unsigned char)(instr & I_SEXB_A),
                              (unsigned char)(instr & I_SEXB_B));
        }
        else if ((instr & I_SEXS) == I_SEXS) {
            retval = mox_sexs((unsigned char)(instr & I_SEXS_A),
                              (unsigned char)(instr & I_SEXS_B));
        }
        else if ((instr & I_SSR) == I_SSR) {
            retval = mox_ssr((unsigned char)(instr & I_SSR_A),
                             (unsigned char)(instr & I_SSR_S));
        }
        else if ((instr & I_STAB) == I_STAB) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_stab((unsigned char)(instr & I_STAB_A), addr);
        }
        else if ((instr & I_STAL) == I_STAL) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_stal((unsigned char)(instr & I_STAL_A), addr);
        }
        else if ((instr & I_STAS) == I_STAS) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_stas((unsigned char)(instr & I_STAS_A), addr);
        }
        else if ((instr & I_STB) == I_STB) {
            retval = mox_stb((unsigned char)(instr & I_STB_A),
                             (unsigned char)(instr & I_STB_B));
        }
        else if ((instr & I_STL) == I_STL) {
            retval = mox_stl((unsigned char)(instr & I_STL_A),
                             (unsigned char)(instr & I_STL_B));
        }
        else if ((instr & I_STOB) == I_STOB) {
            unsigned short addr = instructionSet->instructions[++i];
            retval = mox_stob((unsigned char)(instr & I_STOB_A),
                              (unsigned char)(instr & I_STOB_B), addr);

        }
        else if ((instr & I_STOL) == I_STOL) {
            unsigned short addr = instructionSet->instructions[++i];
            retval = mox_stol((unsigned char)(instr & I_STOL_A),
                              (unsigned char)(instr & I_STOL_B), addr);
        }
        else if ((instr & I_STOS) == I_STOS) {
            unsigned short addr = instructionSet->instructions[++i];
            retval = mox_stos((unsigned char)(instr & I_STOS_A),
                              (unsigned char)(instr & I_STOS_B), addr);
        }
        else if ((instr & I_STS) == I_STS) {
            retval = mox_sts((unsigned char)(instr & I_STS_A),
                             (unsigned char)(instr & I_STS_B));
        }
        else if ((instr & I_SUB) == I_SUB) {
            retval = mox_sub((unsigned char)(instr & I_SUB_A),
                             (unsigned char)(instr & I_SUB_B));
        }
        else if ((instr & I_SWI) == I_SWI) {
            unsigned long addr = (((unsigned long)
                    instructionSet->instructions[i + 1]) << 16)
                                 | instructionSet->instructions[i + 2];
            i += 2;
            retval = mox_swi(addr);
        }
        else if ((instr & I_UDIV) == I_UDIV) {
            retval = mox_udiv((unsigned char)(instr & I_UDIV_A),
                              (unsigned char)(instr & I_UDIV_B));
        }
        else if ((instr & I_UMOD) == I_UMOD) {
            retval = mox_umod((unsigned char)(instr & I_UMOD_A),
                              (unsigned char)(instr & I_UMOD_B));
        }
        else if ((instr & I_UMULX) == I_UMULX) {
            retval = mox_umulx((unsigned char)(instr & I_UMULX_A),
                               (unsigned char)(instr & I_UMULX_B));
        }
        else if ((instr & I_XOR) == I_XOR) {
            retval = mox_xor((unsigned char)(instr & I_XOR_A),
                             (unsigned char)(instr & I_XOR_B));
        }
        else if ((instr & I_ZEXB) == I_ZEXB) {
            retval = mox_zexb((unsigned char)(instr & I_ZEXB_A),
                              (unsigned char)(instr & I_ZEXB_B));
        }
        else if ((instr & I_ZEXS) == I_ZEXS) {
            retval = mox_zexs((unsigned char)(instr & I_ZEXS_A),
                              (unsigned char)(instr & I_ZEXS_B));
        }
        else {
            printf("Unrecognized command: %x", instr);
            retval = MOX_ERR_UNSUPPORTED;
        }

        if (retval != 0) {
            printf("Processor errorcode: %d. Aborting.", retval);
            break;
        }
    }

    return retval;
}

static Register* getRegister(unsigned char virtualAddr,
                             unsigned char *errFlags) {
    for (int i = 0; i < MOX_REG_SIZE; i++) {
        Register *reg = &registers->r[i];
        if (reg->address == NULL) {
            reg->address = virtualAddr;
            *errFlags = 0;
            return reg;
        }
        else if (reg->address == virtualAddr) {
            *errFlags = 0;
            return reg;
        }
    }

    *errFlags = MOX_ERR_REGISTER_NOT_FOUND | MOX_ERR_REGISTER_NO_SPACE;
    return NULL;
}

static unsigned char setRegister(unsigned char virtualAddr,
                                 unsigned long data) {
    unsigned char errFlags;
    Register *reg = getRegister(virtualAddr, &errFlags);
    if (reg == NULL) {
        return errFlags;
    }

    reg->data = data;
    return 0;
}


/**
 * Resets the processor registers. Does not clear instruction set
 * @Returns
 */
unsigned char mox_reset() {
    memset(registers, 0x0, sizeof(Registers));
    return 0;
}

typedef struct RegisterPair_s {
    Register* a;
    Register* b;
} RegisterPair;
RegisterPair getRegisterPair(unsigned char virtualAddrA,
                             unsigned char virtualAddrB,
                             unsigned char *errFlags) {
    Register *aReg = getRegister(virtualAddrA, errFlags);
    if (aReg == NULL) {
        return NULL;
    }
    Register *bReg = getRegister(virtualAddrB, errFlags);
    if (bReg == NULL) {
        return NULL;
    }

    RegisterPair pair;
    pair.a = aReg;
    pair.b = bReg;
    return pair;
}

unsigned char errFlags = 0x0;
unsigned char mox_and(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data & pair.b->data;
    return 0;
}

unsigned char mox_add(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data + pair.b->data;
    return 0;
}

unsigned char mox_ashl(unsigned char a, unsigned char b) {
    // TODO: ASHL vs LSHL?
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data << pair.b->data;
    return 0;
}

unsigned char mox_ashr(unsigned char a, unsigned char b) {
    // TODO: ASHR vs LSHR?
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data >> pair.b->data;
    return 0;
}

unsigned char mox_beq(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_bge(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_bgeu(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_bgt(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_bgtu(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ble(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_bleu(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_blt(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_bltu(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_bne(unsigned short v) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_brk() {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_cmp(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    registers->s[MOX_SREG_STATUS].data =
            (char) (pair.a->data == pair.b->data);
    return 0;
}

unsigned char mox_dec(unsigned char a, unsigned short i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_div(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data / pair.b->data;
    // TODO: Error codes for div by 0 or div by INT_MIN
    return 0;
}

unsigned char mox_gsr(unsigned char a, unsigned char s) {
    if (s > MOX_SREG_SIZE) return MOX_ERR_REGISTER_NOT_FOUND;
    Register *aReg = getRegister(a, &errFlags);
    if (aReg == NULL) return errFlags;

    aReg->data = registers->s[s].data;
    return 0;
}

unsigned char mox_inc(unsigned char a, unsigned short i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_jmp(unsigned char a) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_jmpa(unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_jsr(unsigned char a) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_jsra(unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldb(unsigned char a, unsigned char b) {
    return mox_ldl(a, b);
}

unsigned char mox_ldl(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.b->data;
    return 0;
}

unsigned char mox_lds(unsigned char a, unsigned char b) {
    return mox_ldl(a, b);
}

unsigned char mox_ldab(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldal(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldas(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldil(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldib(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldis(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldob(unsigned char a, unsigned char b, unsigned short i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldol(unsigned char a, unsigned char b, unsigned short i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ldos(unsigned char a, unsigned char b, unsigned short i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_lshr(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_mod(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data % pair.b->data;
    return 0;
}

unsigned char mox_mov(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.b->data;
    return 0;
}

unsigned char mox_mul(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data * pair.b->data;
    return 0;
}

unsigned char mox_mulx(unsigned char a, unsigned char b) {
    // TODO: Might need to implement?
    return mox_mul(a, b);
}

unsigned char mox_neg(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = -pair.b->data;
    return 0;
}

unsigned char mox_nop() {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_not(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data != pair.b->data;
    return 0;
}

unsigned char mox_or(unsigned char a, unsigned char b) {
    RegisterPair pair = getRegisterPair(a, b, &errFlags);
    if (errFlags != 0x0) return errFlags;
    pair.a->data = pair.a->data || pair.b->data;
    return 0;
}

unsigned char mox_pop(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_push(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ret() {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_sexb(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_sexs(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_ssr(unsigned char a, unsigned char s) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_stb(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_stl(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_sts(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_stab(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_stal(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_stas(unsigned char a, unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_stob(unsigned char a, unsigned char b, unsigned short i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_stol(unsigned char a, unsigned char b, unsigned short i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_stos(unsigned char a, unsigned char b, unsigned short i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_sub(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_swi(unsigned long i) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_udiv(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_umod(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_umulx(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_xor(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_zexb(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}

unsigned char mox_zexs(unsigned char a, unsigned char b) {
    return MOX_ERR_UNSUPPORTED;
}
