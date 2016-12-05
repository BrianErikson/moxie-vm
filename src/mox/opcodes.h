/* Instruction set bit masks for the Moxie processor
 * Copyright (C) 2016 Brian Erikson
 * Moxie processor copyright (C) 2012, 2015  Anthony Green
 * Author: Brian Erikson
 * License terms: GNU General Public License (GPL) version 2
*/

typedef enum {AND, ADD, ASHL, ASHR, BEQ, BGE, BGEU, BGT, BGTU, BLE, BLEU,
    BLT, BLTU, BNE, BRK, CMP, DEC, DIV, GSR, INC, JMP, JMPA, JSR, JSRA, LDB,
    LDL, LDS, LDAB, LDAL, LDAS, LDIL, LDIB, LDIS, LDOB, LDOL, LDOS, LSHR,
    MOD, MOV, MUL, MULX, NEG, NOP, NOT, OR, POP, PUSH, RET, SEXB, SEXS, SSR,
    STB, STL, STS, STAB, STAL, }
        Instruction;

/* Logical and. Performs a logical and operation on
 * the contents of registers $rA and $rB and stores
 * the result in $rA.*/
#define I_AND           0x2600
#define I_AND_A         0x00f0
#define I_AND_B         0x000f

/* Add, long. Adds the contents of registers $rA
 * and $rB and stores the result in $rA */
#define I_ADD           0x0500
#define I_ADD_A         0x00f0
#define I_ADD_B         0x000f

/* Arithmetic shift left. Performs an arithmetic
 * shift left of $rA byt $rB bits and stores
 * the result in $rA. */
#define I_ASHL          0x2800
#define I_ASHL_A        0x00f0
#define I_ASHL_B        0x000f

/* Arithmetic shift right. Performs an arithmetic shift right of $rA byt $rB
 * bits and stores the result in $rA */
#define I_ASHR          0x2c00
#define I_ASHR_A        0x00f0
#define I_ASHR_B        0x000f

/* Branch if equal. If the results of the last cmp demonstrated that $rA is
 * equal to $rB, branch to the address computed by adding the signed 10-bit
 * immediate value shifted to the left by 1 to the program counter.
 * The branch is relative to the start of this instruction. */
#define I_BEQ           0xc000
#define I_BEQ_V         0x03ff

/* Branch if greater than or equal. If the results of the last cmp demonstrated
 * that the signed 32-bit value in $rA is greater than or equal to the signed
 * 32-bit value in $rB, branch to the address computed by adding the signed
 * 10-bit immediate value shifted to the left by 1 to the program counter.
 * The branch is relative to the address of this instruction. */
#define I_BGE           0xd800
#define I_BGE_V         0x03ff

/* Branch if greater than or equal, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is greater than or equal
 * to the unsigned 32-bit value in $rB, branch to the address computed by adding
 * the signed 10-bit immediate value shifted to the left by 1 bit to the program
 * counter. The branch is relative to the address of this instruction. */
#define I_BGEU          0xe100
#define I_BGEU_V        0x03ff

/* Branch if greater than. If the results of the last cmp demonstrated that the
 * signed 32-bit value in $rA is greater than the signed 32-bit value in $rB,
 * branch to the address computed by adding the signed 10-bit immediate value
 * shifted to the left by 1 bit to the program counter. The branch is relative
 * to the address of this instruction. */
#define I_BGT           0xcc00
#define I_BGT_V         0x03ff

/* Branch if greater than, unsigned. If the results of the last cmp demonstrated
 * that the unsigned 32-bit value in $rA is greater than the unsigned 32-bit
 * value in $rB, branch to the address computed by the adding the signed 10-bit
 * immediate value shifted to the left by 1 bit to the program counter.
 * The branch is relative to the address of this instruction. */
#define I_BGTU          0xd400
#define I_BGTU_V        0x03ff

/* Branch if less than or equal. If the results of the last cmp demonstrated
 * that the signed 32-bit value in $rA is less than or equal to the signed
 * 32-bit value in $rB, branch to the address computed by adding the signed
 * 10-bit immediate value shifted to the left by 1 bit to the program counter.
 * The branch is relative to the address of this instruction. */
#define I_BLE           0xdc00
#define I_BLE_V         0x03ff

/* Branch if less than or equal, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is less than or equal to
 * the unsigned 32-bit value in $rB, branch to the address computed by adding
 * the signed 10-bit immediate value to the program counter. The branch is
 * relative to the address of this instruction. */
#define I_BLEU          0xe400
#define I_BLEU_V        0x03ff

/* Branch if less than. If the results of the last cmp demonstrated that the
 * signed 32-bit value in $rA is less than the signed 32-bit value in $rB,
 * branch to the address computed by adding the signed 10-bit immediate value
 * shifted to the left by 1 bit to the program counter. The branch is relative
 * to the address of this instruction. */
#define I_BLT           0xc800
#define I_BLT_V         0x03ff

/* Branch if less than, unsigned. If the results of the last cmp demonstrated
 * that the unsigned 32-bit value in $rA is less than the unsigned 32-bit value
 * in $rB, branch to the address computed by adding the signed 10-bit immediate
 * value shifted to the left by 1 bit to the program counter. The branch is
 * relative to the address of this instruction. */
#define I_BLTU          0xd000
#define I_BLTU_V        0x03ff

/* Branch if not equal. If the results of the last cmp demonstrated that $rA is
 * not equal to $rB, branch to the address computed by adding the signed 10-bit
 * immediate value shifted to the left by 1 bit to the program counter. The
 * branch is relative to the address of this instruction. */
#define I_BNE           0xc400
#define I_BNE_V         0x03ff

/* Break. The software breakpoint instruction. */
#define I_BRK           0x3500

/* Compare. Compares the contents of $rA to $rB and store the results in the
 * processor's internal condition code register. This is the only instruction
 * that updates the internal condition code register used by the branch
 * instructions. */
#define I_CMP           0x0e00
#define I_CMP_A         0x00f0
#define I_CMP_B         0x000f

/* Decrement. Decrement register $rA by the 8-bit value encoded in the
 * 16-bit opcode. */
#define I_DEC           0x9000
#define I_DEC_A         0x0f00
#define I_DEC_I         0x00ff

/* Divide, long. Divides the signed contents of registers $rA and $rB and stores
 * the result in $rA. Two special cases are handled here: division by zero
 * asserts an Divide by Zero [[Exceptions|Exception]], and INT_MIN divided by -1
 * results in INT_MIN. */
#define I_DIV           0x3100
#define I_DIV_A         0x00f0
#define I_DIV_B         0x000f

/* Get special register. Move the contents of the special register S into $rA.*/
#define I_GSR           0xb000
#define I_GSR_A         0x0f00
#define I_GSR_S         0x00ff

/* Increment. Increment register $rA by the 8-bit value encoded in the
 * 16-bit opcode. */
#define I_INC           0x8000
#define I_INC_A         0x0f00
#define I_INC_I         0x00ff

/* Jump. Jumps to the 32-bit address stored in $rA. This is not a subroutine
 * call, and therefore the stack is not updated. */
#define I_JMP           0x2500
#define I_JMP_A         0x00f0

/* Jump to address. Jumps to the 32-bit address following the 16-bit opcode.
 * This is not a subroutine call, and therefore the stack is not updated. */
#define I_JMPA          0x1a00

/* Jump to subroutine. Jumps to a subroutine at the address stored in $rA. */
#define I_JSR           0x1900
#define I_JSR_A         0x00f0

/* Jump to subroutine at absolute address. Jumps to a subroutine identified by
 * the 32-bit address following the 16-bit opcode. */
#define I_JSRA          0x0300

/* ld.b - Load byte. Loads the 8-bit contents stored at the address pointed
 * to by $rB into $rA. The loaded value is zero-extended to 32-bits. */
#define I_LDB           0x1c00
#define I_LDB_A         0x00f0
#define I_LDB_B         0x000f

/* ld.l - Load long. Loads the 32-bit contents stored at the address pointed
 * to by $rB into $rA. */
#define I_LDL           0x0a00
#define I_LDL_A         0x00f0
#define I_LDL_B         0x000f

/* ld.s - Load short. Loads the 16-bit contents stored at the address pointed
 * to by $rB into $rA. The loaded value is zero-extended to 32-bits. */
#define I_LDS           0x2100
#define I_LDS_A         0x00f0
#define I_LDS_B         0x000f

/* lda.b - Load absolute, byte. Loads the 8-bit value pointed at by the 32-bit
 * address following the 16-bit opcode into register $rA. The loaded value
 * is zero-extended to 32-bits. */
#define I_LDAB          0x1d00
#define I_LDAB_A        0x00f0

/* Load absolute, long. Loads the 32-bit value pointed at by the 32-bit address
 * following the 16-bit opcode into register $rA. */
#define I_LDAL          0x0800
#define I_LDAL_A        0x00f0

/* lda.s - Load absolute, short. Loads the 16-bit value pointed at by the
 * 32-bit address following the 16-bit opcode into register $rA. The loaded
 * value is zero-extended to 32-bits. */
#define I_LDAS          0x2200
#define I_LDAS_A        0x00f0

/* ldi.l - Load immediate, long. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. */
#define I_LDIL          0x0100
#define I_LDIL_A        0x00f0

/* ldi.b - Load immediate, byte. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. This is a poor encoding, as the 32-bit
 * value really only contains 8-bits of interest. */
#define I_LDIB          0x1b00
#define I_LDIB_A        0x00f0

/* ldi.s - Load immediate, short. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. This is a poor encoding, as the 32-bit
 * value really only contains 16-bits of interest. */
#define I_LDIS          0x2000
#define I_LDIS_A        0x00f0

/* ldo.b - Load offset, byte. Loads into $rA the 8-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. The loaded value is zero-extended to 32-bits. */
#define I_LDOB          0x3600
#define I_LDOB_A        0x00f0
#define I_LDOB_B        0x000f

/* lod-l - Load offset, long. Loads into $rA the 32-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. */
#define I_LDOL          0x0c00
#define I_LDOL_A        0x00f0
#define I_LDOL_B        0x000f

/* ldo.s - Load offset, short. Loads into $rA the 16-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. The loaded value is zero-extended to 32-bits. */
#define I_LDOS          0x3800
#define I_LDOS_A        0x00f0
#define I_LDOS_B        0x000f

/* Logical shift right. Performs a logical shift right of register $rA by $rB
 * bits and stores the result in $rA. */
#define I_LSHR          0x2700
#define I_LSHR_A        0x00f0
#define I_LSHR_B        0x000f

/* Modulus, long. Compute the modulus of the signed contents of registers $rA
 * and $rB and stores the result in $rA. */
#define I_MOD           0x3300
#define I_MOD_A         0x00f0
#define I_MOD_B         0x000f

/* Move register to register. Move the contents of $rB into $rA. */
#define I_MOV           0x0200
#define I_MOV_A         0x00f0
#define I_MOV_B         0x000f

/* Multiply. Multiplies the contents of registers $rA and $rB and stores the
 * lower 32-bits of a 64-bit result in $rA. */
#define I_MUL           0x2f00
#define I_MUL_A         0x00f0
#define I_MUL_B         0x000f

/* mul.x - Signed multiply, upper word. Multiplies the contents of registers $rA
 * and $rB and stores the upper 32-bits of a 64-bit result in $rA. */
#define I_MULX          0x1500
#define I_MULX_A        0x00f0
#define I_MULX_B        0x000f

/* Negative. Changes the sign of $rB and stores the result in $rA */
#define I_NEG           0x2a00
#define I_NEG_A         0x00f0
#define I_NEG_B         0x000f

/* Do nothing. */
#define I_NOP           0x0f00

/* Logical not. Performs a logical not operation on the contents of register $rB
 * and stores the result in register $rA. */
#define I_NOT           0x2c00
#define I_NOT_A         0x00f0
#define I_NOT_B         0x000f

/* Logical or. Performs a logical or operation on the contents of registers $rA
 * and $rB and stores the result in $rA. */
#define I_OR            0x2b00
#define I_OR_A          0x00f0
#define I_OR_B          0x000f

/* Pop the 32-bit contents of the top of the stack pointed to by $rB into $rA
 * and update the stack pointer. Stacks grown down. */
#define I_POP           0x0700
#define I_POP_A         0x00f0
#define I_POP_B         0x000f

/* Push the contents of $rB onto a stack pointed to by $rA and update the stack
 * pointer. Stacks grown down. */
#define I_PUSH          0x0600
#define I_PUSH_A        0x00f0
#define I_PUSH_B        0x000f

/* Return from subroutine. */
#define I_RET           0x0400

/* sex.b - Sign-extend byte. Sign-extend the lower 8-bits of $rB into a $rA
 * as a 32-bit value. */
#define I_SEXB          0x1000
#define I_SEXB_A        0x00f0
#define I_SEXB_B        0x000f

/* sex.s - Sign-extend short. Sign-extend the lower 16-bits of $rB into a $rA
 * as a 32-bit value. */
#define I_SEXS          0x1100
#define I_SEXS_A        0x00f0
#define I_SEXS_B        0x000f

/* Set special register. Move the contents of $rA into special register S. */
#define I_SSR           0xb000
#define I_SSR_A         0x0f00
#define I_SSR_S         0x00ff

/* st.b - Store byte. Stores the 8-bit contents of $rB into memory at the
 * address pointed to by $rA. */
#define I_STB           0x1e00
#define I_STB_A         0x00f0
#define I_STB_B         0x000f

/* st.l - Store long. Stores the 32-bit contents of $rB into memory at the
 * address pointed to by $rA. */
#define I_STL           0x0b00
#define I_STL_A         0x00f0
#define I_STL_B         0x000f

/* st.s - Store short. Stores the 16-bit contents of $rB into memory at the
 * address pointed to by $rA. */
#define I_STS           0x2300
#define I_STS_A         0x00f0
#define I_STS_B         0x000f

/* sta.b - Store absolute, byte. Stores the lower 8-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
#define I_STAB          0x1f00
#define I_STAB_A        0x00f0

/* sta.l - Store absolute, long. Stores the full 32-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
#define I_STAL          0x0900
#define I_STAL_A        0x00f0

/* sta.s - Store absolute, short. Stores the lower 16-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
#define I_STAS          0x2400
#define I_STAS_A        0x00f0

/* sto.b - Store offset, byte. Stores the 8-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
#define I_STOB          0x3700
#define I_STOB_A        0x00f0
#define I_STOB_B        0x000f

/* sto.l - Store offset, long. Stores the 32-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
#define I_STOL          0x0d00
#define I_STOL_A        0x00f0
#define I_STOL_B        0x000f

/* sto.s - Store offset, short. Stores the 16-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
#define I_STOS          0x3900
#define I_STOS_A        0x00f0
#define I_STOS_B        0x000f

/* Subtract, long. Subtracts the contents of registers $rA and $rB and stores
 * the result in $rA. */
#define I_SUB           0x2900
#define I_SUB_A         0x00f0
#define I_SUB_B         0x000f

/* Software interrupt. Trigger a software interrupt, where the interrupt type is
 * encoded in the 32-bits following the 16-bit opcode. */
#define I_SWI           0x3000

/* Divide unsigned, long. Divides the unsigned contents of registers $rA and $rB
 * and stores the result in $rA. */
#define I_UDIV          0x3200
#define I_UDIV_A        0x00f0
#define I_UDIV_B        0x000f

/* Modulus unsigned, long. Compute the modulus of the unsigned contents of
 * registers $rA and $rB and stores the result in $rA. */
#define I_UMOD          0x3400
#define I_UMOD_A        0x00f0
#define I_UMOD_B        0x000f

/* umul.x - Unsigned multiply, upper word. Multiplies the contents of registers
 * $rA and $rB and stores the upper 32-bits of an unsigned 64-bit result in
 * $rA. */
#define I_UMULX         0x1400
#define I_UMULX_A       0x00f0
#define I_UMULX_B       0x000f

/* Logical exclusive or. Performs a logical exclusive or operation on the
 * contents of registers $rA and $rB and stores the result in $rA. */
#define I_XOR           0x2e00
#define I_XOR_A         0x00f0
#define I_XOR_B         0x000f

/* zex.b - Zero-extend byte. Zero-extend the lower 8-bits of $rB into a $rA
 * as a 32-bit value. */
#define I_ZEXB          0x1200
#define I_ZEXB_A        0x00f0
#define I_ZEXB_B        0x000f

/* zex.s - Zero-extend short. Zero-extend the lower 16-bits of $rB into a $rA
 * as a 32-bit value. */
#define I_ZEXS          0x1300
#define I_ZEXS_A        0x00f0
#define I_ZEXS_B        0x000f