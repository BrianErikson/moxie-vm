/* Logical and. Performs a logical and operation on
 * the contents of registers $rA and $rB and stores
 * the result in $rA.*/
#define I_AND       0x2600
#define I_AND_A     0x00f0
#define I_AND_B     0x000f

/* Add, long. Adds the contents of registers $rA
 * and $rB and stores the result in $rA */
#define I_ADD       0x0500
#define I_ADD_A     0x00f0
#define I_ADD_B     0x000f

/* Arithmetic shift left. Performs an arithmetic
 * shift left of $rA byt $rB bits and stores
 * the result in $rA. */
#define I_ASHL      0x2800
#define I_ASHL_A    0x00f0
#define I_ASHL_B    0x000f

/* Arithmetic shift right. Performs an arithmetic shift right of $rA byt $rB
 * bits and stores the result in $rA */
#define I_ASHR      0x2c00
#define I_ASHR_A    0x00f0
#define I_ASHR_B    0x000f

/* Branch if equal. If the results of the last cmp demonstrated that $rA is
 * equal to $rB, branch to the address computed by adding the signed 10-bit
 * immediate value shifted to the left by 1 to the program counter.
 * The branch is relative to the start of this instruction. */
#define I_BEQ       0xc000
#define I_BEQ_V     0x03ff

/* Branch if greater than or equal. If the results of the last cmp demonstrated
 * that the signed 32-bit value in $rA is greater than or equal to the signed
 * 32-bit value in $rB, branch to the address computed by adding the signed
 * 10-bit immediate value shifted to the left by 1 to the program counter.
 * The branch is relative to the address of this instruction. */
#define I_BGE       0xd800
#define I_BGE_V     0x03ff

/* Branch if greater than or equal, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is greater than or equal
 * to the unsigned 32-bit value in $rB, branch to the address computed by adding
 * the signed 10-bit immediate value shifted to the left by 1 bit to the program
 * counter. The branch is relative to the address of this instruction. */
#define I_BGEU      0xe100
#define I_BGEU_V    0x03ff

/* Branch if greater than. If the results of the last cmp demonstrated that the
 * signed 32-bit value in $rA is greater than the signed 32-bit value in $rB,
 * branch to the address computed by adding the signed 10-bit immediate value
 * shifted to the left by 1 bit to the program counter. The branch is relative
 * to the address of this instruction. */
#define I_BGT       0xcc00
#define I_BGT_V     0x03ff

/* Branch if greater than, unsigned. If the results of the last cmp demonstrated
 * that the unsigned 32-bit value in $rA is greater than the unsigned 32-bit
 * value in $rB, branch to the address computed by the adding the signed 10-bit
 * immediate value shifted to the left by 1 bit to the program counter.
 * The branch is relative to the address of this instruction. */
#define I_BGTU      0xd400
#define I_BGTU_V    0x03ff

/* Branch if less than or equal. If the results of the last cmp demonstrated
 * that the signed 32-bit value in $rA is less than or equal to the signed
 * 32-bit value in $rB, branch to the address computed by adding the signed
 * 10-bit immediate value shifted to the left by 1 bit to the program counter.
 * The branch is relative to the address of this instruction. */
#define I_BLE       0xdc00
#define I_BLE_V     0x03ff

/* Branch if less than or equal, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is less than or equal to
 * the unsigned 32-bit value in $rB, branch to the address computed by adding
 * the signed 10-bit immediate value to the program counter. The branch is
 * relative to the address of this instruction. */
#define I_BLEU      0xe400
#define I_BLEU_V    0x03ff

/* Branch if less than. If the results of the last cmp demonstrated that the
 * signed 32-bit value in $rA is less than the signed 32-bit value in $rB,
 * branch to the address computed by adding the signed 10-bit immediate value
 * shifted to the left by 1 bit to the program counter. The branch is relative
 * to the address of this instruction. */
#define I_BLT       0xc800
#define I_BLT_V     0x03ff

/* Branch if less than, unsigned. If the results of the last cmp demonstrated
 * that the unsigned 32-bit value in $rA is less than the unsigned 32-bit value
 * in $rB, branch to the address computed by adding the signed 10-bit immediate
 * value shifted to the left by 1 bit to the program counter. The branch is
 * relative to the address of this instruction. */
#define I_BLTU      0xd000
#define I_BLTU_V    0x03ff

/* Branch if not equal. If the results of the last cmp demonstrated that $rA is
 * not equal to $rB, branch to the address computed by adding the signed 10-bit
 * immediate value shifted to the left by 1 bit to the program counter. The
 * branch is relative to the address of this instruction. */
#define I_BNE       0xc400
#define I_BNE_V     0x03ff