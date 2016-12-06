
#define MOX_ERR_UNSUPPORTED 0x01

#define MOX_ERR_REGISTER_NOT_FOUND 0x01
#define MOX_ERR_REGISTER_NO_SPACE 0x02

#define MOX_REG_SIZE  14
#define MOX_SREG_SIZE 10

/** status regsiter with the following bit values: */
#define MOX_SREG_STATUS             0
/** a pointer to the Exception Handler routine (invoked by swi, IRQs, Divide by
 * Zero and illegal instructions (bad)) */
#define MOX_SREG_EXCPT_ADDR         1
/** upon invocation of the Excecption Handler (see above), special register 2
 * will have one of four values.. */
#define MOX_SREG_EXCPT_RTRN         2
/**the swi request number (by convention) */
#define MOX_SREG_SWI_NUMBER         3
/** address of the supervisor mode stack on which exceptions are executed */
#define MOX_SREG_EXCP_SVR_STACK     4
/** return address upon entering the exception handler */
#define MOX_SREG_EXCPT_RTRN_ADDR    5
/**an optional non-zero pointer to the Device Tree blob describing this device*/
#define MOX_SREG_DEVICE_ADDR        9

typedef struct Register_s {
    long data;
    unsigned char address;
} Register;

typedef struct SpecialRegister_s {
    char data;
    unsigned char address;
} SpecialRegister;

typedef struct Registers_s {
    Register fp;
    Register sp;
    Register r[MOX_REG_SIZE];
    SpecialRegister s[MOX_SREG_SIZE];
} Registers;

typedef struct InstructionSet_s {
    unsigned short *instructions;
    unsigned int size;
} InstructionSet;

/**
 * Initializes the registers for use
 * @return 
 */
unsigned char mox_init();

/**
 * Loads a set of instructions to execute
 * @param instructions
 * @param size
 * @return
 */
unsigned char mox_load(unsigned short *instructions, unsigned int size);

/**
 * Executes loaded instructions
 * @return
 */
unsigned char mox_exec();

/**
 * Sets all registers to 0x0
 * @return
 */
unsigned char mox_reset();

void mox_free();

/** Logical and. Performs a logical and operation on
 * the contents of registers $rA and $rB and stores
 * the result in $rA.*/
unsigned char mox_and(unsigned char a, unsigned char b);

/** Add, long. Adds the contents of registers $rA
 * and $rB and stores the result in $rA */
unsigned char mox_add(unsigned char a, unsigned char b);

/** Arithmetic shift left. Performs an arithmetic
 * shift left of $rA byt $rB bits and stores
 * the result in $rA. */
unsigned char mox_ashl(unsigned char a, unsigned char b);

/** Arithmetic shift right. Performs an arithmetic shift right of $rA byt $rB
 * bits and stores the result in $rA */
unsigned char mox_ashr(unsigned char a, unsigned char b);

/** Branch if equal. If the results of the last cmp demonstrated that $rA is
 * equal to $rB, branch to the address computed by adding the signed 10-bit
 * immediate value shifted to the left by 1 to the program counter.
 * The branch is relative to the start of this instruction. */
unsigned char mox_beq(unsigned short v);

/** Branch if greater than or equal. If the results of the last cmp demonstrated
 * that the signed 32-bit value in $rA is greater than or equal to the signed
 * 32-bit value in $rB, branch to the address computed by adding the signed
 * 10-bit immediate value shifted to the left by 1 to the program counter.
 * The branch is relative to the address of this instruction. */
unsigned char mox_bge(unsigned short v);

/** Branch if greater than or equal, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is greater than or equal
 * to the unsigned 32-bit value in $rB, branch to the address computed by adding
 * the signed 10-bit immediate value shifted to the left by 1 bit to the program
 * counter. The branch is relative to the address of this instruction. */
unsigned char mox_bgeu(unsigned short v);

/** Branch if greater than. If the results of the last cmp demonstrated that the
 * signed 32-bit value in $rA is greater than the signed 32-bit value in $rB,
 * branch to the address computed by adding the signed 10-bit immediate value
 * shifted to the left by 1 bit to the program counter. The branch is relative
 * to the address of this instruction. */
unsigned char mox_bgt(unsigned short v);

/** Branch if greater than, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is greater than the
 * unsigned 32-bit value in $rB, branch to the address computed by the adding
 * the signed 10-bit immediate value shifted to the left by 1 bit to the
 * program counter. The branch is relative to the address of this instruction*/
unsigned char mox_bgtu(unsigned short v);

/** Branch if less than or equal. If the results of the last cmp demonstrated
 * that the signed 32-bit value in $rA is less than or equal to the signed
 * 32-bit value in $rB, branch to the address computed by adding the signed
 * 10-bit immediate value shifted to the left by 1 bit to the program counter.
 * The branch is relative to the address of this instruction. */
unsigned char mox_ble(unsigned short v);

/** Branch if less than or equal, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is less than or equal to
 * the unsigned 32-bit value in $rB, branch to the address computed by adding
 * the signed 10-bit immediate value to the program counter. The branch is
 * relative to the address of this instruction. */
unsigned char mox_bleu(unsigned short v);

/** Branch if less than. If the results of the last cmp demonstrated that the
 * signed 32-bit value in $rA is less than the signed 32-bit value in $rB,
 * branch to the address computed by adding the signed 10-bit immediate value
 * shifted to the left by 1 bit to the program counter. The branch is relative
 * to the address of this instruction. */
unsigned char mox_blt(unsigned short v);

/** Branch if less than, unsigned. If the results of the last cmp demonstrated
 * that the unsigned 32-bit value in $rA is less than the unsigned 32-bit value
 * in $rB, branch to the address computed by adding the signed 10-bit immediate
 * value shifted to the left by 1 bit to the program counter. The branch is
 * relative to the address of this instruction. */
unsigned char mox_bltu(unsigned short v);

/** Branch if not equal. If the results of the last cmp demonstrated that $rA is
 * not equal to $rB, branch to the address computed by adding the signed 10-bit
 * immediate value shifted to the left by 1 bit to the program counter. The
 * branch is relative to the address of this instruction. */
unsigned char mox_bne(unsigned short v);

/** Break. The software breakpoint instruction. */
unsigned char mox_brk();

/** Compare. Compares the contents of $rA to $rB and store the results in the
 * processor's internal condition code register. This is the only instruction
 * that updates the internal condition code register used by the branch
 * instructions. */
unsigned char mox_cmp(unsigned char a, unsigned char b);

/** Decrement. Decrement register $rA by the 8-bit value encoded in the
 * 16-bit opcode. */
unsigned char mox_dec(unsigned char a, unsigned short i);

/** Divide, long. Divides the signed contents of registers $rA and $rB and stores
 * the result in $rA. Two special cases are handled here: division by zero
 * asserts an Divide by Zero [[Exceptions|Exception]], and INT_MIN divided by -1
 * results in INT_MIN. */
unsigned char mox_div(unsigned char a, unsigned char b);

/** Get special register. Move the contents of the special register S into $rA.*/
unsigned char mox_gsr(unsigned char a, unsigned char s);

/** Increment. Increment register $rA by the 8-bit value encoded in the
 * 16-bit opcode. */
unsigned char mox_inc(unsigned char a, unsigned short i);

/** Jump. Jumps to the 32-bit address stored in $rA. This is not a subroutine
 * call, and therefore the stack is not updated. */
unsigned char mox_jmp(unsigned char a);

/** Jump to address. Jumps to the 32-bit address following the 16-bit opcode.
 * This is not a subroutine call, and therefore the stack is not updated. */
unsigned char mox_jmpa(unsigned long i);

/** Jump to subroutine. Jumps to a subroutine at the address stored in $rA. */
unsigned char mox_jsr(unsigned char a);

/** Jump to subroutine at absolute address. Jumps to a subroutine identified by
 * the 32-bit address following the 16-bit opcode. */
unsigned char mox_jsra(unsigned long i);

/** ld.b - Load byte. Loads the 8-bit contents stored at the address pointed
 * to by $rB into $rA. The loaded value is zero-extended to 32-bits. */
unsigned char mox_ldb(unsigned char a, unsigned char b);

/** ld.l - Load long. Loads the 32-bit contents stored at the address pointed
 * to by $rB into $rA. */
unsigned char mox_ldl(unsigned char a, unsigned char b);

/** ld.s - Load short. Loads the 16-bit contents stored at the address pointed
 * to by $rB into $rA. The loaded value is zero-extended to 32-bits. */
unsigned char mox_lds(unsigned char a, unsigned char b);

/** lda.b - Load absolute, byte. Loads the 8-bit value pointed at by the 32-bit
 * address following the 16-bit opcode into register $rA. The loaded value
 * is zero-extended to 32-bits. */
unsigned char mox_ldab(unsigned char a, unsigned long i);

/** Load absolute, long. Loads the 32-bit value pointed at by the 32-bit address
 * following the 16-bit opcode into register $rA. */
unsigned char mox_ldal(unsigned char a, unsigned long i);

/** lda.s - Load absolute, short. Loads the 16-bit value pointed at by the
 * 32-bit address following the 16-bit opcode into register $rA. The loaded
 * value is zero-extended to 32-bits. */
unsigned char mox_ldas(unsigned char a, unsigned long i);

/** ldi.l - Load immediate, long. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. */
unsigned char mox_ldil(unsigned char a, unsigned long i);

/** ldi.b - Load immediate, byte. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. This is a poor encoding, as the 32-bit
 * value really only contains 8-bits of interest. */
unsigned char mox_ldib(unsigned char a, unsigned long i);

/** ldi.s - Load immediate, short. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. This is a poor encoding, as the 32-bit
 * value really only contains 16-bits of interest. */
unsigned char mox_ldis(unsigned char a, unsigned long i);

/** ldo.b - Load offset, byte. Loads into $rA the 8-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. The loaded value is zero-extended to 32-bits. */
unsigned char mox_ldob(unsigned char a, unsigned char b, unsigned short i);

/** lod-l - Load offset, long. Loads into $rA the 32-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. */
unsigned char mox_ldol(unsigned char a, unsigned char b, unsigned short i);

/** ldo.s - Load offset, short. Loads into $rA the 16-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. The loaded value is zero-extended to 32-bits. */
unsigned char mox_ldos(unsigned char a, unsigned char b, unsigned short i);

/** Logical shift right. Performs a logical shift right of register $rA by $rB
 * bits and stores the result in $rA. */
unsigned char mox_lshr(unsigned char a, unsigned char b);

/** Modulus, long. Compute the modulus of the signed contents of registers $rA
 * and $rB and stores the result in $rA. */
unsigned char mox_mod(unsigned char a, unsigned char b);

/** Move register to register. Move the contents of $rB into $rA. */
unsigned char mox_mov(unsigned char a, unsigned char b);

/** Multiply. Multiplies the contents of registers $rA and $rB and stores the
 * lower 32-bits of a 64-bit result in $rA. */
unsigned char mox_mul(unsigned char a, unsigned char b);

/** mul.x - Signed multiply, upper word. Multiplies the contents of registers $rA
 * and $rB and stores the upper 32-bits of a 64-bit result in $rA. */
unsigned char mox_mulx(unsigned char a, unsigned char b);

/** Negative. Changes the sign of $rB and stores the result in $rA */
unsigned char mox_neg(unsigned char a, unsigned char b);

/** Do nothing. */
unsigned char mox_nop();

/** Logical not. Performs a logical not operation on the contents of register $rB
 * and stores the result in register $rA. */
unsigned char mox_not(unsigned char a, unsigned char b);

/** Logical or. Performs a logical or operation on the contents of registers $rA
 * and $rB and stores the result in $rA. */
unsigned char mox_or(unsigned char a, unsigned char b);

/** Pop the 32-bit contents of the top of the stack pointed to by $rB into $rA
 * and update the stack pointer. Stacks grown down. */
unsigned char mox_pop(unsigned char a, unsigned char b);

/** Push the contents of $rB onto a stack pointed to by $rA and update the stack
 * pointer. Stacks grown down. */
unsigned char mox_push(unsigned char a, unsigned char b);

/** Return from subroutine. */
unsigned char mox_ret();

/** sex.b - Sign-extend byte. Sign-extend the lower 8-bits of $rB into a $rA
 * as a 32-bit value. */
unsigned char mox_sexb(unsigned char a, unsigned char b);

/** sex.s - Sign-extend short. Sign-extend the lower 16-bits of $rB into a $rA
 * as a 32-bit value. */
unsigned char mox_sexs(unsigned char a, unsigned char b);

/** Set special register. Move the contents of $rA into special register S. */
unsigned char mox_ssr(unsigned char a, unsigned char s);

/** st.b - Store byte. Stores the 8-bit contents of $rB into memory at the
 * address pointed to by $rA. */
unsigned char mox_stb(unsigned char a, unsigned char b);

/** st.l - Store long. Stores the 32-bit contents of $rB into memory at the
 * address pointed to by $rA. */
unsigned char mox_stl(unsigned char a, unsigned char b);

/** st.s - Store short. Stores the 16-bit contents of $rB into memory at the
 * address pointed to by $rA. */
unsigned char mox_sts(unsigned char a, unsigned char b);

/** sta.b - Store absolute, byte. Stores the lower 8-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
unsigned char mox_stab(unsigned char a, unsigned long i);

/** sta.l - Store absolute, long. Stores the full 32-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
unsigned char mox_stal(unsigned char a, unsigned long i);

/** sta.s - Store absolute, short. Stores the lower 16-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
unsigned char mox_stas(unsigned char a, unsigned long i);

/** sto.b - Store offset, byte. Stores the 8-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
unsigned char mox_stob(unsigned char a, unsigned char b, unsigned short i);

/** sto.l - Store offset, long. Stores the 32-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
unsigned char mox_stol(unsigned char a, unsigned char b, unsigned short i);

/** sto.s - Store offset, short. Stores the 16-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
unsigned char mox_stos(unsigned char a, unsigned char b, unsigned short i);

/** Subtract, long. Subtracts the contents of registers $rA and $rB and stores
 * the result in $rA. */
unsigned char mox_sub(unsigned char a, unsigned char b);

/** Software interrupt. Trigger a software interrupt, where the interrupt type is
 * encoded in the 32-bits following the 16-bit opcode. */
unsigned char mox_swi(unsigned long i);

/** Divide unsigned, long. Divides the unsigned contents of registers $rA and $rB
 * and stores the result in $rA. */
unsigned char mox_udiv(unsigned char a, unsigned char b);

/** Modulus unsigned, long. Compute the modulus of the unsigned contents of
 * registers $rA and $rB and stores the result in $rA. */
unsigned char mox_umod(unsigned char a, unsigned char b);

/** umul.x - Unsigned multiply, upper word. Multiplies the contents of registers
 * $rA and $rB and stores the upper 32-bits of an unsigned 64-bit result in
 * $rA. */
unsigned char mox_umulx(unsigned char a, unsigned char b);

/** Logical exclusive or. Performs a logical exclusive or operation on the
 * contents of registers $rA and $rB and stores the result in $rA. */
unsigned char mox_xor(unsigned char a, unsigned char b);

/** zex.b - Zero-extend byte. Zero-extend the lower 8-bits of $rB into a $rA
 * as a 32-bit value. */
unsigned char mox_zexb(unsigned char a, unsigned char b);

/** zex.s - Zero-extend short. Zero-extend the lower 16-bits of $rB into a $rA
 * as a 32-bit value. */
unsigned char mox_zexs(unsigned char a, unsigned char b);
