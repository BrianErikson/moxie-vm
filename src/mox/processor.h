
typedef struct Register_s {
    long data;
    char address;
} Register;

typedef struct SpecialRegister_s {
    char data;
    char address;
} SpecialRegister;

typedef struct Registers_s {
    Register fp;
    Register sp;
    Register r[14];
    SpecialRegister s[10];
} Registers;

/**
 * Initializes the registers for use
 * @return 
 */
int mox_init();

/**
 * Loads a set of instructions to execute
 * @param instructions
 * @param size
 * @return
 */
int mox_load(unsigned short *instructions, int size);

/**
 * Executes loaded instructions
 * @return
 */
int mox_exec();

/**
 * Sets all registers to 0x0
 * @return
 */
int mox_reset();

/** Logical and. Performs a logical and operation on
 * the contents of registers $rA and $rB and stores
 * the result in $rA.*/
int mox_and(char a, char b);

/** Add, long. Adds the contents of registers $rA
 * and $rB and stores the result in $rA */
int mox_add(char a, char b);

/** Arithmetic shift left. Performs an arithmetic
 * shift left of $rA byt $rB bits and stores
 * the result in $rA. */
int mox_ashl(char a, char b);

/** Arithmetic shift right. Performs an arithmetic shift right of $rA byt $rB
 * bits and stores the result in $rA */
int mox_ashr(char a, char b);

/** Branch if equal. If the results of the last cmp demonstrated that $rA is
 * equal to $rB, branch to the address computed by adding the signed 10-bit
 * immediate value shifted to the left by 1 to the program counter.
 * The branch is relative to the start of this instruction. */
int mox_beq(short v);

/** Branch if greater than or equal. If the results of the last cmp demonstrated
 * that the signed 32-bit value in $rA is greater than or equal to the signed
 * 32-bit value in $rB, branch to the address computed by adding the signed
 * 10-bit immediate value shifted to the left by 1 to the program counter.
 * The branch is relative to the address of this instruction. */
int mox_bge(short v);

/** Branch if greater than or equal, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is greater than or equal
 * to the unsigned 32-bit value in $rB, branch to the address computed by adding
 * the signed 10-bit immediate value shifted to the left by 1 bit to the program
 * counter. The branch is relative to the address of this instruction. */
int mox_bgeu(short v);

/** Branch if greater than. If the results of the last cmp demonstrated that the
 * signed 32-bit value in $rA is greater than the signed 32-bit value in $rB,
 * branch to the address computed by adding the signed 10-bit immediate value
 * shifted to the left by 1 bit to the program counter. The branch is relative
 * to the address of this instruction. */
int mox_bgt(short v);

/** Branch if greater than, unsigned. If the results of the last cmp demonstrated
 * that the unsigned 32-bit value in $rA is greater than the unsigned 32-bit
 * value in $rB, branch to the address computed by the adding the signed 10-bit
 * immediate value shifted to the left by 1 bit to the program counter.
 * The branch is relative to the address of this instruction. */
int mox_bgtu(short v);

/** Branch if less than or equal. If the results of the last cmp demonstrated
 * that the signed 32-bit value in $rA is less than or equal to the signed
 * 32-bit value in $rB, branch to the address computed by adding the signed
 * 10-bit immediate value shifted to the left by 1 bit to the program counter.
 * The branch is relative to the address of this instruction. */
int mox_ble(short v);

/** Branch if less than or equal, unsigned. If the results of the last cmp
 * demonstrated that the unsigned 32-bit value in $rA is less than or equal to
 * the unsigned 32-bit value in $rB, branch to the address computed by adding
 * the signed 10-bit immediate value to the program counter. The branch is
 * relative to the address of this instruction. */
int mox_bleu(short v);

/** Branch if less than. If the results of the last cmp demonstrated that the
 * signed 32-bit value in $rA is less than the signed 32-bit value in $rB,
 * branch to the address computed by adding the signed 10-bit immediate value
 * shifted to the left by 1 bit to the program counter. The branch is relative
 * to the address of this instruction. */
int mox_blt(short v);

/** Branch if less than, unsigned. If the results of the last cmp demonstrated
 * that the unsigned 32-bit value in $rA is less than the unsigned 32-bit value
 * in $rB, branch to the address computed by adding the signed 10-bit immediate
 * value shifted to the left by 1 bit to the program counter. The branch is
 * relative to the address of this instruction. */
int mox_bltu(short v);

/** Branch if not equal. If the results of the last cmp demonstrated that $rA is
 * not equal to $rB, branch to the address computed by adding the signed 10-bit
 * immediate value shifted to the left by 1 bit to the program counter. The
 * branch is relative to the address of this instruction. */
int mox_bne(short v);

/** Break. The software breakpoint instruction. */
int mox_brk();

/** Compare. Compares the contents of $rA to $rB and store the results in the
 * processor's internal condition code register. This is the only instruction
 * that updates the internal condition code register used by the branch
 * instructions. */
int mox_cmp(char a, char b);

/** Decrement. Decrement register $rA by the 8-bit value encoded in the
 * 16-bit opcode. */
int mox_dec(char a, char b);

/** Divide, long. Divides the signed contents of registers $rA and $rB and stores
 * the result in $rA. Two special cases are handled here: division by zero
 * asserts an Divide by Zero [[Exceptions|Exception]], and INT_MIN divided by -1
 * results in INT_MIN. */
int mox_div(char a, char b);

/** Get special register. Move the contents of the special register S into $rA.*/
int mox_gsr(char a, char s);

/** Increment. Increment register $rA by the 8-bit value encoded in the
 * 16-bit opcode. */
int mox_inc(char a, char i);

/** Jump. Jumps to the 32-bit address stored in $rA. This is not a subroutine
 * call, and therefore the stack is not updated. */
int mox_jmp(char a);

/** Jump to address. Jumps to the 32-bit address following the 16-bit opcode.
 * This is not a subroutine call, and therefore the stack is not updated. */
int mox_jmpa(long i);

/** Jump to subroutine. Jumps to a subroutine at the address stored in $rA. */
int mox_jsr(char a);

/** Jump to subroutine at absolute address. Jumps to a subroutine identified by
 * the 32-bit address following the 16-bit opcode. */
int mox_jsra(long i);

/** ld.b - Load byte. Loads the 8-bit contents stored at the address pointed
 * to by $rB into $rA. The loaded value is zero-extended to 32-bits. */
int mox_ldb(char a, char b);

/** ld.l - Load long. Loads the 32-bit contents stored at the address pointed
 * to by $rB into $rA. */
int mox_ldl(char a, char b);

/** ld.s - Load short. Loads the 16-bit contents stored at the address pointed
 * to by $rB into $rA. The loaded value is zero-extended to 32-bits. */
int mox_lds(char a, char b);

/** lda.b - Load absolute, byte. Loads the 8-bit value pointed at by the 32-bit
 * address following the 16-bit opcode into register $rA. The loaded value
 * is zero-extended to 32-bits. */
int mox_ldab(char a, long i);

/** Load absolute, long. Loads the 32-bit value pointed at by the 32-bit address
 * following the 16-bit opcode into register $rA. */
int mox_ldal(char a, long i);

/** lda.s - Load absolute, short. Loads the 16-bit value pointed at by the
 * 32-bit address following the 16-bit opcode into register $rA. The loaded
 * value is zero-extended to 32-bits. */
int mox_ldas(char a, long i);

/** ldi.l - Load immediate, long. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. */
int mox_ldil(char a, long i);

/** ldi.b - Load immediate, byte. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. This is a poor encoding, as the 32-bit
 * value really only contains 8-bits of interest. */
int mox_ldib(char a, long i);

/** ldi.s - Load immediate, short. Loads the 32-bit immediate following the
 * 16-bit opcode into register %rA. This is a poor encoding, as the 32-bit
 * value really only contains 16-bits of interest. */
int mox_ldis(char a, long i);

/** ldo.b - Load offset, byte. Loads into $rA the 8-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. The loaded value is zero-extended to 32-bits. */
int mox_ldob(char a, char b, short i);

/** lod-l - Load offset, long. Loads into $rA the 32-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. */
int mox_ldol(char a, char b, short i);

/** ldo.s - Load offset, short. Loads into $rA the 16-bit value from memory
 * pointed at by the address produced by adding the 16-bit value following
 * the 16-bit opcode to $rB. The loaded value is zero-extended to 32-bits. */
int mox_ldos(char a, char b, short i);

/** Logical shift right. Performs a logical shift right of register $rA by $rB
 * bits and stores the result in $rA. */
int mox_lshr(char a, char b);

/** Modulus, long. Compute the modulus of the signed contents of registers $rA
 * and $rB and stores the result in $rA. */
int mox_mod(char a, char b);

/** Move register to register. Move the contents of $rB into $rA. */
int mox_mov(char a, char b);

/** Multiply. Multiplies the contents of registers $rA and $rB and stores the
 * lower 32-bits of a 64-bit result in $rA. */
int mox_mul(char a, char b);

/** mul.x - Signed multiply, upper word. Multiplies the contents of registers $rA
 * and $rB and stores the upper 32-bits of a 64-bit result in $rA. */
int mox_mulx(char a, char b);

/** Negative. Changes the sign of $rB and stores the result in $rA */
int mox_neg(char a, char b);

/** Do nothing. */
int mox_nop();

/** Logical not. Performs a logical not operation on the contents of register $rB
 * and stores the result in register $rA. */
int mox_not(char a, char b);

/** Logical or. Performs a logical or operation on the contents of registers $rA
 * and $rB and stores the result in $rA. */
int mox_or(char a, char b);

/** Pop the 32-bit contents of the top of the stack pointed to by $rB into $rA
 * and update the stack pointer. Stacks grown down. */
int mox_pop(char a, char b);

/** Push the contents of $rB onto a stack pointed to by $rA and update the stack
 * pointer. Stacks grown down. */
int mox_push(char a, char b);

/** Return from subroutine. */
int mox_ret();

/** sex.b - Sign-extend byte. Sign-extend the lower 8-bits of $rB into a $rA
 * as a 32-bit value. */
int mox_sexb(char a, char b);

/** sex.s - Sign-extend short. Sign-extend the lower 16-bits of $rB into a $rA
 * as a 32-bit value. */
int mox_sexs(char a, char b);

/** Set special register. Move the contents of $rA into special register S. */
int mox_ssr(char a, char s);

/** st.b - Store byte. Stores the 8-bit contents of $rB into memory at the
 * address pointed to by $rA. */
int mox_stb(char a, char b);

/** st.l - Store long. Stores the 32-bit contents of $rB into memory at the
 * address pointed to by $rA. */
int mox_stl(char a, char b);

/** st.s - Store short. Stores the 16-bit contents of $rB into memory at the
 * address pointed to by $rA. */
int mox_sts(char a, char b);

/** sta.b - Store absolute, byte. Stores the lower 8-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
int mox_stab(char a, long i);

/** sta.l - Store absolute, long. Stores the full 32-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
int mox_stal(char a, long i);

/** sta.s - Store absolute, short. Stores the lower 16-bit contents of $rA into
 * memory at the 32-bit address following the 16-bit opcode. */
int mox_stas(char a, long i);

/** sto.b - Store offset, byte. Stores the 8-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
int mox_stob(char a, char b, short i);

/** sto.l - Store offset, long. Stores the 32-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
int mox_stol(char a, char b, short i);

/** sto.s - Store offset, short. Stores the 16-bit contents of $rB into memory at
 * the address roduced by adding the 16-bit value following the 16-bit opcode to
 * $rA. */
int mox_stos(char a, char b, short i);

/** Subtract, long. Subtracts the contents of registers $rA and $rB and stores
 * the result in $rA. */
int mox_sub(char a, char b);

/** Software interrupt. Trigger a software interrupt, where the interrupt type is
 * encoded in the 32-bits following the 16-bit opcode. */
int mox_swi(long i);

/** Divide unsigned, long. Divides the unsigned contents of registers $rA and $rB
 * and stores the result in $rA. */
int mox_udiv(char a, char b);

/** Modulus unsigned, long. Compute the modulus of the unsigned contents of
 * registers $rA and $rB and stores the result in $rA. */
int mox_umod(char a, char b);

/** umul.x - Unsigned multiply, upper word. Multiplies the contents of registers
 * $rA and $rB and stores the upper 32-bits of an unsigned 64-bit result in
 * $rA. */
int mox_umulx(char a, char b);

/** Logical exclusive or. Performs a logical exclusive or operation on the
 * contents of registers $rA and $rB and stores the result in $rA. */
int mox_xor(char a, char b);

/** zex.b - Zero-extend byte. Zero-extend the lower 8-bits of $rB into a $rA
 * as a 32-bit value. */
int mox_zexb(char a, char b);

/** zex.s - Zero-extend short. Zero-extend the lower 16-bits of $rB into a $rA
 * as a 32-bit value. */
int mox_zexs(char a, char b);