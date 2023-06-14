            TTL CMPE 250 Exercise 4 
			SUBT Iteration and Subroutines
;****************************************************************
;This program demonstrates iteration and subroutines through
;the use of a division subroutine.
;---------------------------------------------------------------
;Name:  <Atticus Russell>
;Date:  <2/18/2021>
;Class:  CMPE-250
;Section:  <Section 01L1, Thursday, 2:00-3:55>
;---------------------------------------------------------------
;Keil Simulator Template for KL46
;R. W. Melton
;January 5, 2018
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;EQUates
;Standard data masks
BYTE_MASK         EQU  0xFF
NIBBLE_MASK       EQU  0x0F
;Standard data sizes (in bits)
BYTE_BITS         EQU  8
NIBBLE_BITS       EQU  4
;Architecture data sizes (in bytes)
WORD_SIZE         EQU  4  ;Cortex-M0+
HALFWORD_SIZE     EQU  2  ;Cortex-M0+
;Architecture data masks
HALFWORD_MASK     EQU  0xFFFF
;Return                 
RET_ADDR_T_MASK   EQU  1  ;Bit 0 of ret. addr. must be
                          ;set for BX, BLX, or POP
                          ;mask in thumb mode
;MAX_DATA
MAX_DATA		  EQU  25 
;---------------------------------------------------------------
;Vectors
VECTOR_TABLE_SIZE EQU 0x000000C0  ;KL46
VECTOR_SIZE       EQU 4           ;Bytes per vector
;---------------------------------------------------------------
;CPU CONTROL:  Control register
;31-2:(reserved)
;   1:SPSEL=current stack pointer select
;           0=MSP (main stack pointer) (reset value)
;           1=PSP (process stack pointer)
;   0:nPRIV=not privileged
;        0=privileged (Freescale/NXP "supervisor") (reset value)
;        1=not privileged (Freescale/NXP "user")
CONTROL_SPSEL_MASK   EQU  2
CONTROL_SPSEL_SHIFT  EQU  1
CONTROL_nPRIV_MASK   EQU  1
CONTROL_nPRIV_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PRIMASK:  Interrupt mask register
;31-1:(reserved)
;   0:PM=prioritizable interrupt mask:
;        0=all interrupts unmasked (reset value)
;          (value after CPSIE I instruction)
;        1=prioritizable interrrupts masked
;          (value after CPSID I instruction)
PRIMASK_PM_MASK   EQU  1
PRIMASK_PM_SHIFT  EQU  0
;---------------------------------------------------------------
;CPU PSR:  Program status register
;Combined APSR, EPSR, and IPSR
;----------------------------------------------------------
;CPU APSR:  Application Program Status Register
;31  :N=negative flag
;30  :Z=zero flag
;29  :C=carry flag
;28  :V=overflow flag
;27-0:(reserved)
APSR_MASK     EQU  0xF0000000
APSR_SHIFT    EQU  28
APSR_N_MASK   EQU  0x80000000
APSR_N_SHIFT  EQU  31
APSR_Z_MASK   EQU  0x40000000
APSR_Z_SHIFT  EQU  30
APSR_C_MASK   EQU  0x20000000
APSR_C_SHIFT  EQU  29
APSR_V_MASK   EQU  0x10000000
APSR_V_SHIFT  EQU  28
;----------------------------------------------------------
;CPU EPSR
;31-25:(reserved)
;   24:T=Thumb state bit
;23- 0:(reserved)
EPSR_MASK     EQU  0x01000000
EPSR_SHIFT    EQU  24
EPSR_T_MASK   EQU  0x01000000
EPSR_T_SHIFT  EQU  24
;----------------------------------------------------------
;CPU IPSR
;31-6:(reserved)
; 5-0:Exception number=number of current exception
;      0=thread mode
;      1:(reserved)
;      2=NMI
;      3=hard fault
;      4-10:(reserved)
;     11=SVCall
;     12-13:(reserved)
;     14=PendSV
;     15=SysTick
;     16=IRQ0
;     16-47:IRQ(Exception number - 16)
;     47=IRQ31
;     48-63:(reserved)
IPSR_MASK             EQU  0x0000003F
IPSR_SHIFT            EQU  0
IPSR_EXCEPTION_MASK   EQU  0x0000003F
IPSR_EXCEPTION_SHIFT  EQU  0
;----------------------------------------------------------
PSR_N_MASK           EQU  APSR_N_MASK
PSR_N_SHIFT          EQU  APSR_N_SHIFT
PSR_Z_MASK           EQU  APSR_Z_MASK
PSR_Z_SHIFT          EQU  APSR_Z_SHIFT
PSR_C_MASK           EQU  APSR_C_MASK
PSR_C_SHIFT          EQU  APSR_C_SHIFT
PSR_V_MASK           EQU  APSR_V_MASK
PSR_V_SHIFT          EQU  APSR_V_SHIFT
PSR_T_MASK           EQU  EPSR_T_MASK
PSR_T_SHIFT          EQU  EPSR_T_SHIFT
PSR_EXCEPTION_MASK   EQU  IPSR_EXCEPTION_MASK
PSR_EXCEPTION_SHIFT  EQU  IPSR_EXCEPTION_SHIFT
;----------------------------------------------------------
;Stack
SSTACK_SIZE EQU  0x00000100
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
			IMPORT	InitData
			IMPORT	LoadData
			IMPORT	TestData
Reset_Handler  PROC {}
main
;---------------------------------------------------------------
;Initialize registers R0-R12
            BL      RegInit
;>>>>> begin main program code <<<<<
			LDR		R2,=P		;load ptr of P into R2
			LDR		R3,=Q		;load ptr of Q into R3
			MOVS	R4,#0xFF	;load negative into R4
			SXTB	R4,R4		;sign extend R4 to 0xFFFFFFFF
			BL		InitData
dataLoop
			BL		LoadData	;will load p and q
			BCS		endMe		;if C flag is set no more data and program quit
			;p=r1, q=r0, p/q
			LDR		R1,[R2,#0]	;load P into R1 to be divided
			LDR		R0,[R3,#0]	;load Q into R0 to be divided by
			BL		DIVU		;call division subroutine
			BCS		badResult	;if C set by DIVU is bad result, branch
			;if c not set it is a valid result so store it, R0 quotient, rem R1
			STR		R0,[R2,#0]	;store quotient in R0 in mem address P (ptr R2)
			STR		R1,[R3,#0]	;store remainder in R1 in mem address Q (ptr R3)
			B		storeVars
badResult	
			STR		R4,[R2,#0]	;store 0xFFFFFFFF in R4 in mem address P in R2
			STR		R4,[R3,#0]	;store 0xFFFFFFFF in R4 in mem address Q in R3
storeVars
			BL		TestData	;checks div results stores in Results, inc R6
			B		dataLoop
endMe
			NOP					;just a place for a breakpoint    
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;---------------------------------------------------------------
RegInit     PROC  {}
;****************************************************************
;Initializes register n to value 0xnnnnnnnn, for n in 
;{0x0-0xC,0xE}
;****************************************************************
;Put return on stack
            PUSH    {LR}
;Initialize registers
            LDR     R1,=0x11111111
            ADDS    R2,R1,R1
            ADDS    R3,R2,R1
            ADDS    R4,R3,R1
            ADDS    R5,R4,R1
            ADDS    R6,R5,R1
            ADDS    R7,R6,R1
            ADDS    R0,R7,R1
            MOV     R8,R0
            ADDS    R0,R0,R1
            MOV     R9,R0
            ADDS    R0,R0,R1
            MOV     R10,R0
            ADDS    R0,R0,R1
            MOV     R11,R0
            ADDS    R0,R0,R1
            MOV     R12,R0
            ADDS    R0,R0,R1
            ADDS    R0,R0,R1
            MOV     R14,R0
            MOVS    R0,#0
            POP     {PC}
            ENDP
;---------------------------------------------------------------
;>>>>> begin subroutine code <<<<<
DIVU		PROC	{R2-R14}
;****************************************************************
;Input: R0; divisor (denominator)
;Input: R1; dividend (numerator)
;Output: R0; quotient
;Output: R1; remainder
;C: APSR flag: 0 for valid result; 1 for invalid result
;****************************************************************
			CMP		R0,#0	;check if divisor (denominator) is already zero
			BEQ		div0	;if so branch to div0 to set c flag
			PUSH	{R2,R3}	;store R2 and R3 on the stack
			CMP		R1,#0	;check if dividend(numerator) is zero
			BEQ		zerodiv	;if so set outputs to 0 and clear C flag
			
;actual division part
			MOVS	R2,#0	;quotient =0 (R2 will be quotient while computing)
divuloop	
			CMP		R1,R0	;while dividend>=divisor
			BLO		divuFinish
			SUBS	R1,R1,R0;dividend=dividend-divisor
			ADDS	R2,R2,#1;quotient=quotient+1
			B		divuloop
divuFinish
			MOVS	R0,R2	;R0 =quotient = p
							;R1 = remainder = dividend = q
			B		clearC	;branch and clearC then endit
zerodiv		;if dividing zero by anything the result is 0 rem 0
			MOVS	R0,#0
			MOVS	R1,#0
clearC
			MRS		R2,APSR
			MOVS	R3,#0x20
			LSLS	R3,R3,#24
			BICS	R2,R2,R3
			MSR		APSR,R2
			POP		{R2,R3}	;restore R2 and R3 from stack
			B		endit
div0		;time to set the C flag and leave all else unchanged
			PUSH	{R0,R1}	;push input parameters to stack
			MRS		R0,APSR
			MOVS	R1,#0x20
			LSLS	R1,R1,#24
			ORRS	R0,R0,R1
			MSR		APSR,R0
			POP		{R0,R1} ;restore input parameters from stack
endit
			BX		LR
			ENDP
;>>>>>   end subroutine code <<<<<
            ALIGN
;****************************************************************
;Vector Table Mapped to Address 0 at Reset
;Linker requires __Vectors to be exported
            AREA    RESET, DATA, READONLY
            EXPORT  __Vectors
            EXPORT  __Vectors_End
            EXPORT  __Vectors_Size
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;reset vector
            SPACE  (VECTOR_TABLE_SIZE - (2 * VECTOR_SIZE))
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
;>>>>>   end constants here <<<<<
;****************************************************************
            AREA    |.ARM.__at_0x1FFFE000|,DATA,READWRITE,ALIGN=3
            EXPORT  __initial_sp
;Allocate system stack
            IF      :LNOT::DEF:SSTACK_SIZE
SSTACK_SIZE EQU     0x00000100
            ENDIF
Stack_Mem   SPACE   SSTACK_SIZE
__initial_sp
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
			EXPORT	P
			EXPORT	Q
			EXPORT	Results
;>>>>> begin variables here <<<<<
P			SPACE	4
Q			SPACE	4
Results		SPACE	(2*(MAX_DATA*(4)))
;>>>>>   end variables here <<<<<
            END
