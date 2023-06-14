            TTL CMPE 250 Exercise 3 
			SUBT Memory, Conditional Branching, and Debugging Tools
;****************************************************************
;This program demonstrates memory use and conditional branching
;by calculating the result of three arithmetic expressions using
;two sets of input values.
;---------------------------------------------------------------
;Name:  <Atticus Russell>
;Date:  <2/11/2021>
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
;Amounts to shift by for multiplication
MUL2			  EQU  1
MUL4 			  EQU  2
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
Reset_Handler  PROC {}
main
;---------------------------------------------------------------
;Initialize registers R0-R12
            BL      RegInit
;>>>>> begin main program code <<<<<
;Input Set 1: P = 9; Q =4
;Input Set 2: P = 13; Q = -14
;F = 3P + 2Q - 75
;G = 2P - 4Q + 63
;Result = F+ G
			 
;INPUT SET 1	 
			;store that P is 9 in variable P memory location
			LDR		R1,=P
			MOVS	R0,#9
			STR		R0,[R1,#0]
			
			;store that Q is 4 in variable Q memory location
			LDR		R2,=Q
			MOVS	R0,#4
			STR		R0,[R2,#0]
			
;INPUT SET 2			
			;store that P is 9 in variable P memory location
;			LDR		R1,=P
;			MOVS	R0,#13
;			STR		R0,[R1,#0]
;			
;			
;			;store that Q is -14 in variable Q memory location
;			LDR		R2,=Q
;			MOVS	R0,#14
;			RSBS	R0,R0,#0
;			STR		R0,[R2,#0]
			
;Used in both input sets for F
			;store -128 in R4
			MOVS	R4,#128
			RSBS	R4,R4,#0

;calculations for F
;Px3
			LDR		R0,[R1,#0]  ;load P into R0
			MOV		R3,R0		;copy R0 to R3
			LSLS	R0,R0,#MUL2	;multiply P by 2
			ADDS	R0,R0,R3	;Add copied R0 back to complete Px3 process
;time to check for overflow
			CMP		R0,#127      
			BGT		overflowF
			CMP		R0,R4
			BLT		overflowF
;Qx2
			LDR		R3,[R2,#0]  ;load Q into R3
			LSLS	R3,R3,#MUL2	;multiply Q by 2 
;time to check for overflow
			CMP		R3,#127
			BGT		overflowF
			CMP		R3,R4
			BLT		overflowF
;Px3 + Qx3
			ADDS	R0,R0,R3
;time to check for overflow
			CMP		R0,#127
			BGT		overflowF
			CMP		R0,R4
			BLT		overflowF
;Px3 + Qx3 - 75
			LDR		R3,const_F
			SUBS	R0,R0,R3
;time to check for overflow
			CMP		R0,#127
			BGT		overflowF
			CMP		R0,R4
			BLT		overflowF
;store the calculated value for F
			LDR		R3,=F		;load f ptr
			STR		R0,[R3,#0]
			B		noverflowF	
overflowF
			LDR		R3,=F		;load f ptr
			MOVS	R0,#0		
			STR		R0,[R3,#0]	;store that f is 0
noverflowF
;F calculated successfully with first and second set of inputs - nOverflow
;time to calculate G using flags, and then Result

;Shift Calculations for G (not using V)
;Px2
			LDR		R0,[R1,#0]  ;load P into R0
			LSLS	R0,R0,#MUL2	;multiply P by 2
;time to check for overflow
			CMP		R0,#127      
			BGT		overflowG
			CMP		R0,R4
			BLT		overflowG
;Put result of Px2 away for safekeeping to free up R0
			MOVS	R5,R0
;Qx4
			LDR		R0,[R2,#0]	;load Q into R0
			LSLS	R0,R0,#MUL4	;multiply Q by 4
;time to check for overflow
			CMP		R0,#127      
			BGT		overflowG
			CMP		R0,R4
			BLT		overflowG
;Shift over the results of the last two operations to the MSByte
			MOVS	R6,#8		;load amount to shift into R6		
			MOVS	R7,#0xFF	;load partial mask into R7
			RORS	R7,R6		;shift to MSByte to create ANDS mask
			RORS	R0,R0,R6	;rotate R0, to shift Qx4 to MSByte
			ANDS	R0,R0,R7	;ANDS with mask to clear non MSBytes	
			RORS	R5,R5,R6	;rotate R5, to shift Px2 to MSByte
			ANDS	R0,R0,R7	;ANDS with mask to clear non MSBytes
;Calculations for G in MSByte using V to check for overflow
			SUBS	R0,R5,R0	;Subtract 4Q from 2P and store in R0
			BVS		overflowG	;check for overflow and proceed accordingly
			LDR		R4,const_G	;load 63 into R1
			RORS	R4,R4,R6	;rotate R4, to shift 63 to MSByte
			ADDS	R0,R0,R4 	;add shifted 63 to the result and store in R0
			BVS		overflowG	;check for overflow and proceed accordingly
;store G and proceed
			LDR		R1,=G		;load G ptr to R1
			STR		R0,[R1,#0]	;store result in mem addr for G
			B		noverflowG
overflowG
			LDR		R1,=G		;load G ptr to R1
			MOVS	R0,#0		
			STR		R0,[R1,#0]	;store that G is 0
noverflowG
;calculate Result
			LDR		R4,[R3,#0]	;get the result of F and put it in R4
			RORS	R4,R4,R6	;rotate R5, to shift F to MSByte
			ANDS	R4,R4,R7	;ANDS with mask to clear non MSBytes
			LDR		R0,[R1,#0]	;get the result of G and put it in R0
			ADDS	R0,R0,R4	;add F and G and store in R0
			BVS		overflowResult
			B		noverflowResult
overflowResult
			MOVS	R0,#0		
noverflowResult
			LDR		R1,=Result	;put Result ptr in R1
			MOVS	R6,#24		;load amount to rotate into R6
			RORS	R0,R0,R6	;rotate R0 back so LSB is in correct place
			SXTB	R0,R0		;Sign extend
			STR		R0,[R1,#0]	;store R0 in mem location for Result
;>>>>>   end main program code <<<<<
;Stay here
			NOP
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
const_F 	DCD		75
const_G		DCD		63	
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
;>>>>> begin variables here <<<<<
Result 		SPACE 	4
F			SPACE 	4
G			SPACE 	4
P			SPACE 	4
Q			SPACE 	4
;>>>>>   end variables here <<<<<
            END
