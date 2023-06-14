            TTL CMPE 250 Exercise 8 
			SUBT Multiprecision Arithmetic
;****************************************************************
;Implements and tests subroutines to perform multi-word
;number I/O and addition on the KL05Z board.
;---------------------------------------------------------------
;Name:  <Atticus Russell>
;Date:  <3/18/2021>
;Class:  CMPE-250
;Section:  <Section 01L1, Thursday, 2:00-3:55>
;---------------------------------------------------------------
;Keil Template for KL05
;R. W. Melton
;September 13, 2020
;****************************************************************
;Assembler directives
            THUMB
            OPT    64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s     ;Included by start.s
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
;MAX_STRING	EQU		79
CR   		 EQU  0x0D			;moves cursor beginning line
LF   		 EQU  0x0A			;adds new line
NULL 		 EQU  0x00
;---------------------------------------------------------------
;PORTx_PCRn (Port x pin control register n [for pin n])
;___->10-08:Pin mux control (select 0 to 8)
;Use provided PORT_PCR_MUX_SELECT_2_MASK
;---------------------------------------------------------------
;Port B
PORT_PCR_SET_PTB2_UART0_RX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
PORT_PCR_SET_PTB1_UART0_TX  EQU  (PORT_PCR_ISF_MASK :OR: \
                                  PORT_PCR_MUX_SELECT_2_MASK)
;---------------------------------------------------------------
;SIM_SCGC4
;1->10:UART0 clock gate control (enabled)
;Use provided SIM_SCGC4_UART0_MASK
;---------------------------------------------------------------
;SIM_SCGC5
;1->10:Port B clock gate control (enabled)
;Use provided SIM_SCGC5_PORTB_MASK
;---------------------------------------------------------------
;SIM_SOPT2
;01=27-26:UART0SRC=UART0 clock source select (MCGFLLCLK)
;---------------------------------------------------------------
SIM_SOPT2_UART0SRC_MCGFLLCLK  EQU  \
                                 (1 << SIM_SOPT2_UART0SRC_SHIFT)
;---------------------------------------------------------------
;SIM_SOPT5
; 0->   16:UART0 open drain enable (disabled)
; 0->   02:UART0 receive data select (UART0_RX)
;00->01-00:UART0 transmit data select source (UART0_TX)
SIM_SOPT5_UART0_EXTERN_MASK_CLEAR  EQU  \
                               (SIM_SOPT5_UART0ODE_MASK :OR: \
                                SIM_SOPT5_UART0RXSRC_MASK :OR: \
                                SIM_SOPT5_UART0TXSRC_MASK)
;---------------------------------------------------------------
;UART0_BDH
;    0->  7:LIN break detect IE (disabled)
;    0->  6:RxD input active edge IE (disabled)
;    0->  5:Stop bit number select (1)
;00001->4-0:SBR[12:0] (UART0CLK / [9600 * (OSR + 1)]) 
;UART0CLK is MCGFLLCLK
;MCGPLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR ~=~ 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGPLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR ~=~ 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDL_9600  EQU  0x38
;---------------------------------------------------------------
;UART0_C1
;0-->7:LOOPS=loops select (normal)
;0-->6:DOZEEN=doze enable (disabled)
;0-->5:RSRC=receiver source select (internal--no effect LOOPS=0)
;0-->4:M=9- or 8-bit mode select 
;        (1 start, 8 data [lsb first], 1 stop)
;0-->3:WAKE=receiver wakeup method select (idle)
;0-->2:IDLE=idle line type select (idle begins after start bit)
;0-->1:PE=parity enable (disabled)
;0-->0:PT=parity type (even parity--no effect PE=0)
UART0_C1_8N1  EQU  0x00
;---------------------------------------------------------------
;UART0_C2
;0-->7:TIE=transmit IE for TDRE (disabled)
;0-->6:TCIE=transmission complete IE for TC (disabled)
;0-->5:RIE=receiver IE for RDRF (disabled)
;0-->4:ILIE=idle line IE for IDLE (disabled)
;1-->3:TE=transmitter enable (enabled)
;1-->2:RE=receiver enable (enabled)
;0-->1:RWU=receiver wakeup control (normal)
;0-->0:SBK=send break (disabled, normal)
UART0_C2_T_R  EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
;---------------------------------------------------------------
;UART0_C3
;0-->7:R8T9=9th data bit for receiver (not used M=0)
;           10th data bit for transmitter (not used M10=0)
;0-->6:R9T8=9th data bit for transmitter (not used M=0)
;           10th data bit for receiver (not used M10=0)
;0-->5:TXDIR=UART_TX pin direction in single-wire mode
;            (no effect LOOPS=0)
;0-->4:TXINV=transmit data inversion (not inverted)
;0-->3:ORIE=overrun IE for OR (disabled)
;0-->2:NEIE=noise error IE for NF (disabled)
;0-->1:FEIE=framing error IE for FE (disabled)
;0-->0:PEIE=parity error IE for PF (disabled)
UART0_C3_NO_TXINV  EQU  0x00
;---------------------------------------------------------------
;UART0_C4
;    0-->  7:MAEN1=match address mode enable 1 (disabled)
;    0-->  6:MAEN2=match address mode enable 2 (disabled)
;    0-->  5:M10=10-bit mode select (not selected)
;01111-->4-0:OSR=over sampling ratio (16)
;               = 1 + OSR for 3 <= OSR <= 31
;               = 16 for 0 <= OSR <= 2 (invalid values)
UART0_C4_OSR_16           EQU  0x0F
UART0_C4_NO_MATCH_OSR_16  EQU  UART0_C4_OSR_16
;---------------------------------------------------------------
;UART0_C5
;  0-->  7:TDMAE=transmitter DMA enable (disabled)
;  0-->  6:Reserved; read-only; always 0
;  0-->  5:RDMAE=receiver full DMA enable (disabled)
;000-->4-2:Reserved; read-only; always 0
;  0-->  1:BOTHEDGE=both edge sampling (rising edge only)
;  0-->  0:RESYNCDIS=resynchronization disable (enabled)
UART0_C5_NO_DMA_SSR_SYNC  EQU  0x00
;---------------------------------------------------------------
;UART0_S1
;0-->7:TDRE=transmit data register empty flag; read-only
;0-->6:TC=transmission complete flag; read-only
;0-->5:RDRF=receive data register full flag; read-only
;1-->4:IDLE=idle line flag; write 1 to clear (clear)
;1-->3:OR=receiver overrun flag; write 1 to clear (clear)
;1-->2:NF=noise flag; write 1 to clear (clear)
;1-->1:FE=framing error flag; write 1 to clear (clear)
;1-->0:PF=parity error flag; write 1 to clear (clear)
UART0_S1_CLEAR_FLAGS  EQU  (UART0_S1_IDLE_MASK :OR: \
                            UART0_S1_OR_MASK :OR: \
                            UART0_S1_NF_MASK :OR: \
                            UART0_S1_FE_MASK :OR: \
                            UART0_S1_PF_MASK)
;---------------------------------------------------------------
;UART0_S2
;1-->7:LBKDIF=LIN break detect interrupt flag (clear)
;             write 1 to clear
;1-->6:RXEDGIF=RxD pin active edge interrupt flag (clear)
;              write 1 to clear
;0-->5:(reserved); read-only; always 0
;0-->4:RXINV=receive data inversion (disabled)
;0-->3:RWUID=receive wake-up idle detect
;0-->2:BRK13=break character generation length (10)
;0-->1:LBKDE=LIN break detect enable (disabled)
;0-->0:RAF=receiver active flag; read-only
UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS  EQU  \
        (UART0_S2_LBKDIF_MASK :OR: UART0_S2_RXEDGIF_MASK)
;---------------------------------------------------------------
;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
;			IMPORT	GetHexIntMulti	;from ex8_lib
;			IMPORT	PutHexIntMulti	;from ex8_lib
			IMPORT  Startup
			EXPORT	GetStringSB
			EXPORT	PutNumHex
			
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL05 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<<
			BL		Init_UART0_Polling
			;initializing op string better
			LDR		R0,=lenOpStr;load address to store the length of op str in
			MOVS	R1,#0
			STRB	R1,[R0,#0]	;store the length of the op str as 0
			B 		loopHereInput
loopHereInput
			LDR		R0,=FIRST_PROMPT
getFirstHex	
			BL		CRLF
			MOVS	R1,#33				;num bytes in the first prompt
			BL		PutStringSB			;print first prompt
			LDR		R0,=firstNum		;load mem addr first hex 
			MOVS	R1,#3				;num of words in 96 bits
			BL		GetHexIntMulti
			BCC 	goSecondHex			;if first input is valid hex proceed
			LDR		R0,=INVALID_MESSAGE	;else make them repeat
			B 		getFirstHex			;branch to try again
goSecondHex			
			LDR		R0,=SECOND_PROMPT
secondHex
			BL		CRLF
			MOVS	R1,#33				;num bytes in the second prompt	
			BL		PutStringSB			;print second prompt
			LDR		R0,=secondNum		;load mem addr second num 
			MOVS	R1,#3				;num of words in 96 bits
			BL		GetHexIntMulti
			BCC 	validHexTwo			;if first input is valid hex proceed
			LDR		R0,=INVALID_MESSAGE	;else make them repeat
			B 		secondHex			;branch to try again
validHexTwo
			LDR		R0,=SUM_MESSAGE		
			MOVS	R1,#33				;num bytes in the sum message
			BL		CRLF
			BL		PutStringSB			;print sum message
			LDR		R1,=firstNum		;load mem addr first hex 
			LDR		R2,=secondNum		;load mem addr second num 
			MOVS	R3,#3				;number of words in addition source
			BL		AddIntMultiU		;branch to the hex addition
			
			BCC		goodAddition		;if c is not set it's valid
			LDR		R0,=OVERFLOW_MESSAGE
			MOVS	R1,#8				;num bytes in the overflow messsage
			BL		PutStringSB			;print overflow prompt
			B 		loopHereInput		;go to top and start again
goodAddition
			LDR		R0,=sumMem			;load mem addr sum
			MOVS	R1,#3				;number words in sum mem
			BL		PutHexIntMulti
			B		loopHereInput
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
CRLF		PROC	{R0-R14}
;****************************************************************
;Description:
;	This subroutine outputs a carriage return and a line-feed to
;	the terminal screen.
;Input Parameter:
;	none
;Output Parameter:
;	none
;modified registers:
;	none
;subroutines utilized:
;	PutChar
;****************************************************************
			PUSH	{LR,R0}
			MOVS	R0,#CR				;load carriage return
			BL		PutChar				;print carriage return
			MOVS	R0,#LF				;load line feed
			BL		PutChar				;print line feed
			POP		{PC,R0}
			BX		LR
			ENDP
				
JustClearC	PROC	{R0-R14}
;****************************************************************
;Description:
;	A subroutine with the sole purpose of setting the PSR 
;	bit "C" to 0
;Input Parameter:
;	none
;Output Parameter:
;	C: APSR bit set to 0, other PSR bits unchanged
;modified registers:
;	no registers other than PSR bit C have changed values
;	after return
;****************************************************************
			PUSH	{LR,R0-R1}
			MRS		R0,APSR
			MOVS	R1,#0x20
			LSLS	R1,R1,#24
			BICS	R0,R0,R1
			MSR		APSR,R0
			POP		{PC,R0-R1}
			BX		LR
			ENDP
				
JustSetC	PROC	{R0-R14}
;****************************************************************
;Description:
;	A subroutine with the sole purpose of setting the APSR 
;	bit "C" to 1
;Input Parameter:
;	none
;Output Parameter:
;	C: APSR bit set to 1, other PSR bits unchanged
;modified registers:
;	no registers other than PSR bit C have changed values
;	after return
;****************************************************************
			PUSH	{LR,R0-R1}
			MRS		R0,APSR
			MOVS	R1,#0x20
			LSLS	R1,R1,#24
			ORRS	R0,R0,R1
			MSR		APSR,R0
			POP		{PC,R0-R1}
			BX		LR
			ENDP
	
AddIntMultiU		PROC	{R0-R14}
;****************************************************************
;Description:
;	Adds the n-word unsigned number in memory starting at the address 
;	in R2 to the n-word unsigned number in memory starting at the address in 
;	R1, and stores the result to memory starting at the address in R0, where 
;	the value in R3 is n. The subroutine uses ADCS to add word by word, starting
;	from the least significant words of the augend and addend and working to 
;	the most significant words. If the result is a valid n-word unsigned 
;	number, it returns with the APSR C bit clear as the return code for 
;	success; otherwise it returns with the APSR C bit set as the return code 
;	for unsigned n-word overflow.
;Input Parameter: 
;~~~R0: n-word address to store sum in
;	R1: n-word augend for addition (unsigned word address)
;	R2: n-word addend for addition (unsigned word address)
;	R3: n, the numbr of words in addition source and result operands
;		(unsigned word value)
;Output Parameter: 
;	R0: n-word sum from addition (unsigned word address)
;	C: addition operation status: 0 success; 1 overflow (APSR bit flag)
;modified registers:
;	no registers other than PSR and any output parameters) have changed values
;	after return
;subroutines used:
;	JustClearC
;	JustSetC
;****************************************************************
			PUSH	{LR,R1-R7}		;store operating registers on stack
			LDR		R0,=sumMem	
			MOVS	R7,#0		;clear c for LSW of addition
loopHereMain
			CMP		R7,#0
			BNE		setC	;interprets r7 and modifies C of PSR accordingly
			;if didnt branch - clear c
			BL		JustClearC
			B		branchHereAdd
setC		
			;set C 
			BL		JustSetC
			B 		branchHereAdd
			
branchHereAdd	
			LDR		R4,[R1,#0]	;load augend value
			LDR		R5,[R2,#0]	;load addend value
			MOV		R6,R5		;move to R6 because everuthing is arbitrary
			ADCS	R6,R6,R4	;add with carry set, store res in R6
			;preserve C by using R7 to indicate its state
			BCS		indCSet
			;if doesn't branch there then c cleared
			MOVS	R7,#0	
			B		skipSet
indCSet
			MOVS	R7,#1
skipSet
			STR		R6,[R0,#0]	;store word of result in addr R0
			ADDS	R0,R0,#4	;increment address of result storage
			ADDS	R1,R1,#4	;increment address of augend to next word
			ADDS	R2,R2,#4	;increment address of addend to next word
			SUBS	R3,R3,#1	;decrement R3 (num words in source and res)
			CMP		R3,#0
			BGT		loopHereMain
			;if didn't loop then we're done with addition and need to evaluate
			;whether its a valid result and indicate with C flag
				;does it fit within available bits?
					;taken from a slide but aren't these the same thing?
				;if c set after final add its then invalid
				
			
			CMP		R7,#0
			BEQ		clearC2		;interprets r7 and modifies C of PSR accordingly 
			;if didnt branch - set c
setC2		
			BL		JustSetC
			B 		finished
clearC2			
			BL		JustClearC	
finished
			LDR		R0,=sumMem
			POP		{PC,R1-R7}
			BX		LR
			ENDP	

	
	
GetHexIntMulti	PROC	{R1-R14}
;****************************************************************
;Description:
;	Gets an n-word unsigned number from the user typed in text 
;	hexadecimal representation, and stores it in binary in memory starting 
;	at the address in R0, where the value in R1 is n. the subroutine reads 
;	characters typed by the user until the enter key is pressed by calling 
;	the subroutine GetStringSB. It then converts the ASCII hexadecimal 
;	representation input by the user to binary, and it stores the binary 
;	value to memory at the address specified in R0. If the result is a valid 
;	n-word unsigned number, it returns with the APSR C bit clear; 
;	otherwise, it returns with the APSR C bit set. 
;Input Parameter: 
;	R0: mem address to store at (unsigned word address)
;	R1: n, the number of words in the input number (unsigned word value)
;Output Parameter: 
;	R0: n-word number input by user (unsigned word address)
;	C: input number status: 0 valid; 1 invalid (APSR bit flag)
;modified registers:
;	no registers other than PSR (and any output parameters) have changed values
;	after return
;subroutines used:
;	GetStringSB
;	JustSetC
;	JustClearC
;****************************************************************
			PUSH	{LR,R0-R7}
			MOVS	R5,R1		;backup number of words, n, in R5
			MOVS	R2,#3		;we want number of bytes in the n words 
			LSLS	R1,R1,R2	;Store 8*n in R1 (using shift)
			MOVS	R2,R1		;copy 8n to R2
			ADDS	R1,R1,#1	;change value in R1 to 8n+1 to use as 
								;GetStringSB buffer capacity
			PUSH	{R0}		;back up original mem addr
			LDR		R0,=convertMem
			BL		GetStringSB
			;now convert ASCII to hex and store at addr
			MOVS	R1,#0		;initialize counter i as 0
loopGHIM
			LDRB	R3,[R0,R1]	;get R1th byte
		
			MOVS	R4,#'f'
			CMP		R3,R4
			BGT		invalidInput;if greater than ascii lowercase f its not valid
			MOVS	R4,#'a'
			CMP		R3,R4
			BGE		lowercaseASCII;if  'a'<=R3<='f'	then its in that range
			MOVS	R4,#'F'
			CMP		R3,R4
			BGT		invalidInput;in no-mans land of ascii
			MOVS	R4,#'A'
			CMP		R3,R4
			BGE		uppercaseASCII;if  'A'<=R3<='F'	then its in that range
			MOVS	R4,#'9'
			CMP		R3,R4
			BGT		invalidInput;in no-mans land of ascii
			MOVS	R4,#'0'
			CMP		R3,R4
			BGE		numberASCII;if  '0'<=R3<='9'	then its in that range
			B 		invalidInput;else its probably not valid
invalidInput
			BL		JustSetC	;set C to indicate invalid and leave
			B   	endGHIM
lowercaseASCII
			SUBS	R3,R3,#0x20	;convert to upper case ASCII 
			B 		uppercaseASCII
numberASCII
			SUBS	R3,R3,#0x30	;convert to regular hex number
			B 		storeChar
uppercaseASCII	
			SUBS	R3,R3,#0x31	;convert to decimalish double-digit num 
			;convert to single digit Hex (0xA through 0xF)
			MOVS	R7,R3
			MOVS	R3,#0x0A
			MOVS	R6,#2_11110000
			BICS	R7,R7,R6
			ADDS	R3,R3,R7
			B 		storeChar
storeChar
			STRB	R3,[R0,R1]	;store char at relevant byte
			ADDS	R1,R1,#1	;increment i

			CMP		R1,R2		;compare loops with num chars to process
			BGE		packIntoBytes;end if equals the number of chars			
			B		loopGHIM	;loop again
packIntoBytes
			
			;MOVS	R7,R0
			;make r7 at the end of the memory buffer
			;ADDS	R7,R0,R2
			POP		{R7}		;original mem addr into r7
			MOVS	R3,#0
			MOVS	R6,R2		
			
			LSRS	R6,R6,#1;need to divide by 2
			SUBS	R6,R6,#1;subtract 1 because zero based
packLoop
			MOVS	R4,R3
			ADDS	R4,R4,#1
			LDRB	R1,[R0,R4]
			LDRB	R5,[R0,R3]
			;now move first nibble in R5 into second nibble
			MOVS	R4,#28
			RORS	R5,R5,R4
			ADDS	R1,R1,R5
			STRB	R1,[R7,R6]
			
			ADDS	R3,R3,#2
			SUBS	R6,R6,#1
			CMP		R3,R2
			BEQ		windDownGHIM
			B		packLoop
windDownGHIM		;swap to little-endian
			;LDRB	R1,[R0,#0]
			;REV		R1,R1 
			MOVS	R0,R7		;swap to original mem addr
			
			BL		JustClearC
endGHIM			
			POP		{PC,R0-R7}
			BX		LR
			ENDP
			
				
PutHexIntMulti	PROC	{R0-R14}
;****************************************************************
;Description:
;	Outputs an n-word unsigned number, from memory starting at the 
;	address in R0, to the terminal in text hexadecimal representation 
;	using 8n hex digits, where the value in R1 is n
;Input Parameter: 
;	R0: n-word number to output (unsigned word address)
;	R1: n, the number of words in the input number (unsigned word value)
;Output Parameter: 
;	(none)
;modified registers:
;	no registers other than PSR (and any output parameters) have changed values
;	after return
;****************************************************************
			PUSH	{LR,R0-R4}
			MOVS	R2,R0
			MOVS	R3,R1		;copy n to r3
			SUBS	R3,R3,#1	;update r3 to n-1
			LSLS	R3,R3,#2	;Store 4*(n-1) in R3 (using shift)
			MOVS	R4,#0

PHIMLoop	
			LDR 	R0,[R2,R3]			;load hex sum from addr in r0 into r0
			BL		PutNumHex			;use putnumhex thingy
			SUBS	R3,R3,#4
			CMP		R3,R4
			BGE		PHIMLoop			;loop till R3 less than 0
			POP		{PC,R0-R4}
			BX		LR
			ENDP	

PutNumHex	PROC	{R0-R14}
;****************************************************************
;Description:
;	Prints to the terminal screen the text hexadecimal representation of the
;	unsigned word value in R0. (For example, if R0 contains 0x000012FF, then 
;	000012FF should print on the terminal. Note: 12FF would not be acceptable. 
;	Do not use division to determine the hexadecimal digit values�use bit masks 
;	and shifts.)
;Input Parameter: 
;	R0: number to print in hexadecimal (unsigned word value)
;Output Parameter: none
;modified registers:
;	no registers other than PSR and any output parameters) have changed values
;	after return
;subroutines utilized:
;	PutChar
;****************************************************************
			PUSH	{LR,R0-R7}
			;REV		R0,R0			;reverse order of bytes
			MOVS	R2,R0			;copy unsigned word value to r2
			MOVS	R3,#8			;move 8 to r3 as a comparator
			MOVS	R4,#0			;move 0 to r4 as a loop counter (i)
			;need to put #0xFFFFFFF0 in r6 as clearing mask
			MOVS	R6,#2_0000000000001111 ;will try just like this
			
			
			MOVS	R7,#9			;comporator for ascii
convertLoopHex	
			MOVS	R5,#4			;move 4 to R5 as a multiplication const.
			MOVS	R0,R2			;restore original hex value
			MULS	R5,R4,R5		;R5<-4*1
			LSRS	R0,R0,R5		;shift right 4*i bits(=1 nibble= 1 hex char)
									;and pad with zeros on the left
			ANDS	R0,R0,R6		;clear all but LSB of R0
			CMP		R0,R7			;compare single hex digit with 
			BLE		hexLessThan9	;if 9 or less just add 0x30
			MOVS	R1,#0x37		;if  >9, add 0x32 (idk why not 0x40) 
			B		nowConvert		
hexLessThan9	
			MOVS	R1,#0x30		;ascii conversion mask
nowConvert
			ADDS	R0,R0,R1		;convert hex character to ascii
			PUSH	{R0}			;push R0 to print later in reverse order
			ADDS	R4,R4,#1		;increment i
			CMP		R3,R4
			BEQ		endConvertHex	;once counter equal to 8 its done			
			B		convertLoopHex
endConvertHex
			;pop 8 times from the stack and print using putchar (LIFO)
			MOVS	R4,#0			;new loop counter n at 0
LIFOPrintLoop
			CMP		R3,R4			;compare n to 8
			BEQ		finishedLIFOPrint ;when equal stop print looping
			POP		{R0}
			BL		PutChar
			ADDS	R4,R4,#1 		;increment n
			B		LIFOPrintLoop
finishedLIFOPrint
			POP		{PC,R0-R7}
			BX		LR
			ENDP
			
PutChar		PROC	{R0-R14}
;****************************************************************
;Polled Transmit 
;Description from provided doc:
;A polled transmit, (i.e., send character), polls UART0 status 
;register 1 for the transmit data register empty (TDRE) condition 
;in bit 7. If TDRE = 1, a byte may be transmitted (i.e., sent); 
;otherwise, UART0 is not yet ready to transmit a byte, and the
;status register must be polled until TDRE = 1.
;Input Parameter: 
;	R0:  character to send to terminal (unsigned byte ASCII code)
;Output Parameter: none
;no registers but LR, PC, PSR changed after return
;****************************************************************
;code from pdf
;Poll TDRE until UART0 ready to transmit
			PUSH	{LR,R0-R3}
			LDR 	R1,=UART0_BASE
			MOVS 	R2,#UART0_S1_TDRE_MASK
PollTx 		LDRB 	R3,[R1,#UART0_S1_OFFSET]
			ANDS 	R3,R3,R2
			BEQ 	PollTx
;Transmit character stored in R0
			STRB 	R0,[R1,#UART0_D_OFFSET]
			POP		{PC,R0-R3}
			BX		LR
			ENDP
				
				
GetChar		PROC	{R1-R14}
;****************************************************************
;Polled Recieve 
;Description from provided doc:
;a polled receive, (i.e., get character), polls UART0 status 
;register 1 for the receive data register full (RDRF) condition 
;in bit 5. If RDRF = 1, a byte may be read by the KL05, 
;(i.e., received); otherwise, UART0 has not yet received a 
;byte, and the status register must be polled until RDRF = 1.
;Input Parameter: none
;Output Parameter: 
;	R0: character recieved from terminal (unsigned byte ASCII code)
;no registers but R0, LR, PC, PSR changed after return
;****************************************************************
;code from doc provided
			PUSH	{LR,R1-R3}
;Poll RDRF until UART0 ready to receive
			LDR 	R1,=UART0_BASE
			MOVS 	R2,#UART0_S1_RDRF_MASK
PollRx 		LDRB R3,[R1,#UART0_S1_OFFSET]
			ANDS 	R3,R3,R2
			BEQ 	PollRx
;Receive character and store in R0
			LDRB 	R0,[R1,#UART0_D_OFFSET]
			POP		{PC,R1-R3}
			BX		LR
			ENDP
				
GetStringSB	PROC	{R1-R14}
;****************************************************************
;Inputs string from terminal reventing overrun 
	;Preventing overrun of the buffer capacity specified in R1, this
;subroutine inputs a string from the terminal keyboard to memory 
;starting at the address in R0 and adds null termination. It ends 
;terminal keyboard input when the user presses the enter key. 
;For each of up to R1 - 1 characters typed on the terminal 
;keyboard, it uses GetChar to input the character, uses PutChar 
;to echo the character to the terminal screen, and stores the 
;character at the next position in the string. For any character
;typed after the first R1 - 1 characters, it uses GetChar to input 
;the character, but it neither stores the character in the string 
;nor echoes the character to the terminal screen.When the carriage 
;return character has been received, it null terminates the string,
;advances the cursor on the terminal screen to the beginning of 
;the next line, and returns.
;Input Parameter: 
;~~~R0: should be mem adress to start at. Not specified in doc
;	R1: bytes in string buffer where R0 points(unsigned word value)
;Output Parameter: 
;	R0: string buffer in memory for input from user (unsigned word 
;		address)
;no registers but PSR changed after return
;Subroutines used:
;	GetChar
;	PutChar
;****************************************************************
			PUSH	{LR,R0-R5}
			MOVS	R3,R0		;copy address to store in to R3
			SUBS	R4,R1,#1	;store R1-1 in R4
			MOVS	R2,#0		;initialize r2 as 0 for num chars typed
loopHere	
			BL		GetChar		;Get char from terminal (now in R0)
			MOVS	R5,R2		;r5 1 less then r2 for storage purposes
			ADDS	R2,R2,#1	;increment r2
			
			CMP		R0,#CR		;compare the character typed to carriage return
			BEQ		enterPressed;if equal branch accordingly
			
			CMP		R2,R4		;compare r1-1 with num chars typed so far
			BGT		loopHere	;if r2 > r4 then no output or store
			
			;output and store 
			BL		PutChar		;output char in r0 to terminal
			STRB	R0,[R3,R5]  ;store in addr r0 inc offset
			B		loopHere
			
enterPressed
			LDR		R0,=lenOpStr;load address to store the length of op str in
			CMP		R5,R4		;if length greater than max length
			BLE		notExceed
			MOVS	R5,R4		;store length as max length
notExceed
			STRB	R5,[R0,#0]	;store the length of the op str 
			MOVS	R0,#NULL	;move equate null into r0 
			STRB	R0,[R3,R5]  ;store null in addr r0 inc offset			
			POP		{PC,R0-R5}
			BX		LR
			ENDP			
				
PutStringSB	PROC	{R0-R14}
;****************************************************************
;Displays string from memory to terminal screen
	;Preventing overrun of the buffer capacity specified in R1, 
;this subroutine displays a null-terminated string to the terminal 
;screen from memory starting at the address in R0. It uses PutChar 
;to display characters from the string and leaves the terminal 
;screen cursor positioned after the last character of the string.
;Input Parameters: 
;	R0: string buffer in memory for output to simulated output stream 
;	(unsigned word address)
;	R1: bytes in string buffer where R0 points (unsigned word value)
;Output Parameter:
;	none
;no registers but PSR changed after return
;Subroutines used:
;	PutChar
;****************************************************************
			
			PUSH	{LR,R0-R3}
			MOVS	R2,R0		;copy starting address of string
			MOVS	R3,#0		;initialize counter
printLoop
			LDRB	R0,[R2,R3]	;configure input to PutChar	correctly with
								;offset
			CMP		R0,#NULL	;compare to see if null character an
			BEQ		stopPrint	;if so terminate
			
			BL		PutChar		;use putchar to output char R0 to terminal 		
			
			ADDS	R3,R3,#1	;increment counter
			CMP		R3,R1
			BLE		printLoop	;if the counter is less than or equal to the 
stopPrint						;num bytes in the string branch and get next
			POP		{PC,R0-R3}	;otherwise restore registers and exit
			BX		LR
			ENDP			
				
PutNumU		PROC	{R0-R14}
;****************************************************************
	;Displays text decimal representation of value to terminal screen
;This subroutine displays the text decimal representation to the
;terminal screen of the unsigned word value in R0, using PutChar 
;to output each decimal digit character. (For example, if R0 
;contains 0x00000100, then 256 should be output 0000000256 would 
;also be acceptable.) (hint use divU)
;Input Parameters: 
;R0: number for output to terminal(unsigned word value)
;Output Parameter:
;	none
;no registers but PSR changed after return
;Subroutines used:
;	DIVU
;	PutChar
;****************************************************************
			PUSH	{LR,R0-R5}
			MOVS	R2,R0		;backup word value in R2
			MOVS 	R3,#10		;store 10 to be the denominator
			MOVS	R4,#0		;store counter in R4
			MOVS	R1,R2		;move word value to numerator
convertLoop
			MOVS    R0,R3		;move 10 to be the denominator
			
			BL		DIVU		;use DIVU  		
			PUSH	{R1}		;store the remainder in the stack
			ADDS	R4,R4,#1	;increment counter
			MOVS	R1,R0		;move the quotient so that it will be the ~~~~
								;numerator next time around
			CMP		R0,#0		;check if quotient is 0, if so done
			BNE		convertLoop	;if quotient not equal 0 go again
popLoop		
			POP		{R0}		;pop individual result to the register r0
			MOVS	R5,#0x30	;move hex 30 into the register
			ADDS	R0,R0,R5	;add to r0 to make ascii from dec
			BL		PutChar		;use putchar to output to the terminal		
			SUBS	R4,R4,#1	;decrement counter
			CMP		R4,#0		;see if counter is back to 0
			BGT		popLoop		;if counter still positive loop again		
			POP		{PC,R0-R5}	;restore registers
			BX		LR			;branch back to where putNumU was called
			ENDP	
				
DIVU		PROC	{R2-R14}
;****************************************************************
;Subroutine from lab 4 created to perform integer division
;Input: Parameters:
;	R0: divisor (denominator)
;	R1: dividend (numerator)
;Output Parameters
;	R0: quotient
;	R1: remainder
;	C: APSR flag: 0 for valid result; 1 for invalid result
;Subroutines Used:
;	JustSetC
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
			BL		JustSetC
endit
			BX		LR
			ENDP			
			
Init_UART0_Polling	PROC	{R0-R14}
;****************************************************************
;select/configure UART0 sources 
;enable clocks for UART0 and Port B
;set Port B mux pins to connect to UART0 
;Configure UART0 (register initialization)
;Input Parameter: none
;Output Parameter: none
;no registers but LR, PC, PSR changed after return
;****************************************************************
			PUSH	{R0-R2}
			;code copied from provided PDF
			;Select MCGFLLCLK as UART0 clock source
			LDR 	R0,=SIM_SOPT2
			LDR 	R1,=SIM_SOPT2_UART0SRC_MASK
			LDR 	R2,[R0,#0]
			BICS 	R2,R2,R1
			LDR 	R1,=SIM_SOPT2_UART0SRC_MCGFLLCLK
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
			;Set UART0 for external connection
			LDR 	R0,=SIM_SOPT5
			LDR 	R1,=SIM_SOPT5_UART0_EXTERN_MASK_CLEAR
			LDR 	R2,[R0,#0]
			BICS 	R2,R2,R1
			STR 	R2,[R0,#0]
			;Enable UART0 module clock
			LDR 	R0,=SIM_SCGC4
			LDR 	R1,=SIM_SCGC4_UART0_MASK
			LDR 	R2,[R0,#0]
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
			;Enable PORT B module clock
			LDR 	R0,=SIM_SCGC5
			LDR 	R1,=SIM_SCGC5_PORTB_MASK
			LDR 	R2,[R0,#0]
			ORRS 	R2,R2,R1
			STR 	R2,[R0,#0]
			;Select PORT B Pin 2 (D0) for UART0 RX (J8 Pin 01)
			LDR 	R0,=PORTB_PCR2
			LDR 	R1,=PORT_PCR_SET_PTB2_UART0_RX
			STR 	R1,[R0,#0]
			; Select PORT B Pin 1 (D1) for UART0 TX (J8 Pin 02)
			LDR 	R0,=PORTB_PCR1
			LDR 	R1,=PORT_PCR_SET_PTB1_UART0_TX
			STR 	R1,[R0,#0]
			;Disable UART0 receiver and transmitter
			LDR 	R0,=UART0_BASE
			MOVS	R1,#UART0_C2_T_R
			LDRB 	R2,[R0,#UART0_C2_OFFSET]
			BICS 	R2,R2,R1
			STRB 	R2,[R0,#UART0_C2_OFFSET]
			;Set UART0 for 9600 baud, 8N1 protocol
			MOVS 	R1,#UART0_BDH_9600
			STRB 	R1,[R0,#UART0_BDH_OFFSET]
			MOVS	R1,#UART0_BDL_9600
			STRB 	R1,[R0,#UART0_BDL_OFFSET]
			MOVS 	R1,#UART0_C1_8N1
			STRB 	R1,[R0,#UART0_C1_OFFSET]
			MOVS 	R1,#UART0_C3_NO_TXINV
			STRB 	R1,[R0,#UART0_C3_OFFSET]
			MOVS 	R1,#UART0_C4_NO_MATCH_OSR_16
			STRB 	R1,[R0,#UART0_C4_OFFSET]
			MOVS 	R1,#UART0_C5_NO_DMA_SSR_SYNC
			STRB 	R1,[R0,#UART0_C5_OFFSET]
			MOVS 	R1,#UART0_S1_CLEAR_FLAGS
			STRB 	R1,[R0,#UART0_S1_OFFSET]
			MOVS 	R1, \
					#UART0_S2_NO_RXINV_BRK10_NO_LBKDETECT_CLEAR_FLAGS
			STRB 	R1,[R0,#UART0_S2_OFFSET]
			;Enable UART0 receiver and transmitter
			MOVS	R1,#UART0_C2_T_R
			STRB 	R1,[R0,#UART0_C2_OFFSET]
			;end code from PDF
			POP		{R0-R2}
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
            IMPORT  __initial_sp
            IMPORT  Dummy_Handler
            IMPORT  HardFault_Handler
__Vectors 
                                      ;ARM core vectors
            DCD    __initial_sp       ;00:end of stack
            DCD    Reset_Handler      ;01:reset vector
            DCD    Dummy_Handler      ;02:NMI
            DCD    HardFault_Handler  ;03:hard fault
            DCD    Dummy_Handler      ;04:(reserved)
            DCD    Dummy_Handler      ;05:(reserved)
            DCD    Dummy_Handler      ;06:(reserved)
            DCD    Dummy_Handler      ;07:(reserved)
            DCD    Dummy_Handler      ;08:(reserved)
            DCD    Dummy_Handler      ;09:(reserved)
            DCD    Dummy_Handler      ;10:(reserved)
            DCD    Dummy_Handler      ;11:SVCall (supervisor call)
            DCD    Dummy_Handler      ;12:(reserved)
            DCD    Dummy_Handler      ;13:(reserved)
            DCD    Dummy_Handler      ;14:PendSV (PendableSrvReq)
                                      ;   pendable request 
                                      ;   for system service)
            DCD    Dummy_Handler      ;15:SysTick (system tick timer)
            DCD    Dummy_Handler      ;16:DMA channel 0 transfer 
                                      ;   complete/error
            DCD    Dummy_Handler      ;17:DMA channel 1 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;18:DMA channel 2 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;19:DMA channel 3 transfer
                                      ;   complete/error
            DCD    Dummy_Handler      ;20:(reserved)
            DCD    Dummy_Handler      ;21:FTFA command complete/
                                      ;   read collision
            DCD    Dummy_Handler      ;22:low-voltage detect;
                                      ;   low-voltage warning
            DCD    Dummy_Handler      ;23:low leakage wakeup
            DCD    Dummy_Handler      ;24:I2C0
            DCD    Dummy_Handler      ;25:(reserved)
            DCD    Dummy_Handler      ;26:SPI0
            DCD    Dummy_Handler      ;27:(reserved)
            DCD    Dummy_Handler      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    Dummy_Handler      ;38:PIT
            DCD    Dummy_Handler      ;39:(reserved)
            DCD    Dummy_Handler      ;40:(reserved)
            DCD    Dummy_Handler      ;41:DAC0
            DCD    Dummy_Handler      ;42:TSI0
            DCD    Dummy_Handler      ;43:MCG
            DCD    Dummy_Handler      ;44:LPTMR0
            DCD    Dummy_Handler      ;45:(reserved)
            DCD    Dummy_Handler      ;46:PORTA
            DCD    Dummy_Handler      ;47:PORTB
__Vectors_End
__Vectors_Size  EQU     __Vectors_End - __Vectors
            ALIGN
;****************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
;>>>>> begin constants here <<<<<
FIRST_PROMPT  	 DCB	" Enter first 96-bit hex number: 0x"
SECOND_PROMPT 	 DCB	"Enter 96-bit hex number to add: 0x"
INVALID_MESSAGE	 DCB	"	    Invalid number--try again: 0x"
SUM_MESSAGE		 DCB	"                           Sum: 0x"
OVERFLOW_MESSAGE DCB	"OVERFLOW"
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
convertMem	SPACE	28
firstNum	SPACE 	12
secondNum	SPACE 	12
sumMem		SPACE 	12
lenOpStr	SPACE	2				;length of the operational string

;>>>>>   end variables here <<<<<
            ALIGN
            END
