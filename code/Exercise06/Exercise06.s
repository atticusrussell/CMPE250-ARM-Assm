            TTL CMPE 250 Exercise 6
;****************************************************************
;Polled Serial I/O
;Name:  <Atticus Russell>
;Date:  <3/04/2021>
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
;EQUates (copied from Useful equates mycourses)
;Characters
CR   		 EQU  0x0D			;moves cursor beginning line
LF   		 EQU  0x0A			;adds new line
NULL 		 EQU  0x00

MAX_STRING	 EQU	 79 		;the maximum number of string characters, 
;	including the null termination character
LEN_LENGTH 	 EQU	7			;the num characters in "Length:"
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
			IMPORT	LengthStringSB	;from ex6_lib
            IMPORT  Startup
			EXPORT 	PutChar
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
loopHereInput
			MOVS	R0,#CR				;load carriage return
			BL		PutChar				;print carriage return
			MOVS	R0,#LF				;load line feed
			BL		PutChar				;print line feed
			LDR		R0,=PROMPT_STRING
			MOVS	R1,#31			;store num bytes in prompt string
			BL		PutStringSB		;print prompt to terminal from mem
			BL		GetChar			;get command char from terminal
			BL		PutChar			;show typed char on terminal
			;checking if char is lowercase
			CMP		R0,#'z'			;check if ASCII is greater than 'z;
			BHI 	notLowerCase
			CMP		R0,#'a'
			BLO		notLowerCase	;check if ASCII is less than 'a'
			;Converting lowercase to uppercase
			SUBS	R0,R0,#32		;otherwise make uppercase subtract 32
notLowerCase
			CMP		R0,#'G'
			BEQ		GInstruction
			CMP		R0,#'I'
			BEQ		IInstruction
			CMP		R0,#'L'
			BEQ		LInstruction
			CMP		R0,#'P'
			BEQ		PInstruction
;otherwise repeat regular 
			B		loopHereInput
;special instructions
GInstruction
			MOVS	R0,#CR				;load carriage return
			BL		PutChar				;print carriage return
			MOVS	R0,#LF				;load line feed
			BL		PutChar				;print line feed
			MOVS	R0,#'<'				;load '<'
			BL		PutChar				;print '<'
			LDR		R0,=String			;mem address of string
			MOVS	R1,#MAX_STRING		;length of max string
			BL		GetStringSB			;branch to get string input 
			B		loopHereInput		;branch to start again
IInstruction
			LDR		R0,=String			;mem address of op string
			MOVS	R1,#NULL			;load null character
			STRB	R1,[R0,#0]			;null char in first byte of op string
			MOVS	R0,#CR				;load carriage return
			BL		PutChar				;print carriage return
			B		loopHereInput
LInstruction
			MOVS	R0,#CR				;load carriage return
			BL		PutChar				;print carriage return
			MOVS	R0,#LF				;load line feed
			BL		PutChar				;print line feed
			LDR		R0,=LEN_STR			;mem address of "Length:"
			MOVS	R1,#LEN_LENGTH		;load num characters of "length
			BL		PutStringSB			;print "Length:"
			LDR		R1,=lenOpStr		;load mem addr of length op str
			LDRB	R0,[R1,#0]			;load value of len op str to R0
			BL		PutNumU				;print length of op string in dec
			B		loopHereInput
PInstruction	
			MOVS	R0,#CR				;load carriage return
			BL		PutChar				;print carriage return
			MOVS	R0,#LF				;load line feed
			BL		PutChar				;print line feed
			MOVS	R0,#'>'				;load '>'
			BL		PutChar				;print '>'
			LDR		R0,=String			;load start mem addr op str
			LDR		R1,=MAX_STRING		;load length of max string
			BL		PutStringSB			;branch to print the string
			MOVS	R0,#'>'				;load '>'
			BL		PutChar				;print '>'
			B		loopHereInput

;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
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
				
				
PutChar		PROC	{R0-R14}
;****************************************************************
;Polled Transmit 
;Description from provided doc:
;A polled transmit, (i.e., send character), polls UART0 status 
;register 1 for the transmit data register empty (TDRE) condition 
;in bit 7. If TDRE = 1, a byte may be transmitted (i.e., sent); 
;otherwise, UART0 is not yet ready to transmit a byte, and the
;status register must be polled until TDRE = 1.
;Input Parameter: R0,  character to send to terminal (unsigned
;byte ASCII code)
;Output Parameter: none
;no registers but LR, PC, PSR changed after return
;****************************************************************
;code from pdf
;Poll TDRE until UART0 ready to transmit
			PUSH	{R0-R3}
			LDR 	R1,=UART0_BASE
			MOVS 	R2,#UART0_S1_TDRE_MASK
PollTx 		LDRB 	R3,[R1,#UART0_S1_OFFSET]
			ANDS 	R3,R3,R2
			BEQ 	PollTx
;Transmit character stored in R0
			STRB 	R0,[R1,#UART0_D_OFFSET]
			POP		{R0-R3}
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
;Output Parameter: R0, character recieved from terminal unsigned
;byte ASCII code)
;no registers but R0, LR, PC, PSR changed after return
;****************************************************************
;code from doc provided
			PUSH	{R1-R3}
;Poll RDRF until UART0 ready to receive
			LDR 	R1,=UART0_BASE
			MOVS 	R2,#UART0_S1_RDRF_MASK
PollRx 		LDRB 	R3,[R1,#UART0_S1_OFFSET]
			ANDS 	R3,R3,R2
			BEQ 	PollRx
;Receive character and store in R0
			LDRB 	R0,[R1,#UART0_D_OFFSET]
			POP		{R1-R3}
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
;	R0: should be mem adress to start at. Not specified in doc
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
			;MOVS	R0,#LF		;load carriage return
			;BL	PutChar		;output carriage return to terminal
			;MOVS	R0,#LF		;load line feed
			;BL		PutChar		;print line feed
			
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
			BX		LR			;branch back to where putNUmU was called
			ENDP	
				
DIVU		PROC	{R2-R14}
;****************************************************************
;Subroutine from lab 4 created to perform integer division
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
PROMPT_STRING	DCB		"Type a string command (G,I,L,P):"
LEN_STR			DCB		"Length:"
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
String 		SPACE	MAX_STRING
lenOpStr	SPACE	2				;length of the operational string
;>>>>>   end variables here <<<<<
            ALIGN
            END
