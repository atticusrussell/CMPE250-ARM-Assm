            TTL CMPE 250 Exercise 11
			SUBT PWM and LEDs with Mixed Assembly Language and C
;****************************************************************
;A PWM waveform from the KL05 is used to control the brightness 
; of the red LED on the KL05Z board. 
;----------------------------------------------------------------
;Name:  <Atticus Russell>
;Date:  <April 26,2021>
;Class:  CMPE-250
;Section:  <Section 01L1, Thursday 2:00-3:55>
;---------------------------------------------------------------
;Keil Template for KL05 Assembly with Keil C startup
;R. W. Melton
;November 3, 2020
;****************************************************************
;Assembler directives
            THUMB
            GBLL  MIXED_ASM_C
MIXED_ASM_C SETL  {TRUE}
            OPT   64  ;Turn on listing macro expansions
;****************************************************************
;Include files
            GET  MKL05Z4.s
            OPT  1          ;Turn on listing
;****************************************************************
;EQUates
;DAC0
DAC0_BITS	 EQU 12
DAC0_STEPS	 EQU 4096
;LED
LED_LEVELS 	EQU 5
;PWM	
PWM_DUTY_MAX EQU 49970	;(PWM_PERIOD - 1) 

;Below EQUates are from lab exercise 7
;---------------------------------------------------------------
; Queue management record field offsets
IN_PTR      EQU   0
OUT_PTR     EQU   4
BUF_STRT    EQU   8
BUF_PAST    EQU   12
BUF_SIZE    EQU   16
NUM_ENQD    EQU   17
; Queue structure sizes
Q_BUF_SZ    EQU   4   ;Queue contents
Q_REC_SZ    EQU   18  ;Queue management record
; Number of bytes in prompts
PROMPT_QUEUE_LEN	EQU		32;
FAILURE_STR_LEN	EQU		7;
ENQUEUE_STR_LEN	EQU		20;
SUCCESS_STR_LEN	EQU 	7;
STATUS_STR_LEN	EQU		6;
IN_STR_LEN		EQU		4;
OUT_STR_LEN		EQU		8;
NUM_STR_LEN		EQU		3;
HELP_STR_LEN	EQU		56;
;---------------------------------------------------------------
;For text output and related subroutines
CR   		 EQU  0x0D			;moves cursor beginning line
LF   		 EQU  0x0A			;adds new line
NULL 		 EQU  0x00			;the null character
;below equates are from "useful EQUates for UART0 serial driver"
;---------------------------------------------------------------
;NVIC_ICER
;31-00:CLRENA=masks for HW IRQ sources;
;             read:   0 = unmasked;   1 = masked
;             write:  0 = no effect;  1 = mask
;12:UART0 IRQ mask
NVIC_ICER_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_ICPR
;31-00:CLRPEND=pending status for HW IRQ sources;
;             read:   0 = not pending;  1 = pending
;             write:  0 = no effect;
;                     1 = change status to not pending
;12:UART0 IRQ pending status
NVIC_ICPR_UART0_MASK  EQU  UART0_IRQ_MASK
;---------------------------------------------------------------
;NVIC_IPR0-NVIC_IPR7
;2-bit priority:  00 = highest; 11 = lowest
UART0_IRQ_PRIORITY    EQU  3
NVIC_IPR_UART0_MASK   EQU (3 << UART0_PRI_POS)
NVIC_IPR_UART0_PRI_3  EQU (UART0_IRQ_PRIORITY << UART0_PRI_POS)
;---------------------------------------------------------------
;NVIC_ISER
;31-00:SETENA=masks for HW IRQ sources;
;             read:   0 = masked;     1 = unmasked
;             write:  0 = no effect;  1 = unmask
;12:UART0 IRQ mask
NVIC_ISER_UART0_MASK  EQU  UART0_IRQ_MASK
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
;MCGFLLCLK is 47972352 Hz ~=~ 48 MHz
;SBR ~=~ 48 MHz / (9600 * 16) = 312.5 --> 312 = 0x138
;SBR = 47972352 / (9600 * 16) = 312.32 --> 312 = 0x138
UART0_BDH_9600  EQU  0x01
;---------------------------------------------------------------
;UART0_BDL
;26->7-0:SBR[7:0] (UART0CLK / [9600 * (OSR + 1)])
;UART0CLK is MCGFLLCLK
;MCGFLLCLK is 47972352 Hz ~=~ 48 MHz
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
UART0_C2_T_R    EQU  (UART0_C2_TE_MASK :OR: UART0_C2_RE_MASK)
UART0_C2_T_RI   EQU  (UART0_C2_RIE_MASK :OR: UART0_C2_T_R)
UART0_C2_TI_RI  EQU  (UART0_C2_TIE_MASK :OR: UART0_C2_T_RI)
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
;MACROs
;****************************************************************
;Program
;C source will contain main ()
;Only subroutines and ISRs in this assembly source
            AREA    MyCode,CODE,READONLY
			EXPORT	GetChar
			EXPORT	GetStringSB
			EXPORT	Init_UART0_IRQ
			EXPORT 	PutChar
			EXPORT	PutNumHex
			EXPORT	PutNumUB			
			EXPORT	PutStringSB
			EXPORT	UART0_IRQHandler
;>>>>> begin subroutine code <<<<<
UART0_IRQHandler
UART0_ISR	PROC	{R0-R14}
;****************************************************************
;ISR that handles UART0 transmit and recieve interrupts. You 
;must write the ISR so that no registers have changed value after 
;return. (Note: the Cortex-M0+ automatically preserves R0–R3, 
;R12, LR, PC, and PSR for ISRs.)
;****************************************************************
			CPSID	I 	;Mask interrupts
			PUSH	{LR} 
						;push any registers used except R0–R3, R12
						;interrupt source can be found in UART0_S1
			LDR 	R3,=UART0_BASE
			MOVS	R1,#UART0_C2_TIE_MASK
						; 	if (TxInteruptEnabled) then ;TIE = 1 in UART0_C2 
			LDRB	R2,[R3,#UART0_C2_OFFSET]
			ANDS	R2,R2,R1	;if the bit was set non zero value 
			CMP		R2,#0
			BEQ		if_TIE_0
						; 		if (TxInterrupt) then ;TDRE = 1 in UART0_S1 
			MOVS	R1,#UART0_S1_TDRE_MASK
			LDRB	R2,[R3,#UART0_S1_OFFSET]
			ANDS	R2,R2,R1	;if the bit was set non zero value 
			CMP		R2,#0
			BEQ		if_TIE_0
						; 			Dequeue character from TxQueue 
			LDR		R1,=TxQRecord
			BL		Dequeue
			BCS		unsuccessful_DQ
						; 			if (dequeue successful) then 
						; 				Write character to UART0_D ;Tx data reg. 
			STRB	R0,[R3,#UART0_D_OFFSET]
			B		if_TIE_0
						; 			else
unsuccessful_DQ
						; 				Disable TxInterrupt ;UART0_C2_T_RI 
			MOVS	R0,#UART0_C2_T_RI
			STRB	R0,[R3,#UART0_C2_OFFSET]
if_TIE_0	
			; 	if (RxInterrupt) then ;RDRF = 1 in UART0_S1 
			MOVS	R1,#UART0_S1_RDRF_MASK
			LDRB	R2,[R3,#UART0_S1_OFFSET]
			ANDS	R2,R2,R1	;if the bit was set non zero value 
			CMP		R2,#0
			BEQ		endGoodNaming
		; 		Read character from UART0_D ;receive data register 
			LDRB	R0,[R3,#UART0_D_OFFSET]
						; 		Enqueue character in RxQueue 
			LDR		R1,=RxQRecord
			BL		Enqueue
						; 		;Character lost if RxQueue full 
endGoodNaming						
						; Pop any registers pushed above 
			CPSIE	I   ; Unmask other interrupts ;CPSIE I 
			POP		{PC}
			ENDP
			
GetChar		PROC	{R1-R14}
;****************************************************************
;Description:
;	Dequeues a character from the recieve queue and returns it in
;	R0
;Input Parameter:
;	none
;Output Parameter:
;	R0: returned character (unsigned byte ASCII code)
;modified registers:
;	Nothing but output parameter registers and PSR
;subroutines utilized:
;	Dequeue
;****************************************************************
;code from doc provided
			PUSH	{LR,R1}
			LDR		R1,=RxQRecord	;load addr RxQRecord
tryDequeueLoop
			CPSID	I
			BL		Dequeue			;R0<- char from RxQBuffer
			CPSIE	I
			BCS		tryDequeueLoop	;if c set (Dequeue failed) try again
			POP		{PC,R1}
			ENDP
			
Enqueue		PROC	{R2-R14}
;****************************************************************
;Description:
;	Attempts to put a character in the queue whose queue record structure’s
;	address is in R1: if the queue is not full, enqueues the single character 
;	from R0 to the queue, and returns with the PSR C bit cleared to report 
;	enqueue success, otherwise, returns with the PSR C bit set to report 
;	enqueue failure.
;Input Parameter: 
;	R0: character to enqueue (unsigned byte ASCII code)
;~~	R1: queue record structure (unsigned word address)
;Output Parameter: 
;	R1: queue record structure (unsigned word address)
;	C: enqueue operation status: 0 success; 1 failure (PSR bit flag)
;modified registers:
;	no registers other than PSR and any output parameters) have changed values
;	after return
;subroutines utilized:
;	JustClearC
;	JustSetC
;****************************************************************
			PUSH	{LR,R2-R6}
			LDRB	R2,[R1,#NUM_ENQD]	;R2<-num enqueued (byte)
			LDRB	R3,[R1,#BUF_SIZE]	;R3<-max num Q items (byte)
			CMP		R2,R3			;see if queueFull
			BGE		queueFull		;if queueFull go to queueFull
			LDR		R4,[R1,#IN_PTR]	;R4<-Q IN_PTR
			STRB	R0,[R4,#0]		;store char at IN_PTR location 
			ADDS	R2,R2,#1		;increment num_enqued
			STRB	R2,[R1,#NUM_ENQD];store updated value
			ADDS	R4,R4,#1		;increment IN_PTR past new item
			STR		R4,[R1,#IN_PTR] ;store updated valueof inpointer
			LDR		R5,[R1,#BUF_PAST]	;R5<-BUF_PAST
			CMP		R4,R5			;Compare IN_PTR and BUF_PAST
			BLO		notExceeded;if IN_PTR less than BUF_PAST: notExceeded
			;if queue now full, set inptr to start of queue buffer
			LDR		R6,[R1,#BUF_STRT];R6 <- start Q Buff
			STR		R6,[R1,#IN_PTR]	;store updated inpointer
			BL		JustClearC		;indicate successful enqueue
			B		endImmediate
notExceeded
			STR		R4,[R1,#IN_PTR];store updated valueof inpointer
			BL		JustClearC		;indicate successful enqueue
			B		endImmediate
queueFull	;set c flag and end
			BL		JustSetC
endImmediate
			POP		{PC,R2-R6}
			BX		LR
			ENDP	
		
Dequeue		PROC	{R2-R14}
;****************************************************************
;Description:
;	Attempts to get a character from the queue whose record structure’s
;	address is in R1: if the queue is not empty, dequeues a single character
;	from the queue to R0, and returns with the PSR C bit cleared, (i.e., 0),
;	to report dequeue success, otherwise, returns with the PSR C bit set, 
;	(i.e., 1), to report dequeue failure.
;Input Parameter: 
;	R1: queue record structure (unsigned word address)
;Output Parameter/: 
;	R0: character dequeued (unsigned byte ASCII code)
;	R1: queue record structure (unsigned word address)
;	C: dequeue operation status: 0 success; 1 failure (PSR bit flag)
;modified registers:
;	no registers other than PSR (and any output parameters) have changed values
;	after return
;subroutines utilized:
;	JustClearC
;	JustSetC
;****************************************************************
			PUSH	{LR,R2-R4}
			;first check if the queue is empty - if so indicate failure + end
			LDRB	R2,[R1,#NUM_ENQD];R2 <- number of items in queue
			MOVS	R3,#0			;R3 <- 0
			CMP		R2,R3			
			BEQ		queueEmpty		;if num_enqued==0,end and fail
			;otherwise dequeue into r0, update record, and indicate success
			LDR		R4,[R1,#OUT_PTR];load OUT_PTR addr into R4
			LDRB	R0,[R4,#0]		;load byte value stored in addr in R4
			;update records and such
			SUBS	R2,R2,#1		;r2 <- r2-1
			STRB	R2,[R1,#NUM_ENQD];store decremented number s
			;increment OUT_PTR (loop it to BUF_STRT if it hits BUF_PAST)
			LDR		R2,[R1,#OUT_PTR]
			ADDS	R2,R2,#1		;increment OUT_PTR
			LDR		R3,[R1,#BUF_PAST]
			CMP		R2,R3			
			BGE		passedBuffer	;branch if hit buf_past
			STR		R2,[R1,#OUT_PTR];if no branch, store incremented out ptr
			B		successDequeue	;branch to clear c and end
passedBuffer	
			LDR		R3,[R1,#BUF_STRT]
			STR		R3,[R1,#OUT_PTR];store new out pointer as start of buf	
successDequeue						;clear c and end subroutine
			BL		JustClearC
			B		endDequeueHere
queueEmpty
			BL		JustSetC		;run subroutine to set c and then end
endDequeueHere
			POP		{PC,R2-R4}
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

Init_UART0_IRQ		PROC	{R0-R14}
;****************************************************************
;Description:
;	This subroutine initializes the KL05 for interrupt -based serial
;	I/0 with UART0 using the format: 1 start bit, 8 data bits, no	
;	parity, 1 stop bit at 9600 baud. same format and speed as previous
;	but with interrupts instead of polling. Should configure UART0,
;	initialize the UART0 interrupt in NVIC, and should initialize the 
;	receive and transmit queue management record structures (RxQRecord
;	and TxQRecord) for 80-character queue buffers (RxQBuffer and 
;	TxQBuffer, respectively) using InitQueue from Lab Exercise Seven. 
;	(Suggestion: copy Init_UART0_Polling that has been used since 
;	Lab Exercise Five, and modify it to use interrupts instead of 
;	polling, including changing UART0 initialization to support 
;	UART0_ISR and calling InitQueue from Lab Exercise Seven to 
;	initialize the receive and transmit queue management record 
;	structures.) 	 
;Input Parameter:
;	none
;Output Parameter:
;	none
;modified registers:
;	none
;subroutines utilized:
;	InitQueue
;****************************************************************
			PUSH	{LR,R0-R2}
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
			;Set UART0 IRQ priority 
			LDR 	R0,=UART0_IPR 
			;LDR R1,=NVIC_IPR_UART0_MASK 
			LDR 	R2,=NVIC_IPR_UART0_PRI_3 
			LDR 	R3,[R0,#0] 
			;BICS R3,R3,R1 
			ORRS 	R3,R3,R2 
			STR 	R3,[R0,#0] 
			;Clear any pending UART0 interrupts 
			LDR 	R0,=NVIC_ICPR 
			LDR 	R1,=NVIC_ICPR_UART0_MASK 
			STR 	R1,[R0,#0] 
			;Unmask UART0 interrupts 
			LDR 	R0,=NVIC_ISER 
			LDR 	R1,=NVIC_ISER_UART0_MASK 
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
			MOVS	R1,#UART0_C2_T_RI		;edited this for IRQ
			STRB 	R1,[R0,#UART0_C2_OFFSET]
			;end code from PDF
			LDR		R0,=TxQBuffer
			LDR		R1,=TxQRecord
			MOVS	R2,#80
			BL		InitQueue
			LDR		R0,=RxQBuffer
			LDR		R1,=RxQRecord
			MOVS	R2,#80
			BL		InitQueue
			
			POP		{PC,R0-R2}
			ENDP
			
InitQueue	PROC	{R0-R14}
;****************************************************************
;Description:
;	Initializes the queue record structure at the address in R1 for the empty
;	queue buffer at the address in R0 of size given in R2, (i.e., character 
;	capacity).
;Input Parameter: 
;	R0: queue buffer (unsigned word address)
;	R1: queue record structure (unsigned word address)
;	R2: queue capacity in bytes (unsigned byte value)
;Output Parameter: none
;modified registers:
;	no registers other than PSR and any output parameters) have changed values
;	after return
;****************************************************************
			PUSH  	{R0-R2}
			STR   	R0,[R1,#IN_PTR]		;store addr in ptr of Q in rec struct  
			STR   	R0,[R1,#OUT_PTR] 	;store addr out ptr of Q in rec struct 
			STR   	R0,[R1,#BUF_STRT] 	;store addr Q start in rec struct 
			ADDS  	R0,R0,R2 			;R0 = R0 + Q capacity
			STR   	R0,[R1,#BUF_PAST] 	;str 1st addr past Q buff end
			STRB  	R2,[R1,#BUF_SIZE] 	;store Q capacity in rec struct
			MOVS  	R0,#0 				;R0 = 0
			STRB  	R0,[R1,#NUM_ENQD] 	;store that nothing enqueued so far
			POP		{R0-R2}
			BX		LR
			ENDP	
			
PutChar		PROC	{R0-R14}
;****************************************************************
;Description:
;	Enqueues the character from R0 to the transmit queue
;Input Parameter:
;	R0: character to enqueue (unsigned byte ASCII code)
;Output Parameter:
;	none
;modified registers:
;	nothing but PSR
;Subroutines used:
;	Enqueue
;****************************************************************
			PUSH	{LR,R0-R1}
			LDR		R1,=TxQRecord	;load addr TxQRecord
tryEnqueueLoop
			CPSID	I
			BL		Enqueue			;put char in R0 into TxQBuffer
			CPSIE	I
			BCS		tryEnqueueLoop	;if c set (Enqueue failed) try again		
			;enable transmit interrupt
			LDR		R0,=UART0_BASE
			MOVS	R1,#UART0_C2_TI_RI
			STRB 	R1,[R0,#UART0_C2_OFFSET]
			POP		{PC,R0-R1}
			ENDP			
			
			
			
PutNumHex	PROC	{R0-R14}
;****************************************************************
;Description:
;	Prints to the terminal screen the text hexadecimal representation of the
;	unsigned word value in R0. (For example, if R0 contains 0x000012FF, then 
;	000012FF should print on the terminal. Note: 12FF would not be acceptable. 
;	Do not use division to determine the hexadecimal digit values—use bit masks 
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
					
PutNumUB	PROC	{R0-R14}
;****************************************************************
;Description:
;	Prints to the terminal screen the text decimal representation of the
;	unsigned byte value in R0. (For example, if R0 contains 0x003021101, 
;	then 1 should print on the terminal. Note: 001 would also be acceptable, 
;	as the text decimal representation of 0x01.) (Hint: use a mask to preserve 
;	only the least byte of the word in R0, and call your PutNumU subroutine
;	from Lab Exercise Six.)
;Input Parameter: 
;	R0: number to print in decimal (unsigned byte value)
;Output Parameter: none
;modified registers:
;	no registers other than PSR and any output parameters) have changed values
;	after return
;subroutines utilized:
;	PutNumU
;****************************************************************
			PUSH	{LR,R0-R1}			
			MOVS	R1,#2_00001111	;put mask into r1-1
			SXTB	R1,R1			;sign extend to finish
			ANDS	R0,R0,R1		;AND with the mask to preserve only LSB
			BL		PutNumU			;use PutNumU to print the byte
			POP		{PC,R0-R1}			
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
;>>>>>   end subroutine code <<<<<
            ALIGN
;**********************************************************************
;Constants
            AREA    MyConst,DATA,READONLY
			EXPORT PWM_duty_table_0 ;include if accessed from C
			EXPORT DAC0_table_0 ;make available to C program
;>>>>> begin constants here <<<<<
PWM_duty_table 						;not sure if this label useful
PWM_duty_table_0 ;include if accessed from C
;LED brightness from dimmest (1) to brightest (5)
			 DCW 	PWM_DUTY_MAX 				;0% bright (off)
			 DCW 	(75 * PWM_DUTY_MAX / 100) 	;25% bright
			 DCW 	(50 * PWM_DUTY_MAX / 100) 	;50% bright
			 DCW 	(25 * PWM_DUTY_MAX / 100) 	;75% bright
			 DCW 	0 							;100% bright (always on)
			 ALIGN		;added by me- not sure if necessary
;ROM lookup table of digital values for conversion to analog
DAC0_table_0
DAC0_table
			DCW 	(DAC0_STEPS / (LED_LEVELS * 2))
			DCW 	((DAC0_STEPS * 3) / (LED_LEVELS * 2))
			DCW 	((DAC0_STEPS * 5) / (LED_LEVELS * 2))
			DCW 	((DAC0_STEPS * 7) / (LED_LEVELS * 2))
			DCW 	((DAC0_STEPS * 9) / (LED_LEVELS * 2))
;>>>>>   end constants here <<<<<
			ALIGN	;added by atticus as precaution
;**********************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
TxQRecord	SPACE	18			;record structure for TxQBuffer
TxQBuffer	SPACE	80			;80 character transmit queue buffer
			ALIGN
RxQRecord	SPACE	18			;record structure for RxQBuffer
RxQBuffer	SPACE	80			;80 character recieve queue buffer
			ALIGN
;following from Lab 7
QRecord 	SPACE	Q_REC_SZ	;allocate 18 bytes to variable QRecord
QBuffer		SPACE 	Q_BUF_SZ	;allocate 4	 bytes to variable QBuffer
			ALIGN
lenOpStr	SPACE	2				;length of the operational string
;>>>>>   end variables here <<<<<
            END
