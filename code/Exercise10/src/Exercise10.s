			TTL Lab Exercise Ten
			SUBT Timer Driver Input Timing
;****************************************************************
;Implements a timer driver for the KL05Z board. It uses the KL05 
; periodic interrupt timer (PIT) with channel zero. 
;Timing measurements accurate to within 0.01 s, (10 ms). 
; Uses interrupt service routine (ISR) and driver program
;---------------------------------------------------------------
;Name:  <Atticus Russell>
;Date:  <4/8/2021>
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
            GET	MKL05Z4.s     ;Included by start.s
				;NOT SURE IF THIS WILL WORK V!
			GET	NVIC+PIT+UART0_EQUates.s	;get some equates??
            OPT  1   ;Turn on listing
;****************************************************************
;EQUates
NamePromptSize	EQU	16;
CountSuffixSize EQU 8;
DatePromptSize	EQU	13;
TAPromptSize	EQU	34;
EndMessageSize	EQU	20;
	
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

;****************************************************************
;Program
;Linker requires Reset_Handler
            AREA    MyCode,CODE,READONLY
            ENTRY
            EXPORT  Reset_Handler
            IMPORT  Startup
Reset_Handler  PROC  {}
main
;---------------------------------------------------------------
;Mask interrupts
            CPSID   I
;KL05 system startup with 48-MHz system clock
            BL      Startup
;---------------------------------------------------------------
;>>>>> begin main program code <<<<
			BL		Init_UART0_IRQ
			MOVS	R7,#0
mainLoop
;Clear runstopwatch -> set equal 0
			LDR		R0,=RunStopWatch
			MOVS	R1,#0
			STRB	R1,[R0,#0]
			LDR		R0,=Count
			STR 	R1,[R0,#0]
;init PIT to initialize the KL05 PIT to generate an interrupt 
;from channel zero every 0.01s.
			BL		Init_PIT_IRQ
;Unmask interrupts (CPSIE I
			CPSIE 	I	
;prompt user PutStr
			CMP		R7,#0
			BNE		tryDatePrompt
			LDR		R0,=Name_Prompt
			MOVS	R1,#NamePromptSize
			B		printThisPrompt
tryDatePrompt
			CMP		R7,#1
			BNE		tryTAPrompt
			LDR		R0,=Date_Prompt
			MOVS	R1,#DatePromptSize
			B		printThisPrompt
tryTAPrompt
			CMP		R7,#2
			BNE		endMain
			LDR		R0,=TA_Prompt
			MOVS	R1,#TAPromptSize

printThisPrompt
			BL		PutStringSB
			BL		CRLF
			MOVS	R0,#'>'
			BL		PutChar
;clear Count (the global var)
			MOVS	R1,#0
			LDR		R0,=Count
			STR 	R1,[R0,#0]
;Set RSW to 1
			MOVS	R1,#1
			LDR		R0,=RunStopWatch
			STRB 	R1,[R0,#0]
;call getString for user input and echo to terminal
			LDR		R0,=StringMem
			MOVS	R1,#80
			BL		GetStringSB
			BL		CRLF		;return
			MOVS	R0,#'<'
			BL		PutChar
;Clear RSW
			LDR		R0,=RunStopWatch
			MOVS	R1,#0
			STRB	R1,[R0,#0]
;Print the time -> print the Count
			LDR		R1,=Count
			LDR		R0,[R1,#0]
			BL		PutNumU
;print the count suffix
			LDR		R0,=Count_Suffix
			MOVS	R1,#CountSuffixSize
			BL		PutStringSB
			BL		CRLF
			ADDS	R7,R7,#1		;increment loop counter
			B 		mainLoop
;print goodbye and end
endMain
			LDR		R0,=EndMessage
			MOVS	R1,#EndMessageSize
			BL		PutStringSB
;>>>>>   end main program code <<<<<
;Stay here
            B       .
            ENDP
;>>>>> begin subroutine code <<<<<
PIT_ISR	PROC	{R0-R14}
;****************************************************************
;ISR for the PIT module. On a PIT interrupt, if the (byte) variable 
;RunStopWatch is not zero, PIT_ISR increments the (word) variable Count; 
;otherwise it leaves Count unchanged. In either case, make sure the ISR clears 
;the interrupt condition before exiting. You must write the ISR so that no 
;registers have changed value after return. 
;(Note: the Cortex-M0+ automatically preserves R0�R3, 
;R12, LR, PC, and PSR for ISRs.)
;****************************************************************
			CPSID	I 	;Mask interrupts
						;push any registers used except R0�R3, R12
;							if (RunStopWatch) {
			LDR		R0,=RunStopWatch	;load addr into r0
			LDRB	R0,[R0,#0]			;load value RSW
			CMP		R0,#0				;compare with zero
			BEQ		PITclr_int
;							Increment Count
			LDR		R0,=Count
			LDR		R1,[R0,#0]
			ADDS	R1,R1,#1
			STR		R1,[R0,#0]
;							}
;							clear interrupt
PITclr_int
			LDR 	R0,=PIT_CH0_BASE 
			MOVS	R1,#PIT_TFLG_TIF_MASK
			STR		R1,[R0,#PIT_TFLG_OFFSET]
;							return			
;			 
						; Pop any registers pushed above 
			CPSIE	I   ; Unmask other interrupts ;CPSIE I 
			BX		LR	; Return 
			;POP		{PC}
			ENDP

Init_PIT_IRQ		PROC	{R0-R14}
;****************************************************************
;Description:
;	initialize the KL05 as discussed in class and 
;	presented in the class notes for an interrupt every 0.01 s from PIT channel
;	0. You must write the subroutine so that no registers have changed value 
;	after return. 
;Input Parameter:
;	none
;Output Parameter:
;	none
;modified registers:
;	none
;subroutines utilized:
;	
;****************************************************************
;			Push any registers that will be modified
			PUSH	{R0-R3}
;			Enable module clock for PIT (SIM_SCGC6)
			LDR 	R0,=SIM_SCGC6
			LDR 	R1,=SIM_SCGC6_PIT_MASK
			LDR 	R2,[R0,#0] ;current SIM_SCGC6 value
			ORRS 	R2,R2,R1 ;only PIT bit set
			STR 	R2,[R0,#0] ;update SIM_SCGC6
;			Disable PIT timer 0 (PIT_TCTRL0) 
			LDR 	R0,=PIT_CH0_BASE
			LDR 	R1,=PIT_TCTRL_TEN_MASK 
			LDR 	R2,[R0,#PIT_TCTRL_OFFSET] 
			BICS	R2,R2,R1
			STR		R2,[R0,#PIT_TCTRL_OFFSET]
;			Set PIT IRQ priority to 0 (NVIC_IPR5)
		;PIT_IRQ_PRI EQU 0 ;Highest priority
			;Set PIT interrupt priority
			LDR 	R0,=PIT_IPR
			LDR 	R1,=(NVIC_IPR_PIT_MASK)
			;LDR Rk,=(PIT_IRQ_PRI << PIT_PRI_POS)
			LDR 	R2,[R0,#0]
			BICS 	R2,R2,R1	;bit clear sets it to zero
			;ORRS Rl,Rl,Rk		;general case to set a bit we need both ^
			STR 	R1,[R0,#0]
;			Clear any pending PIT interrupts (NVIC_ICPR)
			LDR	 	R0,=NVIC_ICPR
			LDR 	R1,=PIT_IRQ_MASK
			STR 	R1,[R0,#0]
;			Unmask PIT interupts (NVIC_ISER)
			;Unmask PIT interrupts
			LDR	 	R0,=NVIC_ISER
			LDR 	R1,=PIT_IRQ_MASK
			STR 	R1,[R0,#0]
;			Enable PIT module (PIT_MCR)
			LDR 	R0,=PIT_BASE 
			LDR 	R1,=PIT_MCR_EN_FRZ 
			STR 	R1,[R0,#PIT_MCR_OFFSET]
;			Set PIT timer 0 period for 0.01 s (PIT_LDVAL0)
			LDR 	R0,=PIT_CH0_BASE
			LDR		R1,=PIT_LDVAL_10ms
			STR 	R1,[R0,#PIT_LDVAL_OFFSET]
;			Enable PIT timer 0 with interrupt (PIT_TCTRL0)
			LDR 	R0,=PIT_CH0_BASE
			MOVS 	R1,#PIT_TCTRL_CH_IE
			STR 	R1,[R0,#PIT_TCTRL_OFFSET]
;			Pop any registers pushed
			POP		{R0-R3}
;			Return
			BX		LR
			ENDP
			
UART0_ISR	PROC	{R0-R14}
;****************************************************************
;ISR that handles UART0 transmit and recieve interrupts. You 
;must write the ISR so that no registers have changed value after 
;return. (Note: the Cortex-M0+ automatically preserves R0�R3, 
;R12, LR, PC, and PSR for ISRs.)
;****************************************************************
			CPSID	I 	;Mask interrupts
			PUSH	{LR} 
						;push any registers used except R0�R3, R12
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
;	
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

Dequeue		PROC	{R2-R14}
;****************************************************************
;Description:
;	Attempts to get a character from the queue whose record structure�s
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
				
Enqueue		PROC	{R2-R14}
;****************************************************************
;Description:
;	Attempts to put a character in the queue whose queue record structure�s
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
			;LDR		R0,=lenOpStr;load address to store the length of op str in
			CMP		R5,R4		;if length greater than max length
			BLE		notExceed
			MOVS	R5,R4		;store length as max length
notExceed
			;STRB	R5,[R0,#0]	;store the length of the op str 
			MOVS	R0,#NULL	;move equate null into r0 
			STRB	R0,[R3,R5]  ;store null in addr r0 inc offset			
			POP		{PC,R0-R5}
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
            DCD    UART0_ISR      ;28:UART0 (status; error)
            DCD    Dummy_Handler      ;29:(reserved)
            DCD    Dummy_Handler      ;30:(reserved)
            DCD    Dummy_Handler      ;31:ADC0
            DCD    Dummy_Handler      ;32:CMP0
            DCD    Dummy_Handler      ;33:TPM0
            DCD    Dummy_Handler      ;34:TPM1
            DCD    Dummy_Handler      ;35:(reserved)
            DCD    Dummy_Handler      ;36:RTC (alarm)
            DCD    Dummy_Handler      ;37:RTC (seconds)
            DCD    PIT_ISR		      ;38:PIT
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
Name_Prompt DCB		"Enter your name."
Count_Suffix DCB	" x 0.01 s"
Date_Prompt	DCB		"Enter the Date"
TA_Prompt	DCB		"Enter the last name of a 250 lab TA."
EndMessage	DCB		"Thank you.  Goodbye!"
;>>>>>   end constants here <<<<<
            ALIGN
;****************************************************************
;Variables
            AREA    MyData,DATA,READWRITE
;>>>>> begin variables here <<<<<
Count			SPACE	4	;word
RunStopWatch	SPACE	1	;byte variable (boolean)
			ALIGN		
TxQRecord	SPACE	18			;record structure for TxQBuffer
TxQBuffer	SPACE	80			;80 character transmit queue buffer
			ALIGN
RxQRecord	SPACE	18			;record structure for RxQBuffer
RxQBuffer	SPACE	80			;80 character recieve queue buffer
			ALIGN
;following from Lab 7
QRecord 	SPACE	18	;allocate 18 bytes to variable QRecord
QBuffer		SPACE 	4	;allocate 4	 bytes to variable QBuffer
			ALIGN
StringMem	SPACE	80
;>>>>>   end variables here <<<<<
            ALIGN
            END
