/*********************************************************************/
/* <Lab Ex. 11: PWM and LEDs with Mixed Assembly Language and C>     */
/* Name:  <Atticus Russell>                                          */
/* Date:  <April 26,2021>                                            */
/* Class:  CMPE 250                                                  */
/* Section:  <Section 01L1, Thursday 2:00-3:55>                      */
/*-------------------------------------------------------------------*/
/* Template:  R. W. Melton                                           */
/*            November 2, 2020                                       */
/*********************************************************************/
#include <MKL05Z4.h>
#include "Exercise11_C.h"

#define FALSE      (0)
#define TRUE       (1)

#define MAX_STRING (79u)

#define PWM_FREQ        (60u)
#define TPM_SOURCE_FREQ (47972352u)
#define TPM_PWM_PERIOD ((TPM_SOURCE_FREQ / (1 << TPM_SC_PS_VAL)) / \
                            PWM_FREQ)

#define LED_LEVELS (5u)

/* ADC0 */
#define ADC_MAX (0x3FFu)
#define ADC_VALUES (0x400u)
/* ------------------------------------------------------------------*/
/* ADC0_CFG1:  ADC0 configuration register 1                         */
/*  0-->31-8:(reserved):read-only:0                                  */
/*  1-->   7:ADLPC=ADC low-power configuration                       */
/* 10--> 6-5:ADIV=ADC clock divide select                            */
/*           Internal ADC clock = input clock / 2^ADIV               */
/*  1-->   4:ADLSMP=ADC long sample time configuration               */
/* 10--> 3-2:MODE=conversion mode selection                          */
/*           00=single-ended 8-bit conversion                        */
/*           01=single-ended 12-bit conversion                       */
/*           10=single-ended 10-bit conversion                       */
/*           11=(reserved) do not use this value                     */
/* 01--> 1-0:ADICLK=ADC input clock select                           */
/*           00=bus clock                                            */
/*           01=bus clock / 2                                        */
/*           10=alternate clock (ALTCLK)                             */
/*           11=asynchronous clock (ADACK)                           */
/* BUSCLK = CORECLK / 2 = FLLCLK / 2                                 */
/* FLLCLK is ~48 MHz                                                 */
/* FLLCLK is 47972352 Hz                                             */
/* BUSCLK is ~24 MHz                                                 */
/* BUSCLK is 23986176 Hz                                             */
/* ADCinCLK is BUSCLK / 2 = ~12 MHz                                  */
/* BUSCLK is 11993088 Hz                                             */
/* ADCCLK is ADCinCLK / 4 = ~3 MHz                                   */
/* ADCCLK is 2998272 Hz                                              */
#define ADC0_CFG1_ADIV_BY2 (2u)
#define ADC0_CFG1_MODE_SGL10 (2u)
#define ADC0_CFG1_ADICLK_BUSCLK_DIV2 (1u)
#define ADC0_CFG1_LP_LONG_SGL10_3MHZ (ADC_CFG1_ADLPC_MASK | \
              (ADC0_CFG1_ADIV_BY2 << ADC_CFG1_ADIV_SHIFT) | \
                                     ADC_CFG1_ADLSMP_MASK | \
            (ADC0_CFG1_MODE_SGL10 << ADC_CFG1_MODE_SHIFT) | \
                               ADC0_CFG1_ADICLK_BUSCLK_DIV2)
/*-------------------------------------------------------------*/
/* ADC0_CFG2:  ADC0 configuration register 2                   */
/*  0-->31-8:(reserved):read-only:0                            */
/*  0--> 7-5:(reserved):read-only:0                            */
/*  0-->   4:MUXSEL=ADC mux select:  A channel                 */
/*  0-->   3:ADACKEN=ADC asynchronous clock output enable      */
/*  0-->   2:ADHSC=ADC high-speed configuration:  normal       */
/* 00--> 1-0:ADLSTS=ADC long sample time select (ADK cycles)   */
/*           default longest sample time:  24 total ADK cycles */
#define ADC0_CFG2_CHAN_A_NORMAL_LONG (0x00u)
/*---------------------------------------------------------   */
/* ADC0_SC1:  ADC0 channel status and control register 1      */
/*     0-->31-8:(reserved):read-only:0                        */
/*     0-->   7:COCO=conversion complete flag (read-only)     */
/*     0-->   6:AIEN=ADC interrupt enabled                    */
/*     0-->   5:(reserved):should not be changed              */
/* 00101--> 4-0:ADCH=ADC input channel select (5 = DAC0_OUT) */
#define ADC0_SC1_ADCH_AD5 (5u)
#define ADC0_SC1_SGL_DAC0 (ADC0_SC1_ADCH_AD5)
/*-----------------------------------------------------------*/
/* ADC0_SC2:  ADC0 status and control register 2             */
/*  0-->31-8:(reserved):read-only:0                          */
/*  0-->   7:ADACT=ADC conversion active                     */
/*  0-->   6:ADTRG=ADC conversion trigger select:  software  */
/*  0-->   5:ACFE=ADC compare function enable                */
/*  X-->   4:ACFGT=ADC compare function greater than enable  */
/*  0-->   3:ACREN=ADC compare function range enable         */
/*            0=disabled; only ADC0_CV1 compared             */
/*            1=enabled; both ADC0_CV1 and ADC0_CV2 compared */
/*  0-->   2:DMAEN=DMA enable                                */
/* 01--> 1-0:REFSEL=voltage reference selection:  VDDA       */
#define ADC0_SC2_REFSEL_VDDA (1u)
#define ADC0_SC2_SWTRIG_VDDA (ADC0_SC2_REFSEL_VDDA)
/*-------------------------------------------------------------*/
/* ADC0_SC3:  ADC0 status and control register 3               */
/* 31-8:(reserved):read-only:0                                 */
/*  0-->   7:CAL=calibration                                   */
/*          write:0=(no effect)                                */
/*                1=start calibration sequence                 */
/*          read:0=calibration sequence complete               */
/*               1=calibration sequence in progress            */
/*  0-->   6:CALF=calibration failed flag                      */
/* 00--> 5-4:(reserved):read-only:0                            */
/*  0-->   3:ADCO=ADC continuous conversion enable             */
/*           (if ADC0_SC3.AVGE = 1)                            */
/*  0-->   2:AVGE=hardware average enable                      */
/* XX--> 1-0:AVGS=hardware average select:  2^(2+AVGS) samples */
#define ADC0_SC3_SINGLE (0u)
#define ADC0_SC3_CAL (ADC_SC3_CAL_MASK | ADC_SC3_AVGE_MASK | \
                                           ADC_SC3_AVGS_MASK)
/*-------------------------------------------------------------*/
/* DAC */
#define DAC_DATH_MIN   (0x00u)
#define DAC_DATL_MIN   (0x00u)
/*---------------------------------------------------------------------*/
/* DAC_C0:  DAC control register 0                                     */
/* 1-->7:DACEN=DAC enabled                                             */
/* 1-->6:DACRFS=DAC reference select VDDA                              */
/* 0-->5:DACTRGSEL=DAC trigger select (X)                              */
/* 0-->4:DACSWTRG=DAC software trigger (X)                             */
/* 0-->3:LPEN=DAC low power control:high power                         */
/* 0-->2:(reserved):read-only:0                                        */
/* 0-->1:DACBTIEN=DAC buffer read pointer top flag interrupt enable    */
/* 0-->0:DACBBIEN=DAC buffer read pointer bottom flag interrupt enable */
#define DAC_C0_ENABLE  (DAC_C0_DACEN_MASK | DAC_C0_DACRFS_MASK)
/*----------------------------------------*/
/* DAC_C1:  DAC control register 1        */
/* 0-->  7:DMAEN=DMA disabled             */
/* 0-->6-3:(reserved)                     */
/* 0-->  2:DACBFMD=DAC buffer mode normal */
/* 0-->  1:(reserved)                     */
/* 0-->  0:DACBFEN=DAC buffer disabled    */
#define DAC_C1_BUFFER_DISABLED  (0x00u)
/*-------------------------------------------------------------*/
/* PORTx_PCRn                                                  */
/*     -->31-25:(reserved):read-only:0                         */
/*    1-->   24:ISF=interrupt status flag (write 1 to clear)   */
/*     -->23-20:(reserved):read-only:0                         */
/* 0000-->19-16:IRQC=interrupt configuration (IRQ/DMA diabled) */
/*     -->15-11:(reserved):read-only:0                         */
/*  ???-->10- 8:MUX=pin mux control                            */
/*     -->    7:(reserved):read-only:0                         */
/*    ?-->    6:DSE=drive strengh enable                       */
/*     -->    5:(reserved):read-only:0                         */
/*    0-->    4:PFE=passive filter enable                      */
/*     -->    3:(reserved):read-only:0                         */
/*    0-->    2:SRE=slew rate enable                           */
/*    0-->    1:PE=pull enable                                 */
/*    x-->    0:PS=pull select                                 */
/* Port B */
/*   DAC0_OUT connects through PTB1 to FRDM-KL05Z J8 pin 2 (D1)*/
/*   This connection is not compatible with UART0 USB COM port */
#define PTB1_MUX_DAC0_OUT (0u << PORT_PCR_MUX_SHIFT)
#define SET_PTB1_DAC0_OUT (PORT_PCR_ISF_MASK | PTB1_MUX_DAC0_OUT)
/*   TPM0_CH3_OUT connects through PTB8                        */
/*     to FRDM-KL05Z red LED connects to PTB8                  */
#define PTB8_MUX_TPM0_CH3_OUT (2u << PORT_PCR_MUX_SHIFT)
#define SET_PTB8_TPM0_CH3_OUT (PORT_PCR_ISF_MASK | \
                               PTB8_MUX_TPM0_CH3_OUT)
/*-------------------------------------------------------*/
/* SIM_SOPT2                                             */
/*   -->31-28:(reserved):read-only:0                     */
/*   -->27-26:UART0SRC=UART0 clock source select         */
/* 01-->25-24:TPMSRC=TPM clock source select (MCGFLLCLK) */
/*   -->23-19:(reserved):read-only:0                     */
/*   -->   18:USBSRC=USB clock source select             */
/*   -->17-16:(reserved):read-only:00                    */
/*   -->15- 8:(reserved):read-only:0                     */
/*   --> 7- 5:CLKOUTSEL=CLKOUT select                    */
/*   -->    4:RTCCLKOUTSEL=RTC clock out select          */
/*   --> 3- 0:(reserved):read-only:0                     */
#define SIM_SOPT2_TPMSRC_MCGFLLCLK (1u << SIM_SOPT2_TPMSRC_SHIFT)
/*------------------------------------------------------------------*/
/* TPMx_CONF:  Configuration (recommended to use default values     */
/*     -->31-28:(reserved):read-only:0                              */
/* 0000-->27-24:TRGSEL=trigger select (external pin EXTRG_IN)       */
/*     -->23-19:(reserved):read-only:0                              */
/*    0-->   18:CROT=counter reload on trigger                      */
/*    0-->   17:CSOO=counter stop on overflow                       */
/*    0-->   16:CSOT=counter stop on trigger                        */
/*     -->15-10:(reserved):read-only:0                              */
/*    0-->    9:GTBEEN=global time base enable                      */
/*     -->    8:(reserved):read-only:0                              */
/*   00-->  7-6:DBGMODE=debug mode (paused in debug)                */
/*    0-->    5:DOZEEN=doze enable                                  */
/*     -->  4-0:(reserved):read-only:0                              */
/* 15- 0:COUNT=counter value (writing any value will clear counter) */
#define TPM_CONF_DEFAULT (0u)
/*------------------------------------------------------------------*/
/* TPMx_CNT:  Counter                                               */
/* 31-16:(reserved):read-only:0                                     */
/* 15- 0:COUNT=counter value (writing any value will clear counter) */
#define TPM_CNT_INIT (0u)
/*------------------------------------------------------------------------*/
/* TPMx_MOD:  Modulo                                                      */
/* 31-16:(reserved):read-only:0                                           */
/* 15- 0:MOD=modulo value (recommended to clear TPMx_CNT before changing) */
/* Period = TPM_SOURCE_FREQ / (2 << TPM_SC_PS_DIV16) / PWM_FREQ           */
#define TPM_PWM_PERIOD ((TPM_SOURCE_FREQ / (1 << TPM_SC_PS_VAL)) / \
                            PWM_FREQ)
#define TPM_MOD_PWM_PERIOD (TPM_PWM_PERIOD - 1)
/*------------------------------------------------------------------*/
/* TPMx_CnSC:  Channel n Status and Control                         */
/* 0-->31-8:(reserved):read-only:0                                  */
/* 0-->   7:CHF=channel flag                                        */
/*              set on channel event                                */
/*              write 1 to clear                                    */
/* 0-->   6:CHIE=channel interrupt enable                           */
/* 1-->   5:MSB=channel mode select B (see selection table below)   */
/* 0-->   4:MSA=channel mode select A (see selection table below)   */
/* 1-->   3:ELSB=edge or level select B (see selection table below) */
/* 0-->   2:ELSA=edge or level select A (see selection table below) */
/* 0-->   1:(reserved):read-only:0                                  */
/* 0-->   0:DMA=DMA enable                                          */
#define TPM_CnSC_PWMH (TPM_CnSC_MSB_MASK | TPM_CnSC_ELSB_MASK)
/*--------------------------------------------------------*/
/* TPMx_CnV:  Channel n Value                             */
/* 31-16:(reserved):read-only:0                           */
/* 15- 0:MOD (all bytes must be written at the same time) */
#define TPM_CnV_PWM_DUTY_LED_OFF (TPM_MOD_PWM_PERIOD)
#define TPM_CnV_PWM_DUTY_LED_ON  (0)
/*--------------------------------------------------------------*/
/* TPMx_SC:  Status and Control                                 */
/*    -->31-9:(reserved):read-only:0                            */
/*   0-->   8:DMA=DMA enable                                    */
/*    -->   7:TOF=timer overflow flag                           */
/*   0-->   6:TOIE=timer overflow interrupt enable              */
/*   0-->   5:CPWMS=center-aligned PWM select (edge align)      */
/*  01--> 4-3:CMOD=clock mode selection (count each TPMx clock) */
/* 100--> 2-0:PS=prescale factor selection                      */
/*    -->        can be written only when counter is disabled   */
#define TPM_SC_CMOD_CLK (1u)
#define TPM_SC_PS_VAL  (0x4u)
#define TPM_SC_CLK_DIV ((TPM_SC_CMOD_CLK << TPM_SC_CMOD_SHIFT) | \
                        TPM_SC_PS_VAL)

const UInt16 *DAC0_table = &DAC0_table_0;
const UInt16 *PWM_duty_table = &PWM_duty_table_0;

void Init_TPM0 (void) {
/*********************************************************************/
/* Initialize TPM0 channel 3 for edge-aligned PWM at 60 Hz           */
/* (8.3333 ms) on Port B Pin 8 (red LED)                             */
/*********************************************************************/
/* Enable TPM0 module clock */ 
SIM->SCGC6 |= SIM_SCGC6_TPM0_MASK; 
/* Enable port B module clock */ 
SIM->SCGC5 |= SIM_SCGC5_PORTB_MASK; 
/* Connect TPM0 channel 3 to port B pin 8 */ 
PORTB->PCR[8] = SET_PTB8_TPM0_CH3_OUT; 
/* Set TPM clock source */ 
SIM->SOPT2 &= ~SIM_SOPT2_TPMSRC_MASK; 
SIM->SOPT2 |= SIM_SOPT2_TPMSRC_MCGFLLCLK; 
/* Set TPM0 configuration register to default values */ 
TPM0->CONF = TPM_CONF_DEFAULT; 
/* Set TPM0 counter modulo value */ 
TPM0->CNT = TPM_CNT_INIT; 
TPM0->MOD = TPM_MOD_PWM_PERIOD - 1; 
/* Set TPM0 counter clock configuration */ 
TPM0->SC = TPM_SC_CLK_DIV; 
/* Set TPM0 channel 3 edge-aligned PWM */ 
TPM0->CONTROLS[3].CnSC = TPM_CnSC_PWMH; 
/* Set TPM0 channel 3 value */ 
TPM0->CONTROLS[3].CnV = TPM_CnV_PWM_DUTY_LED_ON;
} /* Init_TPM0 */ 

void Init_DAC0(void){
/*********************************************************************/
/* Initialize the KL05 DAC0 for continuous (non- buffer) 12-bit 		 */
/* conversion of DAC0_DAT0 to an analog value of range (0, 3.3] V    */                        
/*********************************************************************/
/* Enable DAC0 module clock */ 
SIM->SCGC6 |= SIM_SCGC6_DAC0_MASK; 
/* Set DAC0 DMA disabled and buffer disabled */ 
DAC0->C1 = DAC_C1_BUFFER_DISABLED; 
/* Set DAC0 enabled with VDDA as reference voltage */
/* and read pointer interrupts disabled */ 
DAC0->C0 = DAC_C0_ENABLE; 
/* Set DAC0 output voltage at minimum value */ 
DAC0->DAT[0].DATL = DAC_DATL_MIN; 
DAC0->DAT[0].DATH = DAC_DATH_MIN; 
}

void Init_and_Cal_ADAC0(void){
/*********************************************************************/
/* Initialize and calibrate the KL05 ADC0 for polled conversion of   */
/* single-ended channel 5 (AD5), which is connected to DAC0 output,  */
/* to an unsigned 10-bit value right-justified in ADC0_RA,           */
/* (ADC0->R[0] in C).                                                */
/*********************************************************************/
/* Enable ADC0 module clock */ 
SIM->SCGC6 |= SIM_SCGC6_ADC0_MASK; 
/* Set ADC0 power and timing */ 
ADC0->CFG1 = ADC0_CFG1_LP_LONG_SGL10_3MHZ; 
/* Select channel A and set timing */ 
ADC0->CFG2 = ADC0_CFG2_CHAN_A_NORMAL_LONG; 
/* Select SW trigger and VDDA reference */ 
ADC0->SC2 = ADC0_SC2_SWTRIG_VDDA; 
/* Start calibration */ 
//do { 
		 ADC0->SC3 = ADC0_SC3_CAL; 
		 /* Wait for calibration to complete */ 
	while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK));
  /* Wait for calibration to complete */
  while (ADC0->SC3 & ADC_SC3_CAL_MASK);
  /* Check for calibration error */
  if (ADC0->SC3 & ADC_SC3_CALF_MASK) {
    PutStringSB ("Calibration failed\r\n", MAX_STRING);
  }

  /* Compute and store plus-side calibration value */
  /* Add CLPS and CLP4 through CLP0, divide by 2, and set MSB */
//		 while (ADC0->SC3 & ADC_SC3_CAL_MASK); 
//		 /* Check for calibration failure */ 
//} while (ADC0->SC3 & ADC_SC3_CALF_MASK);
/* Compute and store plus-side calibration value */ 
ADC0->PG = (((Int16) ADC0->CLPS + (Int16) ADC0->CLP4 + 
						 (Int16) ADC0->CLP3 + (Int16) ADC0->CLP2 + 
						 (Int16) ADC0->CLP1 + (Int16) ADC0->CLP0 
						) >> 1) | (Int16) 0x8000; 
/* Select single conversion */ 
ADC0->SC3 = ADC0_SC3_SINGLE;
//}
}

int main (void) {

  __asm("CPSID   I");  /* mask interrupts */
  /* Perfrom all device initialization here */
  /* before unmasking interrupts            */
  Init_TPM0 ();
	/*E1*/
	Init_DAC0();
	/*E2*/
	Init_and_Cal_ADAC0();
  Init_UART0_IRQ ();
  __asm("CPSIE   I");  /* unmask interrupts */

	char numPrompt[32]="Type a number from 1 to 5:  ";
	char ODV[32]			="   Original Digital Value:  0x";
	char NDV[32]			="        New Digital Value:  0x";
	char LEDB[32]			="            LED brighness:  ";


	
	int BrightLevel; 
	//int Segment;
  for (;;) { /* do forever */
			/* Main program loop here */
			PutStringSB(numPrompt, 32);
			char recievedNum = GetChar();
			int recievedNumInt = recievedNum - '0'; /* converts ascii num to int */
			while (recievedNumInt > 5 || recievedNumInt < 1){
						recievedNum = GetChar();
						recievedNumInt = recievedNum - '0'; /* converts ascii num to int */
			}
			PutChar (recievedNum);
			PutChar	('\r');
			PutChar	('\n');
			
			/* E5
			convert the ASCII character to a binary value, subtract one from the 
			value, and use the value as an index for accessing the look-up table of 
			DAC0 digital values */
			int lutResult = DAC0_table [recievedNumInt -1];
			
			/* E6
			Set DAC0 output voltage to midpoint of desired segment of voltage range */ 
			UInt8 setDATL = (UInt8) ((lutResult)& 0xFF); 
			UInt8	setDATH = (UInt8) ((lutResult) >> 8);
			DAC0->DAT[0].DATL =setDATL;
			DAC0->DAT[0].DATH =setDATH;
			
			/* E7 
			want to output result of lut */
			PutStringSB(ODV,32);
			PutNumHex(lutResult);
			/*CRLF*/
			PutChar	('\r');
			PutChar	('\n');
			
			
			/*E8 
			Get a digital value from ADC0 that is the 10-bit analog-to-digital 
			conversion of the voltage output from DAC0. */
			/*Start conversion*/
			ADC0->SC1[0] = ADC0_SC1_SGL_DAC0; 
			/*Wait for conversion to complete*/
			while (!(ADC0->SC1[0] & ADC_SC1_COCO_MASK)); 
			/*Read digital value*/
			UInt16 ADCresult = ADC0->R[0];
			
			/*E9*/
			/*output to terminal*/
			PutStringSB(NDV,32);
			PutNumHex(ADCresult);
			/*CRLF*/
			PutChar	('\r');
			PutChar	('\n');
			
			/*E10
			convert ADC result by scale the 10-bit digital value to a TPM PWM duty 
			count value for a duty cycle between 100% and 0%. One way is first to 
			scale the 10-bit digital value to an index between 0 and 4 (inclusive,
			which corresponds to LED brightness levels between 1 and 5, respectively) 
			and then to use the index to access the look-up table of PWM duty period 
			values */
			
//			if (ADCresult >= 406){
//				Segment = 4;
//			}
//			else if (ADCresult >=399){
//				Segment = 3;
//			}
//			else if (ADCresult >=392){
//				Segment = 2;
//			}
//			else if (ADCresult >=306){
//				Segment = 1;
//			}	
//			else{
//				Segment = 0;
//			}

			UInt32 BrightnessLevel = ((UInt32) ADCresult * LED_LEVELS) / ADC_VALUES;

			BrightLevel = PWM_duty_table [BrightnessLevel];
			
			/* Set duty cycle of TMP0 channel 3 for desired brightness level */ 
			TPM0->CONTROLS[3].CnV = BrightLevel;
			
			
			/* E12
			want to output result of lut */
			PutStringSB(LEDB,32);
			PutChar('0' + BrightnessLevel + 1);
			/*CRLF*/
			PutChar	('\r');
			PutChar	('\n');
		} /* do forever */

  //return (0);
} /* main */
