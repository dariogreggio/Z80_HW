// Local Header Files
#include <stdlib.h>
#include <string.h>
#include <xc.h>
#include "Z80_hw_PIC.h"

#include <sys/attribs.h>
#include <sys/kmem.h>




// PIC32MZ1024EFE064 Configuration Bit Settings

// 'C' source line config statements

// DEVCFG3
// USERID = No Setting
#pragma config FMIIEN = OFF             // Ethernet RMII/MII Enable (RMII Enabled)
#pragma config FETHIO = OFF             // Ethernet I/O Pin Select (Alternate Ethernet I/O)
#pragma config PGL1WAY = ON             // Permission Group Lock One Way Configuration (Allow only one reconfiguration)
#pragma config PMDL1WAY = ON            // Peripheral Module Disable Configuration (Allow only one reconfiguration)
#pragma config IOL1WAY = ON             // Peripheral Pin Select Configuration (Allow only one reconfiguration)
#pragma config FUSBIDIO = ON            // USB USBID Selection (Controlled by the USB Module)

// DEVCFG2
/* Default SYSCLK = 200 MHz (8MHz FRC / FPLLIDIV * FPLLMUL / FPLLODIV) */
//#pragma config FPLLIDIV = DIV_1, FPLLMULT = MUL_50, FPLLODIV = DIV_2
#pragma config FPLLIDIV = DIV_2         // System PLL Input Divider (8x Divider)
#pragma config FPLLRNG = RANGE_34_68_MHZ// System PLL Input Range (34-68 MHz Input)
#pragma config FPLLICLK = PLL_FRC       // System PLL Input Clock Selection (FRC is input to the System PLL)
#pragma config FPLLMULT = MUL_50       // System PLL Multiplier (PLL Multiply by 128)
#pragma config FPLLODIV = DIV_2        // System PLL Output Clock Divider (32x Divider)
#pragma config UPLLFSEL = FREQ_24MHZ    // USB PLL Input Frequency Selection (USB PLL input is 24 MHz)

// DEVCFG1
#pragma config FNOSC = FRCDIV           // Oscillator Selection Bits (Fast RC Osc w/Div-by-N (FRCDIV))
#pragma config DMTINTV = WIN_127_128    // DMT Count Window Interval (Window/Interval value is 127/128 counter value)
#pragma config FSOSCEN = ON             // Secondary Oscillator Enable (Enable SOSC)
#pragma config IESO = ON                // Internal/External Switch Over (Enabled)
#pragma config POSCMOD = OFF            // Primary Oscillator Configuration (Primary osc disabled)
#pragma config OSCIOFNC = OFF           // CLKO Output Signal Active on the OSCO Pin (Disabled)
#pragma config FCKSM = CSECME           // Clock Switching and Monitor Selection (Clock Switch Enabled, FSCM Enabled)
#pragma config WDTPS = PS16384          // Watchdog Timer Postscaler (1:16384)
  // circa 6-7 secondi, 24.7.19
#pragma config WDTSPGM = STOP           // Watchdog Timer Stop During Flash Programming (WDT stops during Flash programming)
#pragma config WINDIS = NORMAL          // Watchdog Timer Window Mode (Watchdog Timer is in non-Window mode)
#pragma config FWDTEN = ON             // Watchdog Timer Enable (WDT Enabled)
#pragma config FWDTWINSZ = WINSZ_25     // Watchdog Timer Window Size (Window size is 25%)
#pragma config DMTCNT = DMT31           // Deadman Timer Count Selection (2^31 (2147483648))
#pragma config FDMTEN = OFF             // Deadman Timer Enable (Deadman Timer is disabled)

// DEVCFG0
#pragma config DEBUG = OFF              // Background Debugger Enable (Debugger is disabled)
#pragma config JTAGEN = OFF             // JTAG Enable (JTAG Disabled)
#pragma config ICESEL = ICS_PGx1        // ICE/ICD Comm Channel Select (Communicate on PGEC1/PGED1)
#pragma config TRCEN = OFF              // Trace Enable (Trace features in the CPU are disabled)
#pragma config BOOTISA = MIPS32         // Boot ISA Selection (Boot code and Exception code is MIPS32)
#pragma config FECCCON = OFF_UNLOCKED   // Dynamic Flash ECC Configuration (ECC and Dynamic ECC are disabled (ECCCON bits are writable))
#pragma config FSLEEP = OFF             // Flash Sleep Mode (Flash is powered down when the device is in Sleep mode)
#pragma config DBGPER = PG_ALL          // Debug Mode CPU Access Permission (Allow CPU access to all permission regions)
#pragma config SMCLR = MCLR_NORM        // Soft Master Clear Enable bit (MCLR pin generates a normal system Reset)
#pragma config SOSCGAIN = GAIN_2X       // Secondary Oscillator Gain Control bits (2x gain setting)
#pragma config SOSCBOOST = ON           // Secondary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config POSCGAIN = GAIN_2X       // Primary Oscillator Gain Control bits (2x gain setting)
#pragma config POSCBOOST = ON           // Primary Oscillator Boost Kick Start Enable bit (Boost the kick start of the oscillator)
#pragma config EJTAGBEN = NORMAL        // EJTAG Boot (Normal EJTAG functionality)

// DEVCP0
#pragma config CP = OFF                 // Code Protect (Protection Disabled)

// SEQ3

// DEVADC0

// DEVADC1

// DEVADC2

// DEVADC3

// DEVADC4

// DEVADC7



const char CopyrightString[]= {'Z','8','0','C','P','U',' ','E','m','u','l','a','t','o','r',' ','v',
	VERNUMH+'0','.',VERNUML/10+'0',(VERNUML % 10)+'0',' ','-',' ', '0','6','/','0','3','/','2','2', 0 };

const char Copyr1[]="(C) Dario's Automation 2019-2022 - G.Dar\xd\xa\x0";



// Global Variables:
BOOL debug;


int main(void) {

  // disable JTAG port
//  DDPCONbits.JTAGEN = 0;
  
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  RPB15Rbits.RPB15R = 4;        // Assign RPB15 as U6TX, pin 30
  U6RXRbits.U6RXR = 2;      // Assign RPB14 as U6RX, pin 29 
#ifdef USA_SPI_HW
  RPG8Rbits.RPG8R = 6;        // Assign RPG8 as SDO2, pin 6
//  SDI2Rbits.SDI2R = 1;        // Assign RPG7 as SDI2, pin 5
#endif
  RPD5Rbits.RPD5R = 12;        // Assign RPD5 as OC1, pin 53; anche vaga uscita audio :)
  CFGCONbits.IOLOCK = 1;      // PPS Lock

//  PPSOutput(4,RPC4,OC1);   //buzzer 4KHz , qua rimappabile 

#ifdef DEBUG_TESTREFCLK
// test REFCLK
  PPSOutput(4,RPC4,REFCLKO2);   // RefClk su pin 1 (RG15, buzzer)
	REFOCONbits.ROSSLP=1;
	REFOCONbits.ROSEL=1;
	REFOCONbits.RODIV=0;
	REFOCONbits.ROON=1;
	TRISFbits.TRISF3=1;
#endif

//	PPSLock;

   // Disable all Interrupts
  __builtin_disable_interrupts();
  
//  SPLLCONbits.PLLMULT=10;
  
  OSCTUN=0;
  OSCCONbits.FRCDIV=0;
  
  // Switch to FRCDIV, SYSCLK=8MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x00; // FRC
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
    // At this point, SYSCLK is ~8MHz derived directly from FRC
 //http://www.microchip.com/forums/m840347.aspx
  // Switch back to FRCPLL, SYSCLK=200MHz
  SYSKEY=0xAA996655;
  SYSKEY=0x556699AA;
  OSCCONbits.NOSC=0x01; // SPLL
  OSCCONbits.OSWEN=1;
  SYSKEY=0x33333333;
  while(OSCCONbits.OSWEN) {
    Nop();
    }
  // At this point, SYSCLK is ~200MHz derived from FRC+PLL
//***
  mySYSTEMConfigPerformance();
  //myINTEnableSystemMultiVectoredInt(();

    
	TRISB=0b0000000000000000;			// A0..15
	TRISC=0b0000000000000000;
	TRISD=0b0000000000001100;			// 
	TRISE=0b0000000000000000;			// D0..7
	TRISF=0b0000000000000000;			// 
	TRISG=0b0000000000000000;			// 

  ANSELB=0;
  ANSELE=0;
  ANSELG=0;

//  CNPUDbits.CNPUD2=1;   // switch/pulsanti

      
  
  Timer_Init();
  PWM_Init();
  UART_Init(/*230400L*/ 115200L);

  myINTEnableSystemMultiVectoredInt();
  __delay_ms(50); 



  Emulate(0);

  }


void mySYSTEMConfigPerformance(void) {
  unsigned PLLIDIV;
  unsigned PLLMUL;
  unsigned PLLODIV;
  float CLK2USEC;
  unsigned SYSCLK;
  static char PLLODIVVAL[]={
    2,2,4,8,16,32,32,32
    };

  PLLIDIV=SPLLCONbits.PLLIDIV+1;
  PLLMUL=SPLLCONbits.PLLMULT+1;
  PLLODIV=PLLODIVVAL[SPLLCONbits.PLLODIV];

  SYSCLK=(FCY*PLLMUL)/(PLLIDIV*PLLODIV);
  CLK2USEC=SYSCLK/1000000.0f;

  SYSKEY = 0x0;
  SYSKEY = 0xAA996655;
  SYSKEY = 0x556699AA;

  if(SYSCLK<=60000000)
    PRECONbits.PFMWS=0;
  else if(SYSCLK<=120000000)
    PRECONbits.PFMWS=1;
  else if(SYSCLK<=200000000)
    PRECONbits.PFMWS=2;
  else if(SYSCLK<=252000000)
    PRECONbits.PFMWS=4;
  else
    PRECONbits.PFMWS=7;

  PRECONbits.PFMSECEN=0;    // non c'è nella versione "2019" ...
  PRECONbits.PREFEN=0x1;

  SYSKEY = 0x0;
  }

void myINTEnableSystemMultiVectoredInt(void) {

  PRISS = 0x76543210;
  INTCONSET = _INTCON_MVEC_MASK /*0x1000*/;    //MVEC
  asm volatile ("ei");
  //__builtin_enable_interrupts();
  }


void __attribute__((used)) __delay_us(unsigned int usec) {
  unsigned int tWait, tStart;

  tWait=(GetSystemClock()/2000000)*usec;
  tStart=_mfc0(9,0);
  while((_mfc0(9,0)-tStart)<tWait)
    ClrWdt();        // wait for the time to pass
  }

void __attribute__((used)) __delay_ms(unsigned int ms) {
  
  for(;ms;ms--)
    __delay_us(1000);
  }



void Timer_Init(void) {

  T2CON=0;
  T2CONbits.TCS = 0;                  // clock from peripheral clock
  T2CONbits.TCKPS = 7;                // 1:256 prescaler (pwm clock=390625Hz)
  T2CONbits.T32 = 0;                  // 16bit
//  PR2 = 2000;                         // rollover every n clocks; 2000 = 50KHz
  PR2 = 65535;                         // per ora faccio solo onda quadra, v. SID
  T2CONbits.TON = 1;                  // start timer per PWM
  
  // TIMER 3 INITIALIZATION (TIMER IS USED AS A TRIGGER SOURCE FOR ALL CHANNELS).
  T3CON=0;
  T3CONbits.TCS = 0;                  // clock from peripheral clock
  T3CONbits.TCKPS = 4;                // 1:16 prescaler
  PR3 = 3906;                         // rollover every n clocks; 
  T3CONbits.TON = 1;                  // start timer 

  IPC3bits.T3IP=4;            // set IPL 4, sub-priority 2??
  IPC3bits.T3IS=0;
  IEC0bits.T3IE=1;             // enable Timer 3 interrupt se si vuole

	}

void PWM_Init(void) {

  CFGCONbits.OCACLK=0;      // sceglie timer per PWM
  
  OC1CON = 0x0006;      // TimerX ossia Timer2; PWM mode no fault; Timer 16bit, TimerX
//  OC1R    = 500;		 // su PIC32 è read-only!
//  OC1RS   = 1000;   // 50%, relativo a PR2 del Timer2
  OC1R    = 32768;		 // su PIC32 è read-only!
  OC1RS   = 0;        // per ora faccio solo onda quadra, v. SID reg. 0-1
  OC1CONbits.ON = 1;   // on

  }

void UART_Init(DWORD baudRate) {
  
  U6MODE=0b0000000000001000;    // BRGH=1
  U6STA= 0b0000010000000000;    // TXEN
  DWORD baudRateDivider = ((GetPeripheralClock()/(4*baudRate))-1);
  U6BRG=baudRateDivider;
  U6MODEbits.ON=1;
  
#if 0
  ANSELDCLR = 0xFFFF;
  CFGCONbits.IOLOCK = 0;      // PPS Unlock
  RPD11Rbits.RPD11R = 3;        // Assign RPD11 as U1TX
  U1RXRbits.U1RXR = 3;      // Assign RPD10 as U1RX
  CFGCONbits.IOLOCK = 1;      // PPS Lock

  // Baud related stuffs.
  U1MODEbits.BRGH = 1;      // Setup High baud rates.
  unsigned long int baudRateDivider = ((GetSystemClock()/(4*baudRate))-1);
  U1BRG = baudRateDivider;  // set BRG

  // UART Configuration
  U1MODEbits.ON = 1;    // UART1 module is Enabled
  U1STAbits.UTXEN = 1;  // TX is enabled
  U1STAbits.URXEN = 1;  // RX is enabled

  // UART Rx interrupt configuration.
  IFS1bits.U1RXIF = 0;  // Clear the interrupt flag
  IFS1bits.U1TXIF = 0;  // Clear the interrupt flag

  INTCONbits.MVEC = 1;  // Multi vector interrupts.

  IEC1bits.U1RXIE = 1;  // Rx interrupt enable
  IEC1bits.U1EIE = 1;
  IPC7bits.U1IP = 7;    // Rx Interrupt priority level
  IPC7bits.U1IS = 3;    // Rx Interrupt sub priority level
#endif
  }

char BusyUART1(void) {
  
  return(!U6STAbits.TRMT);
  }

void putsUART1(unsigned int *buffer) {
  char *temp_ptr = (char *)buffer;

    // transmit till NULL character is encountered 

  if(U6MODEbits.PDSEL == 3)        /* check if TX is 8bits or 9bits */
    {
        while(*buffer) {
            while(U6STAbits.UTXBF); /* wait if the buffer is full */
            U6TXREG = *buffer++;    /* transfer data word to TX reg */
        }
    }
  else {
        while(*temp_ptr) {
            while(U6STAbits.UTXBF);  /* wait if the buffer is full */
            U6TXREG = *temp_ptr++;   /* transfer data byte to TX reg */
        }
    }
  }

unsigned int ReadUART1(void) {
  
  if(U6MODEbits.PDSEL == 3)
    return (U6RXREG);
  else
    return (U6RXREG & 0xFF);
  }

void WriteUART1(unsigned int data) {
  
  if(U6MODEbits.PDSEL == 3)
    U6TXREG = data;
  else
    U6TXREG = data & 0xFF;
  }

void __ISR(_UART1_RX_VECTOR) UART1_ISR(void) {
  
  LATDbits.LATD4 ^= 1;    // LED to indicate the ISR.
  char curChar = U1RXREG;
  IFS3bits.U1RXIF = 0;  // Clear the interrupt flag!
  }



void __ISR(_TIMER_3_VECTOR,ipl4AUTO) TMR_ISR(void) {
// https://www.microchip.com/forums/m842396.aspx per IRQ priority ecc
  static BYTE divider,divider2;
  static WORD dividerTim;
  int i;

//  LED2 ^= 1;      // check timing: 1600Hz, 9/11/19 (fuck berlin day))
  
  divider++;
  if(divider>=32) {   // 50 Hz per TOD
    divider=0;
//    CIA1IRQ=1;
    }


    
  IFS0CLR = _IFS0_T3IF_MASK;
  }

// ---------------------------------------------------------------------------------------
// declared static in case exception condition would prevent
// auto variable being created
static enum {
	EXCEP_IRQ = 0,			// interrupt
	EXCEP_AdEL = 4,			// address error exception (load or ifetch)
	EXCEP_AdES,				// address error exception (store)
	EXCEP_IBE,				// bus error (ifetch)
	EXCEP_DBE,				// bus error (load/store)
	EXCEP_Sys,				// syscall
	EXCEP_Bp,				// breakpoint
	EXCEP_RI,				// reserved instruction
	EXCEP_CpU,				// coprocessor unusable
	EXCEP_Overflow,			// arithmetic overflow
	EXCEP_Trap,				// trap (possible divide by zero)
	EXCEP_IS1 = 16,			// implementation specfic 1
	EXCEP_CEU,				// CorExtend Unuseable
	EXCEP_C2E				// coprocessor 2
  } _excep_code;

static unsigned int _epc_code;
static unsigned int _excep_addr;

void __attribute__((weak)) _general_exception_handler(uint32_t __attribute__((unused)) code, uint32_t __attribute__((unused)) address) {
  }

void __attribute__((nomips16,used)) _general_exception_handler_entry(void) {
  
	asm volatile("mfc0 %0,$13" : "=r" (_epc_code));
	asm volatile("mfc0 %0,$14" : "=r" (_excep_addr));

	_excep_code = (_epc_code & 0x0000007C) >> 2;

  _general_exception_handler(_excep_code, _excep_addr);

	while (1)	{
		// Examine _excep_code to identify the type of exception
		// Examine _excep_addr to find the address that caused the exception
    }
  }


