//---------------------------------------------------------------------------
//
#ifndef _Z80_PIC_INCLUDED
#define _Z80_PIC_INCLUDED

//---------------------------------------------------------------------------


/* check if build is for a real debug tool */
#if defined(__DEBUG) && !defined(__MPLAB_ICD2_) && !defined(__MPLAB_ICD3_) && \
   !defined(__MPLAB_PICKIT2__) && !defined(__MPLAB_PICKIT3__) && \
   !defined(__MPLAB_REALICE__) && \
   !defined(__MPLAB_DEBUGGER_REAL_ICE) && \
   !defined(__MPLAB_DEBUGGER_ICD3) && \
   !defined(__MPLAB_DEBUGGER_PK3) && \
   !defined(__MPLAB_DEBUGGER_PICKIT2) && \
   !defined(__MPLAB_DEBUGGER_PIC32MXSK)
    #warning Debug with broken MPLAB simulator
    #define USING_SIMULATOR
#endif


//#define REAL_SIZE    


#define FCY 205000000ul    //Oscillator frequency; ricontrollato con baud rate, pare giusto così!

#define CPU_CLOCK_HZ             (FCY)    // CPU Clock Speed in Hz
#define CPU_CT_HZ            (CPU_CLOCK_HZ/2)    // CPU CoreTimer   in Hz
#define PERIPHERAL_CLOCK_HZ      (FCY/2 /*100000000UL*/)    // Peripheral Bus  in Hz
#define GetSystemClock()         (FCY)    // CPU Clock Speed in Hz
#define GetPeripheralClock()     (PERIPHERAL_CLOCK_HZ)    // Peripheral Bus  in Hz

#define US_TO_CT_TICKS  (CPU_CT_HZ/1000000UL)    // uS to CoreTimer Ticks
    
#define VERNUML 1
#define VERNUMH 0


typedef char BOOL;
typedef unsigned char UINT8;
typedef unsigned char BYTE;
typedef signed char INT8;
typedef unsigned short int WORD;
typedef unsigned short int SWORD;       // v. C64: con int/32bit è più veloce!
typedef unsigned long UINT32;
typedef unsigned long DWORD;
typedef signed long INT32;
typedef unsigned short int UINT16;
typedef signed int INT16;


#define TRUE 1
#define FALSE 0

#define MAKEWORD(a, b)   ((WORD) (((BYTE) (a)) | ((WORD) ((BYTE) (b))) << 8)) 
#define MAKELONG(a, b)   ((unsigned long) (((WORD) (a)) | ((DWORD) ((WORD) (b))) << 16)) 
#define HIBYTE(w)   ((BYTE) ((((WORD) (w)) >> 8) /* & 0xFF*/)) 
//#define HIBYTE(w)   ((BYTE) (*((char *)&w+1)))		// molto meglio :)
#define HIWORD(l)   ((WORD) (((DWORD) (l) >> 16) & 0xFFFF)) 
#define LOBYTE(w)   ((BYTE) (w)) 
#define LOWORD(l)   ((WORD) (l)) 


void mySYSTEMConfigPerformance(void);
void myINTEnableSystemMultiVectoredInt(void);

#define ReadCoreTimer()                  _CP0_GET_COUNT()           // Read the MIPS Core Timer


#define ClrWdt() { WDTCONbits.WDTCLRKEY=0x5743; }


void Timer_Init(void);
void PWM_Init(void);
void UART_Init(DWORD);
void putsUART1(unsigned int *buffer);

BYTE GetValue(SWORD);
SWORD GetIntValue(SWORD);
void PutValue(SWORD, BYTE);
BYTE InValue(BYTE);
void OutValue(BYTE, BYTE);
int Emulate(int);

#define DELAY_MEM_RD() {Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();}
#define DELAY_MEM_WR() {Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();}
#define DELAY_IO_RD() {Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();}
#define DELAY_IO_WR() {Nop();Nop();Nop();Nop();Nop();Nop();Nop();Nop();}

#define LED1 LATEbits.LATE2
#define LED2 LATEbits.LATE3
#define LED3 LATEbits.LATE4
#define SW1  PORTDbits.RD2
#define SW2  PORTDbits.RD3

#define mRD LATDbits.LATD9
#define mWR LATDbits.LATD10
#define mMREQ LATDbits.LATD11
#define mIOREQ LATDbits.LATD5

#define mIRQ PORTFbits.RF3
#define mNMI PORTFbits.RF1
#define mRESET PORTFbits.RF0

#define mHALT 1
#define mWAIT 1

#define mRFSH LATAbits.LATA1
#define mBUSRQ PORTAbits.RA2
#define mBUSAK LATAbits.LATA0


#endif

