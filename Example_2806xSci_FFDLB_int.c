//###########################################################################
//
//!  \addtogroup f2806x_example_list
//!  <h1>SCI Digital Loop Back with Interrupts(scia_loopback_interrupts)</h1>
//!
//!  This program uses the internal loop back test mode of the peripheral.
//!  Other then boot mode pin configuration, no other hardware configuration
//!  is required. Both interrupts and the SCI FIFOs are used.
//!
//!  A stream of data is sent and then compared to the received stream.
//!  The SCI-A sent data looks like this: \n
//!  00 01 \n
//!  01 02 \n
//!  02 03 \n
//!  .... \n
//!  FE FF \n
//!  FF 00 \n
//!  etc.. \n
//!  The pattern is repeated forever.
//!
//!  \b Watch \b Variables \n
//!  - \b sdataA , Data being sent
//!  - \b rdataA , Data received
//!  - \b rdata_pointA ,Keep track of where we are in the datastream.
//!    This is used to check the incoming data
//
//###########################################################################
// $TI Release: F2806x C/C++ Header Files and Peripheral Examples V141 $
// $Release Date: January 19, 2015 $
// $Copyright: Copyright (C) 2011-2015 Texas Instruments Incorporated -
//             http://www.ti.com/ ALL RIGHTS RESERVED $
//###########################################################################

#include "DSP28x_Project.h"     // Device Headerfile and Examples Include File

//#define CPU_FREQ    90E6
//#define LSPCLK_FREQ CPU_FREQ/4
//#define SCI_FREQ    100E3
//#define SCI_PRD     (LSPCLK_FREQ/(SCI_FREQ*8))-1

// Prototype statements for functions found within this file.
__interrupt void sciaRxFifoIsr(void);
void scia_fifo_init(void);
void scib_fifo_init(void);	//NO used
void error(void);

void scia_xmit(int a);
void scia_msg(char *msg, int a);	// msg for Tx, a flag for fill the string buffer
void setFluke45(int loopCount, int setFunc);	// delay count, select Fluke45 AI command

// Global variables
//Uint16 sdataA[2];    // Send data for SCI-A
//Uint16 rdataA[2];    // Received data for SCI-A
//Uint16 rdata_pointA; // Used for checking the received data

// for string buffer
Uint16 stCount;	// string pointer
const Uint16 LenOfstring = 32; // total length
char stringrecv[LenOfstring];	// store RX string


void main(void)
{
   Uint16 i;

// Step 1. Initialize System Control:
// PLL, WatchDog, enable Peripheral Clocks
// This example function is found in the F2806x_SysCtrl.c file.
   InitSysCtrl();

// Step 2. Initalize GPIO:
// This example function is found in the F2806x_Gpio.c file and
// illustrates how to set the GPIO to it's default state.
// InitGpio();
// Setup only the GP I/O only for SCI-A and SCI-B functionality
// This function is found in F2806x_Sci.c
   InitSciGpio();

// Step 3. Clear all interrupts and initialize PIE vector table:
// Disable CPU interrupts
   DINT;

// Initialize PIE control registers to their default state.
// The default state is all PIE interrupts disabled and flags
// are cleared.
// This function is found in the F2806x_PieCtrl.c file.
   InitPieCtrl();

// Disable CPU interrupts and clear all CPU interrupt flags:
   IER = 0x0000;
   IFR = 0x0000;

// Initialize the PIE vector table with pointers to the shell Interrupt
// Service Routines (ISR).
// This will populate the entire table, even if the interrupt
// is not used in this example.  This is useful for debug purposes.
// The shell ISR routines are found in F2806x_DefaultIsr.c.
// This function is found in F2806x_PieVect.c.
   InitPieVectTable();

// Interrupts that are used in this example are re-mapped to
// ISR functions found within this file.
   EALLOW;  // This is needed to write to EALLOW protected registers
   PieVectTable.SCIRXINTA = &sciaRxFifoIsr;
//   PieVectTable.SCITXINTA = &sciaTxFifoIsr;	//NO used
   EDIS;   // This is needed to disable write to EALLOW protected registers

// Step 4. Initialize all the Device Peripherals:
// This function is found in F2806x_InitPeripherals.c
// InitPeripherals(); // Not required for this example
   scia_fifo_init();  // Init SCI-A

// Step 5. User specific code, enable interrupts:

// Init send data.  After each transmission this data
// will be updated for the next transmission
//   for(i = 0; i<2; i++)
//   {
//      sdataA[i] = i;
//   }

//   rdata_pointA = sdataA[0];
// Enable interrupts required for this example
   PieCtrlRegs.PIECTRL.bit.ENPIE = 1;   // Enable the PIE block
   PieCtrlRegs.PIEIER9.bit.INTx1=1;     // PIE Group 9, INT1
   PieCtrlRegs.PIEIER9.bit.INTx2=1;     // PIE Group 9, INT2
   IER = 0x100; // Enable CPU INT
   EINT;

// Step 6. IDLE loop. Just sit and loop forever (optional):
    for(;;)
    	for (i=0;i<2;i++)
    		{
    			setFluke45(10,i); // delay count, send TX for Fluke45 AT command = 0 is freq, 1 is vdc
    		}
}

void error(void)
{
   __asm("     ESTOP0"); // Test failed!! Stop!
    for (;;);
}

__interrupt void sciaRxFifoIsr(void)
{
//    Uint16 i;
//    for(i=0;i<2;i++)
//    {
//       rdataA[i]=SciaRegs.SCIRXBUF.all;  // Read data
//    }
//    for(i=0;i<2;i++)                     // Check received data
//    {
//       if(rdataA[i] != ( (rdata_pointA+i) & 0x00FF) ) error();
//    }
//    rdata_pointA = (rdata_pointA+1) & 0x00FF;

	while (SciaRegs.SCIRXST.bit.RXRDY != 0) {}
	stringrecv[stCount++] = SciaRegs.SCIRXBUF.all;  // Read byte

    SciaRegs.SCIFFRX.bit.RXFFOVRCLR=1;   // Clear Overflow flag
    SciaRegs.SCIFFRX.bit.RXFFINTCLR=1;   // Clear Interrupt flag

    PieCtrlRegs.PIEACK.all|=0x100;       // Issue PIE ack
}

void scia_fifo_init()
{
   SciaRegs.SCICCR.all =0x0007;   // 1 stop bit,  No loopback
                                  // No parity,8 char bits,
                                  // async mode, idle-line protocol
   SciaRegs.SCICTL1.all =0x0003;  // enable TX, RX, internal SCICLK,
                                  // Disable RX ERR, SLEEP, TXWAKE

//   SciaRegs.SCICTL2.bit.TXINTENA =1;
   SciaRegs.SCICTL2.bit.RXBKINTENA =1;

   SciaRegs.SCIHBAUD    =0x0001;  	// 9600 baud @LSPCLK = 22.5MHz (90 MHz SYSCLK).
   SciaRegs.SCILBAUD    =0x0024;	// set BRR = 292 -> 0x0124

    SciaRegs.SCICCR.bit.LOOPBKENA =0; // Enable loop back

   // File:spruh18f 13.2.9 SCI FIFO Registers (SCIFFTX, SCIFFRX, SCIFFCT)
   //   SciaRegs.SCIFFTX.all=0xC022;	//echoback is 0xE040
      SciaRegs.SCIFFTX.all=0xE040;
      SciaRegs.SCIFFRX.all=0x0021;	//intc is 0x0022, echoback is 0x2044
      SciaRegs.SCIFFCT.all=0x00;
      SciaRegs.SCICTL1.all =0x0023;     // Relinquish SCI from Reset
      SciaRegs.SCIFFTX.bit.TXFIFOXRESET=1;
      SciaRegs.SCIFFRX.bit.RXFIFORESET=1;
}

// Transmit a character from the SCI
void scia_xmit(int a)
{
    while (SciaRegs.SCIFFTX.bit.TXFFST != 0) {}
    SciaRegs.SCITXBUF=a;

}

void setFluke45(int loopCount, int setFunc)
{
	int i;

	// Fluke 45 AT command
	char *setFreq = "freq\r\0";
	char *setVdc = "vdc\r\0";
	char *askFunc1 = "func1?\r\0";
	char *askMeas = "meas?\r\0";

	// either freq or vdc
	if (setFunc != 1)
	{
		scia_msg(setFreq,0); // send freq to Tx
	} else
	{
		scia_msg(setVdc,0);	// send vdc to Tx
	}

	// Ask Fluke 45 the existing function
	DELAY_US(1000000);
  	scia_msg(askFunc1,1); // "func1? -> Tx, and fill the buffer

  	// Ask Fulke 45 for get the measured value
	DELAY_US(1000000);
  	scia_msg(askMeas,0); // "meas? -> Tx

    // loop for delay
	for (i=0;i<loopCount;i++){DELAY_US(1000000);}
}

void scia_msg(char * msg, int a)
{
    int i;

    //fill bufrer
    if (a !=0)
    {
    	// FILL "$" the store buffer before Transmit
    	i = 0;
    	while(i != LenOfstring)
    	{
    		stringrecv[i] = 0x0024;
    		i++;
    	}
   		stCount =0;	//reset the string pointer to zero, ready for next
    }

    // send msg to TX
    i = 0;
    while(msg[i] != '\0')
    {
        scia_xmit(msg[i]);
        i++;
    }
}

//===========================================================================
// No more.
//===========================================================================

