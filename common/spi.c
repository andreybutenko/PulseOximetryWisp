/******************************************************************************
 * @file	spi.c
 *
 * @brief	spi function
 * @author	Aaron Parks, Eve (Yi Zhao) - Sensor Systems Lab (UW)
 * @date	03/29/2015
 *****************************************************************************/

//=============================================================================
//								Includes
//=============================================================================
#include "spi.h"
#include <string.h>
#include "globals.h"
//=============================================================================
//								Functions
//=============================================================================

uint8_t gpRxBuf[SPI_GP_RXBUF_SIZE];
/**
 * Description of state of the SPI module.
 */
static struct {
	BOOL bPortInUse;
	//BOOL bNewDataReceived;
	unsigned int uiCurRx;
	unsigned int uiCurTx;
	unsigned int uiBytesToSend;
	uint8_t *pcRxBuffer;
	uint8_t *pcTxBuffer;
	uint8_t target; //Target is 0(Fram), 1(EPD)
}spiSM;


/**
 *
 * @return success or failure
 *
 * @todo Implement this function
 */
BOOL SPI_initialize() {
	// Hardware peripheral initialization
	PSEL_SPI |= (SPI_CLK_BIT | SPI_MOSI_BIT | SPI_MISO_BIT);

	UCB1CTL1 |= UCSWRST;                                	// reset the module
	UCB1CTL0 = 0x00;
	UCB1CTL1 = 0x00;
	UCB1BR0 = 0;
	UCB1BR1 = 1;
	UCB1CTL0 = UCCKPL | UCMSB | UCMST | UCMODE_0 | UCSYNC;  // 8 bit, 3-wire, sync
	UCB1CTL1 = UCSSEL__SMCLK | UCSWRST;                     // CLK = SMCLK, reset the module
	UCB1BR0 = SPI_BR_LOW;                               	// low byte
	UCB1BR1 = SPI_BR_HIGH;                              	// high byte
	UCB1IE &= ~(UCTXIE | UCRXIE);							// disable interrupts
	UCB1IFG	= 0;
	UCB1CTL1 &= ~UCSWRST;                               	// reset completed
	// State variable initialization
	spiSM.bPortInUse = FALSE;
	//spiSM.bNewDataReceived = FALSE;
	spiSM.uiCurRx = 0;
	spiSM.uiCurTx = 0;
	spiSM.uiBytesToSend = 0;
	memset(gpRxBuf,0x00, SPI_GP_RXBUF_SIZE);
	return SUCCESS;
}

/**
 *
 * @return Success - you were able to get the port. Fail - you don't have the port, so don't use it.
 */
BOOL SPI_acquirePort() {

	if(spiSM.bPortInUse) {
		return SUCCESS;
	} else {
		spiSM.bPortInUse=TRUE;
		return FAIL;
	}

}

/**
 *
 * @return success or fail
 * @todo Make this more robust (don't allow release of port if we don't have it)
 */
BOOL SPI_releasePort() {
	if(spiSM.bPortInUse) {
		spiSM.bPortInUse = FALSE;
		return SUCCESS;
	}
	return FAIL;
}

void SPI_SlaveOn(void){
	switch (spiSM.target){
	 case FRAM_SPI:
		 MEM_SPI_ON();
		 break;
	 case EPD_SPI:
		 EPD_SPI_ON();
		 break;
	 case ACCEL_SPI:
		 ACCEL_SPI_ON();
		 break;
	 default:
		 break;
	}
}

void SPI_SlaveOff(void){
	switch (spiSM.target){
	 case FRAM_SPI:
		 MEM_SPI_OFF();
		 break;
	 case EPD_SPI:
		 EPD_SPI_OFF();
		 break;
	 case ACCEL_SPI:
		 ACCEL_SPI_OFF();
		 break;
	 default:
		 break;
	}
}

/**
 * Engage in a synchronous serial transaction of the specified length.
 * This function blocks until transaction is complete.
 * TODO: This function is especially used for FRAM now, need to modify for EPD later
 * @param txBuf
 * @param size
 * @return success or fail
 */
void SPI_transaction(uint8_t* rxBuf, uint8_t* txBuf, uint16_t size, uint8_t spiSlaveIndex) {
	//while(SPI_acquirePort());
	spiSM.target = spiSlaveIndex;
	spiSM.uiCurRx = 0;
	spiSM.uiCurTx = 0;
	spiSM.uiBytesToSend = size;
	spiSM.pcRxBuffer = rxBuf;
	spiSM.pcTxBuffer = txBuf;
	UCB1IE |= UCRXIE;
	SPI_SlaveOn();
	// Start transmission
	UCB1TXBUF = spiSM.pcTxBuffer[spiSM.uiCurTx];
	// Sleep until receive occurs
	__bis_SR_register(LPM4_bits | GIE);
	SPI_SlaveOff();
	//return SPI_releasePort();
}

void SPI_put_wait(uint8_t c) {
	//NOTE: when SPI is slow, sleep in while loop
	while ((EPD_BUSY_IN & EPD_BUSY_BIT)||!(UCB1IFG&UCTXIFG)){
		delay_us(T_5us);
	};
	UCB1TXBUF = c;
	//delay_us(T_10us);
	//while (!(UCB1IFG&UCTXIFG));
}
/*------------------------------------------EPD_SPI--------------------------------------------------------*/
/**
* @brief: send one byte EPD command, include one byte register_index and one byte data
*/
void epd_spi_send_byte (const uint8_t register_index, const uint8_t register_data) {
	uint8_t  Index[2];
	uint8_t  Data[2];
	const unsigned char RxData[2]	= {0x00,0x00};

	delay_us(T_10us);
	Index[0]= 0x70;
	Index[1]= register_index;
	Data[0]= 0x72;
	Data[1]= register_data;

	SPI_transaction((uint8_t*)RxData,Index,2,EPD_SPI);
	delay_us(T_10us);
	SPI_transaction((uint8_t*)RxData,Data,2,EPD_SPI);
}


/**
* @brief: Read one byte from EPD register
*/
uint8_t SPI_READ(const uint8_t Register) {
	uint8_t Index[2];
	uint8_t Data[2]	= {0x73,0x00};
	unsigned char RxData[2]	= {0x00,0x00};
	delay_us(T_10us);
	Index[0]=0x70;
	Index[1]=Register;

	SPI_transaction(RxData,Index,2,EPD_SPI);
	delay_us(T_10us);
	SPI_transaction(RxData,Data,2,EPD_SPI);
	return RxData[1];
}

/**
* @brief: send more than two bytes to EPD register
*/
void epd_spi_send (uint8_t register_index, uint8_t *dataPtr,
		uint8_t length) {
	unsigned char Index[2];
	unsigned char RxData[2]	= {0x00,0x00};
	delay_us(T_10us);
	Index[0]= 0x70;
	Index[1]=register_index;
	SPI_transaction(RxData,Index,2,EPD_SPI);
	delay_us(T_10us);
    //transmit data
	//while(SPI_acquirePort());
	spiSM.target = EPD_LONG_DATA;
	//spiSM.uiCurRx = 0;
	spiSM.uiCurTx = 0;
	spiSM.uiBytesToSend = length+1;
	spiSM.pcRxBuffer = (void *)0;
	spiSM.pcTxBuffer = dataPtr-1;//Tricks:we have to send out the header
	UCB1IE |= UCRXIE;
	EPD_SPI_ON();
	// Start transmission
	UCB1TXBUF = 0x72;
	// Sleep until receive occurs
	__bis_SR_register(LPM4_bits | GIE);
	EPD_SPI_OFF();
	delay_us(T_10us);
	//SPI_releasePort();
}

void I2C_config(void) {
    P4SEL |= 0x06; // Assign P4.1 P4.2 to secondary digital function (ISC_CLK, ISC_SDA)
    UCB1CTL1 |= UCSWRST;    // Enable SW reset
    UCB1CTL0 = 0x00;
    	UCB1CTL1 = 0x00;
    	UCB1BR0 = 0;
    	UCB1BR1 = 1;
    UCB1CTL0 = UCMST + UCMODE_3 + UCSYNC;
    UCB1CTL1 = UCSSEL_2 + UCSWRST; // UCSSEL_2 Clock source 2 (SMCLK)
    UCB1BR0 = 32; /*24;*/ // fSCL = SMCLK/12 = ~100kHz (Need to verify [SMCLK = 13.56MHz])
    UCB1BR1 = 0;
    UCB1IFG = 0;
    UCB1CTL1 &= ~UCSWRST;   // Clear SW reset, resume operation
}

void I2C_Tx_config(unsigned char slave_addr) {
	UCB1CTL1 |= UCSWRST;
	//UCB1CTL1 |= UCTR; // I2C transmitter mode
	UCB1I2CSA = slave_addr;
	UCB1CTL1 &= ~UCSWRST;   // Clear SW reset, resume operation
	UCB1IE |= UCTXIE + /*UCNACKIE +*/ UCRXIE;   // Enable TX interrupt
}



void I2C_Tx(unsigned char tx_addr, unsigned char tx_data) {
	MODE = TX;
	//TXAddr = tx_addr;
	//TXData = tx_data;
	unsigned char dummy[] = {tx_addr, tx_data};
	PTxData = (unsigned char *) dummy;
	TXByteCtr = 0x02;
	//while (UCB1CTL1 & UCTXSTP){UCB1CTL1 &= ~UCTXSTP;}           // Ensure stop condition got sent
	UCB1CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
	__bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
	__no_operation();
	while (UCB1CTL1 & UCTXSTP);
	//__bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
	//__no_operation();
	__delay_cycles(1000);

}



void I2C_Rx(unsigned char rx_addr, unsigned char readlen) {

	TXData = rx_addr;
	//unsigned char rx_data[4];
	unsigned char tx_data = rx_addr;

	MODE = RX;
	PTxData = &tx_data;

	TXByteCtr = 0x01;
	RXByteCtr = readlen;


	//PRxData = (unsigned char *) rx_data;
	while (UCB1CTL1 & UCTXSTP);            // Ensure stop condition got sent
	UCB1CTL1 |= UCTR + UCTXSTT;             // I2C TX, start condition
	UCB1CTL1 |= UCTXSTT;
	__bis_SR_register(LPM0_bits + GIE);     // Enter LPM0 w/ interrupts
	__no_operation();

//	while (UCB1CTL1 & UCTXSTP);
	//UCB1CTL1 &= ~UCTR;
	//while (UCB1CTL1 & UCTXSTP){UCB1CTL1 &= ~UCTXSTP;}             // Ensure stop condition got sent
	UCB1CTL1 &= ~UCTR;
	UCB1CTL1 |= UCTXSTT;                    // I2C start condition
	__bis_SR_register(LPM0_bits + GIE);     // Enter LPM0, enable interrupts
	__no_operation();

	//UCB1CTL1 |= UCTR;
}

#pragma vector = USCI_B1_VECTOR
__interrupt void USCI_B1_ISR(void) {
    switch(__even_in_range(UCB1IV, 12)) {
      case  0: break;                           // Vector  0: No interrupts
      case  2:								   // Vector  2: ALIFG
    	  	  if((UCB1CTL0 & 0x06) == 0x00) { // SPI 3-pin mode
			while (!(UCB1IFG&UCTXIFG));
			if(spiSM.target!=EPD_LONG_DATA){
				spiSM.pcRxBuffer[spiSM.uiCurRx] = UCB1RXBUF;
				spiSM.uiCurRx++;
			}
			// Move to next TX and RX index
			spiSM.uiCurTx++;
			spiSM.uiBytesToSend--;
			//spiSM.bNewDataReceived = TRUE;
			if(!spiSM.uiBytesToSend){
				UCB1IE &= ~(UCTXIE + UCRXIE);
				UCB1IFG &= ~(UCRXIFG);
				debug_interrupt = 6;	// DEBUGGING INTERRUPT
				__bic_SR_register_on_exit(LPM4_bits|GIE);

			}else{
				UCB1TXBUF = spiSM.pcTxBuffer[spiSM.uiCurTx];
			}
    	  	  }
    	  	  break;
      case  4: break;                           // Vector  4: NACKIFG
      case  6: break;                           // Vector  6: STTIFG
      case  8: break;                           // Vector  8: STPIFG
      case 10:                                  // Vector 10: RXIFG
		RXByteCtr--;							// Decrement RX byte counter
		if(RXByteCtr) {
			PRxData[3 - RXByteCtr] = UCB1RXBUF;         // Move RX data to address PRxData
			if (RXByteCtr == 1) {           // Only one byte left?
				UCB1CTL1 |= UCTXSTP;            // Generate I2C stop condition
			}
		} else {
			PRxData[3] = UCB1RXBUF;                 // Move final RX data to PRxData
			__bic_SR_register_on_exit(LPM0_bits); // Exit active CPU
		}
        break;
      case 12:
    	    if (TXByteCtr) {                         // Check TX byte counter
    	      UCB1TXBUF = *PTxData;               // Load TX buffer
    	      PTxData++;
    	    	  //if(TXByteCtr == 0x02) UCB1TXBUF = TXAddr;
    	    	  //if(TXByteCtr == 0x01) UCB1TXBUF = TXData;
    	      TXByteCtr--;                          // Decrement TX byte counter
    	    } else {
    	    		// TX mode needs stop condition to occur after last byte sent.
    	    		// RX mode needs TX data to be followed by repeated start (don't stop)
    	    		switch(MODE) {
				case TX:
					  UCB1CTL1 |= UCTXSTP;                  // I2C stop condition
					  UCB1IFG &= ~UCTXIFG;                  // Clear USCI_B0 TX int flag
					  __bic_SR_register_on_exit(LPM0_bits +GIE); // Exit LPM0
				  //break;

				case RX:
					UCB1IFG &= ~UCTXIFG;                  // Clear USCI_B0 TX int flag
					__bic_SR_register_on_exit(LPM0_bits +GIE); // Exit LPM0
					//break;
			}
    	    }
    	    break;
      default: break;
    }
}

