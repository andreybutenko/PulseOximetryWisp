/******************************************************************************
 * myNFC_Protocol.c
 *	@brief	    Higher Layer Application Response (INF) (to I-Block and \
 *				decides how to fill in INF field of Response to ATTRIB)
 *  @date: 		Aug 22, 2014
 *  @author: 	Yi Zhao (Eve)-Sensor System Lab, UW
 *  @note:		Only Implement response in ISO 14443-B protocol here
 *  @TODO:		Test and Implement _15693_1of256 protocol
  *****************************************************************************/

//=============================================================================
//									Includes
//=============================================================================
#include <string.h>
#include "myNFC_Protocol.h"
#include "../NFC_protocol/doNFC.h"
#include "../common/e-paper.h"
#include "../common/spi.h"
#include "../common/sensor_flag.h"

//=============================================================================
//									Defines
//=============================================================================
// Enumerating indices into INF buffer
#define INF_CMD_INDEX 			0
#define INF_DATA_START_INDEX 	2

//Flag enumeration (R->T and T->R)
#define INF_FLAG_WRITE_SUCCESS 	0x01
#define INF_FLAG_WRITE_COMPLETE 0x02
#define INF_FLAG_IMGE_UPDATE_COMPLETE 0x04

////Command IDs for "transport layer"
//#define INF_CMD_ID_TX_COMPLETE	0x01
//#define INF_CMD_ID_READ_SINGLE_BLOCK 0x20
//#define INF_CMD_ID_WRITE_SINGLE_BLOCK 0x21
//#define INF_CMD_ID_WRITE_N_BLOCKS 0x24


#define BLOCK_SIZE 4 // DEBUG: Change to smaller size, original: 4
#define BLOCKS_PER_CHUNK 15 // DEBUG: Change to smaller size, original: 15
#define CHUNK_SIZE_BYTES (BLOCK_SIZE*BLOCKS_PER_CHUNK)
#define MAX_CHUNK_SIZE ((E_INK_SIZE / CHUNK_SIZE_BYTES) - 1)

//=============================================================================
//									Local Variables
//=============================================================================
typedef struct {
	unsigned char raw[CHUNK_SIZE_BYTES];
} ArrayChunks_t;

static ArrayChunks_t* imageBufferChunked;

static unsigned char curRequestedChunk;
static unsigned char rxChunkIndex;
//DEBUG
//extern uint8_t debugBug[10];

//=============================================================================
//									Functions
//=============================================================================
/**
 * initialize custermized nfc_wisp_protocol response to I-Block
 */
void initialize_nfc_wisp_protocol() {
	//doNFC_state=0;
	curRequestedChunk = 0; 								 // Initialize the image chunk counter
	imageBufferChunked = (ArrayChunks_t*)imageBuffer;   // Map struct array over actual data buffer
	//memset(rx_buffer, 0x00, CMD_BUF_SIZE);
	doNFC_state = NFC_Start;
	imageUpdateState = IMG_HALT;										// Clear flag
	senseState = 0;
	rxChunkIndex=0;
}


/**
 * Interprets Higher Layer Response (INF) to I-Block and \
 *  decides how to fill in INF field of Response to ATTRIB
 *
 * @param inf_received Represents the contents of the INF field of the incoming transaction
 * @param index Current index into the transmit command array, to which this function contributes
 *
 * @return The updated current index into the transmit command array after
 *
 */
/*
unsigned char nfc_wisp_protocol(unsigned char * inf_received, unsigned char index) {


	// Extract chunk index from reader command


	// Status flags we'll send back to reader
	unsigned char tagResponseFlags = 0;

	//Used to indicat finish of image update, if INF_FLAG_IMGE_UPDATE_COMPLETE is set
	tagResponseFlags=inf_received[0];

	rxChunkIndex = inf_received[1];
	//@DEBUG
	//debugBuf[0]=curRequestedChunk;

	// Measure the time it takes for receiving data using this if function
	// The voltage drop in vrect using this function and also draw balck & white and one line

	// Set the switch pin(P1.0) to HIGH
	// SW_1_OUT |= SW_1_BIT;
	if((rxChunkIndex == curRequestedChunk)&&(!(tagResponseFlags&INF_FLAG_IMGE_UPDATE_COMPLETE))) { // If the reader is sending the chunk we requested
	//if((rxChunkIndex == curRequestedChunk)&&(!(rxChunkIndex&INF_FLAG_IMGE_UPDATE_COMPLETE))) { // If the reader is sending the chunk we requested

		//!!!
		// Be careful, everything we do here should be done FAST! If it takes too long, reader will miss the tag's response.
		//!!!

		// Set up some pointers for a buffer copy speed run
		unsigned char* tmpDst = imageBufferChunked[rxChunkIndex].raw;
		unsigned char* finalDst = &(imageBufferChunked[rxChunkIndex].raw[CHUNK_SIZE_BYTES]); // Disregard out-of-range warning
		unsigned char* tmpSrc = &(inf_received[INF_DATA_START_INDEX]);

		//led_2_on();
		// Copy data from INF field SUPER FAST (least dereferencing possible)!
		do {
			*(tmpDst++) = *(tmpSrc++);
		} while(tmpDst != finalDst);

		// Signal to reader that we've successfully copied the previous data
		tagResponseFlags |= INF_FLAG_WRITE_SUCCESS;

		// Since we successfully received that chunk, request the next one unless we're done with everything
		//if(curRequestedChunk >= (E_INK_SIZE / CHUNK_SIZE_BYTES) - 1) {
		if(curRequestedChunk >= MAX_CHUNK_SIZE) {

			// Signal to reader that we've received all the chunks we think we need
			tagResponseFlags |= INF_FLAG_WRITE_COMPLETE;

			// Signal to application that we're done
			// TODO Hack alert! Encapsulate this module better
			imageUpdateState = IMG_FORCE_UPDATE;
			curRequestedChunk = 0;
			//led_2_on(); // DEBUG

		} else {
			//toggle_led_1(); 				// DEBUG
			curRequestedChunk++; 			// Request next chunk
			//debugBuf[9]=curRequestedChunk;
		}
	}
	// Set the switch pin(P1.0) to LOW
	//SW_1_OUT &= ~SW_1_BIT;

	// Tell the reader how things went this round
	transmitCommand[index++] = tagResponseFlags;

	// Send the latest requested address, which the reader should send to us next round
	transmitCommand[index++] = curRequestedChunk;

	return index;
}*/


unsigned char nfc_wisp_protocol(unsigned char * inf_received, unsigned char index) {
	//if(!temp_read) {
		int i, k;
		uint8_t j;
		//uint8_t rxbuffer[128];
		int max_sample = 16;
		uint8_t rxbuf[4];
		//uint8_t ir[128], red[128];
		//int ir_i, red_i;

		if(temp_read) {
			temp_read = 0;
			I2C_config();
			I2C_Tx_config(0x57);
			I2C_Tx(0x06, 0x03);
			I2C_Tx(0x07, 0x47); // 0x47
			I2C_Tx(0x09, 0x64);
			//I2C_Tx(0x09, 0x99);
		}

		P1DIR |= 0x10;
		P1SEL &= ~0x10;
		P1OUT &= ~0x10;

		//ir_i = 0;
		//red_i = 0;
		if(sample_req) {
			sample_req = 0;
			uint32_t bufferAddress = 0;
			uint8_t txbuffer[4];

			for(i = 0; i < max_sample; i++) {
				P1OUT |= 0x10;
				I2C_config();
				I2C_Tx_config(0x57);
				I2C_Rx(0x05, 4);


				for(j = 0; j < 4; j++) {
					txbuffer[j] = PRxData[j];
				}



				SPI_initialize();
				SPI_FRAM_Wake_Up();
				SPI_FRAM_Write_Enable_Latch();
				SPI_FRAM_Write_Memory((uint8_t *) &bufferAddress, txbuffer, 4);

				// 20 bytes, index must be less than 60
				bufferAddress += 4;
				lowPowerSleep(LPM_5ms + LPM_500us * 2 + LPM_50us);
				P1OUT &= ~0x10;
			}

		}
		SPI_initialize();
		//for(k = 0; k < max_sample; k++) { // test loop
		for(i = 0; i < 4; i++) {
			// read from fram
			SPI_FRAM_Read_Memory_func((uint8_t *) &fram_addr, rxbuf, 4);
			for(j = 0; j < 4; j++) {
				transmitCommand[index + i * 4 + j] = rxbuf[j];
			}
			fram_addr += 4;
		}
		//} // end test loop
		index += 16;
		if(fram_addr >= 	max_sample * 4) {
			fram_addr = 0;
			sample_req = 1;
		}
	/*} else {
		temp_read = 0;
		I2C_config();
		I2C_Tx_config(0x57);
		I2C_Tx(0x06, 0x0B);
		I2C_Tx(0x07, 0x47); // 0x47
		I2C_Tx(0x09, 0x63);
		lowPowerSleep(LPM_10ms);
		//I2C_Tx(0x06, 0x0B);
		int i = 0;

		do {
			I2C_Rx(0x00, 1);
		} while(!(PRxData[0] & 0x40));
		I2C_Rx(0x16, 1);
		transmitCommand[i] = PRxData[0];
		I2C_Rx(0x17, 1);
		transmitCommand[i + 1] = PRxData[0];

		index += 16;
	}*/
	return index;
}

