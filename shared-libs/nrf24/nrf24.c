#include "nrf24.h"
#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include "aes.h"

static SPI_HandleTypeDef * nrf24_hspi;
uint8_t nrf24_payload_len;

void nrf24_init(SPI_HandleTypeDef * hspi)
{
	nrf24_hspi=hspi;
  CE_LOW;
  CSN_HIGH;  
}

uint8_t spi_transfer(uint8_t value)
{
	uint8_t rx;
	HAL_SPI_TransmitReceive(nrf24_hspi, &value, &rx, 1, 100);
	return rx;
}

/* send multiple bytes over SPI */
void nrf24_transmitSync(uint8_t* dataout,uint8_t len)
{
	HAL_SPI_Transmit(nrf24_hspi, dataout, len, 100);
}

#define W_ACK_PAYLOAD 0xA8

uint32_t min(uint32_t a, uint32_t b){
	return a<b ? a : b;
}

void nrf24_writeAckPayload(uint8_t pipe, const void* buf, uint8_t len)
{
	const uint8_t * current = buf;
  CSN_LOW;
  spi_transfer( W_ACK_PAYLOAD | ( pipe & 7 ) ); //B111
  const uint8_t max_payload_size = 32;
  uint8_t data_len = min(len,max_payload_size);
  while ( data_len-- )
  spi_transfer(*current++);

  CSN_HIGH;
}

void nrf24_configRegister(uint8_t reg, uint8_t value)
{
    CSN_LOW;
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    spi_transfer(value);
    CSN_HIGH;
}

void nrf24_powerUpRx()
{     
    CSN_LOW;
    spi_transfer(FLUSH_RX);
    CSN_HIGH;

    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 

    CE_LOW;    
    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(1<<PRIM_RX)));    
    CE_HIGH;
}



/* Write to a single register of nrf24 */
void nrf24_writeRegister(uint8_t reg, uint8_t* value, uint8_t len) 
{
    CSN_LOW;
    spi_transfer(W_REGISTER | (REGISTER_MASK & reg));
    nrf24_transmitSync(value,len);
    CSN_HIGH;
}

/* Set the RX address */
void nrf24_rx_address(uint8_t pipe, uint8_t * adr) 
{
  CE_LOW;
	nrf24_writeRegister(pipe,adr, pipe-RX_ADDR_P0 < 2 ? nrf24_ADDR_LEN : 1 );
  CE_HIGH;
}

/* Set the TX address */
void nrf24_tx_address(uint8_t* adr)
{
    /* RX_ADDR_P0 must be set to the sending addr for auto ack to work. */
    nrf24_writeRegister(RX_ADDR_P0,adr,nrf24_ADDR_LEN);
    nrf24_writeRegister(TX_ADDR,adr,nrf24_ADDR_LEN);
}

/* Returns the payload length */
uint8_t nrf24_payload_length()
{
    return nrf24_payload_len;
}

/* configure the module */
void nrf24_config(uint8_t channel, uint8_t pay_length)
{
    /* Use static payload length ... */
    nrf24_payload_len = pay_length;

    // Set RF channel
    nrf24_configRegister(RF_CH,channel);

    // Set length of incoming payload 
    nrf24_configRegister(RX_PW_P0, 32); // Auto-ACK pipe ...
    nrf24_configRegister(RX_PW_P1, 32); // Data payload pipe
    nrf24_configRegister(RX_PW_P2, 32); // Pipe not used 
    nrf24_configRegister(RX_PW_P3, 32); // Pipe not used 
    nrf24_configRegister(RX_PW_P4, 32); // Pipe not used 
    nrf24_configRegister(RX_PW_P5, 32); // Pipe not used 

    // 1 Mbps, TX gain: 0dbm
		nrf24_configRegister(RF_SETUP, (0<<RF_DR)|((0x03)<<RF_PWR));
		//256
		//nrf24_configRegister(RF_SETUP, (1<<RF_DR_LOW)|((0x03)<<RF_PWR));

    // CRC enable, 1 byte CRC length
    nrf24_configRegister(CONFIG,nrf24_CONFIG);

    // Auto Acknowledgment
    nrf24_configRegister(EN_AA,(1<<ENAA_P0)|(1<<ENAA_P1)|(1<<ENAA_P2)|(1<<ENAA_P3)|(1<<ENAA_P4)|(1<<ENAA_P5));

    // Enable RX addresses
    nrf24_configRegister(EN_RXADDR,(1<<ERX_P0)|(1<<ERX_P1)|(1<<ERX_P2)|(1<<ERX_P3)|(1<<ERX_P4)|(1<<ERX_P5));

    // Auto retransmit delay: 1000 us and Up to 15 retransmit trials
    nrf24_configRegister(SETUP_RETR,(0x04<<ARD)|(0x0F<<ARC));

    // Dynamic length configurations: No dynamic length
    //nrf24_configRegister(DYNPD,(0<<DPL_P0)|(0<<DPL_P1)|(0<<DPL_P2)|(0<<DPL_P3)|(0<<DPL_P4)|(0<<DPL_P5));

		// Use dymamic length
    nrf24_configRegister(DYNPD,(1<<DPL_P0)|(1<<DPL_P1)|(1<<DPL_P2)|(1<<DPL_P3)|(1<<DPL_P4)|(1<<DPL_P5));
		nrf24_configRegister(FEATURE,(1<<EN_DPL));		

    // Start listening
    //nrf24_powerUpRx();
}


void nrf24_powerUpTx()
{
    nrf24_configRegister(STATUS,(1<<RX_DR)|(1<<TX_DS)|(1<<MAX_RT)); 

    nrf24_configRegister(CONFIG,nrf24_CONFIG|((1<<PWR_UP)|(0<<PRIM_RX)));
}

void nrf24_powerDown()
{
    CE_LOW;
    nrf24_configRegister(CONFIG,nrf24_CONFIG);

}

void nrf24_sendDL(void * value, uint8_t payload_len) 
{    
    /* Go to Standby-I first */
    CE_LOW;
     
    /* Set to transmitter mode , Power up if needed */
    nrf24_powerUpTx();

    /* Do we really need to flush TX fifo each time ? */
    #if 1
        /* Pull down chip select */
			CSN_LOW;

        /* Write cmd to flush transmit FIFO */
        spi_transfer(FLUSH_TX);     

        /* Pull up chip select */
        CSN_HIGH;
    #endif 

    /* Pull down chip select */
    CSN_LOW;

    /* Write cmd to write payload */
    spi_transfer(W_TX_PAYLOAD);

    /* Write payload */
    nrf24_transmitSync(value, payload_len);   

    /* Pull up chip select */
    CSN_HIGH;

    /* Start the transmission */
    CE_HIGH;
}


// Sends a data package to the default address. Be sure to send the correct
// amount of bytes as configured as payload on the receiver.
void nrf24_send(void * value) 
{    
   nrf24_sendDL(value, nrf24_payload_len);
}

uint8_t nrf24_getStatus()
{
    uint8_t rv;
    CSN_LOW;
    rv = spi_transfer(NOP);
    CSN_HIGH;
    return rv;
}

uint8_t nrf24_isSending()
{
    uint8_t status;

    /* read the current status */
    status = nrf24_getStatus();
                
    /* if sending successful (TX_DS) or max retries exceded (MAX_RT). */
    if((status & ((1 << TX_DS)  | (1 << MAX_RT))))
    {        
        return 0; /* false */
    }

    return 1; /* true */

}

uint8_t nrf24_lastMessageStatus()
{
    uint8_t rv;

    rv = nrf24_getStatus();

    /* Transmission went OK */
    if((rv & ((1 << TX_DS))))
    {
        return NRF24_TRANSMISSON_OK;
    }
    /* Maximum retransmission count is reached */
    /* Last message probably went missing ... */
    else if((rv & ((1 << MAX_RT))))
    {
        return NRF24_MESSAGE_LOST;
    }  
    /* Probably still sending ... */
    else
    {
        return 0xFF;
    }
}

/* send and receive multiple bytes over SPI */
void nrf24_transferSync(uint8_t* dataout,uint8_t* datain,uint8_t len)
{
    uint8_t i;

    for(i=0;i<len;i++)
    {
        datain[i] = spi_transfer(dataout[i]);
    }

}

/* Read single register from nrf24 */
void nrf24_readRegister(uint8_t reg, uint8_t* value, uint8_t len)
{
    CSN_LOW;
    spi_transfer(R_REGISTER | (REGISTER_MASK & reg));
    nrf24_transferSync(value,value,len);
    CSN_HIGH;
}

/* Returns the number of retransmissions occured for the last message */
uint8_t nrf24_retransmissionCount()
{
    uint8_t rv;
    nrf24_readRegister(OBSERVE_TX,&rv,1);
    rv = rv & 0x0F;
    return rv;
}

/* Checks if receive FIFO is empty or not */
uint8_t nrf24_rxFifoEmpty()
{
    uint8_t fifoStatus;

    nrf24_readRegister(FIFO_STATUS,&fifoStatus,1);
    
    return (fifoStatus & (1 << RX_EMPTY));
}

/* Checks if data is available for reading */
/* Returns 1 if data is ready ... */
uint8_t nrf24_dataReady() 
{
    // See note in getData() function - just checking RX_DR isn't good enough
    uint8_t status = nrf24_getStatus();

    // We can short circuit on RX_DR, but if it's not set, we still need
    // to check the FIFO for any pending packets
    if ( status & (1 << RX_DR) ) 
    {
        return 1;
    }

    return !nrf24_rxFifoEmpty();
}

uint8_t nrf24_dataReadyPipeNo()
{
		return (nrf24_getStatus()>>1)&0x7;
}


/* Reads payload bytes into data array */
void nrf24_getData(void * data) 
{
    /* Pull down chip select */
    CSN_LOW;                               

    /* Send cmd to read rx payload */
    spi_transfer( R_RX_PAYLOAD );
    
    /* Read payload */
    nrf24_transferSync(data,data,nrf24_payload_len);
    
    /* Pull up chip select */
    CSN_HIGH;

    /* Reset status register */
    nrf24_configRegister(STATUS,(1<<RX_DR));   
}


typedef struct
{
	uint32_t version;
	uint8_t data[28];
} QMTTMessage;

/*
QMTTMessage * getSettingsStruct()
{
		return (QMTTMessage*)buff;
}
*/

void sendNRF(QMTTMessage * message)
{
	nrf24_send(message);
	while(nrf24_isSending()){};
		
/*
	uint8_t a=nrf24_lastMessageStatus();
	a=a+1;
	uint8_t b=nrf24_retransmissionCount();
		b=b+1;
*/
		
	nrf24_powerUpRx();	
}

void QMTT_SendOutTopic(const char * out_topic, uint8_t * adr, aes128_ctx_t* ctx)
{
	QMTTMessage message;	
	message.version=0;
	memcpy((char*)message.data, adr, 5);
	strcpy((char*)message.data+5, out_topic);
	
	aes128_enc((void*)(&message), ctx);
	aes128_enc((void*)(&message+16), ctx);
	
	sendNRF(&message);
}

void QMTT_SendInTopic(const char * in_topic, uint8_t * adr, aes128_ctx_t* ctx)
{
	QMTTMessage message;	
	message.version=1;
	memcpy((char*)message.data, adr, 5);
	strcpy((char*)message.data+5, in_topic);

	aes128_enc((void*)(&message), ctx);
	aes128_enc((void*)(&message+16), ctx);

	sendNRF(&message);
}


void QMTT_SendTextMessage(const char * name, const char * value, uint8_t * adr, aes128_ctx_t* ctx)
{
	QMTTMessage message;	
	
	message.version=10;
	
	memcpy((char*)message.data, adr, 5);
	
	int len=strlen((char*)name)+1;
	memcpy((char*)message.data+5, name, len);
	
	int len2=strlen((char*)value);
	memcpy((char*)message.data+5+len, value, len2+1);
	
	aes128_enc((void*)(&message), ctx);
	aes128_enc((void*)(&message+16), ctx);

	sendNRF(&message);
}
