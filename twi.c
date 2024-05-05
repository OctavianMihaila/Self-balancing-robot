/* twi.c - TWI library for ATMega324P */

#include "twi.h"
#include "debug.h"

void twi_init(void) {
	/* Enable I2C power (board-specific) */
	DDRA |= (1 << PA6);
	PORTA |= (1 << PA6);
	TWCR = 0; /* reset it again, just to be sure */

    // activate internal pull-ups for SDA and SCL
    // PORTC |= (1 << PC4) | (1 << PC5);
	
    // TODO 0: Set bitrate (TWBR & TWSR!)
    TWBR = TWBR_VAL;
    TWSR &= ~((1 << TWPS0) | (1 << TWPS1));

}

void twi_start(void) {    
    // Send START condition (using TWCR)
	TWCR = 0; /* reset it again, just to be sure */
    // 
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA) | (1 << TWSTA); // TWEA sets acknowledge bit
	// obligatory: wait for START condition to be sent

	while (!(TWCR & (1 << TWINT)));
    // blink_green_LED_5_digit(11111);

}

void twi_write(uint8_t data) {
    // Send a byte of data (TWCR + TWDR)
    TWDR = data;
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // TWEA sets acknowledge bit
	// TODO 1: wait for transfer to complete (TWINT flag)
    while (!(TWCR & (1 << TWINT)));
}

void twi_read_ack(uint8_t *data) {
    // TODO 1: Read a byte of data with ACK enabled 
	TWCR = (1 << TWINT) | (1 << TWEN) | (1 << TWEA); // TWEA sets acknowledge bit
	while (!(TWCR & (1 << TWINT)));
    *data = TWDR;
}

void twi_read_nack(uint8_t *data) {
    // TODO 1: Read a byte of data with ACK disabled 
	// same as above, but don't send acknowledge
    TWCR = (1 << TWINT) | (1 << TWEN); // No acknowledge
	while (!(TWCR & (1 << TWINT)));
    *data = TWDR;
}

void twi_stop(void) {
    // Send STOP condition
    TWCR = (1 << TWEN) | (1 << TWINT) | (1 << TWSTO);
}

void twi_discover(void) {
    // TODO 2: Search for I2C slaves.
    // HINT: An acknowledged SLA_R should enable a flag in TWSR. Check the datasheet!
    for (uint8_t i = 0x00; i < 0x7F; i++)  {
        twi_start(); // Start condition

        // Write address and check acknowledgment
        twi_write(i << 1 | 1); // Write address with SLA_W (R/W bit set to 0)
        uint8_t twst = TWSR & 0xFB; // Get the status register value

        if (twst == 0x40) { // If SLA+W transmitted and ACK received
            printf("Device discovered on 0x%x\n", i); // Print discovered device address
        }

        twi_stop(); // Stop condition
    }
}
