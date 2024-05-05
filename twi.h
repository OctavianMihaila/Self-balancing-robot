/* twi.h - TWI library for ATMega324P */

#ifndef F_CPU
#define F_CPU 16000000UL
#endif

#include <avr/io.h>
#include <util/twi.h>
#include <stdio.h>

#define TWBR         TWBR0         //_SFR_MEM8(0xB8)
#define TWSR         TWSR0         //_SFR_MEM8(0xB9)
#define TWAR         TWAR0         //_SFR_MEM8(0xBA)
#define TWDR         TWDR0         //_SFR_MEM8(0xBB)
#define TWCR         TWCR0         //_SFR_MEM8(0xBC)
#define DDRA    _SFR_IO8(0x01)
#define PORTA   _SFR_IO8(0x02)
#define PA6     6


/**
 * According to ATMega324P/V Datasheet Section 23.5.2, the formula used to get the SCL
 * frequency is:
 * SCL_frequency = CPU_Clock_Frequency / (16 + 2 * TWBR * PrescalerValue)
*/
#define F_SCL 100000UL
#define PRESCALER 1
#define TWBR_VAL ((((F_CPU / F_SCL) / PRESCALER) - 16 ) / (2 * PRESCALER))

/**
 * \brief Initialize TWI by setting the bitrate
*/
void twi_init(void);

/**
 * \brief Transmit START condition
*/
void twi_start(void);

/**
 * \brief Write a byte of data
*/
void twi_write(uint8_t data);

/**
 * \brief Read a byte of data with ACK enabled
*/
void twi_read_ack(uint8_t *data);

/**
 * \brief Read a byte of data without ACK
*/
void twi_read_nack(uint8_t *data);

/**
 * \brief Transmit STOP condition
*/
void twi_stop(void);

/**
 * \brief Search for slave devices and print them to serial
*/
void twi_discover(void);
