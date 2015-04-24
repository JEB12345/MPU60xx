/**
 * An I2C library for the dsPIC33 series processors.
 * Copyright (C) 2015 Pavlo Manovi

 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

// Standard headers
#include <stdbool.h>

// Microchip headers
#include <xc.h>
#include <i2c.h>

// User headers
#include "I2CdsPIC.h"
#include "../sensor_state.h"

void I2C_Init(uint16_t brg)
{
	I2C1CONbits.I2CEN = 0; // Disable I2C1
	I2C1BRG = brg + 1; // Set the baud rate
	I2C1CONbits.I2CEN = 1; // Enable I2C1
}

void I2C_WriteToReg(uint8_t address, uint8_t deviceRegister, uint8_t data)
{

	// Assert the start condition
	StartI2C1();
	while (I2C1CONbits.SEN);
	IFS1bits.MI2C1IF = 0;

	// Send the 7-bit I2C device address
	MasterWriteI2C1(address << 1);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Send the register address
	MasterWriteI2C1(deviceRegister);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Send the data
	MasterWriteI2C1(data);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Assert the stop condition
	StopI2C1();
	while (I2C1CONbits.PEN);

	// Sit idle on the bus
	IdleI2C1();
}

uint8_t I2C_ReadFromReg(uint8_t address, uint8_t deviceRegister)
{
	uint8_t tempData;

	I2C_ReadFromReg_Burst(address, deviceRegister, &tempData, 1);
	return tempData;
}

void I2C_ReadFromReg_Burst(uint8_t address, uint8_t deviceRegister, uint8_t* data, uint8_t burstNum)
{
	// Assert the start condition
	StartI2C1();
	while (I2C1CONbits.SEN);

	// Send the 7-bit device address
	MasterWriteI2C1(address << 1);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Send the register address
	MasterWriteI2C1(deviceRegister);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Start a new I2C transaction
	RestartI2C1();
	while (I2C1CONbits.RSEN);

	// Send the second address
	MasterWriteI2C1((address << 1) + 1);
	while (I2C1STATbits.TBF); // 8 clock cycles
	while (!IFS1bits.MI2C1IF); // Wait for 9th clock cycle
	IFS1bits.MI2C1IF = 0; // Clear interrupt flag

	// Read the data
	data[0] = MasterReadI2C1();

	if (burstNum > 1) {
                uint8_t i;
		for (i = 1; i < burstNum; i++) {
			AckI2C1();
			while (I2C1CONbits.ACKEN == 1);

			data[i] = MasterReadI2C1();
		}
	}

	// No need to ack reception of this data
	NotAckI2C1();
	while (I2C1CONbits.ACKEN == 1);

	// Stop the I2C transaction
	StopI2C1();
	while (I2C1CONbits.PEN);

	// Go idle on the bus
	IdleI2C1();
}


//this struct is for internal use only
typedef struct {
    uint8_t op_type; //0 = read, 1 = write
    uint8_t address;
    uint8_t command;
    uint16_t num_bytes;
    uint8_t* output;
    return_value_t* status_out;
} i2c_operation;

typedef enum {
    I2C_IDLE,
            I2C_WAIT_START_READ_STATE,
            I2C_NEXT_PACKET,
            I2C_ADDRESS_READ_STATE,
            I2C_REGISTER_ADDRESS_READ_STATE,
            I2C_RS_READ_STATE,
            I2C_REGISTER_ADDRESS_READ_READ_STATE,
            I2C_BYTE_READ_READ_STATE,
            I2C_STOP_READ_STATE,
            I2C_NEXT_BYTE_READ_STATE
} i2c_internal_state;

#define I2C_OPS_MAX 2
typedef struct {
    volatile i2c_operation operations[I2C_OPS_MAX];
    volatile uint16_t ops_start;
    volatile uint16_t ops_end;
    volatile i2c_internal_state  state;
    volatile uint16_t byte_cnt;
} i2c_data;

i2c_data i2c_state;

void __attribute__((__interrupt__, no_auto_psv)) _MI2C1Interrupt(void) {

    IFS1bits.MI2C1IF = 0;
i2c_interrupt_start:
    switch(i2c_state.state){
        case I2C_NEXT_PACKET:
        case I2C_IDLE:
            if(i2c_state.ops_end!=i2c_state.ops_start){
                //new operation!
                if(i2c_state.operations[i2c_state.ops_start].op_type==0){
                    //read
                    I2C1CONbits.SEN = 1; //start i2c transaction
                    i2c_state.state = I2C_WAIT_START_READ_STATE;
                } else {
                    //write: TODO implement this
                }
            } else {
                i2c_state.state = I2C_IDLE;
            }
            break;
        case I2C_WAIT_START_READ_STATE:
            if (!I2C1STATbits.S) {
                goto i2c_error;
            } else {
                //send address
                I2C1TRN = i2c_state.operations[i2c_state.ops_start].address<<1;
                i2c_state.state = I2C_ADDRESS_READ_STATE;
            }
            break;
        case I2C_ADDRESS_READ_STATE:
            if (I2C1STATbits.ACKSTAT) {
                goto i2c_error;
            } else {
                //ACK RECEIVED
                //send command
                I2C1TRN = i2c_state.operations[i2c_state.ops_start].command;
                i2c_state.state = I2C_REGISTER_ADDRESS_READ_STATE;
            }
            break;
        case I2C_REGISTER_ADDRESS_READ_STATE:
            if (I2C1STATbits.ACKSTAT) {
                goto i2c_error;
            } else {
                //ACK RECEIVED
                //send a repeated start bit
                I2C1CONbits.RSEN = 1; //repeated start bit
                i2c_state.state = I2C_RS_READ_STATE;
            }
            break;
        case I2C_RS_READ_STATE:
            //send device address with read
            I2C1TRN = (i2c_state.operations[i2c_state.ops_start].address<<1)|0b1;
            i2c_state.state = I2C_REGISTER_ADDRESS_READ_READ_STATE;
            break;
        case I2C_REGISTER_ADDRESS_READ_READ_STATE:
            if (I2C1STATbits.ACKSTAT) {
                goto i2c_error;
            } else {
                //ACK RECEIVED
                //we can now start reading registers
                i2c_state.byte_cnt = 0;
                I2C1CONbits.RCEN = 1;
                i2c_state.state = I2C_BYTE_READ_READ_STATE;
            }
            break;
        case I2C_BYTE_READ_READ_STATE:
            if (!I2C1STATbits.RBF) {
                goto i2c_error;
            } else {
                //data received
                //copy data to output buffer
                i2c_state.operations[i2c_state.ops_start].output[i2c_state.byte_cnt++] = I2C1RCV;
                //check if we need to send a nack or ack
                if (i2c_state.byte_cnt >= i2c_state.operations[i2c_state.ops_start].num_bytes) {
                    //send NACK
                    I2C1CONbits.ACKDT = 1;
                    i2c_state.state = I2C_STOP_READ_STATE;
                    I2C1CONbits.ACKEN = 1;
                } else {
                    //send ACK
                    I2C1CONbits.ACKDT = 0;
                    i2c_state.state = I2C_NEXT_BYTE_READ_STATE;
                    I2C1CONbits.ACKEN = 1;
                }
            }
            break;
        case I2C_NEXT_BYTE_READ_STATE:
            if (I2C1CONbits.ACKEN) {
                goto i2c_error;
            } else {
                i2c_state.state = I2C_BYTE_READ_READ_STATE;
                I2C1CONbits.RCEN = 1; //get next byte
            }
            break;
        case I2C_STOP_READ_STATE:
            if (I2C1CONbits.ACKEN) {
                goto i2c_error;
            } else {
                I2C1CONbits.PEN = 1; //stop bit
                *i2c_state.operations[i2c_state.ops_start].status_out = RET_OK;
                i2c_state.ops_start = 1;
                i2c_state.state = I2C_IDLE;
            }
            break;
        default:
            //WHAT??? we shouldn't be here :)
            break;
    };

    return;

i2c_error:
    //error
    I2C1CONbits.PEN = 1; //stop bit
    *(i2c_state.operations[i2c_state.ops_start].status_out) = RET_ERROR;
    i2c_state.state = I2C_IDLE;
    i2c_state.ops_start = 1;
    goto i2c_interrupt_start;
}

void i2c_1_enable_interrupts()
{
    IEC1bits.MI2C1IE = 1;
    i2c_state.state = I2C_IDLE;
    i2c_state.ops_start = i2c_state.ops_end = 0;
}

/**
 * Schedules an I2C read operation
 * @param address 7 bit address of the slave device
 * @param command 8 bit command (register)
 * @param num_bytes the number of bytes to read
 * @param output pointer to store the read data
 * @param status_out stores the result of the operation (RET_OK or RET_ERROR),
 * this can be used by the user to check the state of the operation
 * MULTIPLE OPERATIONS CODE HAS BEEN REMOVED (not needed for IMU).
 * @return RET_OK if the read has been successfully scheduled, else RET_ERROR
 */
return_value_t i2c_1_read(uint8_t address, uint8_t command, uint16_t num_bytes,
        uint8_t* output, return_value_t* status_out)
{
    unsigned i;
    if(i2c_state.state==I2C_IDLE){
        i2c_operation* op = &i2c_state.operations[0];
        op->address = address;
        op->command = command;
        op->num_bytes = num_bytes;
        for(i=0;i<num_bytes;++i){
            output[i] = 0xFF;
        }
        op->output = output;
        op->status_out = status_out;
        *(op->status_out) = RET_UNKNOWN;
        //update internal state last
        //need to kickstart the I2C processing!
        i2c_state.ops_start = 0;
        i2c_state.ops_end = 1;
        _MI2C1Interrupt();
        return RET_OK;
    } else {
        return RET_ERROR; //busy
    }
}