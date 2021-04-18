#ifndef HULP_I2CBB_H
#define HULP_I2CBB_H

//I2C bitbanger. Read (8/16 bit), write, multiple slaves, lightweight (~76 instr), basic error handling.
//Timing is fixed (Frequency: ~150kHz)
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "soc/sens_reg.h"

#include "hulp.h"

#ifdef __cplusplus
extern "C" {
#endif

/**
 *  Convenience macro for an I2C bitbang read (8 bits) into R0, to be used in your program with M_INCLUDE_I2CBB.
 *  
 *  label: Provide a new unique label number (used as return point).
 *  label_read: The label to jump to for I2C reads (ie. the same one provided to M_INCLUDE_I2CBB)
 *  subaddress: The slave's subaddress to read from.
 */
#define M_I2CBB_RD(label, label_read, subaddress) \
    I_MOVI(R1, ((subaddress) << 8)), \
    M_MOVL(R3, label), \
    M_BX(label_read), \
    M_LABEL(label)

/**
 *  Convenience macro for an I2C bitbang read (16 bits) into R0
 * 
 *  label: Provide a new unique label number (used as return point).
 *  label_read: The label to jump to for I2C reads (ie. the same one provided to M_INCLUDE_I2CBB)
 *  subaddress: The slave's subaddress to read from.
 */
#define M_I2CBB_RD16(label, label_read, subaddress) \
    I_MOVI(R1, ((subaddress) << 8) | 1), \
    M_MOVL(R3, label), \
    M_BX(label_read), \
    M_LABEL(label)

/**
 *  Convenience macro for an I2C bitbang write
 *  
 *  label: Provide a new unique label number (used as return point).
 *  label_write: The label to jump to for I2C writes (ie. the same one provided to M_INCLUDE_I2CBB)
 *  subaddress: The slave's subaddress to write to.
 *  value: The value to write (8 bits)
 */
#define M_I2CBB_WR(label, label_write, subaddress, value) \
    I_MOVI(R1, ((subaddress) << 8) | (value)), \
    M_MOVL(R3, label), \
    M_BX(label_write), \
    M_LABEL(label)

/**
 *  Set the address of the I2C device. Used only with multi device variant.
 */
#define I_I2CBB_SET_SLAVE(slave_addr) \
    I_WR_REG(SENS_SAR_SLAVE_ADDR4_REG, 21 - 6, 21, (slave_addr))


/**
 *  This macro expands to populate your ULP program with subroutines for basic I2C master bitbanging.
 * 
 * 	See examples or M_INCLUDE_I2CBB description for setup.
 */

#define M_INCLUDE_I2CBB(label_read, label_write, label_arblost_error, label_nack_error, scl_gpio, sda_gpio, slave_address) \
    M_INCLUDE_I2CBB_(label_read, label_write, label_arblost_error, label_nack_error, scl_gpio, sda_gpio, slave_address, R1, R2, R3)

/**
 *  This macro expands to populate your ULP program with subroutines for basic I2C master bitbanging functionality.
 * 	Operations: 8 bit read, 16 bit read, 8 bit write. Only one slave device is supported.
 *  Additionally, there are callbacks for slave NACK and arbitration loss.
 * 
 * 	Flow: 
 * 		Read: -Prepare reg_data with the slave's sub address in the upper 8 bits. 		eg. I_MOVI( R1, (SUBADDRESS << 8) )
 *            	[Optional] To read 16 bits, additionally set bit 0 at the same time.	eg. I_MOVI( R1, (SUBADDRESS << 8) | 1) )
 * 			  -Prepare reg_return with the return address. 							eg. M_MOVL(R3, LABEL_I2C_SUCCESS_RETURN)
 * 			  -Branch to label_read. 													eg. M_BX(LABEL_I2C_READ)
 * 			On success: Branches to the address in reg_return; R0 = result, reg_data = reg_data, reg_scratch = 0, reg_return = reg_return, stage = undef
 * 			On error: Branches to label_arblost_error_handler or label_nack_error_handler; R0 = undef, reg_data = reg_data, reg_scratch = 0, reg_return = reg_return, stage = undef
 * 
 *  	Write: -Prepare reg_data with the slave's sub address in the upper 8 bits, and the value in the lower 8 bits. eg. I_MOVI(R1, (SUBADDRESS << 8) | (WRITE_VALUE) )
 * 			   -Prepare reg_return with the return address. eg. M_MOVL(R3, LABEL_I2C_SUCCESS_RETURN)
 * 			   -Branch to label_write. eg. M_BX(LABEL_I2C_WRITE)
 * 			On success: Branches to the address in reg_return; R0 = 0(ACK), reg_data = reg_data, reg_scratch = 0, reg_return = reg_return.
 * 			On error: Branches to label_arblost_error_handler or label_nack_error_handler; R0 = undef, reg_data = reg_data, reg_scratch = 0, reg_return = reg_return.
 *  
 * 
 * 	label_write: A label to identify the I2C write entry. Branch to this label to write.
 *  label_read: A label to identify the I2C read entry. Branch to this label in your program to read (8 or 16 bits).
 *  label_arblost_error_handler / label_nack_error_handler: Provide labels to branch to in the event of an error.
 * 		These must be implemented elsewhere in your ULP program to handle errors. The same label may be used for both.
 *  slave_address: 7-bit slave address
 *  sda_gpio / scl_gpio: I2C GPIOs
 *  reg_data: This register is used to pass the subaddress and data to the subroutine (see above for usage). R1-R3.
 *  reg_scratch: Scratch register. R1-R3.
 *  reg_return: A register containing the address to branch to on success. R1-R3.
 */
#define M_INCLUDE_I2CBB_(label_read, label_write, label_arblost_error_handler, label_nack_error_handler, scl_gpio, sda_gpio, slave_address, reg_data, reg_scratch, reg_return) \
		M_LABEL(label_read), \
			M_MOVL(reg_scratch, label_read), \
			I_ADDI(reg_scratch,reg_scratch,33), \
			I_GPIO_READ(sda_gpio), \
			I_BL(24,1), \
			I_GPIO_READ(scl_gpio), \
			I_BL(22,1), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_MOVI(R0, ((slave_address << 1) | 0) << 8), \
			I_STAGE_RST(), \
			I_BGE(8,1 << 15), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_STAGE_INC(1), \
			I_LSHI(R0,R0,1), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_JUMPS(4,8,JUMPS_GE), \
			I_BGE(-7,0), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_BGE(-7,0), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_GPIO_READ(sda_gpio), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_BGE(2,1), \
			I_BXR(reg_scratch), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_MOVI(reg_scratch, 0), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_BGE(2,1), \
			M_BX(label_arblost_error_handler), \
			M_BX(label_nack_error_handler), \
			I_ADDI(reg_scratch,reg_scratch,5), \
			I_MOVR(R0, reg_data), \
			I_BGE(-26,0), \
			I_ADDI(reg_scratch,reg_scratch,34), \
			I_BGE(-3,0), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_ADDI(reg_scratch,reg_scratch,7), \
			I_MOVI(R0, ((slave_address << 1) | 1) << 8), \
			I_DELAY(10), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_BGE(-35,0), \
			I_STAGE_RST(), \
			I_MOVI(reg_scratch, 0), \
			I_ANDI(R0, reg_data, 1), \
			I_BGE(2,1), \
			I_STAGE_INC(9), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_LSHI(reg_scratch,reg_scratch,1), \
			I_GPIO_READ(sda_gpio), \
			I_ADDR(reg_scratch,reg_scratch,R0), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_STAGE_INC(1), \
			I_JUMPS(4,17,JUMPS_GE), \
			I_JUMPS(-7,8,JUMPS_LT), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_JUMPS(-9,9,JUMPS_GE), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_MOVR(R0, reg_scratch), \
			I_JUMPS(-9,17,JUMPS_LT), \
			I_MOVI(reg_scratch, 0), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_BXR(reg_return), \
		M_LABEL(label_write), \
			M_MOVL(reg_scratch, label_read), \
			I_ADDI(reg_scratch,reg_scratch,36), \
			I_BGE(-67,0), \
			I_ADDI(reg_scratch,reg_scratch,3), \
			I_LSHI(R0, reg_data, 8), \
			I_BGE(-63,0), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_BGE(-11,0)


/*
 * This is a variant of I2CBB that uses a register for the slave address (instead of hardcoding), allowing communication with multiple slave devices.
 *
 * The address of the current I2C slave should be set in the upper 7 bits of SENS_I2C_SLAVE_ADDR6.
 * You can use I_I2CBB_SET_SLAVE(addr) to do this easily in your ULP program.

 * eg. 	I_I2CBB_SET_SLAVE(0x12),							//Set the slave address
 * 		M_I2CBB_WR(LBL_WR_DEV1, LBL_I2C_WRITE, 0x34, 255),	//Perform I2C transactions
 * 		I_I2CBB_SET_SLAVE(0x56),							//Set a different slave address
 * 		M_I2CBB_RD(LBL_RD_DEV2, LBL_I2C_READ, 0x78),		//Perform I2C transactions with new device
*/
#define M_INCLUDE_I2CBB_MULTI(label_read, label_write, label_arblost_error, label_nack_error, scl_gpio, sda_gpio) \
    M_INCLUDE_I2CBB_MULTI_(label_read, label_write, label_arblost_error, label_nack_error, scl_gpio, sda_gpio, R1, R2, R3)

#define M_INCLUDE_I2CBB_MULTI_(label_read, label_write, label_arblost_error_handler, label_nack_error_handler, scl_gpio, sda_gpio, reg_data, reg_scratch, reg_return) \
        M_LABEL(label_read), \
			M_MOVL(reg_scratch, label_read), \
			I_ADDI(reg_scratch,reg_scratch,33), \
			I_GPIO_READ(sda_gpio), \
			I_BL(24,1), \
			I_GPIO_READ(scl_gpio), \
			I_BL(22,1), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_RD_REG(SENS_SAR_SLAVE_ADDR4_REG, 21 - 15, 21), \
			I_STAGE_RST(), \
			I_BGE(8,1 << 15), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_STAGE_INC(1), \
			I_LSHI(R0,R0,1), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_JUMPS(4,8,JUMPS_GE), \
			I_BGE(-7,0), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_BGE(-7,0), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_GPIO_READ(sda_gpio), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_BGE(2,1), \
			I_BXR(reg_scratch), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_MOVI(reg_scratch, 0), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_BGE(2,1), \
			M_BX(label_arblost_error_handler), \
			M_BX(label_nack_error_handler), \
			I_ADDI(reg_scratch,reg_scratch,5), \
			I_MOVR(R0, reg_data), \
			I_BGE(-26,0), \
			I_ADDI(reg_scratch,reg_scratch,34), \
			I_BGE(-3,0), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_ADDI(reg_scratch,reg_scratch,7), \
			I_RD_REG(SENS_SAR_SLAVE_ADDR4_REG, 21 - 15, 21), \
			I_ORI(R0, R0, 1 << 8), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_BGE(-35,0), \
			I_STAGE_RST(), \
			I_MOVI(reg_scratch, 0), \
			I_ANDI(R0, reg_data, 1), \
			I_BGE(2,1), \
			I_STAGE_INC(9), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_LSHI(reg_scratch,reg_scratch,1), \
			I_GPIO_READ(sda_gpio), \
			I_ADDR(reg_scratch,reg_scratch,R0), \
			I_GPIO_OUTPUT_EN(scl_gpio), \
			I_STAGE_INC(1), \
			I_JUMPS(4,17,JUMPS_GE), \
			I_JUMPS(-7,8,JUMPS_LT), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_JUMPS(-9,9,JUMPS_GE), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_MOVR(R0, reg_scratch), \
			I_JUMPS(-9,17,JUMPS_LT), \
			I_MOVI(reg_scratch, 0), \
			I_GPIO_OUTPUT_DIS(sda_gpio), \
			I_BXR(reg_return), \
		M_LABEL(label_write), \
			M_MOVL(reg_scratch, label_read), \
			I_ADDI(reg_scratch,reg_scratch,36), \
			I_BGE(-67,0), \
			I_ADDI(reg_scratch,reg_scratch,3), \
			I_LSHI(R0, reg_data, 8), \
			I_BGE(-63,0), \
			I_GPIO_OUTPUT_EN(sda_gpio), \
			I_GPIO_OUTPUT_DIS(scl_gpio), \
			I_BGE(-11,0)

/**
 * Initialise the header of a I2CBB command array
 *  See HULP_I2C_CMD_1B description for usage
 */
#define HULP_I2C_CMD_HDR(_slave_addr, _slave_reg, _num_bytes) \
    {.val = ((((uint16_t)(_slave_addr)) & 0x7F) << 9) | (((_slave_reg) & 0xFF) << 0)},{.val = (_num_bytes)}

/**
 * Initialise the header of a I2CBB command array for reading without writing pointer
 * By default, a read command consists of a write transaction to set the slave register pointer, followed by the configured read transaction.
 * 		slave_address[WR]+slave_reg + slave_address[RD]+num_bytes...
 * Use this, instead, to skip the pointer write to perform only the read.
 * 		slave_address[RD]+num_bytes...
 *  See HULP_I2C_CMD_1B description for usage
 */
#define HULP_I2C_CMD_HDR_NO_PTR(_slave_addr, _num_bytes) \
    {.val = ((((uint16_t)(_slave_addr)) & 0x7F) << 9) | (1 << 8)},{.val = (_num_bytes)}

/**
 * Initialise two bytes in I2CBB command array
 *  See HULP_I2C_CMD_1B description for usage
 */
#define HULP_I2C_CMD_2B(_first_byte, _second_byte) \
    {.val = (((_first_byte) & 0xFF) << 8) | (((_second_byte) & 0xFF) << 0)}

/**
 * Initialise a byte in I2CBB command array
 *  Should only be used where uneven number of bytes are to be written, else use HULP_I2C_CMD_2B
 * 
 * eg. Initialise a command array to write 5 bytes {0x12, 0x34, 0x56, 0x78, 0x9A} to 0x70:
 * 
 *  RTC_SLOW_ATTR ulp_var_t ulp_write_cmd[] = {
 *      HULP_I2C_CMD_HDR(SLAVE_I2C_ADDRESS, 0x70, 5),
 *      HULP_I2C_CMD_2B(0x12, 0x34),
 *      HULP_I2C_CMD_2B(0x56, 0x78),
 *      HULP_I2C_CMD_1B(0x9A)
 *  };
 */
#define HULP_I2C_CMD_1B(_byte) \
    HULP_I2C_CMD_2B(_byte, 0)

/**
 * Number of header words in I2CBB Command
 * 
 * Can help to access data in a command array.
 *  [0] : Addresses
 *  [1] : Number of bytes to read/write
 *  [2...] : Data
 * 
 * eg. my_read_cmd[HULP_I2C_CMD_DATA_OFFSET + 6].val // Get value of the 7th data byte (index 6)
 */
#define HULP_I2C_CMD_DATA_OFFSET 2

/**
 * Macro for declaring a read command buffer of desired size
 * 
 * eg. Initialise a command array with buffer large enough to read 4 bytes from 0x15:
 * 
 *  RTC_SLOW_ATTR ulp_var_t ulp_read_cmd[HULP_I2C_CMD_BUF_SIZE(4)] = {
 *      HULP_I2C_CMD_HDR(SLAVE_I2C_ADDRESS, 0x15, 4),
 *  };
 */
#define HULP_I2C_CMD_BUF_SIZE(_num_bytes) (HULP_I2C_CMD_DATA_OFFSET + ((_num_bytes) + 1) / 2)

/*
	Allows I2C bitbanging with any number of bytes to read or write.
	Use when needing to write multiple or read many bytes at once.
		eg. for slaves that require 16 bit writes, or to efficiently get large readouts from sensors
	
	See HULP_I2C_CMD_1B description for initialising a write command array.
	See HULP_I2C_CMD_BUF_SIZE description for initialising a read command array.
	
	Flow:
		Prepare reg_ptr with the RTC slow memory offset of the command array
			eg. I_MOVO(R1, example_read_cmd),
		Prepare reg_return with the return address
			eg. M_MOVL(R3, MY_READ_RETURN_LABEL),
		Branch to label_read or label_write for the desired operation.
			eg. M_BX(MY_READ_LABEL)

	Return:
		R0: err code
			0 = Success, -1 = Slave NACK, -2 = Bus Error (checked on first byte only)
		reg_ptr: reg_ptr
		reg_scratch: undefined
		reg_return: reg_return

	Example program:
				// Prepare and branch
				I_MOVO(R1, example_read_cmd),
				M_MOVL(R3, LABEL_I2C_RETURN),
				M_BX(LABEL_I2C_READ),
				M_LABEL(LABEL_I2C_RETURN),
				// Check error code
				M_BGE(LABEL_I2C_ERROR, 1),
				// Check some value you just received and wake if > some threshold
				I_GET(R0, R0, example_read_cmd[HULP_I2C_CMD_DATA_OFFSET]),
				M_BGE(LABEL_WAKE, 1234),
				I_HALT(),
				M_LABEL(LABEL_I2C_ERROR),
				I_GPIO_SET(LED_ERR, 1),
				I_END(),
				M_LABEL(LABEL_WAKE),
				I_WAKE(),
				I_HALT(),
				// Include the subroutine
				M_INCLUDE_I2CBB_CMD(LABEL_I2C_READ, LABEL_I2C_WRITE, GPIO_NUM_25, GPIO_NUM_26)
*/
#define M_INCLUDE_I2CBB_CMD(label_read, label_write, scl_gpio, sda_gpio) \
	M_INCLUDE_I2CBB_CMD_(label_read, label_write, scl_gpio, sda_gpio, R1, R2, R3)

#define M_INCLUDE_I2CBB_CMD_(label_read, label_write, scl_gpio, sda_gpio, reg_ptr, reg_scratch, reg_return) \
	M_LABEL(label_read), \
		I_MOVI(reg_scratch, 39), \
		M_MOVL(R0, label_read), \
		I_ST(reg_return, R0, 90), \
		I_MOVI(reg_return, 0), \
		I_ADDR(reg_scratch, R0, reg_scratch), \
		I_GPIO_READ(sda_gpio), \
		I_BL(26, 1), \
		I_GPIO_READ(scl_gpio), \
		I_BL(24, 1), \
		I_LD(R0, reg_ptr, 0), \
		I_GPIO_OUTPUT_EN(sda_gpio), \
		I_GPIO_OUTPUT_EN(scl_gpio), \
		I_STAGE_RST(), \
		I_BL(17, 32768), \
		I_GPIO_OUTPUT_DIS(sda_gpio), \
		I_STAGE_INC(1), \
		I_GPIO_OUTPUT_DIS(scl_gpio), \
		I_JUMPS(3, 8, JUMPS_LT), \
		I_BL(3, 32768), \
		I_STAGE_INC(50), \
		I_LSHI(R0, R0, 1), \
		I_GPIO_OUTPUT_EN(scl_gpio), \
		I_JUMPS(-9, 8, JUMPS_LT), \
		I_GPIO_OUTPUT_DIS(sda_gpio), \
		I_GPIO_OUTPUT_DIS(scl_gpio), \
		I_GPIO_READ(sda_gpio), \
		I_GPIO_OUTPUT_EN(scl_gpio), \
		I_BGE(5, 1), \
		I_LD(R0, reg_ptr, 0), \
		I_BXR(reg_scratch), \
		I_GPIO_OUTPUT_EN(sda_gpio), \
		I_BGE(-16, 0), \
		I_SUBI(R0, R0, 2), \
		I_GPIO_OUTPUT_EN(sda_gpio), \
		I_GPIO_OUTPUT_DIS(scl_gpio), \
		M_MOVL(reg_return, label_read), \
		I_LD(reg_return, reg_return, 90), \
		I_GPIO_OUTPUT_DIS(sda_gpio), \
		I_BXR(reg_return), \
		I_JUMPS(11, 50, JUMPS_GE), \
		I_ADDI(reg_scratch, reg_scratch, 65504), \
		I_ADDI(reg_scratch, reg_scratch, 37), \
		I_LSHI(R0, R0, 8), \
		I_BGE(-31, 0), \
		I_GPIO_OUTPUT_DIS(scl_gpio), \
		I_ADDI(reg_scratch, reg_scratch, 4), \
		I_ORI(R0, R0, 256), \
		I_BGE(-37, 0), \
		I_GPIO_OUTPUT_EN(scl_gpio), \
		I_GPIO_OUTPUT_DIS(sda_gpio), \
		I_STAGE_RST(), \
		I_GPIO_OUTPUT_DIS(scl_gpio), \
		I_LSHI(reg_scratch, reg_scratch, 1), \
		I_GPIO_READ(sda_gpio), \
		I_ORR(reg_scratch, reg_scratch, R0), \
		I_GPIO_OUTPUT_EN(scl_gpio), \
		I_STAGE_INC(1), \
		I_JUMPS(-6, 8, JUMPS_LT), \
		I_LD(R0, reg_ptr, 1), \
		I_SUBR(R0, R0, reg_return), \
		I_BGE(7, 2), \
		I_GPIO_OUTPUT_DIS(scl_gpio), \
		I_STAGE_INC(1), \
		I_ANDI(R0, reg_return, 1), \
		I_GPIO_OUTPUT_EN(scl_gpio), \
		I_BGE(2, 1), \
		I_LSHI(reg_scratch, reg_scratch, 8), \
		I_GPIO_OUTPUT_EN(sda_gpio), \
		I_RSHI(R0, reg_return, 1), \
		I_ADDR(R0, R0, reg_ptr), \
		I_GPIO_OUTPUT_DIS(scl_gpio), \
		I_ST(reg_scratch, R0, 2), \
		I_ADDI(reg_return, reg_return, 1), \
		I_JUMPS(-25, 9, JUMPS_LT), \
		I_MOVI(R0, 0), \
		I_BGE(-40, 0), \
	M_LABEL(label_write), \
		I_MOVI(reg_scratch, 41), \
		I_BGE(-76, 0), \
		I_LD(R0, reg_ptr, 1), \
		I_SUBR(R0, R0, reg_return), \
		I_BL(-47, 1), \
		I_ANDI(R0, reg_return, 1), \
		I_BL(2, 1), \
		I_STAGE_INC(100), \
		I_RSHI(R0, reg_return, 1), \
		I_ADDR(R0, R0, reg_ptr), \
		I_LD(R0, R0, 2), \
		I_ADDI(reg_return, reg_return, 1), \
		I_JUMPS(-46, 100, JUMPS_GE), \
		I_BGE(-77, 0), \
		I_HALT()

#ifdef __cplusplus
}
#endif

#endif // HULP_I2CBB_H