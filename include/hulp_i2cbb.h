#ifndef HULP_I2CBB_H
#define HULP_I2CBB_H

//I2C bitbanger. Read (8/16 bit), write, multiple slaves, lightweight (~76 instr), basic error handling.
//Timing is fixed (Frequency: ~150kHz)
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "soc/sens_reg.h"

#include "hulp.h"

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

#endif // HULP_I2CBB_H