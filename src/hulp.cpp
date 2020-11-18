#include "hulp.h"

#include <string.h>
#include <math.h>

#include "esp_system.h"
#include "esp_sleep.h"
#include "esp_log.h"
#include "esp32/clk.h"
#include "driver/rtc_cntl.h"
#include "driver/gpio.h"
#include "driver/rtc_io.h"
#include "driver/adc.h"
#include "soc/rtc.h"
#include "soc/rtc_cntl_reg.h"
#include "soc/sens_reg.h"
#include "soc/rtc_i2c_reg.h"
#include "soc/syscon_reg.h"
#include "soc/adc_caps.h"
#include "soc/adc_periph.h"
#include "hal/rtc_io_hal.h"
#include "hal/gpio_types.h"

#include "hulp_ulp.h"

#include "sdkconfig.h"

static const char* TAG = "HULP";

esp_err_t hulp_configure_pin(gpio_num_t pin, rtc_gpio_mode_t mode, gpio_pull_mode_t pull_mode, uint32_t level)
{
    if( ESP_OK != rtc_gpio_set_direction(pin, RTC_GPIO_MODE_DISABLED) ||
        ESP_OK != rtc_gpio_init(pin) ||
        ESP_OK != gpio_set_pull_mode(pin, pull_mode) ||
        ESP_OK != rtc_gpio_set_level(pin, level) ||
        ESP_OK != rtc_gpio_set_direction(pin, mode)
    )
    {
        ESP_LOGE(TAG, "configure gpio %d failed", pin);
        return ESP_FAIL;
    }
    return ESP_OK;
}

int hulp_adc_get_periph_index(gpio_num_t pin)
{
    for(int periph = 0; periph < SOC_ADC_PERIPH_NUM; ++periph)
    {
        for(int channel = 0; channel < SOC_ADC_MAX_CHANNEL_NUM; ++channel)
        {
            if(adc_channel_io_map[periph][channel] == pin) return periph;
        }
    }
    ESP_LOGE(TAG, "no ADC periph for gpio %d", pin);
    return -1;
}

int hulp_adc_get_channel_num(gpio_num_t pin)
{
    for(int periph = 0; periph < SOC_ADC_PERIPH_NUM; ++periph)
    {
        for(int channel = 0; channel < SOC_ADC_MAX_CHANNEL_NUM; ++channel)
        {
            if(adc_channel_io_map[periph][channel] == pin) return channel;
        }
    }
    ESP_LOGE(TAG, "no ADC channel for gpio %d", pin);
    return -1;
}

esp_err_t hulp_configure_analog_pin(gpio_num_t pin, adc_atten_t attenuation, adc_bits_width_t width)
{
    int adc_unit_index = hulp_adc_get_periph_index(pin);
    int adc_channel = hulp_adc_get_channel_num(pin);
    
    if(adc_unit_index < 0 || adc_channel < 0)
    {
        ESP_LOGE(TAG, "invalid ADC pin %d (%d, %d)", pin, adc_unit_index, adc_channel);
        return ESP_ERR_INVALID_ARG;
    }
    
    if(adc_unit_index == 0)
    {
        adc1_config_channel_atten((adc1_channel_t)adc_channel, attenuation); //Does adc_gpio_init() internally
        adc1_config_width(width);
        adc1_ulp_enable();
    }
    else
    {
        adc2_config_channel_atten((adc2_channel_t)adc_channel, attenuation); //Does adc2_pad_init() internally
        //Do a read in order to set some regs
        int adc2val;
        adc2_get_raw((adc2_channel_t)adc_channel, width, &adc2val);
        //Equivalent of adc_set_controller( ADC_UNIT_2, ADC_CTRL_ULP )
        REG_CLR_BIT(SENS_SAR_MEAS_START2_REG, SENS_MEAS2_START_FORCE);
        REG_CLR_BIT(SENS_SAR_MEAS_START2_REG, SENS_SAR2_EN_PAD_FORCE);
        REG_CLR_BIT(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_DIG_FORCE);
        REG_CLR_BIT(SENS_SAR_READ_CTRL2_REG, SENS_SAR2_PWDET_FORCE);
        REG_SET_BIT(SYSCON_SARADC_CTRL_REG, SYSCON_SARADC_SAR2_MUX);
    }
    return ESP_OK;
}

#define RTCIO_FUNC_RTC_I2C 0x3

esp_err_t hulp_configure_i2c_pins(gpio_num_t scl_pin, gpio_num_t sda_pin, bool scl_pullup, bool sda_pullup)
{
    if( !(scl_pin == GPIO_NUM_2 || scl_pin == GPIO_NUM_4) )
    {
        ESP_LOGE(TAG, "invalid i2c hw SCL pin %d, must be 2 or 4", scl_pin);
        return ESP_ERR_INVALID_ARG;
    }
    if( !(sda_pin == GPIO_NUM_0 || sda_pin == GPIO_NUM_15) )
    {
        ESP_LOGE(TAG, "invalid i2c hw SDA pin %d, must be 0 or 15", sda_pin);
        return ESP_ERR_INVALID_ARG;
    }

    ESP_ERROR_CHECK(hulp_configure_pin(scl_pin, RTC_GPIO_MODE_INPUT_ONLY, scl_pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING, 0));
    SET_PERI_REG_BITS(rtc_io_desc[hulp_gtr(scl_pin)].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, RTCIO_FUNC_RTC_I2C, rtc_io_desc[hulp_gtr(scl_pin)].func);
    ESP_ERROR_CHECK(hulp_configure_pin(sda_pin, RTC_GPIO_MODE_INPUT_ONLY, sda_pullup ? GPIO_PULLUP_ONLY : GPIO_FLOATING, 0));
    SET_PERI_REG_BITS(rtc_io_desc[hulp_gtr(sda_pin)].reg, RTC_IO_TOUCH_PAD1_FUN_SEL_V, RTCIO_FUNC_RTC_I2C, rtc_io_desc[hulp_gtr(sda_pin)].func);

    REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SCL_SEL, scl_pin == GPIO_NUM_4 ? 0 : 1);
    REG_SET_FIELD(RTC_IO_SAR_I2C_IO_REG, RTC_IO_SAR_I2C_SDA_SEL, sda_pin == GPIO_NUM_0 ? 0 : 1);
    return ESP_OK;
}

void hulp_configure_i2c_controller(uint32_t scl_low, uint32_t scl_high, uint32_t sda_duty, uint32_t scl_start, uint32_t scl_stop, uint32_t timeout, bool scl_pushpull, bool sda_pushpull, bool rx_lsbfirst, bool tx_lsbfirst)
{
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_RX_LSB_FIRST, rx_lsbfirst ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_TX_LSB_FIRST, tx_lsbfirst ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SCL_FORCE_OUT, scl_pushpull ? 1 : 0);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_SDA_FORCE_OUT, sda_pushpull ? 1 : 0);

    REG_SET_FIELD(RTC_I2C_SCL_LOW_PERIOD_REG, RTC_I2C_SCL_LOW_PERIOD, scl_low);
    REG_SET_FIELD(RTC_I2C_SCL_HIGH_PERIOD_REG, RTC_I2C_SCL_HIGH_PERIOD, scl_high);
    REG_SET_FIELD(RTC_I2C_SDA_DUTY_REG, RTC_I2C_SDA_DUTY, sda_duty);
    REG_SET_FIELD(RTC_I2C_SCL_START_PERIOD_REG, RTC_I2C_SCL_START_PERIOD, scl_start);
    REG_SET_FIELD(RTC_I2C_SCL_STOP_PERIOD_REG, RTC_I2C_SCL_STOP_PERIOD, scl_stop);
    REG_SET_FIELD(RTC_I2C_TIMEOUT_REG, RTC_I2C_TIMEOUT, timeout);
    REG_SET_FIELD(RTC_I2C_CTRL_REG, RTC_I2C_MS_MODE, 1);
}

esp_err_t hulp_register_i2c_slave(uint8_t index, uint8_t address)
{
    if(index > 7)
    {
        ESP_LOGE(TAG, "invalid i2c slave index (%u), range 0-7", index);
        return ESP_ERR_INVALID_ARG;
    }
    SET_PERI_REG_BITS(SENS_SAR_SLAVE_ADDR1_REG + (index / 2) * sizeof(uint32_t), SENS_I2C_SLAVE_ADDR0, address, (index % 2) ? SENS_I2C_SLAVE_ADDR1_S : SENS_I2C_SLAVE_ADDR0_S);
    return ESP_OK;
}

void hulp_tsens_configure(uint8_t clk_div)
{
    REG_SET_FIELD(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_CLK_DIV, clk_div);
    REG_SET_FIELD(SENS_SAR_MEAS_WAIT2_REG, SENS_FORCE_XPD_SAR, SENS_FORCE_XPD_SAR_PU);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_DUMP_OUT);
    REG_CLR_BIT(SENS_SAR_TSENS_CTRL_REG, SENS_TSENS_POWER_UP_FORCE);
}

void hulp_peripherals_on()
{
    esp_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_ON);
}
void hulp_configure_hall_effect_sensor()
{
    //GPIO 36
    adc1_config_channel_atten(ADC1_CHANNEL_0, ADC_ATTEN_DB_6);
    //GPIO 39
    adc1_config_channel_atten(ADC1_CHANNEL_3, ADC_ATTEN_DB_6);
    adc1_config_width(ADC_WIDTH_BIT_12);
    adc1_ulp_enable();
    REG_SET_BIT(SENS_SAR_TOUCH_CTRL1_REG, SENS_HALL_PHASE_FORCE);
    REG_SET_BIT(SENS_SAR_TOUCH_CTRL1_REG, SENS_XPD_HALL_FORCE);
    //Connect sensor to 36 and 39
    REG_SET_BIT(RTC_IO_HALL_SENS_REG, RTC_IO_XPD_HALL);
}

void hulp_clear_program_memory()
{
    memset(RTC_SLOW_MEM, 0, ULP_RESERVE_MEM);
}

void hulp_clear_rtc_slow_memory()
{
    memset(RTC_SLOW_MEM, 0, 0x1000);
}

static uint64_t hulp_us_to_ticks(uint64_t time_us)
{
    return rtc_time_us_to_slowclk(time_us, esp_clk_slowclk_cal_get());
}

uint16_t hulp_ms_to_ulp_ticks_with_shift(uint32_t time_ms, uint8_t shift)
{
    return (uint16_t)((hulp_us_to_ticks(1000ULL * time_ms) >> shift) & 0xFFFF);
}

uint16_t hulp_ms_to_ulp_ticks(uint32_t time_ms)
{
    return hulp_ms_to_ulp_ticks_with_shift(time_ms, hulp_ms_to_ulp_tick_shift(time_ms));
}

uint16_t hulp_get_current_ulp_ticks(uint8_t shift)
{
    return (uint16_t)((rtc_time_get() >> shift) & 0xFFFF);
}

uint8_t hulp_ms_to_ulp_tick_shift(uint32_t time_ms)
{
    uint64_t rtc_slow_ticks = hulp_us_to_ticks(1000ULL * time_ms);
    uint8_t high_bit = log2(rtc_slow_ticks);
    if(high_bit >= 32)
    {
        high_bit = 47;  // [47:32]
    }
    else if(high_bit < 16)
    {
        high_bit = 16; // if high_bit < 16, get all lower 16 bits. Note: tick count is updated every 2 ticks, so bit 0 is not interesting, therefore [16:1] rather than [15:0]
    } // else 32 > high_bit > 16. ie. [31:16] - [16:1]
    ESP_LOGV(TAG, "%s ms: %u, ticks: %llu, range: [%u:%u], overflow: %llu ms, resolution: %llu uS", __func__, time_ms, rtc_slow_ticks, high_bit, high_bit - 15, rtc_time_slowclk_to_us((1ULL << (high_bit+1))-1,esp_clk_slowclk_cal_get()) / 1000, rtc_time_slowclk_to_us(1ULL << (high_bit-15),esp_clk_slowclk_cal_get()));
    return (high_bit - 15);
}

uint16_t hulp_get_label_pc(uint16_t label, const ulp_insn_t *program)
{
	uint16_t pc = 0;

    while(pc < ULP_RESERVE_MEM)
    {
        if(program->macro.opcode == OPCODE_MACRO)
        {
            if(program->macro.sub_opcode == SUB_OPCODE_MACRO_LABEL && program->macro.label == label)
            {
				ESP_LOGD(TAG, "label %u at pc %u", label, pc);
                break;
            }
        }
        else
        {
            ++pc;
        }
		++program;
    }

    assert(pc < ULP_RESERVE_MEM && "label not found");
    return pc;
}

static uint32_t periph_sel_to_reg_base(uint32_t sel) {
	uint32_t ret = 3;
	if(sel == 0) {
		ret = DR_REG_RTCCNTL_BASE;
	} else if (sel == 1) {
		ret = DR_REG_RTCIO_BASE;
	} else if (sel == 2) {
		ret = DR_REG_SENS_BASE;
	} else if (sel == 3) {
		ret = DR_REG_RTC_I2C_BASE;
	} else {
		assert(0 && "invalid periph sel");
	}
	return ret;
}

#define HULP_DUMP_INSN_STR(x) x ",\n"

static int print_insn(const ulp_insn_t *ins)
{
	switch(ins->b.opcode)
	{
		case OPCODE_WR_REG:
		{
			return printf(HULP_DUMP_INSN_STR("I_WR_REG(0x%08X, %u, %u, %u)"), 
				periph_sel_to_reg_base(ins->wr_reg.periph_sel) + ins->wr_reg.addr * sizeof(uint32_t),
				ins->wr_reg.low,
				ins->wr_reg.high,
				ins->wr_reg.data
			);
		}
		case OPCODE_RD_REG:
		{
			return printf(HULP_DUMP_INSN_STR("I_RD_REG(0x%08X, %u, %u)"), 
				periph_sel_to_reg_base(ins->rd_reg.periph_sel) + ins->rd_reg.addr * sizeof(uint32_t),
				ins->rd_reg.low,
				ins->rd_reg.high
			);
		}
		case OPCODE_I2C:
		{
			return printf(HULP_DUMP_INSN_STR("I_I2C_RW(%u, %u, %u, %u, %u, %u)"),
				ins->i2c.i2c_addr,
				ins->i2c.data,
				ins->i2c.low_bits,
				ins->i2c.high_bits,
				ins->i2c.i2c_sel,
				ins->i2c.rw
			);
		}
		case OPCODE_DELAY:
		{
			return printf(HULP_DUMP_INSN_STR("I_DELAY(%u)"),
				ins->delay.cycles
			);
		}
		case OPCODE_ADC:
		{
			return printf(HULP_DUMP_INSN_STR("I_ADC(R%u, %u, %u)"),
				ins->adc.dreg,
				ins->adc.sar_sel,
				ins->adc.mux - 1
			);
		}
		case OPCODE_ST:
		{
			return printf(HULP_DUMP_INSN_STR("I_ST(R%u, R%u, %u)"),
				ins->st.dreg,
				ins->st.sreg,
				ins->st.offset
			);
		}
		case OPCODE_ALU:
		{
			switch(ins->alu_reg.sub_opcode)
			{
				case SUB_OPCODE_ALU_REG:
				{
					switch(ins->alu_reg.sel)
					{
						case ALU_SEL_ADD:
						{
							return printf(HULP_DUMP_INSN_STR("I_ADDR(R%u, R%u, R%u)"),
								ins->alu_reg.dreg,
								ins->alu_reg.sreg,
								ins->alu_reg.treg
							);
						}
						case ALU_SEL_SUB:
						{
							return printf(HULP_DUMP_INSN_STR("I_SUBR(R%u, R%u, R%u)"),
								ins->alu_reg.dreg,
								ins->alu_reg.sreg,
								ins->alu_reg.treg
							);
						}
						case ALU_SEL_AND:
						{
							return printf(HULP_DUMP_INSN_STR("I_ANDR(R%u, R%u, R%u)"),
								ins->alu_reg.dreg,
								ins->alu_reg.sreg,
								ins->alu_reg.treg
							);
						}
						case ALU_SEL_OR:
						{
							return printf(HULP_DUMP_INSN_STR("I_ORR(R%u, R%u, R%u)"),
								ins->alu_reg.dreg,
								ins->alu_reg.sreg,
								ins->alu_reg.treg
							);
						}
						case ALU_SEL_MOV:
						{
							return printf(HULP_DUMP_INSN_STR("I_MOVR(R%u, R%u)"),
								ins->alu_reg.dreg,
								ins->alu_reg.sreg
							);
						}
						case ALU_SEL_LSH:
						{
							return printf(HULP_DUMP_INSN_STR("I_LSHR(R%u, R%u, R%u)"),
								ins->alu_reg.dreg,
								ins->alu_reg.sreg,
								ins->alu_reg.treg
							);
						}
						case ALU_SEL_RSH:
						{
							return printf(HULP_DUMP_INSN_STR("I_RSHR(R%u, R%u, R%u)"),
								ins->alu_reg.dreg,
								ins->alu_reg.sreg,
								ins->alu_reg.treg
							);
						}
						default:
							assert(0 && "unknown alu_reg operation");
					}
					break;
				}
				case SUB_OPCODE_ALU_IMM:
				{
					switch(ins->alu_imm.sel)
					{
						case ALU_SEL_ADD:
						{
							return printf(HULP_DUMP_INSN_STR("I_ADDI(R%u, R%u, %u)"),
								ins->alu_imm.dreg,
								ins->alu_imm.sreg,
								ins->alu_imm.imm
							);
						}
						case ALU_SEL_SUB:
						{
							return printf(HULP_DUMP_INSN_STR("I_SUBI(R%u, R%u, %u)"),
								ins->alu_imm.dreg,
								ins->alu_imm.sreg,
								ins->alu_imm.imm
							);
						}
						case ALU_SEL_AND:
						{
							return printf(HULP_DUMP_INSN_STR("I_ANDI(R%u, R%u, %u)"),
								ins->alu_imm.dreg,
								ins->alu_imm.sreg,
								ins->alu_imm.imm
							);
						}
						case ALU_SEL_OR:
						{
							return printf(HULP_DUMP_INSN_STR("I_ORI(R%u, R%u, %u)"),
								ins->alu_imm.dreg,
								ins->alu_imm.sreg,
								ins->alu_imm.imm
							);
						}
						case ALU_SEL_MOV:
						{
							return printf(HULP_DUMP_INSN_STR("I_MOVI(R%u, %u)"),
								ins->alu_imm.dreg,
								ins->alu_imm.imm
							);
						}
						case ALU_SEL_LSH:
						{
							return printf(HULP_DUMP_INSN_STR("I_LSHI(R%u, R%u, %u)"),
								ins->alu_imm.dreg,
								ins->alu_imm.sreg,
								ins->alu_imm.imm
							);
						}
						case ALU_SEL_RSH:
						{
							return printf(HULP_DUMP_INSN_STR("I_RSHI(R%u, R%u, %u)"),
								ins->alu_imm.dreg,
								ins->alu_imm.sreg,
								ins->alu_imm.imm
							);
						}
						default:
							assert(0 && "unknown alu_imm operation");
					}
					break;
				}
				case SUB_OPCODE_ALU_CNT:
				{
					switch(ins->alu_reg_s.sel)
					{
						case ALU_SEL_SINC:
						{
							return printf(HULP_DUMP_INSN_STR("I_STAGE_INC(%u)"),
								ins->alu_reg_s.imm
							);
						}
						case ALU_SEL_SDEC:
						{
							return printf(HULP_DUMP_INSN_STR("I_STAGE_DEC(%u)"),
								ins->alu_reg_s.imm
							);
						}
						case ALU_SEL_SRST:
						{
							return printf(HULP_DUMP_INSN_STR("I_STAGE_RST()"));
						}
						default:
							assert(0 && "unknown alu_reg_s operation");
					}
					break;
				}
				default:
					assert(0 && "unknown alu subopcode");
			}
			break;
		}
		case OPCODE_BRANCH:
		{
			switch(ins->b.sub_opcode)
			{
				case SUB_OPCODE_BX:
				{
					switch(ins->bx.type)
					{
						case BX_JUMP_TYPE_DIRECT:
						{
							if(ins->bx.reg)
							{
								return printf(HULP_DUMP_INSN_STR("I_BXR(R%u)"),
									ins->bx.dreg
								);
							}
							else
							{
								return printf(HULP_DUMP_INSN_STR("I_BXI(%u)"),
									ins->bx.addr
								);
							}
						}
						case BX_JUMP_TYPE_ZERO:
						{
							if(ins->bx.reg)
							{
								return printf(HULP_DUMP_INSN_STR("I_BXZR(R%u)"),
									ins->bx.dreg
								);
							}
							else
							{
								return printf(HULP_DUMP_INSN_STR("I_BXZI(%u)"),
									ins->bx.addr
								);
							}
						}
						case BX_JUMP_TYPE_OVF:
						{
							if(ins->bx.reg)
							{
								return printf(HULP_DUMP_INSN_STR("I_BXFR(R%u)"),
									ins->bx.dreg
								);
							}
							else
							{
								return printf(HULP_DUMP_INSN_STR("I_BXFI(%u)"),
									ins->bx.addr
								);
							}
						}
						default:
							assert(0 && "unknown bx type");
					}
					break;
				}
				case SUB_OPCODE_BR:
				{
					if(ins->b.cmp == B_CMP_L)
					{
						return printf(HULP_DUMP_INSN_STR("I_BL(%s%u, %u)"),
							ins->b.sign ? "-" : "",
							ins->b.offset,
							ins->b.imm
						);
					}
					else
					{
						return printf(HULP_DUMP_INSN_STR("I_BGE(%s%u, %u)"),
							ins->b.sign ? "-" : "",
							ins->b.offset,
							ins->b.imm
						);
					}
				}
				case SUB_OPCODE_BS:
				{
					return printf(HULP_DUMP_INSN_STR("I_JUMPS(%s%u, %u, %s)"),
						ins->bs.sign ? "-" : "",
						ins->bs.offset,
						ins->bs.imm,
							(	ins->bs.cmp == JUMPS_LT ? 	"JUMPS_LT" :
							(	ins->bs.cmp == JUMPS_GE ? 	"JUMPS_GE" :
															"JUMPS_LE"
							))
					);
				}
				default:
					assert(0 && "unknown branch subopcode");
			}
			break;
		}
		case OPCODE_END:
		{
			switch(ins->end.sub_opcode)
			{
				case SUB_OPCODE_END:
				{
					return printf(HULP_DUMP_INSN_STR("I_WAKE()"));
				}
				case SUB_OPCODE_SLEEP:
				{
					return printf(HULP_DUMP_INSN_STR("I_SLEEP_CYCLE_SEL(%u)"),
						ins->sleep.cycle_sel
					);
				}
				default:
					assert(0 && "unknown end subopcode");
			}
			break;
		}
		case OPCODE_TSENS:
		{
			return printf(HULP_DUMP_INSN_STR("I_TSENS(R%u, %u)"),
				ins->tsens.dreg,
				ins->tsens.wait_delay
			);
		}
		case OPCODE_HALT:
		{
			return printf(HULP_DUMP_INSN_STR("I_HALT()"));
		}
		case OPCODE_LD:
		{
			return printf(HULP_DUMP_INSN_STR("I_LD(R%u, R%u, %u)"),
				ins->ld.dreg,
				ins->ld.sreg,
				ins->ld.offset
			);
		}
		default:
			assert(0 && "unknown opcode");
	}
	assert(0);
}

void hulp_dump_program(uint32_t start_offset, size_t num_instructions)
{
	assert((start_offset + num_instructions) < ULP_RESERVE_MEM);

	const ulp_insn_t *p = (const ulp_insn_t*)&RTC_SLOW_MEM[start_offset];
	const ulp_insn_t *end = (const ulp_insn_t*)&RTC_SLOW_MEM[start_offset + num_instructions];

	while(p < end)
	{
		print_insn(p);
		++p;
	}
}

esp_err_t hulp_ulp_run(uint32_t entry_point)
{
    return ulp_run(entry_point);
}

esp_err_t hulp_ulp_run_once(uint32_t entry_point)
{
    // disable ULP timer
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
    // wait for at least 1 RTC_SLOW_CLK cycle
    ets_delay_us(10);
    // set entry point
    REG_SET_FIELD(SENS_SAR_START_FORCE_REG, SENS_PC_INIT, entry_point);
    // enable SW start
    SET_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_FORCE_START_TOP);
    // make sure voltage is raised when RTC 8MCLK is enabled
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_I2C_FOLW_8M);
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_CORE_FOLW_8M);
    SET_PERI_REG_MASK(RTC_CNTL_OPTIONS0_REG, RTC_CNTL_BIAS_SLEEP_FOLW_8M);
    // start
    CLEAR_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_START_TOP);
    SET_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_ULP_CP_START_TOP);
    return ESP_OK;
}

esp_err_t hulp_ulp_load(const ulp_insn_t *program, size_t size_of_program, uint32_t period_us, uint32_t entry_point)
{
    size_t num_words = size_of_program / sizeof(ulp_insn_t);
    esp_err_t err = ulp_process_macros_and_load(entry_point, program, &num_words);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to load program");
        return err;
    }
    ulp_set_wakeup_period(0, period_us);
    return ESP_OK;
}

void hulp_ulp_end()
{
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
}

bool hulp_is_deep_sleep_wakeup()
{
    return (esp_reset_reason() == ESP_RST_DEEPSLEEP);
}

bool hulp_is_ulp_wakeup()
{
    return (esp_sleep_get_wakeup_cause() == ESP_SLEEP_WAKEUP_ULP);
}

esp_err_t hulp_ulp_isr_register(intr_handler_t handler, void* handler_arg)
{
    esp_err_t err = rtc_isr_register(handler, handler_arg, RTC_CNTL_SAR_INT_ST_M);
    if(err != ESP_OK)
    {
        return err;
    }
    return ESP_OK;
}

esp_err_t hulp_ulp_isr_deregister(intr_handler_t handler, void* handler_arg)
{
    return rtc_isr_deregister(handler, handler_arg);
}

void hulp_ulp_interrupt_en()
{
    REG_SET_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
}

void hulp_ulp_interrupt_dis()
{
    REG_CLR_BIT(RTC_CNTL_INT_ENA_REG, RTC_CNTL_ULP_CP_INT_ENA_M);
}

esp_err_t hulp_configure_pin_int(gpio_num_t gpio_num, gpio_int_type_t intr_type)
{
    //See rtc_gpio_wakeup_enable
    int rtcio = rtc_io_number_get(gpio_num);
    if(rtcio < 0)
    {
        ESP_LOGE(TAG, "invalid rtcio (gpio %d)", gpio_num);
        return ESP_ERR_INVALID_ARG;
    }

    //edge interrupts work, however all behave as if GPIO_INTR_ANYEDGE
    if(intr_type == GPIO_INTR_POSEDGE || intr_type == GPIO_INTR_NEGEDGE)
    {
        ESP_LOGE(TAG, "POSEDGE and NEGEDGE not supported; use ANYEDGE");
        return ESP_ERR_INVALID_ARG;
    }

    REG_SET_FIELD(RTC_GPIO_PIN0_REG + rtcio * sizeof(uint32_t), RTC_GPIO_PIN0_INT_TYPE, intr_type);
    return ESP_OK;
}

ulp_state_t hulp_get_state()
{
    uint32_t ulp_state_bits = REG_READ(RTC_CNTL_LOW_POWER_ST_REG) & (0xF << 13);

    switch(ulp_state_bits)
    {
        case 0:
            return ULP_STATE_IDLE;
        case BIT(13) |  BIT(14):
            return ULP_STATE_RUNNING;
        case BIT(13) |  BIT(14) |             BIT(16):
            return ULP_STATE_HALTED;
        case                        BIT(15) | BIT(16):
            return ULP_STATE_SLEEPING;
        case            BIT(14) |             BIT(16):
        case            BIT(14) |   BIT(15) | BIT(16):
        case BIT(13) |  BIT(14) |   BIT(15) | BIT(16): //if sleep time ~0
            return ULP_STATE_WAKING;
        case                                  BIT(16):
            return ULP_STATE_DONE;
        default:
            ESP_LOGW(TAG, "unknown state: %u", ulp_state_bits);
            return ULP_STATE_UNKNOWN;
    }
}

uint32_t hulp_get_fast_clk_freq(uint32_t slow_clk_cycles)
{
#ifdef CONFIG_HULP_USE_APPROX_FAST_CLK
    return (uint32_t)RTC_FAST_CLK_FREQ_APPROX;
#else
    const bool clk_8m_enabled = rtc_clk_8m_enabled();
    const bool clk_8md256_enabled = rtc_clk_8md256_enabled();
    if (!clk_8m_enabled || !clk_8md256_enabled) {
        rtc_clk_8m_enable(true, true);
    }
    uint32_t ret = (uint32_t)(1000000ULL * (1 << RTC_CLK_CAL_FRACT) * 256 / rtc_clk_cal(RTC_CAL_8MD256, slow_clk_cycles));
    if (!clk_8m_enabled || !clk_8md256_enabled) {
        rtc_clk_8m_enable(clk_8m_enabled, clk_8md256_enabled);
    }
    return ret;
#endif
}