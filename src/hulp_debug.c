#include "hulp_debug.h"

#include "esp_log.h"
#include "soc/rtc_cntl_reg.h"

#include "hulp.h"

#include "sdkconfig.h"

static const char* TAG = "HULP-DBG";

typedef struct {
    uint16_t label_num;
    uint16_t pc;
} hulp_debug_label_pc_pair_t;

struct hulp_debug_bp_state_t {
    ulp_debug_bp_data_t* data;
    struct {
        hulp_debug_label_pc_pair_t* pc_pairs;
        size_t num;
    } labels;
    struct {
        hulp_debug_bp_cb_t fn;
        void* ctx;
    } callback;
};

/**
 * To reduce ULP/RTC_MEM overhead, there are only 3 ST instructions for the 4 registers (ie. scratch reg is excluded).
 * For debugging ease, this parses them into an array of 4 uint16_t, setting scratch reg to its index (eg. R2 = 2).
 */
static void hulp_debug_parse_data_regs_inserting_scratch_val(const ulp_debug_bp_data_t* data, uint16_t* dest)
{
    const uint16_t scr_reg = data->scr.val;
    
    for(uint16_t cur_reg = R0; cur_reg <= R3; ++cur_reg)
    {
        if(cur_reg == scr_reg)
        {
            dest[cur_reg] = cur_reg;
        }
        else
        {
            dest[cur_reg] = data->reg[cur_reg - (cur_reg > scr_reg ? 1 : 0)].val; //if reg > scratch reg then move array index back one to compensate
        }
    }
}

void hulp_debug_bp_continue(hulp_debug_bp_cb_data_t* bp_data)
{
    //Use a known I_ST in the M_INCLUDE... macro to set an anchor point. Other key addresses can then be located by their offset.
    const uint16_t inc_st_pc = bp_data->meta.handle->data->scr.pc;

    //There are 3 reserved instructions at the default re-entry point.
    //Two are WR_REG reserved for the ULP to restore its original entry point (in case user does a normal HALT and reasonably expects ULP to begin again at normal entry)
    //ULP can only do 8 bit reg writes but (very) fringe cases where user may have entry point at very high offset (11 bits), so need 2 instructions to be sure.
    ulp_insn_t* reserved_insn = (ulp_insn_t*)(RTC_SLOW_MEM + inc_st_pc + HULP_DEBUG_BP_INC_ST_REENTRY_DEFAULT_OFFSET);
    //Alter instruction for upper 3 bits
    reserved_insn->wr_reg.data = (uint8_t)(((bp_data->meta.config_backup.entry_point) >> 8) & 0x7);
    ++reserved_insn;
    //Alter next instruction for lower 8 bits
    reserved_insn->wr_reg.data = (uint8_t)((bp_data->meta.config_backup.entry_point) & 0xFF);
    ++reserved_insn;
    //The third reserved instruction is a branch to the address to continue execution from (typically after breakpoint (default) but may have been altered).
    reserved_insn->bx.addr = bp_data->meta.return_addr;

    //Reload the register values from ULP data and compare with BP info to see if altered. Faster for SoC to skip ULP a few instructions ahead than for ULP to load values again even if unaltered.
    uint16_t check_reg_vals[4];
    hulp_debug_parse_data_regs_inserting_scratch_val(bp_data->meta.handle->data, check_reg_vals);
    bool regs_dirty = (
        bp_data->regs.r0 != check_reg_vals[0] ||
        bp_data->regs.r1 != check_reg_vals[1] ||
        bp_data->regs.r2 != check_reg_vals[2] ||
        bp_data->regs.r3 != check_reg_vals[3] );

    // Calculate reentry address based on default (reset entry point then branch) or dirty (reload registers then default)
    uint32_t reentry_point = inc_st_pc;
    if(regs_dirty)
    {
        reentry_point += HULP_DEBUG_BP_INC_ST_REENTRY_DIRTY_OFFSET;
    }
    else
    {
        reentry_point += HULP_DEBUG_BP_INC_ST_REENTRY_DEFAULT_OFFSET;
    }

    // Restore ULP wakeup timer based on state before the breakpoint disabled it. It will be enabled again here via ulp_run.
    if(bp_data->meta.config_backup.timer_en)
    {
        hulp_ulp_run(reentry_point);
    }
    else
    {
        hulp_ulp_run_once(reentry_point);
    }
}

void hulp_debug_bp_print_info(hulp_debug_bp_cb_data_t* bp_data)
{
    ets_printf("D (%u) %s: BP:\t%s%5u\t\t%s%5u\t\t%s%5u\t\t%s%5u\t(PC: %5u, Lab: %5u%s, Line: %5u)\n",
                esp_log_timestamp(),
                TAG,
                bp_data->meta.reg_scr == R0 ? "*" : "", bp_data->regs.r0,
                bp_data->meta.reg_scr == R1 ? "*" : "", bp_data->regs.r1,
                bp_data->meta.reg_scr == R2 ? "*" : "", bp_data->regs.r2,
                bp_data->meta.reg_scr == R3 ? "*" : "", bp_data->regs.r3,
                bp_data->bp.pc,
                bp_data->bp.label.num, bp_data->bp.label.valid ? "" : "*",
                bp_data->bp.line
    );
}

esp_err_t hulp_debug_bp_alter_reg(hulp_debug_bp_cb_data_t* bp_data, uint8_t reg, uint16_t val)
{
    if(reg > R3)
    {
        return ESP_ERR_INVALID_ARG;
    }

    uint8_t scr_reg = bp_data->meta.reg_scr;

    if(reg == scr_reg)
    {
        return ESP_ERR_INVALID_STATE;
    }

    uint8_t arr_ind = reg;
    if(reg > scr_reg) --arr_ind;

    bp_data->meta.handle->data->reg[arr_ind].val = val;
    return ESP_OK;
}

void hulp_debug_bp_callback_default(hulp_debug_bp_cb_data_t* bp_data, void *ctx)
{
    hulp_debug_bp_print_info(bp_data);
    hulp_debug_bp_continue(bp_data);
}

/**
 * Intermediate isr to transpose data set by ulp into a hulp_debug_bp_cb_data_t using metadata and info from config.
 * This is then passed to the callback.
 */
static void hulp_debug_isr_handler(void* ctx)
{
    hulp_debug_bp_handle_t handle = (hulp_debug_bp_handle_t)ctx;

    if(handle->data->marker.pc == 0)
    {
        //No breakpoint data pending. Must be non-breakpoint wake (ie. normal user program I_WAKE). Ignore.
        return;
    }

    hulp_debug_bp_cb_data_t bp_data = {};

    //Backup current config then immediately disable ULP timer to prevent it waking again while this breakpoint is processed.
    // - If this isr is not serviced quickly enough for some reason, there is potential for the ULP to wake again before we are able to disable the timer here, creating a mess. Unlikely/impossible?
        //Other options:
            // - Leave timer setting / enable timer: Not practical. Would require a sufficiently long ULP wakeup interval, else ULP may wake again before debugging complete -> mess.
            // - Always disable timer: If user's ULP program executes a normal halt, the ULP will not wake again if debugger has disabled timer, causing unexpected behaviour. Would require altering halt to perform a timer enable or trigger interrupt so debugger can handle it -> messy, inconvenient
            // - ULP disables its wakeup timer before interrupting: This would require overwriting R0 in order to get the current config, or losing current config -> not ideal as R0 is most likely register to want debugged (though could be worked around) and adds ULP overhead. Best option if current method is unreliable.

#if CONFIG_IDF_TARGET_ESP32
    bp_data.meta.config_backup.timer_en = GET_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
    CLEAR_PERI_REG_MASK(RTC_CNTL_STATE0_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
    bp_data.meta.config_backup.entry_point = GET_PERI_REG_MASK(SENS_SAR_START_FORCE_REG, SENS_PC_INIT_M);
#elif CONFIG_IDF_TARGET_ESP32S2
    bp_data.meta.config_backup.timer_en = REG_GET_FIELD(RTC_CNTL_ULP_CP_TIMER_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
    REG_CLR_BIT(RTC_CNTL_ULP_CP_TIMER_REG, RTC_CNTL_ULP_CP_SLP_TIMER_EN);
    bp_data.meta.config_backup.entry_point = REG_GET_FIELD(RTC_CNTL_ULP_CP_TIMER_REG, RTC_CNTL_ULP_CP_PC_INIT);
#else
    #error "unsupported"
#endif

    //Get which register is scratch
    bp_data.meta.reg_scr = handle->data->scr.val;

    //Get the general purpose registers (scratch reg will be set to a default value)
    hulp_debug_parse_data_regs_inserting_scratch_val(handle->data, (uint16_t*)&(bp_data.regs));

    //Get line number to easily locate breakpoint in file
    bp_data.bp.line = handle->data->marker.val;

    //Get the pc of this breakpoint
    bp_data.bp.pc = handle->data->marker.pc - HULP_DEBUG_SET_BP_START_ST_OFFSET;
    //This marker pc can be reset now
    handle->data->marker.pc = 0;

    //Set default continue point based on the marker pc (done in this way as part of breakpoint struct so that it can be altered between breakpoint and continue if desired)
    bp_data.meta.return_addr = bp_data.bp.pc + HULP_DEBUG_SET_BP_INSN_NUM;
    
    // Get the label number for this pc, if available
    if(handle->labels.pc_pairs != NULL)
    {
        for(size_t i = 0; i < handle->labels.num; ++i)
        {
            if(handle->labels.pc_pairs[i].pc == bp_data.bp.pc)
            {
                bp_data.bp.label.num = handle->labels.pc_pairs[i].label_num;
                bp_data.bp.label.valid = true;
                break;
            }
        }
    }

    //Set handle so other data (eg. label list) can be accessed by callbacks and continue
    bp_data.meta.handle = handle;

    //Pass to handler
    handle->callback.fn(&bp_data, handle->callback.ctx);
}

esp_err_t hulp_debug_bp_deinit(hulp_debug_bp_handle_t handle)
{
    hulp_ulp_isr_deregister(hulp_debug_isr_handler, handle);
    if(handle)
    {
        if(handle->labels.pc_pairs)
        {
            free(handle->labels.pc_pairs);
        }
        free(handle);
        handle = NULL;
    }
    return ESP_OK;
}

static size_t hulp_debug_count_ulp_program_labels(const ulp_insn_t* program, size_t num_words)
{
    size_t num_labels = 0;
    const ulp_insn_t* program_end = program + num_words;
    while(program < program_end)
    {
        if(program->macro.opcode == OPCODE_MACRO && program->macro.sub_opcode == SUB_OPCODE_MACRO_LABEL)
        {
            ++num_labels;
        }
        ++program;
    }
    return num_labels;
}

static size_t hulp_debug_populate_ulp_program_label_pcs(hulp_debug_label_pc_pair_t* label_pairs, size_t max_label_pairs, const ulp_insn_t* program, size_t num_words)
{
    size_t current_pc = 0;
    
    const ulp_insn_t* program_end = program + num_words;
    const hulp_debug_label_pc_pair_t* labelpairs_end = label_pairs + max_label_pairs;

    while(program < program_end && label_pairs < labelpairs_end)
    {
        if(program->macro.opcode == OPCODE_MACRO)
        {
            if(program->macro.sub_opcode == SUB_OPCODE_MACRO_LABEL)
            {
                label_pairs->label_num = program->macro.label;
                label_pairs->pc = current_pc;
                ++label_pairs;
            }
        }
        else
        {
            ++current_pc;
        }
        ++program;
    }

    size_t num_populated = max_label_pairs - (labelpairs_end - label_pairs);
    return num_populated;
}

static esp_err_t hulp_debug_get_pc_from_label_pairs(hulp_debug_bp_handle_t handle, uint16_t label_num, uint16_t* dest)
{
    if(!handle->labels.pc_pairs)
    {
        return ESP_ERR_INVALID_STATE;
    }

    for(size_t i = 0; i < handle->labels.num; ++i)
    {
        if(handle->labels.pc_pairs[i].label_num == label_num)
        {
            *dest = handle->labels.pc_pairs[i].pc;
            return ESP_OK;
        }
    }

    return ESP_ERR_NOT_FOUND;
}

esp_err_t hulp_debug_bp_set_continue_label(hulp_debug_bp_cb_data_t* bp_data, uint16_t label_num)
{
    if(!bp_data->meta.handle->labels.pc_pairs)
    {
        return ESP_ERR_INVALID_STATE;
    }

    return hulp_debug_get_pc_from_label_pairs(bp_data->meta.handle, label_num, &(bp_data->meta.return_addr));
    
}

/**
 * Each BP begins with a offset branch to allow easily enabling/disabling.
 * To enable, set offset to 1 (ie. branch next instruction)
 * To disable, set offset to skip over all instructions for this BP.
 */
static esp_err_t hulp_debug_bp_pc_set_branch_offset(uint16_t pc, uint16_t offset)
{
    //A common misuse will be to try to disable a BP before the program is loaded into RTC memory.
    //For a rudimentary check, a ulp_insn_t array consisting of the M_DEBUG_SET_BP is declared and compared against the instruction at the provided pc. 
    const ulp_insn_t template_insn_check = (ulp_insn_t[]){M_DEBUG_SET_BP(0,0,template_insn_check)}[0];
    ulp_insn_t* bp_en_insn = (ulp_insn_t*)(RTC_SLOW_MEM + pc);

    if(bp_en_insn->b.opcode != template_insn_check.b.opcode || bp_en_insn->b.sub_opcode != template_insn_check.b.sub_opcode)
    {
        return ESP_ERR_INVALID_STATE;
    }

    bp_en_insn->b.offset = offset;
    return ESP_OK;
}

esp_err_t hulp_debug_bp_enable_by_pc(uint16_t pc)
{
    //set to 'branch' ahead by 1 instruction (ie. continue)
    return hulp_debug_bp_pc_set_branch_offset(pc, 1);
}

esp_err_t hulp_debug_bp_disable_by_pc(uint16_t pc)
{
    //set to branch forward HULP_DEBUG_BP_INSN_NUM instructions (ie. past this BP macro block)
    return hulp_debug_bp_pc_set_branch_offset(pc, HULP_DEBUG_SET_BP_INSN_NUM);
}

esp_err_t hulp_debug_bp_enable_by_label(hulp_debug_bp_handle_t handle, uint16_t label_num)
{
    uint16_t pc;
    esp_err_t err = hulp_debug_get_pc_from_label_pairs(handle, label_num, &pc);
    if(err == ESP_OK)
    {
        err = hulp_debug_bp_enable_by_pc(pc);
    }
    return err;
}

esp_err_t hulp_debug_bp_disable_by_label(hulp_debug_bp_handle_t handle, uint16_t label_num)
{
    uint16_t pc;
    esp_err_t err = hulp_debug_get_pc_from_label_pairs(handle, label_num, &pc);
    if(err == ESP_OK)
    {
        err = hulp_debug_bp_disable_by_pc(pc);
    }
    return err;
}


esp_err_t hulp_debug_bp_init(const hulp_debug_bp_config_t* config, hulp_debug_bp_handle_t* handle)
{
    hulp_debug_bp_handle_t dbg_state = calloc(1, sizeof(struct hulp_debug_bp_state_t));

    if(!dbg_state)
    {
        ESP_LOGE(TAG, "no memory for debug state");
        return ESP_ERR_NO_MEM;
    }

    if(config->program.ptr)
    {
        size_t num_labels_in_program = hulp_debug_count_ulp_program_labels(config->program.ptr, config->program.num_words);
        if(num_labels_in_program > 0)
        {
            dbg_state->labels.pc_pairs = (hulp_debug_label_pc_pair_t*)malloc(num_labels_in_program * sizeof(hulp_debug_label_pc_pair_t));
            if(!dbg_state->labels.pc_pairs)
            {
                ESP_LOGE(TAG, "no memory for debug labels");
                hulp_debug_bp_deinit(dbg_state);
                return ESP_ERR_NO_MEM;
            }
            hulp_debug_populate_ulp_program_label_pcs(dbg_state->labels.pc_pairs, num_labels_in_program, config->program.ptr, config->program.num_words);
            dbg_state->labels.num = num_labels_in_program;

        }
    }

    dbg_state->data = config->data;
    dbg_state->callback.fn = config->callback.fn;
    dbg_state->callback.ctx = config->callback.ctx;

    esp_err_t err = hulp_ulp_isr_register(hulp_debug_isr_handler, dbg_state);
    if(err != ESP_OK)
    {
        ESP_LOGE(TAG, "failed to register isr (%d)", err);
        hulp_debug_bp_deinit(dbg_state);
        return err;
    }

    if(handle)
    {
        *handle = dbg_state;
    }

    hulp_ulp_interrupt_en();

    return ESP_OK;
}