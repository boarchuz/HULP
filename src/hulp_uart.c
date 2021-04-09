#include "hulp_uart.h"

#include "hulp_types.h"

static void set_len(ulp_var_t *hulp_string, uint8_t len)
{
    hulp_string[0].val = (hulp_string[0].val & 0xFF00) | len;
}

static uint8_t get_len(ulp_var_t *hulp_string)
{
    return (hulp_string[0].val & 0x00FF);
}

static char char_get(ulp_var_t *hulp_string, uint8_t index)
{
    return (char)((hulp_string[1 + index / 2].val >> ((index % 2) * 8)) & 0xFF);
}
static void char_set(ulp_var_t *hulp_string, uint8_t index, char c)
{
    hulp_string[1 + index / 2].val = (hulp_string[1 + index / 2].val & ~(0xFF << ((index % 2) * 8))) | ((uint16_t)c << ((index % 2) * 8));
}

int hulp_uart_string_set(ulp_var_t *hulp_string, size_t len, const char* str)
{
    if(!hulp_string || len < 2 || !str)
    {
        return -1;
    }

    size_t capacity = (len - 1) * 2;
    uint8_t index = 0;
    while(*str != '\0' && index < capacity)
    {
        char_set(hulp_string, index, *str);
        ++str;
        ++index;
    }
    set_len(hulp_string, index);
    return index;
}

int hulp_uart_string_get(ulp_var_t *hulp_string, char* buffer, size_t buffer_size, bool clear)
{
    if(!hulp_string || !buffer || !buffer_size)
    {
        return -1;
    }

    int string_len = get_len(hulp_string);
    if(buffer_size <= string_len)
    {
        return -1;
    }

    int index = 0;
    while(index < string_len)
    {
        *buffer = char_get(hulp_string, index);
        ++buffer;
        ++index;
    }
    *buffer = '\0';
    if(clear)
    {
        set_len(hulp_string, 0);
    }
    return string_len;
}