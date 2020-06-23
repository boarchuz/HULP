#ifndef HULP_TYPES_H
#define HULP_TYPES_H

#include "hulp.h"

#include "soc/rtc.h"
#include "esp32/clk.h"
#include "soc/soc_memory_layout.h"

struct ulp_var_t {
    ulp_var_t() {
        assert(esp_ptr_in_rtc_slow(this) && "var not in RTC_SLOW_MEM, check attributes");
    }
    void put(uint16_t v)
    {
        val = v;
    }
    uint16_t get() const
    {
        return val;
    }
    union {
        struct {
            uint32_t val : 16;
            uint32_t reg_off : 2;
            uint32_t st : 3;
            uint32_t pc : 11;
        };
        ulp_insn_t insn;
    };
};

template<uint8_t SZ>
struct ulp_string_t {
    ulp_string_t() {
        data.meta.val = 0;
        setSize(SZ);
        setLen(0);
    }
    ulp_string_t(const char* src) {
        data.meta.val = 0;
        setSize(SZ);
        set(src);
    }

    uint8_t set(const char *src)
    {
        uint8_t index = 0;
        while(*src != 0 && index < size())
        {
            char_set(index, *src);
            ++src;
            ++index;
        }
        setLen(index);
        return index;
    }

    void operator=(const char * s)
    {
        set(s);
    }

    uint8_t peek(char *buf, size_t bufferSize)
    {
        uint8_t index = 0;
        while(index < bufferSize && index < len())
        {
            *buf = char_get(index);
            ++buf;
            ++index;
        }
        return index;
    }

    uint8_t read(char *buf, size_t bufferSize)
    {
        uint8_t num = peek(buf, bufferSize);
        setLen(0);
        return num;
    }

    char operator [](uint8_t i) const {
        return data.chars[i/2][i%2];
    }
    char & operator [](uint8_t i) {
        return (char&)data.chars[i/2][i%2];
    }

    uint8_t size()
    {
        return (data.meta.val >> 8) & 0xFF;
    }
    uint8_t len()
    {
        return (data.meta.val >> 0) & 0xFF;
    }

    private:
        void setLen(uint8_t l)
        {
            data.meta.val = ((uint16_t)size() << 8) | l;
        }
        void setSize(uint8_t s)
        {
            data.meta.val = ((uint16_t)s << 8) | len();
        }
        char char_get(uint8_t index)
        {
            return (char)((data.chars[index / 2].val >> ((index % 2) * 8)) & 0xFF);
        }
        void char_set(uint8_t index, char c)
        {
            data.chars[index / 2].val = (data.chars[index / 2].val & ~(0xFF << ((index % 2) * 8))) | ((uint16_t)c << ((index % 2) * 8));
        }
        struct {
            ulp_var_t meta;
            ulp_var_t chars[(SZ+1)/2];
        } data;
};

struct ulp_timestamp_t {
		uint64_t ticks()
		{
			return ((uint64_t)bits.upper.get() << 32) | ((uint32_t)bits.middle.get() << 16) | bits.lower.get();
		}
        uint64_t us()
        {
            return rtc_time_slowclk_to_us(ticks(), esp_clk_slowclk_cal_get());
        }

        struct {
            ulp_var_t upper;
            ulp_var_t middle;
            ulp_var_t lower;
        } bits;
};

#endif // HULP_TYPES_H