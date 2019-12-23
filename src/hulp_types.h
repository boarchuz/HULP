#ifndef HULP_TYPES_H_
#define HULP_TYPES_H_

#include "hulp.h"
#include "soc/rtc.h"
extern "C" {
    #include "esp_clk.h"
}

#define ADDR_IN_RTC_SLOW_MEM(addr) ((uint8_t*)(addr) - (uint8_t*)RTC_SLOW_MEM < 0x2000)

struct rtcslow32_t {
    rtcslow32_t() {
        assert(ADDR_IN_RTC_SLOW_MEM(this) && "var not in RTC_SLOW_MEM, check attributes");
    }
    protected:
        union {
            uint32_t raw;
            struct {
                union {
                    uint16_t val;
                    uint8_t bytes[2];
                } data;
                uint32_t st : 5;
                uint32_t pc : 11;
            } ulpf;
            ulp_insn_t insn;
        };
};

struct ulp_var_t : rtcslow32_t {
    ulp_var_t() {}
    ulp_var_t(uint16_t v)
    {
        ulpf.data.val = v;
    }
    void put(uint16_t v)
    {
        ulpf.data.val = v;
    }
    uint16_t get()
    {
        return ulpf.data.val;
    }
    operator uint16_t*()
    {
        return (uint16_t*)&ulpf.data;
    }
    operator uint16_t()
    {
        return ulpf.data.val;
    }
    ulp_var_t& operator=(const uint16_t rhs)
    {
        ulpf.data.val = rhs;
        return *this;
    }
    ulp_var_t& operator++()
    {
        ++ulpf.data.val;
        return *this;
    }
    ulp_var_t operator++(int)
    {
        ulp_var_t temp = *this;
        ++*this;
        return temp;
    }
    ulp_var_t& operator--()
    {
        --ulpf.data.val;
        return *this;
    }
    ulp_var_t operator--(int)
    {
        ulp_var_t temp = *this;
        --*this;
        return temp;
    }
    bool updated()
    {
        return (ulpf.st != 0);
    }
    void clearUpdated()
    {
        ulpf.st = 0;
    }
    uint8_t operator [](uint8_t i) const    {return ulpf.data.bytes[(uint8_t)i];}
    uint8_t & operator [](uint8_t i) {return ulpf.data.bytes[(uint8_t)i];}
};

template<uint8_t SZ>
struct ulp_string_t {
    ulp_string_t() {}
    ulp_string_t(const char* src) {
        set(src);
    }

    uint8_t set(const char *src)
    {
        uint8_t index = 0;
        while(*src != 0 && index < size())
        {
            str.chars[(uint8_t)(index/2)][(uint8_t)(index%2)] = *src;
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
            *buf++ = str.chars[(uint8_t)(index/2)][(uint8_t)(index%2)];
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
        return str.chars[i/2][i%2];
    }
    char & operator [](uint8_t i) {
        return (char&)str.chars[i/2][i%2];
    }

    bool updated()
    {
        return (str.metadata.updated());
    }
    void clearUpdated()
    {
        str.metadata.clearUpdated();
    }

    uint8_t size()
    {
        return str.metadata[(uint8_t)1];
    }
    uint8_t len()
    {
        return str.metadata[(uint8_t)0];
    }

    private:
        void setLen(uint8_t l)
        {
            str.metadata[(uint8_t)0] = l;
        }
        struct {
            ulp_var_t metadata = (SZ << 8);
            ulp_var_t chars[(SZ+1)/2];
        } str;
};

struct ulp_timestamp_t {
		uint64_t ticks()
		{
			return ((uint64_t)fields.upper.get() << 32) | ((uint32_t)fields.middle.get() << 16) | fields.lower.get();
		}
        uint64_t us()
        {
            return rtc_time_slowclk_to_us(ticks(), esp_clk_slowclk_cal_get());
        }
        bool updated()
        {
            return fields.upper.updated();
        }
        void clearUpdated()
        {
            fields.upper.clearUpdated();
        }
        // private:
        struct {
            ulp_var_t upper;
            ulp_var_t middle;
            ulp_var_t lower;
        } fields;
            
		// uint64_t timeNow = rtc_time_get();
		// uint64_t adcTimestamp;
		// adcTimestamp = (ulp_data_read(DUM_STAMP48_ADC_A) << 32) | (ulp_data_read(DUM_STAMP48_ADC_B) << 16) | (ulp_data_read(DUM_STAMP48_ADC_C));
		// uint64_t pgTimestamp;
		// pgTimestamp = (ulp_data_read(DUM_STAMP48_PG_A) << 32) | (ulp_data_read(DUM_STAMP48_PG_B) << 16) | (ulp_data_read(DUM_STAMP48_PG_C));
		// printf("ADC age: %"PRIu64"ms, PG time: %"PRIu64"ms\n", rtc_time_slowclk_to_us(timeNow - adcTimestamp, esp_clk_slowclk_cal_get()) / 1000, (ulp_data_read(DUM_BQ_PG_VALID) ? (rtc_time_slowclk_to_us(timeNow - pgTimestamp, esp_clk_slowclk_cal_get()) / 1000) : 0));
};

#endif