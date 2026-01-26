#include "zf_common_headfile.h"
#include "bsp_battery.h"
#include "pin_config.h"

// ADC分辨率
#define ADC_RESOLUTION ADC_12BIT

// 参考电压 3.3V
#define ADC_REF_VOLTAGE 3.3f

// 分压比，假设电池电压直接连接，如果有分压电阻需要修改此值
#define VOLTAGE_DIVIDER_RATIO 1.0f

void battery_init(void)
{
    adc_init(BATTERY_ADC, ADC_RESOLUTION);
}

float battery_get_voltage(void)
{
    uint16_t adc_value = adc_convert(BATTERY_ADC);
    float voltage = (adc_value / (float)(1 << ADC_RESOLUTION)) * ADC_REF_VOLTAGE / VOLTAGE_DIVIDER_RATIO / 1000.0f;
    return voltage;
}
