
#include <stdio.h>
#include "driver/pcnt.h"
/* 
pcnt_config_t pcnt_config_l;

void pcnt_init(void)
{
#if (EN_DIR_STEP_OUTPUT == 0)
    pinMode(iStep, INPUT);
    pinMode(iDIR, INPUT);
    pinMode(iEN, INPUT);
    pcnt_config_l.pulse_gpio_num = iStep;   // 脉冲脚
    pcnt_config_l.ctrl_gpio_num = iDIR;     //方向脚
    pcnt_config_l.channel = PCNT_CHANNEL_0; //计数器频道
    pcnt_config_l.unit = PCNT_UNIT_0;       //计数器单元
    pcnt_config_l.pos_mode = PCNT_COUNT_INC;
    pcnt_config_l.neg_mode = PCNT_COUNT_DIS;
    pcnt_config_l.lctrl_mode = PCNT_MODE_REVERSE;
    pcnt_config_l.hctrl_mode = PCNT_MODE_KEEP;
    pcnt_config_l.counter_h_lim = 32767;  //最大计数值
    pcnt_config_l.counter_l_lim = -32767; //最小计数值
    pcnt_unit_config(&pcnt_config_l);
    pcnt_set_filter_value(PCNT_UNIT_0, 1023); //设定检测阈值
    pcnt_filter_enable(PCNT_UNIT_0);
    pcnt_counter_pause(PCNT_UNIT_0);
    pcnt_counter_clear(PCNT_UNIT_0);
    pcnt_counter_resume(PCNT_UNIT_0);
#else
    pinMode(iStep, OUTPUT);
    pinMode(iDIR, OUTPUT);
    pinMode(iEN, OUTPUT);
    ledcSetup(0, 4500, 8);
    ledcAttachPin(iStep, 0);
    ledcAttachPin(iDIR, 0);
    ledcAttachPin(iEN, 0);
    ledcWrite(0, 0x80);
#endif
}

void pcnt_loop(void)
{
#if (EN_DIR_STEP_OUTPUT == 0)
    int16_t count = 0;
    pcnt_get_counter_value(PCNT_UNIT_0, &count); //取0单元的数据
    pcnt_counter_clear(PCNT_UNIT_0);             //计数器清零
    if (count != 0 && digitalRead(iEN))
        stepper->move(Motor_dir * count);
#endif
} */
