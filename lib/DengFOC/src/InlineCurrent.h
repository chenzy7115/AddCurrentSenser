// 两相在线电流采样类。
// ADC 电压先减去零电流偏置，再按采样电阻和运算放大器增益换算为电流。

#include <Arduino.h>

#ifndef INLINE_CURRENT_H
#define INLINE_CURRENT_H

class CurrSense
{
  public:
    //根据电机编号选择 ADC 引脚，并设置采样电阻和放大器增益
    CurrSense(int Mot_Num);

    //读取指定 ADC 引脚，并按 3.3 V/4095 换算为电压
    float readADCVoltageInline(const int pinA);

    //将已配置的 A/B/C 相 ADC 引脚设为输入；C 相可为 NOT_SET
    void configureADCInline(const int pinA,const int pinB, const int pinC);

    //在零电流状态下连续采样 1000 次，计算各相电压偏置
    void calibrateOffsets();

    //依次配置 ADC 引脚并执行零偏置校准
    void init();

    //读取 A/B/C 相电流；未配置的 C 相固定为 0
    void getPhaseCurrents();

    float current_a;  //A 相电流，单位 A
    float current_b;  //B 相电流，单位 A
    float current_c;  //C 相电流，单位 A；两相采样时为 0
    int pinA;         //A 相 ADC 引脚
    int pinB;         //B 相 ADC 引脚
    int pinC;         //C 相 ADC 引脚；NOT_SET 表示不采样
    float offset_ia;  //A 相零电流电压偏置，单位 V
    float offset_ib;  //B 相零电流电压偏置，单位 V
    float offset_ic;  //C 相零电流电压偏置，单位 V
    float _shunt_resistor; //采样电阻阻值，单位 Ω
    float amp_gain;       //电流检测运算放大器电压增益

    float volts_to_amps_ratio; //ADC 电压到电流的换算系数，单位 A/V

    float gain_a;  //A 相电压到电流增益
    float gain_b;  //B 相电压到电流增益
    float gain_c;  //C 相电压到电流增益

  private:
    int _Mot_Num; //电机编号，用于选择引脚配置
};

#endif
