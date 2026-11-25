// 两相电流采样实现：ADC 电压换算、零偏置校准和电流计算。

#include <Arduino.h> 
#include "InlineCurrent.h"


//采样电阻：0.01 Ω；放大器增益：50；A/B 相使用 ADC1 引脚。
//C 相不接线，因此 pinC 保持 NOT_SET，避免访问未初始化的 GPIO。


#define _ADC_VOLTAGE 3.3f            //ESP32 ADC 参考电压，单位 V
#define _ADC_RESOLUTION 4095.0f      //12 位 ADC 最大原始计数

//每个 ADC 计数对应的电压，单位 V/count
#define _ADC_CONV ( (_ADC_VOLTAGE) / (_ADC_RESOLUTION) )

#define NOT_SET -12345.0            //未配置 ADC 引脚的哨兵值
#define _isset(a) ( (a) != (NOT_SET) ) //判断引脚是否已配置

//根据电机编号加载固定硬件参数；0 号电机用于本章三闭环示例。
CurrSense::CurrSense(int Mot_Num)
{
  if(Mot_Num==0)
  {
    pinA = 39; //0 号电机 A 相 ADC：GPIO39，仅输入
    pinB = 36; //0 号电机 B 相 ADC：GPIO36，仅输入
    pinC = NOT_SET; //两相采样，保持 C 相未设置
    _shunt_resistor = 0.01; //采样电阻 10 mΩ
    amp_gain  = 50;        //检测放大器增益 50 倍
    
    volts_to_amps_ratio = 1.0f /_shunt_resistor / amp_gain; //电压除以采样电阻和增益得到电流
    
    //三相使用相同硬件参数时的电压到电流增益
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
  }
  if(Mot_Num==1)
  {
    pinA = 35; //1 号电机 A 相 ADC：GPIO35，仅输入
    pinB = 34; //1 号电机 B 相 ADC：GPIO34，仅输入
    pinC = NOT_SET; //两相采样，保持 C 相未设置
    _shunt_resistor = 0.01;
    amp_gain  = 50;
    
    volts_to_amps_ratio = 1.0f /_shunt_resistor / amp_gain;
    
    gain_a = volts_to_amps_ratio;
    gain_b = volts_to_amps_ratio;
    gain_c = volts_to_amps_ratio;
  }
}

//读取一个 ADC 通道并将原始计数转换为电压。
float CurrSense::readADCVoltageInline(const int pinA){
  uint32_t raw_adc = analogRead(pinA);
  return raw_adc * _ADC_CONV;
}

//配置 ADC 输入引脚；只有明确设置的 C 相才会被配置。
void CurrSense::configureADCInline(const int pinA,const int pinB, const int pinC){
  pinMode(pinA, INPUT);
  pinMode(pinB, INPUT);
  if( _isset(pinC) ) pinMode(pinC, INPUT);
}

//在零电流状态下采集 1000 次并求平均，得到各相 ADC 电压零点。
void CurrSense::calibrateOffsets(){
    const int calibration_rounds = 1000;

    //清零累加器；C 相未使用时仍保持为 0
    offset_ia = 0;
    offset_ib = 0;
    offset_ic = 0;
    //累计采样，两次读取之间延时 1 ms
    for (int i = 0; i < calibration_rounds; i++) {
        offset_ia += readADCVoltageInline(pinA);
        offset_ib += readADCVoltageInline(pinB);
        if(_isset(pinC)) offset_ic += readADCVoltageInline(pinC);
        delay(1);
    }
    //将电压累加值转换为平均零偏置电压
    offset_ia = offset_ia / calibration_rounds;
    offset_ib = offset_ib / calibration_rounds;
    if(_isset(pinC)) offset_ic = offset_ic / calibration_rounds;
}

//先配置 ADC 引脚，再执行零电流校准。
void CurrSense::init(){
    configureADCInline(pinA,pinB,pinC);
    calibrateOffsets();
}

//读取各相电压、减去零点偏置并换算为电流。
void CurrSense::getPhaseCurrents(){
    current_a = (readADCVoltageInline(pinA) - offset_ia)*gain_a;//A 相电流，单位 A
    current_b = (readADCVoltageInline(pinB) - offset_ib)*gain_b;//B 相电流，单位 A
    current_c = (!_isset(pinC)) ? 0 : (readADCVoltageInline(pinC) - offset_ic)*gain_c; //C 相电流，单位 A
}
