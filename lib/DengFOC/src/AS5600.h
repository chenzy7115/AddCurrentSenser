// AS5600 12 位磁编码器驱动。
// 单次读取返回 [0, 2π) 的机械角度，并通过跨圈检测累计整圈数。

#include <Arduino.h>
#include <Wire.h>

#ifndef AS5600_H
#define AS5600_H

class Sensor_AS5600
{
  public:
    //记录电机编号；当前驱动使用同一套 AS5600 引脚和寄存器逻辑
    Sensor_AS5600(int Mot_Num);

    //绑定 I²C 对象并等待传感器稳定，默认使用 Arduino 默认 Wire
    void Sensor_init(TwoWire* _wire = &Wire);

    //读取最新角度，检测跨 0/2π 跳变并更新累计圈数
    void Sensor_update();

    //返回包含累计圈数的机械角度，单位 rad
    float getAngle();

    //根据相邻两次角度采样计算速度，单位 rad/s
    float getVelocity();

    //返回最近一次读取的单圈机械角度，范围 [0, 2π)
    float getMechanicalAngle();

    //直接读取 AS5600 角度寄存器并转换为弧度
    double getSensorAngle();

  private:
    int _Mot_Num;                 //电机编号，保留源项目接口
    //int sensor_direction=1;     //编码器旋转方向定义（当前由 DengFOC 层处理）
    float angle_prev=0;           //最近一次读取的单圈角度，单位 rad
    long angle_prev_ts=0;         //最近一次角度采样的时间戳，单位 us
    float vel_angle_prev=0;       //上一次计算速度时的单圈角度
    long vel_angle_prev_ts=0;     //上一次计算速度时的时间戳，单位 us
    int32_t full_rotations=0;     //用于位置计算的累计整圈数
    int32_t vel_full_rotations=0; //上一次计算速度时的累计整圈数
    TwoWire* wire;                //Sensor_init() 绑定的 I²C 总线对象
};

#endif
