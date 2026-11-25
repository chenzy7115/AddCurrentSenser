// AS5600 寄存器读取、跨圈计数和速度计算实现。

#include "AS5600.h"
#include "Wire.h"
#include <Arduino.h> 

#define _2PI 6.28318530718f //一圈对应的弧度

//读取 AS5600 的 12 位原始角度寄存器 0x0C/0x0D，并转换为 [0, 2π)。
double Sensor_AS5600::getSensorAngle() {
  uint8_t angle_reg_msb = 0x0C; //角度寄存器高字节地址，0x0D 为低字节

  byte readArray[2];           //依次保存高、低字节
  uint16_t readValue = 0;      //拼接后的 12 位原始角度值

  //设置寄存器指针后保持 I²C 连接，下一次直接读取两个角度字节
  wire->beginTransmission(0x36); //AS5600 默认 7 位 I²C 地址
  wire->write(angle_reg_msb);
  wire->endTransmission(false);

  //请求读取角度高字节和低字节
  wire->requestFrom(0x36, (uint8_t)2);
  for (byte i=0; i < 2; i++) {
    readArray[i] = wire->read();
  }

  //12 位角度中高 8 位位于高字节，低 4 位位于低字节高半字节
  int _bit_resolution=12;
  int _bits_used_msb=11-7;
  float cpr = pow(2, _bit_resolution); //一圈对应的 ADC 计数数
  int lsb_used = _bit_resolution - _bits_used_msb;

  uint8_t lsb_mask = (uint8_t)( (2 << lsb_used) - 1 );
  uint8_t msb_mask = (uint8_t)( (2 << _bits_used_msb) - 1 );
  
  readValue = ( readArray[1] &  lsb_mask );
  readValue += ( ( readArray[0] & msb_mask ) << lsb_used );
  //将 0~4095 的计数线性映射到 0~2π
  return (readValue/ (float)cpr) * _2PI; 
}

//=========角度处理相关=============
//保存电机编号，保留与 DengFOC 多电机接口一致的对象结构。
Sensor_AS5600::Sensor_AS5600(int Mot_Num) {
   _Mot_Num=Mot_Num;
   
}

//绑定 I²C 总线，等待传感器启动，并分别初始化速度与位置的历史采样。
void Sensor_AS5600::Sensor_init(TwoWire* _wire) {
    wire=_wire;
    wire->begin(); //启动 I²C 总线；DengFOC 已在调用前设置 SDA/SCL 和频率
    delay(500);   //等待 AS5600 上电稳定

    //第一次采样用于速度历史，第二次采样用于位置历史
    getSensorAngle(); 
    delayMicroseconds(1);
    vel_angle_prev = getSensorAngle(); 
    vel_angle_prev_ts = micros();
    delay(1);
    getSensorAngle(); 
    delayMicroseconds(1);
    angle_prev = getSensorAngle(); 
    angle_prev_ts = micros();
}

//更新单圈角度；若跳变量大于 0.8 圈，则认为发生了 0/2π 跨圈。
void Sensor_AS5600::Sensor_update() {
    float val = getSensorAngle(); //本次单圈角度
    angle_prev_ts = micros();      //记录采样时刻供速度计算使用
    float d_angle = val - angle_prev;

    //2π→0 的负向跳变记为 +1；0→2π 的正向跳变记为 -1，使累计角度保持连续
    if(abs(d_angle) > (0.8f*_2PI) ) full_rotations += ( d_angle > 0 ) ? -1 : 1; 
    angle_prev = val;
}

//返回最近一次读取的单圈机械角度，不包含累计圈数。
float Sensor_AS5600::getMechanicalAngle() {
    return angle_prev;
}

//返回累计整圈数与当前单圈角度之和，可表示多圈位置。
float Sensor_AS5600::getAngle(){
    return (float)full_rotations * _2PI + angle_prev;
}

//按角度差和采样时间差计算速度，并更新下一次速度计算的历史值。
float Sensor_AS5600::getVelocity() {
    //时间戳由 micros() 提供，转换为秒
    float Ts = (angle_prev_ts - vel_angle_prev_ts)*1e-6;
    //避免时间差为零或微秒计数回绕时产生异常速度
    if(Ts <= 0) Ts = 1e-3f;
    //同时考虑单圈角度差和累计圈数差
    float vel = ( (float)(full_rotations - vel_full_rotations)*_2PI + (angle_prev - vel_angle_prev) ) / Ts;    
    //保存本次状态，供下一次速度计算使用
    vel_angle_prev = angle_prev;
    vel_full_rotations = full_rotations;
    vel_angle_prev_ts = angle_prev_ts;
    return vel;
}
