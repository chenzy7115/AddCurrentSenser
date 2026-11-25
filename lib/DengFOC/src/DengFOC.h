//灯哥开源，遵循GNU协议，转载请著名版权！
//GNU开源协议（GNU General Public License, GPL）是一种自由软件许可协议，保障用户能够自由地使用、研究、分享和修改软件。
//该协议的主要特点是，要求任何修改或衍生的作品必须以相同的方式公开发布，即必须开源。此外，该协议也要求在使用或分发软件时，必须保留版权信息和许可协议。GNU开源协议是自由软件基金会（FSF）制定和维护的，常用于GNU计划的软件和其他自由软件。
//仅在DengFOC官方硬件上测试过，欢迎硬件购买/支持作者，淘宝搜索店铺：灯哥开源
//你的支持将是接下来做视频和持续开源的经费，灯哥在这里先谢谢大家了
//函数声明

#include <Arduino.h>

#ifndef DENG_FOC_H
#define DENG_FOC_H

// DengFOC V4 单电机 FOC 公共接口。
// 单位约定：电压为 V，电流为 A，角度为 rad，速度为 rad/s。

//将三相对地电压转换为 PWM 占空比；输入范围由母线电压限制
void setPwm(float Ua, float Ub, float Uc);

//根据 q 轴目标电压和电角度计算三相占空比，Ud 固定为 0
void setTorque(float Uq,float angle_el);

//将任意角度归一化到 [0, 2π)
float _normalizeAngle(float angle);

//设置母线电压，并初始化 PWM、AS5600 和电流采样
void DFOC_Vbus(float power_supply);

//设置极对数和方向，并通过短时励磁确定编码器零电角度
void DFOC_alignSensor(int _PP,int _DIR);

//读取归一化后的电角度，供 Park/Clark 变换使用
float _electricalAngle();

//返回最近一次串口输入的目标值
float serial_motor_target();

//读取串口字符；收到换行符后将完整命令保存到 motor_target
String serialReceiveUserCommand();

//返回经过方向修正和低通滤波后的电机速度，单位 rad/s
float DFOC_M0_Velocity();

//返回支持整圈计数的机械角度，单位 rad
float DFOC_M0_Angle();

//返回经过低通滤波后的 q 轴电流，单位 A
float DFOC_M0_Current();

//设置角度环 P、I、D、输出变化率限制和输出限幅
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp,float limit);

//设置速度环 P、I、D、输出变化率限制和输出限幅
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp,float limit);

//设置电流环 P、I、D 和输出变化率限制；输出限幅由库内参数决定
void DFOC_M0_SET_CURRENT_PID(float P,float I,float D,float ramp);

//执行速度 PID，输入为速度误差
float DFOC_M0_VEL_PID(float error);

//执行角度 PID，输入为角度误差
float DFOC_M0_ANGLE_PID(float error);

//位置-速度-电流三闭环接口，Target 为机械角度目标，单位 rad
void DFOC_M0_set_Velocity_Angle(float Target);

//速度-电流双闭环接口，Target 为速度目标，单位 rad/s
void DFOC_M0_setVelocity(float Target);

//位置（力位）-电流双闭环接口，Target 为机械角度目标，单位 rad
void DFOC_M0_set_Force_Angle(float Target);

//纯电流闭环接口，Target 为 q 轴电流目标，单位 A
void DFOC_M0_setTorque(float Target);

//每次控制循环开始时调用，更新编码器和两相电流采样
void runFOC();

#endif
