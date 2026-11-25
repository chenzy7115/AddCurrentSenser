// PID 控制器实现：离散 P/I/D、积分限幅、输出限幅和变化率限制。

#include "pid.h"
#include <Arduino.h>

//将数值限制在闭区间 [low, high]
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//初始化 PID 参数和内部历史值，时间基准使用微秒。
PIDController::PIDController(float P, float I, float D, float ramp, float limit)
    : P(P)       //比例增益
    , I(I)       //积分增益
    , D(D)       //微分增益
    , output_ramp(ramp) //控制量每秒最大变化量
    , limit(limit)      //输出绝对值上限
    , error_prev(0.0f)  //初始误差为 0
    , output_prev(0.0f) //初始输出为 0
    , integral_prev(0.0f) //初始积分项为 0
{
    timestamp_prev = micros();
}

//输入误差并执行一次 PID；异常采样间隔回退到 1 ms。
float PIDController::operator() (float error){
    unsigned long timestamp_now = micros();
    //将微秒时间差转换为秒，得到本次实际控制周期
    float Ts = (timestamp_now - timestamp_prev) * 1e-6f;
    //间隔为 0 或大于 0.5 s 时视为异常，避免除零和积分突跳
    if(Ts <= 0 || Ts > 0.5f) Ts = 1e-3f;
    
    float proportional = P * error; //比例项
    //使用梯形离散积分，并对积分项单独限幅以防积分饱和
    float integral = integral_prev + I*Ts*0.5f*(error + error_prev);
    integral = _constrain(integral, -limit, limit);
    //用本次误差与上次误差之差计算微分项
    float derivative = D*(error - error_prev)/Ts;

    //合成 P/I/D 输出并限制绝对值
    float output = proportional + integral + derivative;
    output = _constrain(output, -limit, limit);

    //限制相邻两次输出的最大变化量，减小对驱动器的冲击
    if(output_ramp > 0){
        float output_rate = (output - output_prev)/Ts;
        if (output_rate > output_ramp)
            output = output_prev + output_ramp*Ts;
        else if (output_rate < -output_ramp)
            output = output_prev - output_ramp*Ts;
    }
    //保存本次状态，供下一次 PID 计算使用
    integral_prev = integral;
    output_prev = output;
    error_prev = error;
    timestamp_prev = timestamp_now;
    return output;
}
