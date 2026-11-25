// 一阶低通滤波器。
// alpha = Tf / (Tf + dt)，Tf 越大，滤波越强但响应越慢。

#include <Arduino.h>

#ifndef LOWPASS_FILTER_H
#define LOWPASS_FILTER_H

class LowPassFilter
{
public:
    //Tf 为滤波时间常数，单位秒
    LowPassFilter(float Tf);
    ~LowPassFilter() = default;

    //输入本次采样值 x，返回滤波后的值
    float operator() (float x);

    float Tf; //!< 滤波时间常数，单位 s

protected:
    unsigned long timestamp_prev; //!< 上一次滤波的时间戳，单位 us
    float y_prev;                //!< 上一次滤波输出
};

#endif
