// 一阶低通滤波实现，使用实际采样间隔动态计算平滑系数。

#include "lowpass_filter.h"

//保存时间常数并将上一次输出初始化为 0。
LowPassFilter::LowPassFilter(float time_constant)
    : Tf(time_constant) //滤波时间常数，单位 s
    , y_prev(0.0f)     //初始滤波输出为 0
{
    timestamp_prev = micros();
}

//按 y[n] = alpha*y[n-1] + (1-alpha)*x[n] 更新滤波值。
float LowPassFilter::operator() (float x)
{
    unsigned long timestamp = micros();
    float dt = (timestamp - timestamp_prev)*1e-6f; //实际采样间隔，单位 s

    //保留源项目对异常负时间差的保护逻辑
    if (dt < 0.0f ) dt = 1e-3f;
    //采样中断超过 0.3 s 时直接跟随输入，避免旧状态拖慢恢复
    else if(dt > 0.3f) {
        y_prev = x;
        timestamp_prev = timestamp;
        return x;
    }

    //时间常数越长，alpha 越大，输出越平滑
    float alpha = Tf/(Tf + dt);
    float y = alpha*y_prev + (1.0f - alpha)*x;
    y_prev = y;
    timestamp_prev = timestamp;
    return y;
}
