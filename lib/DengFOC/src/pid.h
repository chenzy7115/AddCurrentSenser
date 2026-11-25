// PID 控制器：包含积分限幅、输出限幅和输出变化率限制。
// 采样时间由 micros() 自动计算，调用方式为 controller(error)。

#ifndef PID_H
#define PID_H

class PIDController
{
public:
    //P/I/D 为增益，ramp 为每秒最大输出变化量，limit 为输出绝对值上限
    PIDController(float P, float I, float D, float ramp, float limit);
    ~PIDController() = default;

    //输入本次跟踪误差，返回经过限幅和变化率限制后的控制量
    float operator() (float error);

    float P; //!< 比例增益(P环增益)
    float I; //!< 积分增益（I环增益）
    float D; //!< 微分增益（D环增益）
    float output_ramp; //!< 输出变化率限制，控制量/秒；小于等于 0 时不启用
    float limit;       //!< 积分项和总输出的绝对值限幅

protected:
    float error_prev;       //!< 上一次跟踪误差，用于积分和微分计算
    float output_prev;      //!< 上一次 PID 输出，用于变化率限制
    float integral_prev;    //!< 上一次积分项，用于离散积分累加
    unsigned long timestamp_prev; //!< 上一次执行时间戳，单位 us
};

#endif
