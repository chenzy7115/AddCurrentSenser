// DengFOC V4 单电机有感 FOC 核心实现。
// 控制链路：位置环 -> 速度环 -> 电流环 -> Park/Clarke 逆变换 -> 三相 PWM。
// AS5600 电角度和两相电流为控制环提供反馈。
// 本文件保留源项目的控制公式、PID 参数和硬件映射。

#include <Arduino.h>       //ESP32 GPIO、串口、数学和定时器接口
#include "AS5600.h"       //磁编码器角度与速度
#include "lowpass_filter.h" //速度和电流低通滤波
#include "pid.h"          //离散 PID 控制器
#include "InlineCurrent.h" //两相在线电流采样

//通用限幅宏：将 amt 限制在闭区间 [low, high]
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

float voltage_power_supply; //驱动器母线电压，单位 V；DFOC_Vbus() 中设置
float Ualpha,Ubeta=0,Ua=0,Ub=0,Uc=0; //αβ 轴电压及三相对地电压，单位 V
#define _3PI_2 4.71238898038f //3π/2，编码器对齐时使用的固定电角度
float zero_electric_angle=0; //编码器零电角度偏置，单位 rad
int PP=1,DIR=1;             //电机极对数和传感器方向，由 DFOC_alignSensor() 更新
int pwmA = 32;              //A 相 PWM：GPIO32，LEDC 通道 0
int pwmB = 33;              //B 相 PWM：GPIO33，LEDC 通道 1
int pwmC = 25;              //C 相 PWM：GPIO25，LEDC 通道 2

//速度环和电流环各使用一个一阶低通滤波器
LowPassFilter M0_Vel_Flt = LowPassFilter(0.01); //速度滤波时间常数 10 ms
LowPassFilter M0_Curr_Flt = LowPassFilter(0.05); //电流滤波时间常数 5 ms

//PID 构造函数参数顺序：P、I、D、输出变化率 ramp、输出限幅 limit
PIDController vel_loop_M0 = PIDController(2, 0, 0, 100000, voltage_power_supply/2);
PIDController angle_loop_M0 = PIDController(2, 0, 0, 100000, 100);
PIDController current_loop_M0 = PIDController(1.2, 0, 0, 100000, 12.6);

//0 号电机编码器，使用独立 I²C 对象以便指定 SDA/SCL
Sensor_AS5600 S0=Sensor_AS5600(0);
TwoWire S0_I2C = TwoWire(0);

//Clarke 变换使用的固定系数
#define _1_SQRT3 0.57735026919f //1/√3
#define _2_SQRT3 1.15470053838f //2/√3

//0 号电机电流采样对象，默认使用 GPIO39/36 两相采样
CurrSense CS_M0= CurrSense(0);


//=================PID 参数设置=================
//设置速度环增益、输出变化率和输出限幅；运行时可在 loop 中调用
void DFOC_M0_SET_VEL_PID(float P,float I,float D,float ramp,float limit)
{
  vel_loop_M0.P=P;
  vel_loop_M0.I=I;
  vel_loop_M0.D=D;
  vel_loop_M0.output_ramp=ramp;
  vel_loop_M0.limit=limit;
}

//设置角度环增益、输出变化率和输出限幅
void DFOC_M0_SET_ANGLE_PID(float P,float I,float D,float ramp,float limit)
{
  angle_loop_M0.P=P;
  angle_loop_M0.I=I;
  angle_loop_M0.D=D;
  angle_loop_M0.output_ramp=ramp;
  angle_loop_M0.limit=limit;
}

//设置电流环增益和输出变化率；电流环 limit 保持库内默认值 12.6
void DFOC_M0_SET_CURRENT_PID(float P,float I,float D,float ramp)
{
  current_loop_M0.P=P;
  current_loop_M0.I=I;
  current_loop_M0.D=D;
  current_loop_M0.output_ramp=ramp;
}

//速度 PID 接口：输入速度误差，输出电流目标
float DFOC_M0_VEL_PID(float error)
{
  return vel_loop_M0(error);
}

//角度 PID 接口：输入角度误差，输出速度目标
float DFOC_M0_ANGLE_PID(float error)
{
  return angle_loop_M0(error);
}

//保留源项目的重复限幅宏定义，便于对照原算法结构
#define _constrain(amt,low,high) ((amt)<(low)?(low):((amt)>(high)?(high):(amt)))

//将任意角度归一化到 [0, 2π)，供三角函数和跨圈计算使用。
float _normalizeAngle(float angle){
  //fmod 返回与除数同号的余数，因此负数结果需要加一圈
  float a = fmod(angle, 2*PI);
  return a >= 0 ? a : (a + 2*PI);
}

//将三相对地电压转换为 PWM 占空比并写入三个 LEDC 通道。
void setPwm(float Ua, float Ub, float Uc) {
  //先把各相电压限制在 0~母线电压，避免负占空比或过压
  Ua = _constrain(Ua, 0.0f, voltage_power_supply);
  Ub = _constrain(Ub, 0.0f, voltage_power_supply);
  Uc = _constrain(Uc, 0.0f, voltage_power_supply);

  //电压除以母线电压得到归一化占空比，范围 0~1
  float dc_a = _constrain(Ua / voltage_power_supply, 0.0f , 1.0f );
  float dc_b = _constrain(Ub / voltage_power_supply, 0.0f , 1.0f );
  float dc_c = _constrain(Uc / voltage_power_supply, 0.0f , 1.0f );

  //8 位 PWM 的最大计数为 255；新版 API 使用显式通道号
  ledcWriteChannel(0, dc_a*255);
  ledcWriteChannel(1, dc_b*255);
  ledcWriteChannel(2, dc_c*255);
}

//根据 q 轴电压和电角度执行 Park 逆变换、Clarke 逆变换并输出 PWM。
void setTorque(float Uq,float angle_el) {
  //q 轴电压限制在 ±母线电压/2，保证逆变器线性调制范围
  Uq=_constrain(Uq,-(voltage_power_supply)/2,(voltage_power_supply)/2);
  float Ud=0; //当前算法不使用 d 轴电压，保留源项目变量
  angle_el = _normalizeAngle(angle_el);

  //Park 逆变换：q/d 轴电压转换为 α/β 轴电压，Ud 固定为 0
  Ualpha =  -Uq*sin(angle_el);
  Ubeta =   Uq*cos(angle_el);

  //Clarke 逆变换：α/β 轴电压转换为三相电压，并叠加母线电压中点
  Ua = Ualpha + voltage_power_supply/2;
  Ub = (sqrt(3)*Ubeta-Ualpha)/2 + voltage_power_supply/2;
  Uc = (-Ualpha-sqrt(3)*Ubeta)/2 + voltage_power_supply/2;
  setPwm(Ua,Ub,Uc);
}

//设置母线电压并初始化 PWM、编码器和电流采样，必须在 setup 中调用。
void DFOC_Vbus(float power_supply)
{
  voltage_power_supply=power_supply;

  //配置三相 PWM 引脚
  pinMode(pwmA, OUTPUT);
  pinMode(pwmB, OUTPUT);
  pinMode(pwmC, OUTPUT);

  //Arduino-ESP32 新版 LEDC API：30 kHz、8 位精度，保持通道 0/1/2 映射
  ledcAttachChannel(pwmA, 30000, 8, 0);
  ledcAttachChannel(pwmB, 30000, 8, 1);
  ledcAttachChannel(pwmC, 30000, 8, 2);
  Serial.println("完成PWM初始化设置");

  //AS5600 使用 I²C0：SDA=GPIO19，SCL=GPIO18，通信速率 400 kHz
  S0_I2C.begin(19,18, 400000UL);
  S0.Sensor_init(&S0_I2C);
  Serial.println("编码器加载完毕");

  //母线电压确定后重新加载速度环默认参数，使 limit 等于母线电压的一半
  vel_loop_M0 = PIDController(2, 0, 0, 100000, voltage_power_supply/2);

  //配置 ADC 并采集 1000 次零电流偏置，校准时电机不应有负载电流
  CS_M0.init();
}

//读取归一化电角度：机械角度乘以极对数和方向，再减去零电角度偏置。
float _electricalAngle(){
  return _normalizeAngle((float)(DIR * PP) * S0.getMechanicalAngle()-zero_electric_angle);
}

//通过固定方向励磁对齐编码器，确定机械角与电角度之间的零点偏置。
void DFOC_alignSensor(int _PP,int _DIR)
{
  PP=_PP;   //写入实际电机极对数
  DIR=_DIR; //写入实际传感器方向

  setTorque(3, _3PI_2); //施加短时对齐力矩，使转子转到已知电角度
  delay(1000);          //等待转子稳定
  S0.Sensor_update();   //读取对齐后的机械角度
  zero_electric_angle=_electricalAngle(); //记录零电角度
  setTorque(0, _3PI_2); //撤去对齐力矩
  Serial.print("0电角度：");Serial.println(zero_electric_angle);
}

//返回经过传感器方向修正的累计机械角度，单位 rad。
float DFOC_M0_Angle()
{
  return DIR*S0.getAngle();
}


//================两相电流到 q 轴电流的变换================
//根据 A/B 相电流和电角度计算 q 轴电流；当前接口只返回 Iq。
float cal_Iq_Id(float current_a,float current_b,float angle_el)
{
  //Clarke 变换：两相电流转换为 α/β 轴电流，C 相电流由三相电流和为零推出
  float I_alpha=current_a;
  float I_beta = _1_SQRT3 * current_a + _2_SQRT3 * current_b;

  //Park 变换：α/β 轴电流转换为 d/q 轴电流
  float ct = cos(angle_el);
  float st = sin(angle_el);
  //d 轴电流计算式保留作参考：float I_d = I_alpha * ct + I_beta * st;
  float I_q = I_beta * ct - I_alpha * st;
  return I_q;
}

//返回滤波后的 q 轴电流，供电流环和串口输出使用，单位 A。
float DFOC_M0_Current()
{
  //使用当前电角度将两相电流变换到 q 轴
  float I_q_M0_ori=cal_Iq_Id(CS_M0.current_a,CS_M0.current_b,_electricalAngle());
  //5 ms 时间常数低通滤波，抑制采样噪声
  float I_q_M0_flit=M0_Curr_Flt(I_q_M0_ori);
  return I_q_M0_flit;
}

//返回经过方向修正和 10 ms 低通滤波后的速度，单位 rad/s。
float DFOC_M0_Velocity()
{
  float vel_M0_ori=S0.getVelocity();       //编码器原始速度
  float vel_M0_flit=M0_Vel_Flt(DIR*vel_M0_ori); //按安装方向修正后滤波
  return vel_M0_flit;
}

//================串口目标值接收================
float motor_target; //最近一次解析出的电机目标值
int commaPosition;  //换行符在命令字符串中的位置，保留源项目变量名

//读取串口字符并以换行符结束一条命令，将数值保存到 motor_target。
String serialReceiveUserCommand() {
  static String received_chars; //跨多次 loop 保留尚未接收完整的命令
  String command = "";         //本次 loop 中已完成的命令

  while (Serial.available()) {
    char inChar = (char)Serial.read(); //读取一个串口字符
    received_chars += inChar;         //追加到命令缓冲区

    if (inChar == '\n') { //收到换行符，表示一条命令结束
      command = received_chars;

      //定位换行符，并提取其前面的数值；命令格式为 "数值 + 换行"
      commaPosition = command.indexOf('\n');
      if(commaPosition != -1)
      {
          motor_target = command.substring(0,commaPosition).toDouble();
          Serial.println(motor_target); //回显已解析的目标值
      }
      received_chars = ""; //清空缓冲区，准备接收下一条命令
    }
  }
  return command;
}

//返回当前串口目标值，供各闭环接口使用。
float serial_motor_target()
{
  return motor_target;
}

//================简易闭环接口================
//纯电流力矩环：电流 PID 根据目标 Iq 与实际 Iq 的误差输出 q 轴电压。
void DFOC_M0_setTorque(float Target)
{
  setTorque(current_loop_M0(Target-DFOC_M0_Current()),_electricalAngle());
}

//位置-速度-电流三闭环：位置环 -> 速度环 -> 电流环。
void DFOC_M0_set_Velocity_Angle(float Target)
{
  //旧接口直接向 PWM 输出力矩，保留作算法演进参考：
  //setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)),_electricalAngle());
  //先将 rad 位置误差转换为度，再由速度环和电流环逐级计算目标
  DFOC_M0_setTorque(DFOC_M0_VEL_PID(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI)-DFOC_M0_Velocity()));
}

//速度-电流双闭环：速度环输出作为电流环目标。
void DFOC_M0_setVelocity(float Target)
{
  //旧接口直接向 PWM 输出力矩，保留作参考：
  //setTorque(DFOC_M0_VEL_PID((Target-DFOC_M0_Velocity())*180/PI),_electricalAngle());
  DFOC_M0_setTorque(DFOC_M0_VEL_PID((Target-DFOC_M0_Velocity())*180/PI));
}

//位置（力位）-电流双闭环：角度环输出作为电流环目标。
void DFOC_M0_set_Force_Angle(float Target)
{
  //旧接口直接向 PWM 输出力矩，保留作参考：
  //setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI),_electricalAngle());
  DFOC_M0_setTorque(DFOC_M0_ANGLE_PID((Target-DFOC_M0_Angle())*180/PI));
}

//每个控制周期先执行传感器更新，再调用相应闭环接口。
void runFOC()
{
  S0.Sensor_update();      //更新编码器单圈角度、累计圈数和采样时间戳
  CS_M0.getPhaseCurrents(); //更新 A/B 相电流采样值
}
