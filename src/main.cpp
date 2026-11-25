#include <Arduino.h>
#include <DengFOC.h>

int Sensor_DIR = 1;  //传感器方向：1 或 -1，用于统一电角度和速度方向
int Motor_PP = 7;     //电机极对数，必须与实际电机一致

// 初始化串口、驱动器使能、PWM、编码器、电流采样和电角度零点。
void setup() {
  //USB 转串口通常使用 UART0，波特率与 platformio.ini 的 monitor_speed 一致
  Serial.begin(115200);

  //V4 驱动器使能脚；上电后拉高
  pinMode(12, OUTPUT);
  digitalWrite(12, HIGH);

  //设置母线电压并初始化 PWM、AS5600 和电流传感器
  DFOC_Vbus(12.6);

  //校准编码器零电角度；校准时电机会输出短暂力矩，应移除负载
  DFOC_alignSensor(Motor_PP, Sensor_DIR);
}

//控制循环计数，用于降低串口打印频率
int count = 0;

//主循环：采集传感器数据，执行三闭环，接收目标值并周期输出状态。
void loop() {
  //先更新机械角度、圈数和两相电流，保证后续控制使用同一轮采样
  runFOC();

  //力位闭环示例（默认注释）：角度环输出直接作为电流环目标
  // DFOC_M0_SET_ANGLE_PID(0.5,0,0.003,100000,0.1);
  // DFOC_M0_SET_CURRENT_PID(1.25,50,0,100000);
  // DFOC_M0_set_Force_Angle(serial_motor_target());

  //速度闭环示例（默认注释）：速度环输出作为电流环目标
  // DFOC_M0_SET_VEL_PID(3,2,0,100000,0.5);
  // DFOC_M0_SET_CURRENT_PID(0.5,50,0,100000);
  // DFOC_M0_setVelocity(serial_motor_target());

  //默认模式：位置-速度-电流三闭环
  //角度环将位置误差转换为速度目标，速度环生成电流目标，电流环输出三相电压
  DFOC_M0_SET_ANGLE_PID(1, 0, 0, 100000, 30);
  DFOC_M0_SET_VEL_PID(0.02, 1, 0, 100000, 0.5);
  DFOC_M0_SET_CURRENT_PID(5, 200, 0, 100000);
  DFOC_M0_set_Velocity_Angle(serial_motor_target());

  //纯电流力矩环示例（默认注释）：Target 为 q 轴电流目标，单位 A
  // DFOC_M0_SET_CURRENT_PID(5,200,0,100000);
  // DFOC_M0_setTorque(serial_motor_target());

  //每约 31 个控制周期输出一次滤波后的 q 轴电流和速度
  count++;
  if (count > 30) {
    count = 0;
    //仅输出电流：Serial.printf("%f\n", DFOC_M0_Current());
    Serial.printf("%f,%f\n", DFOC_M0_Current(), DFOC_M0_Velocity());
  }

  //读取以换行符结尾的串口目标值，例如发送 "1.57\n"
  serialReceiveUserCommand();
}
