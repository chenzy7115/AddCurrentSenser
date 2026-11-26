# ESP32 DengFOC 有感三闭环项目

本项目由 DengFOC V4 第九章电流采样 Arduino 示例迁移而来，运行于 `lolin32_lite`。默认使用 **位置环 → 速度环 → 电流环** 三闭环控制，包含 AS5600 编码器读取、两相电流采样、Clarke/Park 变换、PID、低通滤波和三相 PWM 输出。

> **安全提示：** FOC 上电会使电机输出力矩。首次烧录或修改参数前，请移除负载并确认驱动器、电源和电流采样电路安全。编码器对齐期间会施加约 1 秒的对齐力矩。

## 1. 项目结构

```text
AddCurrentSenser/
├── platformio.ini                 # PlatformIO 环境和串口监视配置
├── src/
│   └── main.cpp                   # Arduino setup()/loop() 主程序
└── lib/
    └── DengFOC/
        ├── library.json           # 本地库元数据
        └── src/
            ├── DengFOC.cpp/.h     # FOC 核心、PID 设置、闭环接口
            ├── AS5600.cpp/.h      # 编码器读取、跨圈计数、速度计算
            ├── InlineCurrent.cpp/.h # 两相 ADC 采样和零偏置校准
            ├── pid.cpp/.h         # 离散 PID 控制器
            └── lowpass_filter.cpp/.h # 一阶低通滤波器
```

主程序通过 `#include <DengFOC.h>` 使用本地库。所有 `.cpp` 和 `.h` 均已补充中文注释，说明接口参数、单位、坐标变换、采样流程和控制链路。

## 2. 环境与依赖

- 开发板：`lolin32_lite`（WEMOS LOLIN32 Lite）
- MCU：ESP32
- 框架：ESP32 Arduino
- PlatformIO 环境名：`lolin32_lite`
- 串口波特率：`115200`

项目没有需要额外下载的第三方 Arduino 库：

- `Arduino.h`、`Wire`、LEDC、ADC 等由 ESP32 Arduino 框架提供。
- DengFOC、AS5600、电流采样、PID 和低通滤波均来自源项目，保存在 `lib/DengFOC`。
- 因此 `platformio.ini` 中无需配置 `lib_deps`。

## 3. 硬件配置

| 功能 | 配置 |
|---|---|
| A/B/C 相 PWM | GPIO32 / GPIO33 / GPIO25 |
| PWM 参数 | 30 kHz、8 位精度、LEDC 通道 0/1/2 |
| AS5600 SDA | GPIO19 |
| AS5600 SCL | GPIO18 |
| AS5600 I²C | I²C0，400 kHz，地址 `0x36` |
| A 相电流 ADC | GPIO39 |
| B 相电流 ADC | GPIO36 |
| C 相电流 ADC | 未使用 |
| ADC 参数 | 3.3 V 参考、12 位、4095 最大计数 |
| 采样电阻 | `0.01 Ω` |
| 电流放大器增益 | `50` |
| 电机使能 | GPIO12，高电平 |
| 电机极对数 | `7` |
| 传感器方向 | `-1` |
| 设定母线电压 | `12.6 V` |
| 调试串口 | `Serial`，115200 baud |

GPIO39 和 GPIO36 是 ESP32 的仅输入引脚，适合 ADC 采样。若实际电机极对数或编码器方向不同，应修改 `src/main.cpp` 中的 `Motor_PP` 和 `Sensor_DIR`。

## 4. 编译、烧录和监视

### 使用 PlatformIO IDE

1. 在 VS Code 中安装 PlatformIO IDE 扩展。
2. 打开本项目根目录，即包含 `platformio.ini` 的目录。
3. 等待 PlatformIO 完成依赖和工具链初始化。
4. 连接开发板，选择正确的 COM 端口。
5. 点击 **Build** 编译，或点击 **Upload** 烧录。
6. 点击 **Monitor** 打开串口监视器。项目已将 `monitor_speed` 设为 `115200`，无需再次选择波特率。

### 使用 PlatformIO Core

在已安装 PlatformIO Core 的终端中执行：

```powershell
# 编译 lolin32_lite 环境
pio run -e lolin32_lite

# 烧录到已连接的开发板
pio run -e lolin32_lite -t upload

# 打开 115200 波特率串口监视器
pio device monitor -e lolin32_lite -b 115200
```

Windows PowerShell 中多条命令应使用分号 `;` 分隔，不要使用 `&&`。

## 5. 串口使用方法

默认三闭环模式下，串口目标值是机械角度，单位为 `rad`。

1. 打开 115200 波特率串口监视器。
2. 发送一个数值并按 Enter 换行，例如：

```text
1.57
```

3. 固件收到换行符后会解析目标值并回显。
4. 程序约每 31 个控制周期输出一次：

```text
滤波后的q轴电流,滤波后的速度
```

其中电流单位为 `A`，速度单位为 `rad/s`。串口目标值解析函数位于 `lib/DengFOC/src/DengFOC.cpp`。

## 6. 控制模式与参数

`src/main.cpp` 的 `loop()` 中保留了四种模式。同一时间应只启用一种模式，避免多个闭环接口同时写入 PWM。

### 默认：位置—速度—电流三闭环

```cpp
DFOC_M0_SET_ANGLE_PID(1, 0, 0, 100000, 30);
DFOC_M0_SET_VEL_PID(0.02, 1, 0, 100000, 0.5);
DFOC_M0_SET_CURRENT_PID(5, 200, 0, 100000);
DFOC_M0_set_Velocity_Angle(serial_motor_target());
```

参数顺序为 `P, I, D, ramp, limit`；电流环设置函数不接收 `limit`，使用库内默认值 `12.6`。

其他保留模式：

- 位置—电流双闭环：启用 `DFOC_M0_set_Force_Angle()` 对应代码块。
- 速度—电流双闭环：启用 `DFOC_M0_setVelocity()` 对应代码块。
- 纯电流力矩环：启用 `DFOC_M0_setTorque()` 对应代码块，目标值单位为 `A`。

切换模式时，请注释默认三闭环代码块，并启用目标模式的 PID 设置和控制接口。

## 7. 启动和校准流程

1. `Serial.begin(115200)` 初始化调试串口。
2. GPIO12 拉高，使能 V4 驱动器。
3. `DFOC_Vbus(12.6)` 初始化 PWM、AS5600 和电流采样。
4. 电流采样连续读取 1000 次，计算零电流偏置，约耗时 1 秒。
5. `DFOC_alignSensor(7, -1)` 施加对齐力矩并等待约 1 秒，记录零电角度后撤去力矩。
6. `loop()` 每次先调用 `runFOC()` 更新编码器和电流，再执行闭环控制。

校准时应保证：

- 电机可以安全转动，且已移除机械负载。
- 电流传感器处于零电流状态。
- AS5600 已正确安装并连接。
- 驱动器使能极性与硬件一致。

## 8. Arduino 项目迁移结果

### 源项目检查结果

源目录为 `DengFOC_Lib_Lesson9_Current_Sense`，其中实际参与运行的文件包括：

- 1 个 Arduino 草图：`DengFOC_Lib_Lesson9_Current_Sense.ino`
- 10 个库源文件和头文件：`DengFOC`、`AS5600`、`InlineCurrent`、`pid`、`lowpass_filter` 各自的 `.cpp/.h`
- 未发现额外的第三方库清单、构建配置或独立示例目录

### 迁移后的文件

- 草图入口迁移至 `src/main.cpp`。
- 10 个库源文件和头文件迁移至 `lib/DengFOC/src/`。
- 新增 `lib/DengFOC/library.json`，用于声明本地库元数据。
- `platformio.ini` 保持 `board = lolin32_lite` 和 `framework = arduino`，并新增 `monitor_speed = 115200`。
- 新增本 `README.md`。

### 兼容性调整

1. 将 Arduino 草图入口适配为 PlatformIO 的 `src/main.cpp`，显式包含 `Arduino.h` 和 `DengFOC.h`。
2. 将新版 ESP32 Arduino 不支持的 `ledcSetup()`、`ledcAttachPin()` 和通道式 `ledcWrite()` 改为 `ledcAttachChannel()`、`ledcWriteChannel()`，保持 30 kHz、8 位精度和通道 0/1/2 不变。
3. 将 `PIDController` 的 designated initializer 改为普通构造函数调用，保持 P/I/D、ramp 和 limit 参数不变。
4. 修正 `DengFOC.h` 中 `setTorque()` 的返回类型，使其与实现一致。
5. 为未使用的 C 相 ADC 引脚设置 `NOT_SET`，避免读取未初始化 GPIO。
6. 为头文件增加包含保护，并补充中文注释。

原项目的控制公式、默认 PID 参数、位置/速度/电流环调用、串口逻辑和硬件引脚均保留。

### 编译验证结果

使用以下命令完成编译：

```powershell
pio run -e lolin32_lite
```

结果为 `SUCCESS`。迁移编译环境为：

- `espressif32 8.0.0`
- Arduino-ESP32 `4.30312.261116`
- Flash 使用约 `321575 bytes`，占 24.5%
- RAM 使用约 `24240 bytes`，占 7.4%

### 整机运行验证结果

项目已完成实际上板测试，全部控制模式均测试通过：

| 控制模式 | 控制接口 | 验证结果 |
|---|---|---|
| 位置—速度—电流三闭环 | `DFOC_M0_set_Velocity_Angle()` | 通过 |
| 位置—电流双闭环（力位） | `DFOC_M0_set_Force_Angle()` | 通过 |
| 速度—电流双闭环 | `DFOC_M0_setVelocity()` | 通过 |
| 纯电流力矩环 | `DFOC_M0_setTorque()` | 通过 |

### 新增依赖

无外部新增依赖，`lib_deps` 保持为空。

### 仍存在的问题

- 编译无错误，但源项目中的未使用变量 `Ud` 会产生一条警告；为保持原算法结构未删除该变量。

## 9. `library.json` 的作用

[`lib/DengFOC/library.json`](lib/DengFOC/library.json) 是 PlatformIO 本地库的清单文件。PlatformIO 通过它识别库的名称、版本、适用框架和平台，并在 Library Dependency Finder 扫描到 `#include <DengFOC.h>` 时自动编译、归档和链接该库。

当前字段含义如下：

| 字段 | 作用 |
|---|---|
| `name` | 库名称，PlatformIO 依赖图中显示为 `DengFOC` |
| `version` | 库的语义化版本元数据，不会自动改变控制算法 |
| `description` | 库用途说明 |
| `keywords` | 检索关键字 |
| `frameworks` | 声明适用于 `arduino` 框架 |
| `platforms` | 声明适用于 `espressif32` 平台 |

`library.json` 使用标准 JSON，**不能添加注释**，因此字段说明统一放在本 README 中。该文件不是 Arduino 草图，也不会生成固件入口；主程序入口仍由 `src/main.cpp` 提供。

## 10. 版权与许可

源项目的 DengFOC 版权和 GNU/GPL 许可说明保留在主程序和相关库文件注释中。修改、分发或衍生使用时应遵守源项目许可要求。
