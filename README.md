# CtrBoard-H7_CAN · STM32H723 电机 CAN 与遥控实验

面向 STM32H723 的裸机 C 工程，用于学习和验证 **达妙电机速度模式、C620/M3508 速度闭环、DBUS 接收和串口诊断**。固件采用 STM32 HAL 与 CMake/Ninja，业务代码集中在 `User/`，周期处理由主循环组织。

**当前源码中 `REMOTE_MOTOR_CONTROL_ENABLE=0`**：DBUS 继续接收和记录，但遥控器不驱动两路电机。不能仅看到遥控在线就判断输出已使能；旧文档中的使能操作须结合当前宏值理解。

## 硬件与各总线职责

| 接口 | 当前用途 | 控制方式 |
| --- | --- | --- |
| CAN1 | 达妙 J4310-2EC | ID/参数查询、速度模式；电机内部闭合速度环，命令单位 rad/s |
| CAN2 | 一台 C620 + M3508 | 接收反馈识别 ID，MCU 运行速度 PI，发送电流命令 |
| CAN3 | 预留 | 当前不初始化、不发送 |
| UART7 / PE7 RX | DT7/DR16 DBUS | 100 kbit/s、偶校验、18 字节帧，接收反相配置见源码 |
| USART10 / PE3 TX | 诊断日志 | 115200、8N1，3.3V TTL，需与电脑串口共地 |

CAN1 与 CAN2 当前均使用 1 Mbps Classic CAN。C620 的电流命令不能当成目标转速；`iq_cmd` 和 `iq_fb` 也不直接代表安培值。详细接线、供电和终端电阻说明见 [CAN2 首次使用](docs/can2-m3508-start.md)。

## 代码结构

| 路径 | 职责 |
| --- | --- |
| [Core/Src/main.c](Core/Src/main.c) | 外设初始化、遥控控制开关、周期调度与日志 |
| [Core/Src/fdcan.c](Core/Src/fdcan.c) | 三路 FDCAN 的底层配置 |
| [User/bsp_fdcan.c](User/bsp_fdcan.c) | CAN 收发、达妙协议相关处理和总线统计 |
| [User/dji_motor.c](User/dji_motor.c) | C620 反馈、ID 识别、速度 PI、限制与故障状态 |
| [User/dji_motor.h](User/dji_motor.h) | 电机编号、减速比、速度和保护配置 |
| [User/dbus.c](User/dbus.c) | 遥控接收、解码、在线状态 |
| [User/dbus.h](User/dbus.h) | 遥控开关值与超时配置 |
| [cmake/](cmake/) | ARM GCC/Clang 工具链与 CubeMX 源码列表 |
| [tests/](tests/) | 使用 HAL 替身的主机 CAN/电机测试 |
| [docs/serial-diagnostics.md](docs/serial-diagnostics.md) | 日志字段及排错顺序 |

## 控制与诊断流程

```text
CAN RX 中断 → 帧检查/反馈计数 → 主循环快照 → 电机状态与速度控制
DBUS 接收   → 解码/在线状态  → 控制许可开关 → 目标值与停机
                                      ↓
                        周期 USART10 日志与 CAN 健康统计
```

中断接收路径不做阻塞串口打印。C620 自动识别模式会观察反馈数量、时效和出现的 ID；发现多个 ID 时不会任意选择一台电机。此项目是单 M3508 实验程序，其他电流槽位保持零，不是多电机控制框架。

## 构建

准备 **CMake 3.22+、Ninja、GNU Arm Embedded 工具链**，确保 `arm-none-eabi-gcc`、`objcopy`、`size` 等可从 PATH 找到：

```bash
git clone https://github.com/huj911419-ctrl/CtrBoard-H7_CAN.git
cd CtrBoard-H7_CAN
cmake --preset Debug
cmake --build --preset Debug
```

Release 配置：

```bash
cmake --preset Release
cmake --build --preset Release
```

预设输出目录为 `build/Debug/` 或 `build/Release/`，目标名为 `CtrBoard-H7_FDCAN`。具体生成的固件格式以 CMake 配置与构建输出为准。构建命令只生成文件，不连接或写入控制板。

旧的首次使用说明还提到 Keil 工程，但当前仓库没有 `MDK-ARM/` 目录；本仓库提供的构建入口是 CMake 预设。

## 主机测试

电脑端需要 PowerShell 与主机 GCC；使用替身头文件，不连接 CAN 或电机：

```powershell
pwsh -File tests/run_host_tests.ps1 -Compiler gcc
```

脚本分别编译 `DJI_MOTOR_ID=0` 与 `6` 的测试，检查自动识别和固定编号的相关行为。此测试不包含真实总线时序、电机机械响应或实际供电验证。

## 修改配置前先看这些位置

- **遥控是否允许输出**：[main.c](Core/Src/main.c) 中的 `REMOTE_MOTOR_CONTROL_ENABLE`，当前为 0。
- **C620 编号**：[dji_motor.h](User/dji_motor.h) 中的 `DJI_MOTOR_ID`；0 为自动识别，1～8 为固定编号。
- **速度和反馈**：核对减速比、转子/输出轴单位、PI 参数、温度和反馈超时。
- **遥控映射**：[dbus.h](User/dbus.h) 与主循环中的 S1/S2、通道映射。
- **达妙参数**：先确认实际 ESC_ID、MASTER_ID、模式与反馈，再讨论速度命令；不要混用 DJI 协议。

读取状态与改变电机输出是两个步骤。首次连接 CAN2 时按文档仅接一台 C620，核对反馈后再按现场测试流程启用控制。

## 日志与排错

先看 `[BOOT]`、`[CFG]`，再看 `[RC]`、`[CAN1]/[CAN2]`，最后看 `[DM]`、`[DJI]`。`rx` 表示收到反馈，`txq` 只表示放入发送队列，不能证明对端已经接收。

遇到不转动时，先确认当前控制开关，再检查供电、CAN_H/L、终端电阻、波特率、模式、反馈年龄和故障状态。详细字段见 [串口诊断约定](docs/serial-diagnostics.md)。

## 许可证与验证状态

仓库包含部分厂商组件的许可文件，但根目录尚无覆盖项目应用代码的 `LICENSE`。本说明依据当前默认分支源码整理；构建通过、主机测试和实机验收应分别记录，不能互相替代。
