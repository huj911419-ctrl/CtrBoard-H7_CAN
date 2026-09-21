# 串口诊断约定

当前调试串口为 USART10，115200 baud。日志目标是“先看状态，再查链路”，避免逐帧打印 CAN 报文干扰实时控制。

## 日志标签

- `[BOOT]`：单片机启动。
- `[CFG]`：一次性配置摘要。
- `[DM][SCAN]`：CAN1 达妙 ESC_ID/MST_ID 扫描结果。
- `[DM][PARAM]` / `[DM][PARAM-WR]`：达妙参数读取/写入应答。
- `[RC]`：DBUS 遥控器状态，约 5 Hz。
- `[DJI]`：CAN2 C620/M3508 状态，约 2 Hz。
- `[DM]`：CAN1 达妙反馈，约 2 Hz。
- `[CAN1]` / `[CAN2]`：总线健康状态，约 1 Hz。
- `[EVENT]`：链路、状态发生变化时立即打印。

周期日志由主循环发送；CAN RX 中断只做收包、解析、计数，不阻塞打印。

## CAN1 与 CAN2 的关键区别

CAN1 和 CAN2 都配置为 1 Mbps Classic CAN，但控制协议不同。

CAN1 接 DM-J4310-2EC，当前使用达妙“速度模式”。命令 ID 为 `0x200 + ESC_ID`，数据是 4 字节小端 `float v_des`，单位 rad/s。速度闭环主要在达妙驱动器内部。

CAN2 接 C620 + M3508。C620 的 CAN 命令 `0x200/0x1FF` 是电流控制量，不是目标转速。项目中的速度 PI 在 STM32H723 内运行，PI 输出再变成 C620 电流命令。

## 常用字段

### [RC]

```text
[RC] link=ON ctrl=OFF arm=0 fire=0 s1=3 s2=3 ch=1024,1024,1024,1024 frames=1234
```

- `link`：DBUS 是否在线。
- `ctrl`：遥控器是否允许控制电机。当前学习版本为 OFF。
- `arm` / `fire`：根据 S1/S2 计算出的逻辑状态，即使 ctrl=OFF 也继续显示。
- `ch`：CH0~CH3 原始值。

### [DJI]

```text
[DJI] st=STOP id=1 ecd=4096 rpm=0 out=0 iq_fb=0 iq_cmd=0 T=31 age=0ms rx=12345
```

- `st`：WAIT/RUN/STOP 或故障状态。
- `ecd`：C620 反馈的转子机械角度，0~8191。
- `rpm`：电机转子转速。
- `out`：按减速比换算后的输出轴 rpm（整数显示）。
- `iq_fb`：C620 反馈中的“实际转矩电流”原始有符号值，不直接当作 A。
- `iq_cmd`：STM32 发给 C620 的电流命令值。
- `age`：距最近一帧有效反馈的时间。
- `rx`：累计有效反馈帧数。

### [CAN1] / [CAN2]

```text
[CAN2] rx=1000/s txq=200/s fail=0 TEC=0 REC=0 BO=0 EP=0 LEC=0
```

- `rx`：最近一个日志周期收到的有效帧数。
- `txq`：成功放入 FDCAN 发送队列的帧数；它不等于“对端一定 ACK”。
- `fail`：放入发送队列失败的次数。
- `TEC` / `REC`：CAN 发送/接收错误计数器。
- `BO`：Bus-Off。
- `EP`：Error Passive。
- `LEC`：Last Error Code。

## 排错顺序

看到电机不动时，先看对应 `[CANx]`。如果 RX 为 0，优先检查供电、CAN_H/CAN_L、终端电阻、波特率和设备的 CAN 模式。如果 RX 正常且 TEC/REC 接近 0，再看 `[DM]` 或 `[DJI]`。

C620 场景里，如果 `iq_cmd=0`，说明软件没有要求输出力矩；如果 `iq_cmd` 很大、`iq_fb` 也明显变化但 `rpm=0`，再检查机械堵转或负载。当前版本 `REMOTE_MOTOR_CONTROL_ENABLE=0`，因此遥控器只接收和记录，不会让电机运动。
