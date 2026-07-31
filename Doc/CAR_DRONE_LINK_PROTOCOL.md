# 小车-无人机无线通信协议与联调流程

本文是当前工程的小车端对接依据。小车 MCU 是发送端，无人机 STM32F429 是接收端。当前链路为单向透明串口，只传输小车航向、启动和中止指令；UWB 测距数据通过无人机 UC/UART4 单独接入，不经过本链路。

## 1. 硬件与无线配置

无人机使用 UG/UART8，配置为 `115200 8N1`，接收引脚为 `PE0`。推荐连接如下：

```text
小车 MCU TX -> 小车无线模块 RX
无人机无线模块 TX -> 飞控 UG RX / PE0
两端 MCU 与各自模块共地
```

无人机当前不通过 UG 发送应用数据，因此 UG TX/PE1 和无人机模块 RX 可不接。模块供电按具体版本丝印和说明书连接，UART 信号必须兼容 3.3 V 电平。

两只无线模块的空中速率、频率、无线 CRC 和通信版本必须一致。当前可使用 `2 Mbps`、`2.500 GHz`、`16 位 CRC`、`V210 及以上`。实时控制建议自动重发 `0` 或 `1` 次，达到上限后丢弃当前包并发送新包，不要无限重发旧包。

## 2. 应用帧格式

每帧固定 11 字节，是二进制数据，不带换行符或字符串结束符。

| 偏移 | 字段 | 长度 | 格式与说明 |
|---:|---|---:|---|
| 0 | HEAD0 | 1 | 固定 `0xAA` |
| 1 | HEAD1 | 1 | 固定 `0x55` |
| 2 | VERSION | 1 | 固定 `0x01` |
| 3 | SEQ | 1 | 每生成一个新应用帧加 1，`255 -> 0` 合法 |
| 4 | FLAGS | 1 | bit0=`START`，bit1=`ABORT`，bit2=`TASK_DYNAMIC`，bit3=`CAR_AT_A` |
| 5-8 | YAW_X100 | 4 | 有符号 `int32_t`，小端，单位 `0.01 deg` |
| 9-10 | CRC16 | 2 | CRC16-CCITT-FALSE，小端，低字节在前 |

`YAW_X100` 是小车相对上电朝向的累计航向角。小车与无人机摆放时应保持车头和机头同向；上电值为 0，逆时针为正。如果传感器是顺时针为正，应在打包前取负。允许发送负角度或超过一圈的累计角度，无人机会按 `360.00 deg` 归一化。

FLAGS 定义：

| FLAGS | 含义 | 无人机处理 |
|---:|---|---|
| `0x00` | 正常航向心跳 | 只更新航向与链路状态 |
| `0x01` | START + 抛投任务 | 锁存抛投任务启动请求 |
| `0x02` | ABORT | 锁存中止请求，进入降落保护流程 |
| `0x04` | TASK_DYNAMIC | 仅在 START 帧中使用，选择动态起降任务 |
| `0x05` | START + 动态起降任务 | 锁存动态起降任务启动请求 |
| `0x08` | CAR_AT_A | 小车已回到 A 点并停车，飞控锁存该事件 |
| `0x03` | START+ABORT | 禁止使用；无人机按 ABORT 优先处理 |

## 3. CRC 与打包参考

CRC 参数：多项式 `0x1021`、初值 `0xFFFF`、不反射、结果异或 `0x0000`。计算范围为字节 0-8，不包含 CRC 自身；结果以小端写入字节 9-10。

```c
static uint16_t car_crc16(const uint8_t *data, uint8_t length)
{
    uint16_t crc = 0xFFFFU;

    for (uint8_t i = 0U; i < length; i++)
    {
        crc ^= (uint16_t)data[i] << 8;
        for (uint8_t bit = 0U; bit < 8U; bit++)
        {
            crc = (crc & 0x8000U) ?
                  (uint16_t)((crc << 1) ^ 0x1021U) :
                  (uint16_t)(crc << 1);
        }
    }
    return crc;
}

static void car_build_frame(uint8_t frame[11], uint8_t sequence,
                            uint8_t flags, int32_t yaw_x100)
{
    const uint32_t yaw = (uint32_t)yaw_x100;
    uint16_t crc;

    frame[0] = 0xAAU;
    frame[1] = 0x55U;
    frame[2] = 0x01U;
    frame[3] = sequence;
    frame[4] = flags & 0x0FU;
    frame[5] = (uint8_t)yaw;
    frame[6] = (uint8_t)(yaw >> 8);
    frame[7] = (uint8_t)(yaw >> 16);
    frame[8] = (uint8_t)(yaw >> 24);
    crc = car_crc16(frame, 9U);
    frame[9] = (uint8_t)crc;
    frame[10] = (uint8_t)(crc >> 8);
}
```

已知正确测试帧：

```text
IDLE,  SEQ=00, YAW=0.00 deg:
AA 55 01 00 00 00 00 00 00 1F D4

START, SEQ=01, YAW=0.00 deg:
AA 55 01 01 01 00 00 00 00 EE 3B

RUN,   SEQ=02, YAW=+90.00 deg:
AA 55 01 02 00 28 23 00 00 44 32

ABORT, SEQ=03, YAW=+90.00 deg:
AA 55 01 03 02 28 23 00 00 67 33
```

## 4. 小车端发送状态机

建议固定以 `20-50 Hz` 发送，每帧都更新 SEQ 和航向。20 Hz 对应 50 ms 一帧；飞控超时为 200 ms，因此不要使用 5 Hz 临界频率。

### 4.1 IDLE - 等待启动

- 持续发送 `FLAGS=0x00`，保证飞控已获得有效航向和链路心跳。
- 操作员按下小车一键启动时，应在启动小车电机的同一事件中进入 `START_BURST`。
- 启动前应确认 UWB、光流、ToF、无线航向和 LX IMU 0x07 速度帧都已在无人机端有效。飞控仅在滤波 UWB 新鲜时接受 START 并冻结该时刻的 H 点坐标；若 UWB 无效，必须让小车停在 A 点，待数据恢复后重新发送 START。

### 4.2 START_BURST - 请求无人机启动

- 抛投任务连续 500 ms 发送 `FLAGS=0x01`；动态起降任务发送 `FLAGS=0x05`。20 Hz 时共 10 帧，每帧 SEQ 必须递增。
- 无人机收到滤波 UWB 新鲜的 START 帧后才会锁存请求；START 突发必须在小车仍位于 A 点期间完成。
- START 清除后不会取消已经锁存的请求。不要一直保持 START，否则任务结束后无法形成新的低到高启动事件。

### 4.3 RUNNING - 小车正常行驶

- 持续发送 `FLAGS=0x00` 和最新航向。
- 不需要等待无人机 ACK；当前协议没有无人机到小车的返回帧。
- 正常通信中断不会等价于 ABORT。追逐阶段无人机会暂时沿用最后一次目标，因此车端故障或主动停车前应尽可能发送 ABORT。

### 4.4 ABORT_BURST - 主动中止

- 故障、人工中止或比赛急停事件发生时，清除 START，连续 500 ms 发送 `FLAGS=0x02`，SEQ 继续递增。
- 无人机收到一帧有效 ABORT 后会锁存中止请求，停止水平目标并进入自动降落保护；一帧之后即使 FLAGS 清零，中止仍然有效。
- 发送完成后进入 `ABORTED`，继续发送 `FLAGS=0x00` 航向心跳或停止业务发送均可，但不要立即再次发送 START。

### 4.5 CAR_AT_A - 小车回到 A 点

- 小车完成一圈并在 A 点停车后，连续发送至少 500 ms 的 `FLAGS=0x08`，期间 SEQ 必须递增。
- `CAR_AT_A` 只用于通知无人机进入 H 点返航，不表示新的任务启动。
- 任务完成后无人机保持 `COMPLETE`，下一次任务必须重新上电，以恢复舵机夹紧状态。

推荐的小车端伪代码：

```c
every_20_to_50_hz()
{
    flags = 0x00U;

    if (state == CAR_START_BURST)
        flags = 0x01U | (task_dynamic ? 0x04U : 0x00U);
    else if (state == CAR_ABORT_BURST)
        flags = 0x02U;
    else if (state == CAR_AT_A_BURST)
        flags = 0x08U;

    car_build_frame(frame, tx_sequence++, flags, relative_yaw_x100);
    uart_send(frame, 11U);
}
```

## 5. 无人机端任务流程

飞控上电后始终接管自动任务，反复请求 LX IMU Mode 3 并保持上锁。仅在滤波 UWB 新鲜时接受 START，并在同一调度周期锁存任务类型和 H 点相对 A 点的返航坐标。

1. `WAIT_START -> PREPARE -> UNLOCK -> TAKEOFF -> HOVER`：起飞到 150 cm，稳定 500 ms 后悬停 3 s。
2. `TRACK_MODE -> ACQUIRE_CAR`：切换 LX IMU Mode 2，在巡航高度对准小车 UWB 目标点。
3. 抛投任务：`DROP_DESCEND(100 cm) -> DROP_DELAY(500 ms) -> DROP_RELEASE -> CLIMB_CRUISE`。
4. 动态起降任务：`CAR_LAND_DESCEND(15 cm) -> CAR_LOCK -> CAR_DWELL(5 s) -> RETAKEOFF -> Mode 2`。
5. 两类任务完成并恢复到巡航高度后进入 `WAIT_CAR_A`，在 Mode 2 原地定高悬停；小车独自回到 A 点并发送 `CAR_AT_A` 后，无人机以启动时保存的相对 UWB 坐标返回 H 点并执行 `LAND_H`。
6. `COMPLETE` 是终止状态，保持上锁并等待重新上电。状态切换会向 UD、UA 和 USB 上位机发送 `AUTO: ...` 文本。

追逐阶段的 UWB、航向、光流或 ToF 临时失效会沿用最后目标；低空抛投、车顶锁定和 H 点降落的进入条件必须重新获得有效 UWB、航向、光流、ToF 和 0x07 速度帧，其中光流、ToF 和速度帧均须在最近 500 ms 内更新。车顶上锁还要求 ToF 高度不高于 15 cm、水平对准且垂直速度绝对值不超过 20 cm/s，并在每次上锁前重新确认。ABORT、遥控急停、起飞超时或 90 s 任务超时进入 `LAND_FAILSAFE`。

## 6. 序号、超时和拒收规则

- 首帧或 200 ms 超时后的首帧接受任意 SEQ。
- 正常情况下只接受相对上一帧前进 `1-127` 的 SEQ。
- 重复帧、倒序帧和跨度 `128-255` 的帧被拒绝，且不会刷新 200 ms 心跳。
- `255 -> 0` 的自然回绕会被接受。
- 版本错误、CRC 错误、重复或旧 SEQ 都会增加 `CarLink.rejected_frame_count`。
- 无线模块无限重发会重复同一个 SEQ，既不能刷新链路，又可能阻塞新航向帧，因此必须避免。

## 7. 当前协议边界

目前小车无法通过该链路获知以下信息：

- 无人机是否已收到 START 或 ABORT；
- 无人机当前处于起飞、悬停、追逐、投放、降落中的哪个阶段；
- 无人机是否已完成投放、落到小车或返回起降点；
- 小车是否到达 B、C、D 点没有对应字段发送给无人机；正常任务在 `ACQUIRE_CAR` 后立即执行，以留出 D 点前完成的时间。
- `CAR_AT_A` 是唯一的路线完成标志，用于触发无人机返航 H 点；它必须在小车停车后发送。
- 自动投放由无人机状态机根据 UWB、高度和延时执行，小车无需发送单独的开爪命令。

如果小车程序必须根据无人机状态切换自身流程，需要后续增加无人机到小车的反向状态帧；在协议扩展完成前，小车端不得等待不存在的 ACK。

## 8. 联调检查表

1. 用串口助手直接接小车 MCU TX，确认 `115200 8N1`、每帧恰好 11 字节、无换行，并核对上述测试帧。
2. 两只无线模块参数完全一致；关闭“达到重发上限后继续发送”。
3. 飞控调试器观察 `CarLink.valid_frame_count` 持续增加，`rejected_frame_count` 不增长。
4. 静止同向上电时 `CarLink.yaw_x100` 接近 0；逆时针转动车头时该值增加。
5. 停止发送超过 200 ms，确认 `CarLink.valid=0`；恢复后新帧重新有效。
6. 发送一组 START 帧，确认无人机只在传感器就绪、上锁和 Mode 3 确认后解锁起飞。
7. START 清零后继续航向心跳，确认已锁存的任务不会被取消。
8. 飞行中发送一组 ABORT 帧，确认无人机进入降落流程，而不是继续追逐。
9. 分别发送 `FLAGS=0x01` 与 `FLAGS=0x05`，确认抛投和动态起降任务被正确锁存。
10. 任务完成后移动小车，确认无人机保持原地悬停；小车停在 A 点后发送 `FLAGS=0x08`，确认无人机切换为 H 点返航。
11. 任务完成后确认飞控保持 `COMPLETE` 和上锁；重新上电后舵机恢复夹紧。

飞控实现的代码依据为 [`Drv_CarLink.c`](../DriversBsp/Drv_CarLink.c) 和 [`User_Task.c`](../FcSrc/User_Task.c)。修改协议时必须同步更新两端程序和本文档。
