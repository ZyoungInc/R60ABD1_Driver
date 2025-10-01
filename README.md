# R60ABD1 60GHz 呼吸/睡眠雷达 — 协议库

> 适配文档版：**R60ABD1-呼吸睡眠雷达 V3.0**
> 串口：TTL/115200/8N1，固定帧头 `53 59`，帧尾 `54 43`，**SUM** 为“帧头+控制字+命令字+长度(2B)+数据”的**低 8 位**。

---

## 功能概览

* ✅ 发送完整协议帧（含 SUM）
* ✅ 接收状态机（丢包恢复、越界丢弃、快速重同步）
* ✅ 内置**回调路由**：自动把“查询/应答（cmd|0x80）”映射到**基础子命令**
* ✅ **产品信息特例**：`A1~A4` 自动映射至 `01~04` 回调
* ✅ 已覆盖**人体存在(0x80)**、**呼吸(0x81)**、**睡眠(0x84)**、**心率(0x85)**、**系统(0x01/0x05/0x07)**、**OTA(0x03)**
* ✅ 关键数据结构：`g_body / g_breath / g_heart / g_sleep / g_sys / g_product`
* ✅ 典型查询的**一键封装**（如 `radar_query_exist()`、`radar_query_night_summary()` 等）

---

## 目录结构

```
/your-project
├─ src/
│  ├─ radar_protocol.h
│  └─ radar_protocol.cpp
└─ your_sketch.ino
```

---

## 快速开始

### 1) 硬件连接与串口

* 默认使用 `Serial1`，引脚：`RX=7`、`TX=6`（可在 `radar_init(rx, tx, baud)` 里改）
* 波特率：`115200`，TTL 电平，8N1

### 2) 初始化与主循环

```cpp
#include "radar_protocol.h"

void setup() {
  Serial.begin(115200);
  radar_init(/*rx=*/7, /*tx=*/6, /*baud=*/115200);

  // 可选：注册一个默认打印回调，用于未覆盖的命令调试
  radar_set_default_handler([](uint8_t ctrl, uint8_t cmd, const uint8_t* d, uint16_t n){
    Serial.printf("[RX RAW] ctrl=%02X cmd=%02X len=%u\n", ctrl, cmd, n);
    for (uint16_t i = 0; i < n; ++i) Serial.printf(" %02X", d[i]);
    Serial.println();
  });

  // 例：打开人体存在&呼吸&心率&睡眠
  radar_set_body_enable(true);
  radar_set_breath_enable(true);
  radar_set_heart_enable(true);
  radar_set_sleep_enable(true);

  // 例：查询一次产品信息与系统状态
  radar_query_product_model();
  radar_query_fw_version();
  radar_query_heartbeat();
}

void loop() {
  // 必须周期调用以驱动接收状态机
  radar_process_rx();

  // 演示：周期查询
  static uint32_t t = 0;
  if (millis() - t > 1000) {
    t = millis();
    radar_query_exist();            // 人体存在
    radar_query_breath_value();     // 呼吸值
    radar_query_heart_value();      // 心率
    radar_query_composite10m();     // 睡眠10分钟综合
  }

  // 读取解析后的全局数据（示例）
  if (g_body.exist) {
    // 有人，打印距离和运动状态
    Serial.printf("Exist=1, Dist=%ucm, Motion=%u\n",
      g_body.distance_cm, g_body.motion);
  }
}
```

---

## 协议与校验

### 帧格式

```
[53 59] [CTRL] [CMD] [LEN_H LEN_L] [DATA ...] [SUM] [54 43]
```

* **SUM**：`(HDR1 + HDR2 + CTRL + CMD + LEN_H + LEN_L + DATA...) & 0xFF`

### 查询统一构造

* 所有“查询”命令：`CMD = (SUBCMD | 0x80)`，`DATA = 0x0F`，`LEN = 0x0001`
* 库内统一接口：`radar_send_query(ctrl, subcmd)`

### 示例 1：存在信息查询

* 逻辑：`CTRL=0x80`，`SUBCMD=0x01` → `CMD=0x81`，`DATA=0x0F`
* 完整帧（SUM 已算好）：
  `53 59 80 81 00 01 0F **BD** 54 43`

### 示例 2：睡眠统计查询（0x84 8F）

* 和值：`53+59+84+8F+00+01+0F=0x01CF` → **SUM=0xCF**
* 完整帧：`53 59 84 8F 00 01 0F **CF** 54 43`

---

## 数据结构要点

### 人体存在（`g_body`）

* `exist`：0无人/1有人
* `motion`：0无/1静止/2活跃
* `distance_cm`：2B, cm
* `pos_xyz[3]`：每轴“**最高位符号+14位幅值**” → 已在库中 `decode_smag14()` 解码

### 呼吸（`g_breath`）

* `info`：1正常/2过高/3过低/4无
* `value`：0~35
* `wave[5]`：1s 内 5 点，中轴 128
* `low_slow_thresh`：10~20（0x0B/0x8B）

### 心率（`g_heart`）

* `value`：60~120
* `wave[5]`：同上

### 睡眠（`g_sleep`）

* `bed_state`：0离床/1入床/2无
* `state`：0深/1浅/2清醒/3无
* `score`：**过程结束时**上报的单字节评分（0x06）
* `composite10m`（0x0C/0x8D 共 8B）：存在、状态、平均呼吸、平均心率、翻身次数、大/小体动占比、呼吸暂停次数
* `night`（整夜统计）：

  * **来源 0x0D 上报 或 0x8F 查询应答**，**顺序固定 12B**：

    1. `score`(0~100) → 2) `total_min`(2B, BE) → 3) `awake_pct` → 4) `light_pct` → 5) `deep_pct` →
    2. `outbed_min`(0~255) → 7) `outbed_cnt` → 8) `turn_cnt` → 9) `avg_breath`(0~25) → 10) `avg_heart`(0~100) → 11) `apnea_cnt`(0~10)
  * **评级 `grade`**：来自 0x10/0x90（0无/1良/2般/3差），与评分分离存储
* `cfg`：模式、挣扎/无人开关、无人/截止时间、挣扎灵敏度
* `abnormal`（0x0E/0x8E）：睡眠异常标志

> 注：库中已将 **整夜评分(`night.score`)** 与 **质量评级(`night.grade`)** 明确分离，避免混淆。

---

## 常用 API 速写

```cpp
// 开关
radar_set_body_enable(true);
radar_set_breath_enable(true);
radar_set_heart_enable(true);
radar_set_sleep_enable(true);

// 查询（人体存在）
radar_query_exist();
radar_query_motion();
radar_query_distance();
radar_query_direction();

// 查询（呼吸 & 心率）
radar_query_breath_info();
radar_query_breath_value();
radar_query_breath_wave();
radar_query_heart_value();
radar_query_heart_wave();

// 查询（睡眠）
radar_query_composite10m();     // 0x8D
radar_query_night_summary();    // 0x8F（12B 顺序见上）
radar_query_rating();           // 0x90（评级）
radar_query_abnormal();         // 0x8E
radar_query_mode();             // 0x8C

// 系统/产品
radar_query_heartbeat();        // 0x01/0x80
radar_query_initdone();         // 0x05/0x81
radar_query_posout();           // 0x07/0x87
radar_query_product_model();    // 0x02/A1
radar_query_fw_version();       // 0x02/A4
```

---

## 回调与扩展

* 已内置所有常用解析回调并完成注册
* 若你有**新增命令**，可注册自定义回调：

```cpp
bool ok = radar_register_handler(CTRL_SLEEP, /*cmd=*/0x23, my_cb);
```

* 未命中的帧会走 `default handler`（如已设置）

---

## OTA 示例（简化）

```cpp
// 1) 开始：传入固件总大小（大端存放）
radar_ota_begin(total_size);

// 2) 分片发送：frame_size/offset + 数据
radar_ota_send_chunk(frame_size, offset, chunk, len);

// 3) 结束：status=01 成功 / 02 失败
radar_ota_end(0x01);
```

---

## 调试与联调建议

1. 开始仅开**一种功能**，逐步叠加：
   `radar_set_body_enable(true)` → 确认 `radar_query_exist()` 正确应答，再加呼吸/心率/睡眠。
2. 打开 `RADAR_DEBUG 1`，观察 SUM 错误与重同步日志。
3. 有些帧为**状态变化上报**（无需查询），注意在 `loop()` 中持续调用 `radar_process_rx()`。
4. 板卡若没有 `Serial1`，可修改源文件改为其它硬件串口或 SoftwareSerial（不推荐高波特率）。

---

## 资源与限制

* 接收缓冲：512B（超长帧自动丢弃数据并保持同步）
* 线程安全：**非中断**，在 `loop()` 轮询解析
* 平台：基于 Arduino API（已在常见 ESP32/STM32 Arduino Core 上通过）

---

## 变更点摘要

* 明确区分**睡眠评分**（0x06/0x0D/0x8F 的首字节）与**质量评级**（0x10/0x90）
* `find_handler()`：

  * 统一映射 `cmd|0x80` 到基础子命令
  * **产品信息特例**：`A1~A4 → 01~04`
* 系统心跳应答 `0x01/0x80` 已注册到与上报 `0x01/0x01` 相同回调

---

## FAQ

**Q1：为什么查询时总要带 `0x0F` 数据？**
A：协议表明所有“查询”命令需携带 1B 数据 `0x0F`，库已在 `radar_send_query()` 内统一处理。

**Q2：0x0D 与 0x8F 有何区别？**
A：`0x0D` 为**上报**（判断为睡眠过程结束时上报整晚统计），`0x8F` 为**查询应答**。两者 12B 数据**字段顺序一致**，库都解析进 `g_sleep.night`。

**Q3：收包时偶现 SUM 错？**
A：串口抖动/丢字节常见。库会打印错误并尝试把当前字节作为新帧头快速重同步。

---



需要我把这个 README 直接落成 `README.md` 文件内容或添加几张典型帧的计算表吗？
