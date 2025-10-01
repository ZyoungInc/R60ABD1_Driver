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

# 快速开始（Quick Start）

1. **导入文件**
   把你贴的 `radar_protocol.h` / `radar_protocol.cpp` 放到同一工程（或库）里。

2. **初始化串口与雷达**
   在 `setup()` 里：

```cpp
Serial.begin(115200);
radar_init(/*rxPin=*/7, /*txPin=*/6, /*baud=*/115200);   // 用 Serial1，8N1，115200
```

3. **循环处理接收 + 发查询**
   在 `loop()` 里持续调用：

```cpp
radar_process_rx();  // 必须频繁调用，让协议状态机吃数据
```

需要数据时发起查询（例如存在、呼吸、心率、睡眠统计等）。

---

# 示例 1：BasicMonitor.ino（最常用：启用 → 周期查询 → 打印变化）

把下面整段粘到一个新 `BasicMonitor.ino` 里即可运行（串口监视器 115200）：

```cpp
#include "radar_protocol.h"

// —— 简单的“上一次值”，仅用于变化打印，避免刷屏 ——
static uint8_t  prev_exist      = 0xFF;
static uint8_t  prev_motion     = 0xFF;
static uint8_t  prev_breath     = 0xFF;
static uint8_t  prev_heart      = 0xFF;
static uint8_t  prev_sleep_state= 0xFF;
static uint8_t  prev_bed_state  = 0xFF;
static bool     was_in_bed      = false;

static uint32_t t1s   = 0;   // 1秒节拍：常规查询
static uint32_t t2s   = 0;   // 2秒节拍：距离/方位
static uint32_t t10s  = 0;   // 10秒节拍：综合 & 心跳/系统

void setup() {
  Serial.begin(115200);
  // Radar: Serial1 @ GPIO7(RX), GPIO6(TX), 115200 8N1
  radar_init(/*rxPin=*/7, /*txPin=*/6, /*baud=*/115200);

  // 打开各功能（根据需要选择）
  radar_set_body_enable(true);
  radar_set_breath_enable(true);
  radar_set_heart_enable(true);
  radar_set_sleep_enable(true);
  // 实时/睡眠模式（0=实时，1=睡眠状态模式）：按需选择
  radar_set_sleep_mode(1);

  Serial.println("Radar ready.");
}

static void print_body_if_changed() {
  if (g_body.exist != prev_exist) {
    prev_exist = g_body.exist;
    Serial.printf("[BODY] exist=%u (%s)\n", g_body.exist, g_body.exist ? "有人" : "无人");
  }
  if (g_body.motion != prev_motion) {
    prev_motion = g_body.motion;
    const char* ms = (g_body.motion==0? "无" : (g_body.motion==1? "静止":"活跃"));
    Serial.printf("[BODY] motion=%u (%s)\n", g_body.motion, ms);
  }
}

static void print_breath_if_changed() {
  if (g_breath.value != prev_breath) {
    prev_breath = g_breath.value;
    Serial.printf("[BREATH] value=%u 次/min, info=%u\n", g_breath.value, g_breath.info);
  }
}

static void print_heart_if_changed() {
  if (g_heart.value != prev_heart) {
    prev_heart = g_heart.value;
    Serial.printf("[HEART] value=%u 次/min\n", g_heart.value);
  }
}

static void print_sleep_if_changed() {
  if (g_sleep.bed_state != prev_bed_state) {
    prev_bed_state = g_sleep.bed_state;
    Serial.printf("[SLEEP] bed=%u (%s)\n", g_sleep.bed_state,
                  g_sleep.bed_state==0? "离床" : (g_sleep.bed_state==1? "入床":"无"));
  }
  if (g_sleep.state != prev_sleep_state) {
    prev_sleep_state = g_sleep.state;
    const char* ss = (g_sleep.state==0? "深睡":
                     (g_sleep.state==1? "浅睡":
                     (g_sleep.state==2? "清醒":"无")));
    Serial.printf("[SLEEP] state=%u (%s)\n", g_sleep.state, ss);
  }
}

static void try_fetch_night_summary_on_leave_bed() {
  // 检测“入床(1) -> 离床(0)”边沿，离床后请求整夜统计（0x8F）
  bool in_bed = (g_sleep.bed_state == 1);
  if (was_in_bed && !in_bed) {
    Serial.println("[SLEEP] Leave bed detected, query night summary...");
    radar_query_night_summary();   // 0x84/0x8F
    delay(30);
    // 让协议机先处理一下
    radar_process_rx();

    // 打印整夜统计（解析在 rx_sleep_night_sum 里把 12B 写到了 g_sleep.night）
    Serial.printf("[NIGHT] score=%u, total=%u min, awake=%u%%, light=%u%%, deep=%u%%\n",
                  g_sleep.night.score, g_sleep.night.total_min,
                  g_sleep.night.awake_pct, g_sleep.night.light_pct, g_sleep.night.deep_pct);
    Serial.printf("[NIGHT] outbed=%u min, outbed_cnt=%u, turn_cnt=%u, avg_breath=%u, avg_heart=%u, apnea=%u\n",
                  g_sleep.night.outbed_min, g_sleep.night.outbed_cnt, g_sleep.night.turn_cnt,
                  g_sleep.night.avg_breath, g_sleep.night.avg_heart, g_sleep.night.apnea_cnt);

    // 可选：查询“质量评级 0/1/2/3” 0x84/0x90
    radar_query_rating();
    delay(20);
    radar_process_rx();
    Serial.printf("[NIGHT] grade=%u (0无/1良/2般/3差)\n", g_sleep.night.grade);
  }
  was_in_bed = in_bed;
}

void loop() {
  radar_process_rx();  // 非常重要：越勤快越不丢包

  const uint32_t now = millis();

  // —— 1秒查询：存在/运动/呼吸/心率（轻量）——
  if (now - t1s >= 1000) {
    t1s = now;
    radar_query_exist();        // 0x80/0x81
    radar_query_motion();       // 0x80/0x82
    radar_query_breath_value(); // 0x81/0x82
    radar_query_heart_value();  // 0x85/0x82

    // 打印变化
    print_body_if_changed();
    print_breath_if_changed();
    print_heart_if_changed();
    print_sleep_if_changed();
    try_fetch_night_summary_on_leave_bed();
  }

  // —— 2秒查询：距离/方位（略重）——
  if (now - t2s >= 2000) {
    t2s = now;
    radar_query_distance();     // 0x80/0x84
    // 简单示例：收到后读 g_body.distance_cm（单位：cm）
    // 如需方位：radar_query_direction(); 然后查看 g_body.pos_xyz[3]
  }

  // —— 10秒查询：系统心跳/初始化/探测范围状态 + 睡眠10分钟综合 —— 
  if (now - t10s >= 10000) {
    t10s = now;
    radar_query_heartbeat();       // 0x01/0x80
    radar_query_initdone();        // 0x05/0x81
    radar_query_posout();          // 0x07/0x87
    radar_query_composite10m();    // 0x84/0x8D（8B）
  }

  // 轻微让步 CPU
  delay(2);
}
```

> 运行效果：串口里会看到 `BODY/ BREATH/ HEART/ SLEEP` 的数据变化；离床时自动拉一次整夜统计并打印。

---

# 示例 2：手动构造一条查询帧（可选，进阶）

一般无需自己拼帧，但若你想发“睡眠统计查询（0x84/0x8F）”，用库的底层接口也很方便：

```cpp
const uint8_t d = 0x0F;                           // 所有查询 data 固定 0x0F
radar_send_frame(CTRL_SLEEP, 0x8F, &d, 1);        // = 53 59 84 8F 00 01 0F CF 54 43
```

> 校验举例：`53+59+84+8F+00+01+0F = 0x01CF → 低8位 0xCF`

---

# API 速查表（最常用函数）

## 初始化与循环

| 函数                               | 作用                                         |
| -------------------------------- | ------------------------------------------ |
| `radar_init(rxPin, txPin, baud)` | 初始化串口与内置回调（ESP32-C6: `rx=7, tx=6, 115200`） |
| `radar_process_rx()`             | **必须频繁调用**；解析串口帧并填充全局结构体                   |

## 人体存在（CTRL=0x80）

| 函数                            | 含义                      | 典型返回                 |
| ----------------------------- | ----------------------- | -------------------- |
| `radar_set_body_enable(bool)` | 开/关存在检测                 | `g_body.enabled`     |
| `radar_query_exist()`         | 查询是否有人（0无人/1有人）         | `g_body.exist`       |
| `radar_query_motion()`        | 查询运动状态（0无/1静止/2活跃）      | `g_body.motion`      |
| `radar_query_body_param()`    | 体动强度（0~100）             | `g_body.body_param`  |
| `radar_query_distance()`      | 距离（cm, 2B）              | `g_body.distance_cm` |
| `radar_query_direction()`     | 方位（x,y,z 各 2B，符号+14位幅值） | `g_body.pos_xyz[3]`  |

## 呼吸（CTRL=0x81）

| 函数                              | 含义                  | 典型返回                              |
| ------------------------------- | ------------------- | --------------------------------- |
| `radar_set_breath_enable(bool)` | 开/关呼吸检测             | `g_breath.enabled`                |
| `radar_query_breath_info()`     | 状态 01正常/02高/03低/04无 | `g_breath.info`                   |
| `radar_query_breath_value()`    | 呼吸率（0~35 次/min）     | `g_breath.value`                  |
| `radar_query_breath_wave()`     | 5点波形（0~255，128为中轴）  | `g_breath.wave[5]`                |
| `radar_set_breath_low_slow(v)`  | 设置“低缓呼吸阈值” 10~20    | `radar_query_breath_lowslow()` 查询 |
| `radar_query_breath_lowslow()`  | 读取低缓阈值              | `g_breath.low_slow_thresh`        |

## 心率（CTRL=0x85）

| 函数                             | 含义               | 典型返回              |
| ------------------------------ | ---------------- | ----------------- |
| `radar_set_heart_enable(bool)` | 开/关心率检测          | `g_heart.enabled` |
| `radar_query_heart_value()`    | 心率（60~120 次/min） | `g_heart.value`   |
| `radar_query_heart_wave()`     | 5点波形（0~255）      | `g_heart.wave[5]` |

## 睡眠（CTRL=0x84）

| 函数                                                                        | 含义                  | 写入字段（收到后）                                           |
| ------------------------------------------------------------------------- | ------------------- | --------------------------------------------------- |
| `radar_set_sleep_enable(bool)`                                            | 开/关睡眠检测             | `g_sleep.enabled`                                   |
| `radar_set_sleep_mode(0/1)`                                               | 0=实时 / 1=睡眠模式       | `g_sleep.cfg.mode`                                  |
| `radar_query_sleep_bed_state()`                                           | 入床/离床 0离/1入/2无      | `g_sleep.bed_state`                                 |
| `radar_query_sleep_state()`                                               | 0深/1浅/2醒/3无         | `g_sleep.state`                                     |
| `radar_query_awake_min()`                                                 | 清醒时长（2B）            | `g_sleep.awake_min`                                 |
| `radar_query_light_min()`                                                 | 浅睡时长（2B）            | `g_sleep.light_min`                                 |
| `radar_query_deep_min()`                                                  | 深睡时长（2B）            | `g_sleep.deep_min`                                  |
| `radar_query_sleep_score()`                                               | 单字节“过程评分”           | `g_sleep.score`                                     |
| `radar_query_composite10m()`                                              | 10分钟综合（8B）          | `g_sleep.composite10m.*`                            |
| `radar_query_abnormal()`                                                  | 睡眠异常标志              | `g_sleep.abnormal`                                  |
| **`radar_query_night_summary()`**                                         | **整夜统计（0x8F, 12B）** | `g_sleep.night.*（见下）`                               |
| `radar_query_rating()`                                                    | 质量评级 0无/1良/2般/3差    | `g_sleep.night.grade`                               |
| 开关/阈值：`radar_set_sleep_struggle_enable` / `radar_query_struggle_switch` 等 | 挣扎/无人计时/阈值/灵敏度      | `g_sleep.cfg.*`                                     |
| 状态：`radar_query_struggle_state()` / `radar_query_noperson_state()`        | 0无/1正常/2异常          | `g_sleep.struggle_state` / `g_sleep.noperson_state` |

**整夜统计字段（收到 0x8F/0x0D 时）** → `g_sleep.night`

* `score`（评分 0~100）
* `total_min`（总时长，分钟）
* `awake_pct / light_pct / deep_pct`（三段占比 %）
* `outbed_min / outbed_cnt / turn_cnt`
* `avg_breath (0~25) / avg_heart (0~100) / apnea_cnt (0~10)`
* `grade`（另由 0x90 查询得到：0无/1良/2般/3差）

## 系统与产品

| 函数                            | 含义                | 返回/备注                            |
| ----------------------------- | ----------------- | -------------------------------- |
| `radar_query_heartbeat()`     | 系统心跳（0x01/0x80）   | `g_sys.heartbeat=1`              |
| `radar_query_initdone()`      | 初始化完成？（0x05/0x81） | `g_sys.init_done=0/1`            |
| `radar_query_posout()`        | 探测范围状态（0x07/0x87） | `g_sys.pos_outbound=0/1`         |
| `radar_query_product_model()` | 产品型号（0x02/A1）     | `g_product.model[..], model_len` |
| `radar_query_product_id()`    | 产品ID（0x02/A2）     | 同上                               |
| `radar_query_hw_model()`      | 硬件型号（0x02/A3）     | 同上                               |
| `radar_query_fw_version()`    | 固件版本（0x02/A4）     | 同上                               |

> **说明**：产品信息查询时，命令字用 A1~A4，库已自动把应答分派到 01~04 的回调去填 `g_product.*`。

## OTA（可选）

| 函数                                                     | 作用 |
| ------------------------------------------------------ | -- |
| `radar_ota_begin(total_size)`                          |    |
| `radar_ota_send_chunk(frame_size, offset, chunk, len)` |    |
| `radar_ota_end(status)`                                |    |

---

# 全局数据结构（只看你用得到的字段）

* `g_body`: `{ exist, motion, body_param, distance_cm, pos_xyz[3], enabled }`
* `g_breath`: `{ info, value, wave[5], low_slow_thresh, enabled }`
* `g_heart`: `{ value, wave[5], enabled }`
* `g_sleep`:

  * 即时：`bed_state, state, awake_min, light_min, deep_min, score, abnormal`
  * 10分钟：`composite10m.{ exist,state,avg_breath,avg_heart,turn_cnt,big_move_pct,small_move_pct,apnea_cnt }`
  * 整夜：`night.{ score,total_min,awake_pct,light_pct,deep_pct,outbed_min,outbed_cnt,turn_cnt,avg_breath,avg_heart,apnea_cnt,grade }`
  * 配置：`cfg.{ mode,struggle_enable,noperson_enable,noperson_min,cutoff_min,struggle_sens }`
  * 纯状态：`struggle_state, noperson_state`
  * `enabled`
* `g_sys`: `{ heartbeat, init_done, pos_outbound }`
* `g_product`: `{ model[..],id[..],hw_model[..],fw_version[..] + 各自长度 }`

---

# 帧格式与校验（遇到你要自己拼帧时）

```
53 59  <CTRL> <CMD>  <LEN_H> <LEN_L>  <DATA...>  <SUM>  54 43
SUM = (帧头53+59 + CTRL + CMD + LEN_H + LEN_L + 所有DATA) 低8位
```

* **查询帧**统一 `DATA=0x0F`、长度=1。
* 示例：**存在信息查询**（0x80/0x81）
  `53 59 80 81 00 01 0F BD 54 43`（SUM=0xBD）
* 示例：**整夜统计查询**（0x84/0x8F）
  `53 59 84 8F 00 01 0F CF 54 43`（SUM=0xCF）

> 你无需自己算 SUM，库会自动拼帧、求和与发送；接收时状态机会校验 SUM 并自动丢弃错误帧。

---

# 实用小贴士

* **一定要频繁调用** `radar_process_rx()`（如 loop 每次都调）。
* 查询**无需等待**：直接发，数据到时会被回调解析并写入 `g_*` 全局；你在 loop 里按需读取。
* 需要“变化上报”效果，就像示例里那样自己做个上次值缓存，变化时打印/处理。
* **长度单位**：距离是 cm；整夜总时长是分钟；占比是 %；波形 0~255，中轴 128。
* **睡眠统计时机**：

  * 0x0D 是“过程结束自动上报”；
  * 0x8F 是“你主动查询应答”。
    建议在**离床**时拉一次 0x8F。
* 如果想直接看“未知帧”调试，调用 `radar_set_default_handler(...)` 设置一个打印函数即可。


# 一、发出的 API（库函数 → 发送帧）

## A. 人体存在（CTRL=0x80）

| 调用函数                             | 含义        | CTRL |  CMD |   DATA | 说明                 |
| -------------------------------- | --------- | ---: | ---: | -----: | ------------------ |
| `radar_set_body_enable(bool en)` | 开/关存在检测   | 0x80 | 0x00 | 1B=0/1 | 设置                 |
| `radar_query_body_enable()`      | 查询开关      | 0x80 | 0x80 |   0x0F | 应答含 1B 开关          |
| `radar_query_exist()`            | 是否有人      | 0x80 | 0x81 |   0x0F | 应答 1B：0无人/1有人      |
| `radar_query_motion()`           | 运动状态      | 0x80 | 0x82 |   0x0F | 应答 1B：0无/1静/2活     |
| `radar_query_body_param()`       | 体动强度      | 0x80 | 0x83 |   0x0F | 应答 1B：0~100        |
| `radar_query_distance()`         | 距离        | 0x80 | 0x84 |   0x0F | 应答 2B：cm（BE）       |
| `radar_query_direction()`        | 方位（x,y,z） | 0x80 | 0x85 |   0x0F | 应答 6B：每轴“符号+14位幅值” |

## B. 呼吸（CTRL=0x81）

| 调用函数                                   | 含义      | CTRL |  CMD |     DATA | 说明                     |
| -------------------------------------- | ------- | ---: | ---: | -------: | ---------------------- |
| `radar_set_breath_enable(bool en)`     | 开/关呼吸检测 | 0x81 | 0x00 |   1B=0/1 | 设置                     |
| `radar_query_breath_enable()`          | 查询开关    | 0x81 | 0x80 |     0x0F | 应答 1B 开关               |
| `radar_query_breath_info()`            | 呼吸状态    | 0x81 | 0x81 |     0x0F | 应答 1B：01正常/02高/03低/04无 |
| `radar_query_breath_value()`           | 呼吸率     | 0x81 | 0x82 |     0x0F | 应答 1B：0~35 次/min       |
| `radar_query_breath_wave()`            | 呼吸波形    | 0x81 | 0x85 |     0x0F | 应答 5B：0~255（中轴128）     |
| `radar_set_breath_low_slow(uint8_t v)` | 低缓阈值设定  | 0x81 | 0x0B | 1B=10~20 | 设置                     |
| `radar_query_breath_lowslow()`         | 低缓阈值查询  | 0x81 | 0x8B |     0x0F | 应答 1B=10~20            |

## C. 心率（CTRL=0x85）

| 调用函数                              | 含义      | CTRL |  CMD |   DATA | 说明                 |
| --------------------------------- | ------- | ---: | ---: | -----: | ------------------ |
| `radar_set_heart_enable(bool en)` | 开/关心率检测 | 0x85 | 0x00 | 1B=0/1 | 设置                 |
| `radar_query_heart_enable()`      | 查询开关    | 0x85 | 0x80 |   0x0F | 应答 1B 开关           |
| `radar_query_heart_value()`       | 心率      | 0x85 | 0x82 |   0x0F | 应答 1B：60~120 次/min |
| `radar_query_heart_wave()`        | 心率波形    | 0x85 | 0x85 |   0x0F | 应答 5B：0~255        |

## D. 睡眠（CTRL=0x84）

### D1. 设置/开关

| 调用函数                                          | 含义      | CTRL |  CMD |            DATA |
| --------------------------------------------- | ------- | ---: | ---: | --------------: |
| `radar_set_sleep_enable(bool en)`             | 开/关睡眠检测 | 0x84 | 0x00 |          1B=0/1 |
| `radar_set_sleep_mode(uint8_t mode)`          | 上报模式    | 0x84 | 0x0F |      1B：0实时/1睡眠 |
| `radar_set_sleep_struggle_enable(bool en)`    | 挣扎开关    | 0x84 | 0x13 |          1B=0/1 |
| `radar_set_sleep_noperson_enable(bool en)`    | 无人计时开关  | 0x84 | 0x14 |          1B=0/1 |
| `radar_set_sleep_noperson_minutes(uint8_t m)` | 无人计时分钟  | 0x84 | 0x15 |        1B：5~120 |
| `radar_set_sleep_cutoff_minutes(uint8_t m)`   | 截止分钟    | 0x84 | 0x16 | 1B：30~180（步长10） |
| `radar_set_sleep_struggle_sens(uint8_t lvl)`  | 挣扎灵敏度   | 0x84 | 0x1A |     1B：0低/1中/2高 |

### D2. 按“子命令”查询（0x80+）

| 调用函数                            | 含义       | CTRL |  CMD | DATA |
| ------------------------------- | -------- | ---: | ---: | ---: |
| `radar_query_sleep_enable()`    | 睡眠开关     | 0x84 | 0x80 | 0x0F |
| `radar_query_sleep_bed_state()` | 入床/离床    | 0x84 | 0x81 | 0x0F |
| `radar_query_sleep_state()`     | 深/浅/醒/无  | 0x84 | 0x82 | 0x0F |
| `radar_query_awake_min()`       | 清醒分钟(2B) | 0x84 | 0x83 | 0x0F |
| `radar_query_light_min()`       | 浅睡分钟(2B) | 0x84 | 0x84 | 0x0F |
| `radar_query_deep_min()`        | 深睡分钟(2B) | 0x84 | 0x85 | 0x0F |
| `radar_query_sleep_score()`     | 过程评分(1B) | 0x84 | 0x86 | 0x0F |

### D3. “显式 opcode”查询（协议表里的 0x8C~）

| 调用函数                              | 含义            | CTRL |  CMD | DATA |
| --------------------------------- | ------------- | ---: | ---: | ---: |
| `radar_query_mode()`              | 上报模式          | 0x84 | 0x8C | 0x0F |
| `radar_query_composite10m()`      | 10分钟综合（8B）    | 0x84 | 0x8D | 0x0F |
| `radar_query_abnormal()`          | 睡眠异常(1B)      | 0x84 | 0x8E | 0x0F |
| **`radar_query_night_summary()`** | **整夜统计（12B）** | 0x84 | 0x8F | 0x0F |
| `radar_query_rating()`            | 质量评级(1B)      | 0x84 | 0x90 | 0x0F |
| `radar_query_struggle_state()`    | 挣扎状态(1B)      | 0x84 | 0x91 | 0x0F |
| `radar_query_noperson_state()`    | 无人状态(1B)      | 0x84 | 0x92 | 0x0F |
| `radar_query_struggle_switch()`   | 挣扎开关          | 0x84 | 0x93 | 0x0F |
| `radar_query_noperson_switch()`   | 无人计时开关        | 0x84 | 0x94 | 0x0F |
| `radar_query_noperson_minutes()`  | 无人计时分钟        | 0x84 | 0x95 | 0x0F |
| `radar_query_cutoff_minutes()`    | 截止分钟          | 0x84 | 0x96 | 0x0F |
| `radar_query_struggle_sens()`     | 挣扎灵敏度         | 0x84 | 0x9A | 0x0F |

## E. 系统/产品信息/OTA

| 调用函数                                         | 含义     | CTRL |  CMD |   DATA | 说明          |
| -------------------------------------------- | ------ | ---: | ---: | -----: | ----------- |
| `radar_query_heartbeat()`                    | 心跳查询   | 0x01 | 0x80 |   0x0F | 应答/上报均可     |
| `radar_query_initdone()`                     | 初始化完成？ | 0x05 | 0x81 |   0x0F | 1=已完成       |
| `radar_query_posout()`                       | 探测范围状态 | 0x07 | 0x87 |   0x0F | 0范围外/1范围内   |
| `radar_query_product_model()`                | 产品型号   | 0x02 | 0xA1 |   0x0F | 变长字符串       |
| `radar_query_product_id()`                   | 产品ID   | 0x02 | 0xA2 |   0x0F | 变长字符串       |
| `radar_query_hw_model()`                     | 硬件型号   | 0x02 | 0xA3 |   0x0F | 变长字符串       |
| `radar_query_fw_version()`                   | 固件版本   | 0x02 | 0xA4 |   0x0F | 变长字符串       |
| `radar_system_reset()`                       | 模组复位   | 0x01 | 0x02 |   0x0F | 复位命令        |
| `radar_ota_begin(total)`                     | OTA开始  | 0x03 | 0x01 |  4B总大小 | BE          |
| `radar_ota_send_chunk(frame_sz,off,buf,len)` | OTA分包  | 0x03 | 0x02 | 8B头+数据 | 头=4B帧长+4B偏移 |
| `radar_ota_end(status)`                      | OTA结束  | 0x03 | 0x03 |   1B状态 | 01成功/02失败   |

---

# 二、接收/应答/上报（库解析 → 写入的全局字段）

> 你只要读 `g_*` 就行。下表列出**收到哪些帧**时，库会把数据**写到哪里**、**单位/范围**和**常见上报节奏**。

## A. 人体存在（CTRL=0x80）

| 来源    | CTRL |         CMD | 负载        | 写入字段                 | 单位/范围       | 上报/频率     |
| ----- | ---: | ----------: | --------- | -------------------- | ----------- | --------- |
| 回复/上报 | 0x80 | 0x00 / 0x80 | 1B 开关     | `g_body.enabled`     | 0关/1开       | 变化或查询应答   |
| 回复/上报 | 0x80 | 0x01 / 0x81 | 1B 存在     | `g_body.exist`       | 0无人/1有人     | 变化或查询     |
| 回复/上报 | 0x80 | 0x02 / 0x82 | 1B 运动     | `g_body.motion`      | 0无/1静/2活    | 变化或查询     |
| 回复/上报 | 0x80 | 0x03 / 0x83 | 1B 体动强度   | `g_body.body_param`  | 0~100       | ~1s/变化或查询 |
| 回复/上报 | 0x80 | 0x04 / 0x84 | 2B 距离(BE) | `g_body.distance_cm` | cm(0~65535) | ~2s或查询    |
| 回复/上报 | 0x80 | 0x05 / 0x85 | 6B(x,y,z) | `g_body.pos_xyz[3]`  | 符号+14位幅值    | ~2s或查询    |

## B. 呼吸（CTRL=0x81）

| 来源    | CTRL |         CMD | 负载    | 写入字段                       | 范围/备注            | 上报/频率  |
| ----- | ---: | ----------: | ----- | -------------------------- | ---------------- | ------ |
| 回复/上报 | 0x81 | 0x00 / 0x80 | 1B 开关 | `g_breath.enabled`         | 0/1              | 变化/应答  |
| 回复/上报 | 0x81 | 0x01 / 0x81 | 1B 信息 | `g_breath.info`            | 01正常/02高/03低/04无 | 变化/查询  |
| 回复/上报 | 0x81 | 0x02 / 0x82 | 1B 速率 | `g_breath.value`           | 0~35 次/min       | ~3s或查询 |
| 回复/上报 | 0x81 | 0x05 / 0x85 | 5B 波形 | `g_breath.wave[5]`         | 0~255，中轴128      | ~1s或查询 |
| 回复/上报 | 0x81 | 0x0B / 0x8B | 1B 阈值 | `g_breath.low_slow_thresh` | 10~20            | 设置/查询  |

## C. 心率（CTRL=0x85）

| 来源    | CTRL |         CMD | 负载    | 写入字段              | 范围/备注        | 上报/频率  |
| ----- | ---: | ----------: | ----- | ----------------- | ------------ | ------ |
| 回复/上报 | 0x85 | 0x00 / 0x80 | 1B 开关 | `g_heart.enabled` | 0/1          | 变化/应答  |
| 回复/上报 | 0x85 | 0x02 / 0x82 | 1B 心率 | `g_heart.value`   | 60~120 次/min | ~3s或查询 |
| 回复/上报 | 0x85 | 0x05 / 0x85 | 5B 波形 | `g_heart.wave[5]` | 0~255        | ~1s或查询 |

## D. 睡眠（CTRL=0x84）

### D1. 即时状态/分钟/评分（子命令映射）

| 来源    | CTRL |  CMD(上报/应答) | 负载      | 写入字段                | 说明          |
| ----- | ---: | ----------: | ------- | ------------------- | ----------- |
| 回复/上报 | 0x84 | 0x00 / 0x80 | 1B 开关   | `g_sleep.enabled`   |             |
| 回复/上报 | 0x84 | 0x01 / 0x81 | 1B 入/离床 | `g_sleep.bed_state` | 0离/1入/2无    |
| 回复/上报 | 0x84 | 0x02 / 0x82 | 1B 状态   | `g_sleep.state`     | 0深/1浅/2醒/3无 |
| 回复/上报 | 0x84 | 0x03 / 0x83 | 2B 清醒分钟 | `g_sleep.awake_min` | BE，min      |
| 回复/上报 | 0x84 | 0x04 / 0x84 | 2B 浅睡分钟 | `g_sleep.light_min` | BE，min      |
| 回复/上报 | 0x84 | 0x05 / 0x85 | 2B 深睡分钟 | `g_sleep.deep_min`  | BE，min      |
| 上报/应答 | 0x84 | 0x06 / 0x86 | 1B 过程评分 | `g_sleep.score`     | 0~100       |

### D2. 10分钟综合（0x0C/0x8D）

| 来源    | CTRL |         CMD | 负载(8B)                                                                                       | 写入字段                     |
| ----- | ---: | ----------: | -------------------------------------------------------------------------------------------- | ------------------------ |
| 上报/应答 | 0x84 | 0x0C / 0x8D | exist(1B), state(1B), avg_breath(1B), avg_heart(1B), turn(1B), big(1B), small(1B), apnea(1B) | `g_sleep.composite10m.*` |

### D3. 整夜统计（0x0D 上报 / 0x8F 应答）

> 两者字段一致；0x0D 在“判断为睡眠过程结束时**上报**整晚统计”；0x8F 为你**主动查询**的应答。

| 来源 | CTRL |  CMD | 负载(12B) 顺序                                                                                                                                                       | 写入字段                                                                                                                         |
| -- | ---: | ---: | ---------------------------------------------------------------------------------------------------------------------------------------------------------------- | ---------------------------------------------------------------------------------------------------------------------------- |
| 上报 | 0x84 | 0x0D | score(1B), total_min(2B), awake_pct(1B), light_pct(1B), deep_pct(1B), outbed_min(1B), outbed_cnt(1B), turn_cnt(1B), avg_breath(1B), avg_heart(1B), apnea_cnt(1B) | `g_sleep.night.{score,total_min,awake_pct,light_pct,deep_pct,outbed_min,outbed_cnt,turn_cnt,avg_breath,avg_heart,apnea_cnt}` |
| 应答 | 0x84 | 0x8F | 同上 12B                                                                                                                                                           | 同上                                                                                                                           |

> 质量**评级**不是这 12B 里的字段，需要另外查询 `0x90`，库会写入 `g_sleep.night.grade`。

### D4. 异常/评级/模式/状态/开关/阈值（显式 opcode）

| 来源    | CTRL |         CMD | 负载 | 写入字段                          | 说明                   |
| ----- | ---: | ----------: | -- | ----------------------------- | -------------------- |
| 上报/应答 | 0x84 | 0x0E / 0x8E | 1B | `g_sleep.abnormal`            | 0不足4h/1>12h/2长时无人/3无 |
| 上报/应答 | 0x84 | 0x10 / 0x90 | 1B | `g_sleep.night.grade`         | 0无/1良/2般/3差          |
| 应答    | 0x84 |        0x8C | 1B | `g_sleep.cfg.mode`            | 0实时/1睡眠              |
| 上报/应答 | 0x84 | 0x11 / 0x91 | 1B | `g_sleep.struggle_state`      | 0无/1正常/2异常           |
| 上报/应答 | 0x84 | 0x12 / 0x92 | 1B | `g_sleep.noperson_state`      | 0无/1正常/2异常           |
| 应答    | 0x84 |        0x93 | 1B | `g_sleep.cfg.struggle_enable` | 0/1                  |
| 应答    | 0x84 |        0x94 | 1B | `g_sleep.cfg.noperson_enable` | 0/1                  |
| 应答    | 0x84 |        0x95 | 1B | `g_sleep.cfg.noperson_min`    | 5~120                |
| 应答    | 0x84 |        0x96 | 1B | `g_sleep.cfg.cutoff_min`      | 30~180               |
| 应答    | 0x84 |        0x9A | 1B | `g_sleep.cfg.struggle_sens`   | 0/1/2                |

## E. 系统/产品/OTA

| 来源    | CTRL |            CMD | 负载 | 写入字段                                       | 说明       |
| ----- | ---: | -------------: | -- | ------------------------------------------ | -------- |
| 上报/应答 | 0x01 |    0x01 / 0x80 | —  | `g_sys.heartbeat=1`                        | 心跳       |
| 上报/应答 | 0x05 |    0x01 / 0x81 | 1B | `g_sys.init_done`                          | 0未/1已    |
| 上报/应答 | 0x07 |    0x07 / 0x87 | 1B | `g_sys.pos_outbound`                       | 0外/1内    |
| 上报/应答 | 0x02 |    0x01 / 0xA1 | N  | `g_product.model[..], model_len`           | 字符串      |
| 上报/应答 | 0x02 |    0x02 / 0xA2 | N  | `g_product.product_id[..], product_id_len` | 字符串      |
| 上报/应答 | 0x02 |    0x03 / 0xA3 | N  | `g_product.hw_model[..], hw_model_len`     | 字符串      |
| 上报/应答 | 0x02 |    0x04 / 0xA4 | N  | `g_product.fw_version[..], fw_version_len` | 字符串      |
| 应答    | 0x03 | 0x01/0x02/0x03 | 见上 | —                                          | OTA 过程返回 |

# ① 常态（查询/拉取类）

## A. 人体存在（CTRL=0x80）

| 调用函数                        | 作用   | CTRL | CMD(发送) | 预期应答CMD | DATA(TX) | 应答负载      | 写入字段                 | 备注           |
| --------------------------- | ---- | ---: | ------: | ------: | -------: | --------- | -------------------- | ------------ |
| `radar_query_body_enable()` | 查询开关 | 0x80 |    0x80 |    0x80 |     0x0F | 1B 开关     | `g_body.enabled`     | 0关/1开        |
| `radar_query_exist()`       | 是否有人 | 0x80 |    0x81 |    0x81 |     0x0F | 1B 存在     | `g_body.exist`       | 0无人/1有人      |
| `radar_query_motion()`      | 运动状态 | 0x80 |    0x82 |    0x82 |     0x0F | 1B 运动     | `g_body.motion`      | 0无/1静/2活     |
| `radar_query_body_param()`  | 体动强度 | 0x80 |    0x83 |    0x83 |     0x0F | 1B 强度     | `g_body.body_param`  | 0~100        |
| `radar_query_distance()`    | 距离   | 0x80 |    0x84 |    0x84 |     0x0F | 2B cm(BE) | `g_body.distance_cm` | 0~65535      |
| `radar_query_direction()`   | 方位   | 0x80 |    0x85 |    0x85 |     0x0F | 6B x/y/z  | `g_body.pos_xyz[3]`  | 16位：符号+14位幅值 |

## B. 呼吸（CTRL=0x81）

| 调用函数                           | 作用   | CTRL | CMD(发) | 应答CMD | DATA | 应答负载     | 写入字段                       | 备注               |
| ------------------------------ | ---- | ---: | -----: | ----: | ---: | -------- | -------------------------- | ---------------- |
| `radar_query_breath_enable()`  | 查询开关 | 0x81 |   0x80 |  0x80 | 0x0F | 1B 开关    | `g_breath.enabled`         | 0/1              |
| `radar_query_breath_info()`    | 呼吸信息 | 0x81 |   0x81 |  0x81 | 0x0F | 1B 状态    | `g_breath.info`            | 01正常/02高/03低/04无 |
| `radar_query_breath_value()`   | 呼吸速率 | 0x81 |   0x82 |  0x82 | 0x0F | 1B 次/min | `g_breath.value`           | 0~35             |
| `radar_query_breath_wave()`    | 呼吸波形 | 0x81 |   0x85 |  0x85 | 0x0F | 5B       | `g_breath.wave[5]`         | 中轴128            |
| `radar_query_breath_lowslow()` | 低缓阈值 | 0x81 |   0x8B |  0x8B | 0x0F | 1B       | `g_breath.low_slow_thresh` | 10~20            |

## C. 心率（CTRL=0x85）

| 调用函数                         | 作用   | CTRL | CMD(发) | 应答CMD | DATA | 应答负载     | 写入字段              | 备注     |
| ---------------------------- | ---- | ---: | -----: | ----: | ---: | -------- | ----------------- | ------ |
| `radar_query_heart_enable()` | 查询开关 | 0x85 |   0x80 |  0x80 | 0x0F | 1B 开关    | `g_heart.enabled` | 0/1    |
| `radar_query_heart_value()`  | 心率数值 | 0x85 |   0x82 |  0x82 | 0x0F | 1B 次/min | `g_heart.value`   | 60~120 |
| `radar_query_heart_wave()`   | 心率波形 | 0x85 |   0x85 |  0x85 | 0x0F | 5B       | `g_heart.wave[5]` | 0~255  |

## D. 睡眠（CTRL=0x84）

### D1. 子命令查询（0x80+）

| 调用函数                            | 作用      | CTRL | CMD(发) | 应答CMD | DATA | 应答负载     | 写入字段                           |
| ------------------------------- | ------- | ---: | -----: | ----: | ---: | -------- | ------------------------------ |
| `radar_query_sleep_enable()`    | 睡眠开关    | 0x84 |   0x80 |  0x80 | 0x0F | 1B       | `g_sleep.enabled`              |
| `radar_query_sleep_bed_state()` | 入/离床    | 0x84 |   0x81 |  0x81 | 0x0F | 1B       | `g_sleep.bed_state` (0离/1入/2无) |
| `radar_query_sleep_state()`     | 深/浅/醒/无 | 0x84 |   0x82 |  0x82 | 0x0F | 1B       | `g_sleep.state` (0深/1浅/2醒/3无)  |
| `radar_query_awake_min()`       | 清醒分钟    | 0x84 |   0x83 |  0x83 | 0x0F | 2B(BE)   | `g_sleep.awake_min`            |
| `radar_query_light_min()`       | 浅睡分钟    | 0x84 |   0x84 |  0x84 | 0x0F | 2B(BE)   | `g_sleep.light_min`            |
| `radar_query_deep_min()`        | 深睡分钟    | 0x84 |   0x85 |  0x85 | 0x0F | 2B(BE)   | `g_sleep.deep_min`             |
| `radar_query_sleep_score()`     | 过程评分    | 0x84 |   0x86 |  0x86 | 0x0F | 1B 0~100 | `g_sleep.score`                |

### D2. 显式 opcode 查询（协议表里的 0x8C~）

| 调用函数                              | 作用       | CTRL | CMD(发) | 应答CMD | DATA | 应答负载    | 写入字段                          | 备注                   |
| --------------------------------- | -------- | ---: | -----: | ----: | ---: | ------- | ----------------------------- | -------------------- |
| `radar_query_mode()`              | 上报模式     | 0x84 |   0x8C |  0x8C | 0x0F | 1B      | `g_sleep.cfg.mode`            | 0实时/1睡眠              |
| `radar_query_composite10m()`      | 10分钟综合   | 0x84 |   0x8D |  0x8D | 0x0F | 8B      | `g_sleep.composite10m.*`      | 次均值/翻身/体动/暂停         |
| `radar_query_abnormal()`          | 睡眠异常     | 0x84 |   0x8E |  0x8E | 0x0F | 1B      | `g_sleep.abnormal`            | 0不足4h/1>12h/2长时无人/3无 |
| **`radar_query_night_summary()`** | **整夜统计** | 0x84 |   0x8F |  0x8F | 0x0F | **12B** | **`g_sleep.night.*`**         | 字段顺序见下               |
| `radar_query_rating()`            | 质量评级     | 0x84 |   0x90 |  0x90 | 0x0F | 1B      | `g_sleep.night.grade`         | 0无/1良/2般/3差          |
| `radar_query_struggle_state()`    | 挣扎状态     | 0x84 |   0x91 |  0x91 | 0x0F | 1B      | `g_sleep.struggle_state`      | 0无/1正常/2异常           |
| `radar_query_noperson_state()`    | 无人状态     | 0x84 |   0x92 |  0x92 | 0x0F | 1B      | `g_sleep.noperson_state`      | 0无/1正常/2异常           |
| `radar_query_struggle_switch()`   | 挣扎开关     | 0x84 |   0x93 |  0x93 | 0x0F | 1B      | `g_sleep.cfg.struggle_enable` | 0/1                  |
| `radar_query_noperson_switch()`   | 无人计时开关   | 0x84 |   0x94 |  0x94 | 0x0F | 1B      | `g_sleep.cfg.noperson_enable` | 0/1                  |
| `radar_query_noperson_minutes()`  | 无人分钟     | 0x84 |   0x95 |  0x95 | 0x0F | 1B      | `g_sleep.cfg.noperson_min`    | 5~120                |
| `radar_query_cutoff_minutes()`    | 截止分钟     | 0x84 |   0x96 |  0x96 | 0x0F | 1B      | `g_sleep.cfg.cutoff_min`      | 30~180               |
| `radar_query_struggle_sens()`     | 挣扎灵敏度    | 0x84 |   0x9A |  0x9A | 0x0F | 1B      | `g_sleep.cfg.struggle_sens`   | 0/1/2                |

**整夜统计（0x0D 上报/0x8F 应答）12B 字段顺序**
`score(1B), total_min(2B,BE), awake_pct(1B), light_pct(1B), deep_pct(1B), outbed_min(1B), outbed_cnt(1B), turn_cnt(1B), avg_breath(1B), avg_heart(1B), apnea_cnt(1B)`
→ 库写入 `g_sleep.night` 的同名字段。

## E. 系统/产品信息（常用查询）

| 调用函数                          | 作用     | CTRL | CMD(发) |     应答CMD | DATA | 应答负载 | 写入字段                            |
| ----------------------------- | ------ | ---: | -----: | --------: | ---: | ---- | ------------------------------- |
| `radar_query_heartbeat()`     | 心跳查询   | 0x01 |   0x80 | 0x80/0x01 | 0x0F | —/1B | `g_sys.heartbeat=1`             |
| `radar_query_initdone()`      | 初始化完成？ | 0x05 |   0x81 | 0x81/0x01 | 0x0F | 1B   | `g_sys.init_done` (0/1)         |
| `radar_query_posout()`        | 探测范围状态 | 0x07 |   0x87 | 0x87/0x07 | 0x0F | 1B   | `g_sys.pos_outbound` (0外/1内)    |
| `radar_query_product_model()` | 产品型号   | 0x02 |   0xA1 | 0xA1/0x01 | 0x0F | NB   | `g_product.model[..], len`      |
| `radar_query_product_id()`    | 产品ID   | 0x02 |   0xA2 | 0xA2/0x02 | 0x0F | NB   | `g_product.product_id[..], len` |
| `radar_query_hw_model()`      | 硬件型号   | 0x02 |   0xA3 | 0xA3/0x03 | 0x0F | NB   | `g_product.hw_model[..], len`   |
| `radar_query_fw_version()`    | 固件版本   | 0x02 |   0xA4 | 0xA4/0x04 | 0x0F | NB   | `g_product.fw_version[..], len` |

---

# ② 接收 + 上报（设备→主机，库已解析）

> 设备可能**主动上报**或对你的查询**应答**。下表列出 RX 帧进入库后**会写到哪些全局变量**与常见上报节奏（若有）。

## A. 人体存在（0x80）

| 来源    | CTRL |   CMD(RX) | 负载    | 写入字段                 | 频率/触发   |
| ----- | ---: | --------: | ----- | -------------------- | ------- |
| 上报/应答 | 0x80 | 0x00/0x80 | 1B 开关 | `g_body.enabled`     | 开关变化/查询 |
| 上报/应答 | 0x80 | 0x01/0x81 | 1B 存在 | `g_body.exist`       | 状态变化/查询 |
| 上报/应答 | 0x80 | 0x02/0x82 | 1B 运动 | `g_body.motion`      | 变化/查询   |
| 上报/应答 | 0x80 | 0x03/0x83 | 1B 强度 | `g_body.body_param`  | ~1s/查询  |
| 上报/应答 | 0x80 | 0x04/0x84 | 2B 距离 | `g_body.distance_cm` | ~2s/查询  |
| 上报/应答 | 0x80 | 0x05/0x85 | 6B 方位 | `g_body.pos_xyz[]`   | ~2s/查询  |

## B. 呼吸（0x81）

| 来源    | CTRL |   CMD(RX) | 负载    | 写入字段                       | 频率/触发   |
| ----- | ---: | --------: | ----- | -------------------------- | ------- |
| 上报/应答 | 0x81 | 0x00/0x80 | 1B 开关 | `g_breath.enabled`         | 变化/查询   |
| 上报/应答 | 0x81 | 0x01/0x81 | 1B 信息 | `g_breath.info`            | 状态变化/查询 |
| 上报/应答 | 0x81 | 0x02/0x82 | 1B 速率 | `g_breath.value`           | ~3s/查询  |
| 上报/应答 | 0x81 | 0x05/0x85 | 5B 波形 | `g_breath.wave[]`          | ~1s/查询  |
| 上报/应答 | 0x81 | 0x0B/0x8B | 1B 阈值 | `g_breath.low_slow_thresh` | 设置回显/查询 |

## C. 心率（0x85）

| 来源    | CTRL |   CMD(RX) | 负载    | 写入字段              | 频率/触发  |
| ----- | ---: | --------: | ----- | ----------------- | ------ |
| 上报/应答 | 0x85 | 0x00/0x80 | 1B 开关 | `g_heart.enabled` | 变化/查询  |
| 上报/应答 | 0x85 | 0x02/0x82 | 1B 心率 | `g_heart.value`   | ~3s/查询 |
| 上报/应答 | 0x85 | 0x05/0x85 | 5B 波形 | `g_heart.wave[]`  | ~1s/查询 |

## D. 睡眠（0x84）

| 来源     | CTRL |                  CMD(RX) | 负载           | 写入字段                     | 频率/触发       |
| ------ | ---: | -----------------------: | ------------ | ------------------------ | ----------- |
| 上报/应答  | 0x84 |                0x00/0x80 | 1B 开关        | `g_sleep.enabled`        | —           |
| 上报/应答  | 0x84 |                0x01/0x81 | 1B 入/离床      | `g_sleep.bed_state`      | 变化时         |
| 上报/应答  | 0x84 |                0x02/0x82 | 1B 状态        | `g_sleep.state`          | 10min 上报    |
| 上报/应答  | 0x84 |                0x03/0x83 | 2B           | `g_sleep.awake_min`      | 随 10min     |
| 上报/应答  | 0x84 |                0x04/0x84 | 2B           | `g_sleep.light_min`      | 随 10min     |
| 上报/应答  | 0x84 |                0x05/0x85 | 2B           | `g_sleep.deep_min`       | 随 10min     |
| 上报/应答  | 0x84 |                0x06/0x86 | 1B 评分        | `g_sleep.score`          | 过程或查询       |
| 上报/应答  | 0x84 |                0x0C/0x8D | 8B 10min综    | `g_sleep.composite10m.*` | 10min 上报    |
| **上报** | 0x84 |                 **0x0D** | **12B 整夜统计** | **`g_sleep.night.*`**    | **睡眠过程结束时** |
| **应答** | 0x84 |                 **0x8F** | **同上 12B**   | **`g_sleep.night.*`**    | 主动查询        |
| 上报/应答  | 0x84 |                0x0E/0x8E | 1B 异常        | `g_sleep.abnormal`       | 条件触发/查询     |
| 上报/应答  | 0x84 |                0x10/0x90 | 1B 评级        | `g_sleep.night.grade`    | 结束或查询       |
| 应答     | 0x84 |                     0x8C | 1B 模式        | `g_sleep.cfg.mode`       | 查询          |
| 上报/应答  | 0x84 |                0x11/0x91 | 1B 挣扎态       | `g_sleep.struggle_state` | 状态/查询       |
| 上报/应答  | 0x84 |                0x12/0x92 | 1B 无人态       | `g_sleep.noperson_state` | 状态/查询       |
| 应答     | 0x84 | 0x93/0x94/0x95/0x96/0x9A | 各 1B         | `g_sleep.cfg.*`          | 查询或设置回显     |

## E. 系统/产品/OTA

| 来源    | CTRL |             CMD(RX) | 负载    | 写入字段                 | 说明     |
| ----- | ---: | ------------------: | ----- | -------------------- | ------ |
| 上报/应答 | 0x01 |           0x01/0x80 | —/1B  | `g_sys.heartbeat=1`  | 心跳     |
| 上报/应答 | 0x05 |           0x01/0x81 | 1B    | `g_sys.init_done`    | 0未/1已  |
| 上报/应答 | 0x07 |           0x07/0x87 | 1B    | `g_sys.pos_outbound` | 0外/1内  |
| 上报/应答 | 0x02 | 0x01/0xA1…0x04/0xA4 | N     | `g_product.*`        | 字符串    |
| 应答    | 0x03 |      0x01/0x02/0x03 | 见 OTA | —                    | OTA 过程 |

---

# ③ 设置（配置/开关/阈值/模式/OTA/复位）

## A. 人体存在/呼吸/心率开关

| 函数（发）                              | 作用   | CTRL |  CMD |   DATA | 推荐验证（查询）                      |
| ---------------------------------- | ---- | ---: | ---: | -----: | ----------------------------- |
| `radar_set_body_enable(bool en)`   | 开关存在 | 0x80 | 0x00 | 1B=0/1 | `radar_query_body_enable()`   |
| `radar_set_breath_enable(bool en)` | 开关呼吸 | 0x81 | 0x00 | 1B=0/1 | `radar_query_breath_enable()` |
| `radar_set_heart_enable(bool en)`  | 开关心率 | 0x85 | 0x00 | 1B=0/1 | `radar_query_heart_enable()`  |

## B. 睡眠配置

| 函数（发）                                         | 作用     | CTRL |  CMD | DATA 值域     | 推荐验证                             |
| --------------------------------------------- | ------ | ---: | ---: | ----------- | -------------------------------- |
| `radar_set_sleep_enable(bool en)`             | 开关睡眠   | 0x84 | 0x00 | 0/1         | `radar_query_sleep_enable()`     |
| `radar_set_sleep_mode(uint8_t mode)`          | 上报模式   | 0x84 | 0x0F | 0实时/1睡眠     | `radar_query_mode()`             |
| `radar_set_sleep_struggle_enable(bool en)`    | 挣扎判读开关 | 0x84 | 0x13 | 0/1         | `radar_query_struggle_switch()`  |
| `radar_set_sleep_noperson_enable(bool en)`    | 无人计时开关 | 0x84 | 0x14 | 0/1         | `radar_query_noperson_switch()`  |
| `radar_set_sleep_noperson_minutes(uint8_t m)` | 无人分钟   | 0x84 | 0x15 | 5~120       | `radar_query_noperson_minutes()` |
| `radar_set_sleep_cutoff_minutes(uint8_t m)`   | 截止分钟   | 0x84 | 0x16 | 30~180 步长10 | `radar_query_cutoff_minutes()`   |
| `radar_set_sleep_struggle_sens(uint8_t lvl)`  | 挣扎灵敏度  | 0x84 | 0x1A | 0低/1中/2高    | `radar_query_struggle_sens()`    |

## C. 呼吸阈值

| 函数（发）                                  | 作用   | CTRL |  CMD |  DATA | 推荐验证                           |
| -------------------------------------- | ---- | ---: | ---: | ----: | ------------------------------ |
| `radar_set_breath_low_slow(uint8_t v)` | 低缓阈值 | 0x81 | 0x0B | 10~20 | `radar_query_breath_lowslow()` |

## D. 系统控制/查询

| 函数（发）                     | 作用     | CTRL |  CMD | DATA | 备注      |
| ------------------------- | ------ | ---: | ---: | ---: | ------- |
| `radar_system_reset()`    | 模组复位   | 0x01 | 0x02 | 0x0F | 复位命令    |
| `radar_query_heartbeat()` | 心跳     | 0x01 | 0x80 | 0x0F | 应答/上报均可 |
| `radar_query_initdone()`  | 初始化完成？ | 0x05 | 0x81 | 0x0F | 0未/1已   |
| `radar_query_posout()`    | 探测范围   | 0x07 | 0x87 | 0x0F | 0外/1内   |

## E. 产品信息（字符串）

| 函数（发）                         | 作用   | CTRL |  CMD | DATA | 应答负载 |
| ----------------------------- | ---- | ---: | ---: | ---: | ---- |
| `radar_query_product_model()` | 产品型号 | 0x02 | 0xA1 | 0x0F | N 字节 |
| `radar_query_product_id()`    | 产品ID | 0x02 | 0xA2 | 0x0F | N 字节 |
| `radar_query_hw_model()`      | 硬件型号 | 0x02 | 0xA3 | 0x0F | N 字节 |
| `radar_query_fw_version()`    | 固件版本 | 0x02 | 0xA4 | 0x0F | N 字节 |

## F. OTA（可选）

| 函数（发）                                              | 作用     | CTRL |  CMD | DATA 结构                | 备注        |
| -------------------------------------------------- | ------ | ---: | ---: | ---------------------- | --------- |
| `radar_ota_begin(total_size)`                      | 开始 OTA | 0x03 | 0x01 | 4B 总大小（BE）             | 设备回包含建议帧长 |
| `radar_ota_send_chunk(frame_sz, offset, buf, len)` | 分片发送   | 0x03 | 0x02 | 8B 头(4B 帧长+4B 偏移) + 数据 | 遵循回包建议长度  |
| `radar_ota_end(status)`                            | 结束     | 0x03 | 0x03 | 1B 状态(01成功/02失败)       | —         |

---

