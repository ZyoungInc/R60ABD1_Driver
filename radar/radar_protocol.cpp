#include "radar_protocol.h"
#include <string.h>  // for memcpy

/* ================== 全局数据 ================== */
body_exist_t g_body   = {};
breath_t     g_breath = {};
heart_t      g_heart  = {};
sleep_t      g_sleep  = {};
system_t     g_sys    = {};
product_t    g_product = {};

/* ================== 调试选项 ================== */
#ifndef RADAR_DEBUG
#define RADAR_DEBUG 1
#endif

/* ================== 工具 ================== */
static uint8_t sum8(const uint8_t* p, size_t n) {
  uint16_t s = 0;
  for (size_t i = 0; i < n; ++i) s += p[i];
  return (uint8_t)(s & 0xFF);
}

static uint16_t be16(const uint8_t* d) {
  return (uint16_t(d[0]) << 8) | d[1];
}

// 解码 “最高位符号 + 14位幅值”的16位（人体方位）
static int16_t decode_smag14(uint16_t raw_be) {
  uint8_t hi = (raw_be >> 8) & 0xFF;
  uint8_t lo = raw_be & 0xFF;
  uint16_t raw = ((uint16_t)hi << 8) | lo;
  int sign = (raw & 0x8000) ? -1 : +1;
  int mag  = (raw & 0x3FFF);
  return (int16_t)(sign * mag);
}

/* ================== 发送 ================== */
bool radar_send_frame(uint8_t ctrl, uint8_t cmd, const uint8_t* data, uint16_t len) {
  uint8_t hdr[6] = { RADAR_HDR1, RADAR_HDR2, ctrl, cmd, uint8_t(len >> 8), uint8_t(len) };
  uint8_t s = sum8(hdr, sizeof(hdr)) + (data ? sum8(data, len) : 0);
  const uint8_t tail[2] = { RADAR_END1, RADAR_END2 };

  if (Serial1.write(hdr, sizeof(hdr)) != sizeof(hdr)) return false;
  if (len && data && Serial1.write(data, len) != len) return false;
  Serial1.write(&s, 1);
  Serial1.write(tail, sizeof(tail));

#if RADAR_DEBUG
  // Serial.printf("[TX] ctrl=0x%02X cmd=0x%02X len=%u sum=0x%02X\n", ctrl, cmd, len, s);
#endif
  return true;
}

bool radar_send_query(uint8_t ctrl, uint8_t subcmd) {
  const uint8_t d = 0x0F;
  return radar_send_frame(ctrl, RADAR_QUERY(subcmd), &d, 1);
}

bool radar_send_set_u8(uint8_t ctrl, uint8_t subcmd, uint8_t value) {
  return radar_send_frame(ctrl, subcmd, &value, 1);
}

/* ================== 回调注册表 ================== */
struct Slot {
  bool used;
  uint8_t ctrl, cmd;
  RadarRecvFunc cb;
};
static Slot s_slots[64];
static RadarRecvFunc s_default_cb = nullptr;

bool radar_register_handler(uint8_t ctrl, uint8_t cmd, RadarRecvFunc cb) {
  for (auto &sl : s_slots) if (sl.used && sl.ctrl == ctrl && sl.cmd == cmd) {
      sl.cb = cb;
      return true;
    }
  for (auto &sl : s_slots) if (!sl.used) {
      sl.used = true;
      sl.ctrl = ctrl;
      sl.cmd = cmd;
      sl.cb = cb;
      return true;
    }
  return false;
}
bool radar_unregister_handler(uint8_t ctrl, uint8_t cmd) {
  for (auto &sl : s_slots) if (sl.used && sl.ctrl == ctrl && sl.cmd == cmd) {
      sl.used = false;
      sl.cb = nullptr;
      return true;
    }
  return false;
}
bool radar_set_default_handler(RadarRecvFunc cb) {
  s_default_cb = cb;
  return true;
}

static RadarRecvFunc find_handler(uint8_t ctrl, uint8_t cmd) {
  for (auto &sl : s_slots)
    if (sl.used && sl.ctrl == ctrl && sl.cmd == cmd) return sl.cb;

  // 规则一：0x80 查询/应答，映射到基础子命令
  if (cmd & 0x80) {
    uint8_t base = RADAR_SUBCMD(cmd);
    for (auto &sl : s_slots)
      if (sl.used && sl.ctrl == ctrl && sl.cmd == base) return sl.cb;
  }

  // 规则二：产品信息特例（A1~A4 → 01~04）
  if (ctrl == CTRL_PRODUCT && (cmd & 0xF0) == 0xA0) {
    uint8_t base = (cmd & 0x0F);
    for (auto &sl : s_slots)
      if (sl.used && sl.ctrl == ctrl && sl.cmd == base) return sl.cb;
  }

  return nullptr;
}

/* ================== 内置解析回调 ================== */
/* ---- 0x80 人体存在 ---- */
static void rx_body_enable(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_body.enabled = d[0];
}
static void rx_body_exist(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_body.exist = d[0];
}
static void rx_body_motion(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_body.motion = d[0];
}
static void rx_body_param(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_body.body_param = d[0];
}
static void rx_body_distance(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 2) g_body.distance_cm = be16(d);
}
static void rx_body_direction(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 6) {
    g_body.pos_xyz[0] = decode_smag14(be16(d + 0));
    g_body.pos_xyz[1] = decode_smag14(be16(d + 2));
    g_body.pos_xyz[2] = decode_smag14(be16(d + 4));
  }
}

/* ---- 0x81 呼吸 ---- */
static void rx_breath_enable(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_breath.enabled = d[0];
}
static void rx_breath_info(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_breath.info = d[0];
}
static void rx_breath_value(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_breath.value = d[0];
}
static void rx_breath_wave(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 5) for (int i = 0; i < 5; i++) g_breath.wave[i] = d[i];
}
static void rx_breath_low_slow(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_breath.low_slow_thresh = d[0];
}

/* ---- 0x85 心率 ---- */
static void rx_heart_enable(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_heart.enabled = d[0];
}
static void rx_heart_value(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_heart.value = d[0];
}
static void rx_heart_wave(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 5) for (int i = 0; i < 5; i++) g_heart.wave[i] = d[i];
}

/* ---- 0x84 睡眠 ---- */
static void rx_sleep_enable(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.enabled = d[0];
}
static void rx_sleep_bed(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.bed_state = d[0];
}
static void rx_sleep_state(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.state = d[0];
}
static void rx_sleep_awake(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 2) g_sleep.awake_min = be16(d);
}
static void rx_sleep_light(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 2) g_sleep.light_min = be16(d);
}
static void rx_sleep_deep(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 2) g_sleep.deep_min = be16(d);
}
static void rx_sleep_score(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.score = d[0];
}
static void rx_sleep_comp10m(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 8) {
    g_sleep.composite10m.exist          = d[0];
    g_sleep.composite10m.state          = d[1];
    g_sleep.composite10m.avg_breath     = d[2];
    g_sleep.composite10m.avg_heart      = d[3];
    g_sleep.composite10m.turn_cnt       = d[4];
    g_sleep.composite10m.big_move_pct   = d[5];
    g_sleep.composite10m.small_move_pct = d[6];
    g_sleep.composite10m.apnea_cnt      = d[7];
  }
}
static void rx_sleep_night_sum(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 12) {
    g_sleep.night.score     = d[0];                    // 1B 评分 0~100
    g_sleep.night.total_min = (uint16_t(d[1]) << 8) | d[2]; // 2B 总时长 BE
    g_sleep.night.awake_pct = d[3];
    g_sleep.night.light_pct = d[4];
    g_sleep.night.deep_pct  = d[5];
    g_sleep.night.outbed_min = d[6];
    g_sleep.night.outbed_cnt = d[7];
    g_sleep.night.turn_cnt  = d[8];
    g_sleep.night.avg_breath = d[9];
    g_sleep.night.avg_heart = d[10];
    g_sleep.night.apnea_cnt = d[11];
  }
}


static void rx_sleep_abnormal(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.abnormal = d[0];
}
static void rx_sleep_rating(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.night.grade = d[0];   // 0无/1良/2般/3差
}
// 睡眠模式（0x0F / 查询0x8C）
static void rx_sleep_mode(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.cfg.mode = d[0]; // 0实时/1睡眠状态
}

// 异常挣扎开关（0x13 / 查询0x93）
static void rx_sleep_struggle_switch(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.cfg.struggle_enable = d[0];
}

// 无人计时开关（0x14 / 查询0x94）
static void rx_sleep_noperson_switch(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.cfg.noperson_enable = d[0];
}

// 无人计时时长（0x15 / 查询0x95）
static void rx_sleep_noperson_min(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.cfg.noperson_min = d[0];
}

// 睡眠截止时长（0x16 / 查询0x96）
static void rx_sleep_cutoff_min(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.cfg.cutoff_min = d[0];
}

// 挣扎判读灵敏度（0x1A / 查询0x9A）
static void rx_sleep_struggle_sens(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.cfg.struggle_sens = d[0]; // 0低/1中/2高
}

/* ---- 系统组 ---- */
static void rx_sys_heartbeat(uint8_t, uint8_t, const uint8_t* d, uint16_t) {
  (void)d;
  g_sys.heartbeat = 1; // 仅标记收到心跳
}
static void rx_sys_initdone(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sys.init_done = d[0];
}
static void rx_sys_posout(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sys.pos_outbound = d[0];
}

/* ---- 产品信息 ---- */
static void rx_prod_model(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  g_product.model_len = (n > sizeof(g_product.model)) ? sizeof(g_product.model) : n;
  if (g_product.model_len && d) memcpy(g_product.model, d, g_product.model_len);
}
static void rx_prod_id(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  g_product.product_id_len = (n > sizeof(g_product.product_id)) ? sizeof(g_product.product_id) : n;
  if (g_product.product_id_len && d) memcpy(g_product.product_id, d, g_product.product_id_len);
}
static void rx_prod_hw(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  g_product.hw_model_len = (n > sizeof(g_product.hw_model)) ? sizeof(g_product.hw_model) : n;
  if (g_product.hw_model_len && d) memcpy(g_product.hw_model, d, g_product.hw_model_len);
}
static void rx_prod_fw(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  g_product.fw_version_len = (n > sizeof(g_product.fw_version)) ? sizeof(g_product.fw_version) : n;
  if (g_product.fw_version_len && d) memcpy(g_product.fw_version, d, g_product.fw_version_len);
}

/* ---- 睡眠补充：状态（非开关） ---- */
static void rx_sleep_struggle_state(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.struggle_state = d[0];   // 0无/1正常/2异常
}
static void rx_sleep_noperson_state(uint8_t, uint8_t, const uint8_t* d, uint16_t n) {
  if (n >= 1) g_sleep.noperson_state = d[0];   // 0无/1正常/2异常
}

/* ================== 内置注册 ================== */
static void register_builtin() {
  // 0x80 BODY
  radar_register_handler(CTRL_BODY,  BODY_CMD_ENABLE,     rx_body_enable);
  radar_register_handler(CTRL_BODY,  BODY_CMD_EXIST,      rx_body_exist);
  radar_register_handler(CTRL_BODY,  BODY_CMD_MOTION,     rx_body_motion);
  radar_register_handler(CTRL_BODY,  BODY_CMD_BODY_PARAM, rx_body_param);
  radar_register_handler(CTRL_BODY,  BODY_CMD_DISTANCE,   rx_body_distance);
  radar_register_handler(CTRL_BODY,  BODY_CMD_DIRECTION,  rx_body_direction);

  // 0x81 BREATH
  radar_register_handler(CTRL_BREATH, BREATH_CMD_ENABLE,       rx_breath_enable);
  radar_register_handler(CTRL_BREATH, BREATH_CMD_INFO,         rx_breath_info);
  radar_register_handler(CTRL_BREATH, BREATH_CMD_VALUE,        rx_breath_value);
  radar_register_handler(CTRL_BREATH, BREATH_CMD_WAVE,         rx_breath_wave);
  radar_register_handler(CTRL_BREATH, BREATH_CMD_LOW_SLOW_SET, rx_breath_low_slow);

  // 0x85 HEART
  radar_register_handler(CTRL_HEART, HEART_CMD_ENABLE, rx_heart_enable);
  radar_register_handler(CTRL_HEART, HEART_CMD_VALUE,  rx_heart_value);
  radar_register_handler(CTRL_HEART, HEART_CMD_WAVE,   rx_heart_wave);

  // 0x84 SLEEP（基础子命令）
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_ENABLE,        rx_sleep_enable);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_BED_STATE,     rx_sleep_bed);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_STATE,         rx_sleep_state);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_AWAKE_MIN,     rx_sleep_awake);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_LIGHT_MIN,     rx_sleep_light);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_DEEP_MIN,      rx_sleep_deep);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_SCORE,         rx_sleep_score);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_COMPOSITE10M,  rx_sleep_comp10m);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_NIGHT_SUMMARY, rx_sleep_night_sum);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_ABNORMAL,      rx_sleep_abnormal);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_RATING,        rx_sleep_rating);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_STRUGGLE,      rx_sleep_struggle_state);
  radar_register_handler(CTRL_SLEEP, SLEEP_CMD_NOPERSON_TIM,  rx_sleep_noperson_state);

  // ==== 睡眠“显式查询 opcode”映射（0x8C~0x9A） ====
  radar_register_handler(CTRL_SLEEP, 0x8C, rx_sleep_mode);          // 模式
  radar_register_handler(CTRL_SLEEP, 0x8D, rx_sleep_comp10m);       // 10分钟综合
  radar_register_handler(CTRL_SLEEP, 0x8E, rx_sleep_abnormal);      // 异常
  radar_register_handler(CTRL_SLEEP, 0x8F, rx_sleep_night_sum);     // 整夜统计
  radar_register_handler(CTRL_SLEEP, 0x90, rx_sleep_rating);        // 睡眠质量评级（0/1/2/3）
  radar_register_handler(CTRL_SLEEP, 0x91, rx_sleep_struggle_state);// 挣扎状态
  radar_register_handler(CTRL_SLEEP, 0x92, rx_sleep_noperson_state);// 无人状态
  radar_register_handler(CTRL_SLEEP, 0x93, rx_sleep_struggle_switch);// 挣扎开关
  radar_register_handler(CTRL_SLEEP, 0x94, rx_sleep_noperson_switch);// 无人开关
  radar_register_handler(CTRL_SLEEP, 0x95, rx_sleep_noperson_min);  // 无人分钟
  radar_register_handler(CTRL_SLEEP, 0x96, rx_sleep_cutoff_min);    // 截止分钟
  radar_register_handler(CTRL_SLEEP, 0x9A, rx_sleep_struggle_sens); // 挣扎灵敏度

  // 系统（注册基础子命令，0x81/0x87 会由 find_handler 映射）
  // 系统（心跳）
  radar_register_handler(0x01, 0x01, rx_sys_heartbeat); // 上报
  radar_register_handler(0x01, 0x80, rx_sys_heartbeat); // 查询/应答  <<< 新增
  radar_register_handler(0x05, 0x01, rx_sys_initdone);
  radar_register_handler(0x07, 0x07, rx_sys_posout);


  // 产品信息（注册基础子命令，A1~A4 会由 find_handler 映射）
  radar_register_handler(CTRL_PRODUCT, PRODUCT_CMD_MODEL, rx_prod_model);
  radar_register_handler(CTRL_PRODUCT, PRODUCT_CMD_ID,    rx_prod_id);
  radar_register_handler(CTRL_PRODUCT, PRODUCT_CMD_HW,    rx_prod_hw);
  radar_register_handler(CTRL_PRODUCT, PRODUCT_CMD_FW,    rx_prod_fw);
}

/* ================== 初始化 ================== */
void radar_init(uint8_t rxPin, uint8_t txPin, uint32_t baud) {
  Serial1.begin(baud, SERIAL_8N1, rxPin, txPin); // RX=7, TX=6
  for (auto &sl : s_slots) {
    sl.used = false;
    sl.cb = nullptr;
  }
  s_default_cb = nullptr;
  register_builtin();
}

/* ================== 调试：未处理帧转储 ================== */
static void dump_hex(uint8_t ctrl, uint8_t cmd, const uint8_t* d, uint16_t n) {
#if RADAR_DEBUG
  Serial.printf("[RX] Unhandled ctrl=0x%02X cmd=0x%02X len=%u data:", ctrl, cmd, n);
  const uint16_t max_show = (n > 64) ? 64 : n;
  for (uint16_t i = 0; i < max_show; i++) Serial.printf(" %02X", d[i]);
  if (n > max_show) Serial.printf(" ...(+%u)", (unsigned)(n - max_show));
  Serial.printf("\n");
#endif
}

/* ================== 接收状态机（带丢包恢复） ================== */
void radar_process_rx() {
  static uint8_t  state = 0;
  static uint8_t  ctrl = 0, cmd = 0;
  static uint16_t len = 0, got = 0;
  static uint16_t sum = 0;
  static bool     drop_data = false;  // 当 len > buf 时进入丢弃数据模式
  static uint8_t  buf[512];

  while (Serial1.available()) {
    uint8_t b = Serial1.read();
    switch (state) {
      case 0: // HDR1
        if (b == RADAR_HDR1) {
          sum = b;
          state = 1;
        } else state = 0;
        break;

      case 1: // HDR2
        if (b == RADAR_HDR2) {
          sum += b;
          state = 2;
        } else { // 恢复：此字节可能是新的HDR1
          if (b == RADAR_HDR1) {
            sum = b;
            state = 1;
          } else state = 0;
        }
        break;

      case 2: // CTRL
        ctrl = b; sum += b; state = 3; break;

      case 3: // CMD
        cmd = b; sum += b; state = 4; break;

      case 4: // LEN HI
        len = (uint16_t)b << 8; sum += b; state = 5; break;

      case 5: // LEN LO
        len |= b; sum += b; got = 0;
        drop_data = (len > sizeof(buf));
        if (drop_data) {
#if RADAR_DEBUG
          Serial.printf("[RX] oversize len=%u > %u, drop data\n", len, (unsigned)sizeof(buf));
#endif
        }
        state = (len == 0) ? 7 : 6;
        break;

      case 6: // DATA
        if (!drop_data) {
          buf[got] = b;
        }
        got++;
        sum += b;
        if (got >= len) state = 7;
        break;

      case 7: { // SUM
          uint8_t sum_low = (uint8_t)(sum & 0xFF);
          if (b == sum_low) state = 8;
          else {
#if RADAR_DEBUG
            Serial.printf("[RX] sum error: got=0x%02X need=0x%02X (ctrl=0x%02X cmd=0x%02X len=%u)\n",
                          b, sum_low, ctrl, cmd, len);
#endif
            // 恢复：当前字节可能就是新的HDR1
            if (b == RADAR_HDR1) {
              sum = b;
              state = 1;
            } else state = 0;
          }
        } break;

      case 8: // END1
        if (b == RADAR_END1) state = 9;
        else {
          // 恢复：当前字节可能是新的HDR1
          if (b == RADAR_HDR1) {
            sum = b;
            state = 1;
          } else {
            state = 0;
          }
        }
        break;

      case 9: // END2
        if (b == RADAR_END2) {
          // 分发：仅当不是 drop_data 时才传数据
          if (!drop_data) {
            if (RadarRecvFunc cb = find_handler(ctrl, cmd)) {
              cb(ctrl, cmd, buf, len);
            } else if (s_default_cb) {
              s_default_cb(ctrl, cmd, buf, len);
            } else {
              dump_hex(ctrl, cmd, buf, len);
            }
          }
          // 成功帧，无论如何 reset
          drop_data = false;
          state = 0;
        } else {
          // 错误帧尾：尝试快速 resync
          if (b == RADAR_HDR1) {
            sum = b;
            state = 1;   // 直接当新帧开始
          } else {
            state = 0;   // 回到初始状态
          }
          drop_data = false; // 清除超长标志
        }
        break;
    }
  }
}

/* ================== OTA 示例 ================== */
bool radar_ota_begin(uint32_t total_size) {
  uint8_t d[4] = {
    uint8_t(total_size >> 24), uint8_t(total_size >> 16),
    uint8_t(total_size >> 8),  uint8_t(total_size)
  };
  return radar_send_frame(CTRL_OTA, 0x01, d, 4);
}

bool radar_ota_send_chunk(uint32_t frame_size, uint32_t offset, const uint8_t* chunk, uint16_t len) {
  uint8_t hdr[8] = {
    uint8_t(frame_size >> 24), uint8_t(frame_size >> 16), uint8_t(frame_size >> 8), uint8_t(frame_size),
    uint8_t(offset >> 24),     uint8_t(offset >> 16),     uint8_t(offset >> 8),     uint8_t(offset)
  };
  static uint8_t tmp[512];
  if (8 + len > sizeof(tmp)) return false;
  memcpy(tmp, hdr, 8);
  if (len && chunk) memcpy(tmp + 8, chunk, len);
  return radar_send_frame(CTRL_OTA, 0x02, tmp, 8 + len);
}

bool radar_ota_end(uint8_t status) {
  return radar_send_frame(CTRL_OTA, 0x03, &status, 1);
}
