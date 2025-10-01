#ifndef __RADAR_PROTOCOL_H__
#define __RADAR_PROTOCOL_H__

#include <Arduino.h>
#include <stdint.h>

/* ========= 帧常量 ========= */
#define RADAR_HDR1  0x53
#define RADAR_HDR2  0x59
#define RADAR_END1  0x54
#define RADAR_END2  0x43

/* ========= 控制字（ctrl） ========= */
enum : uint8_t {
  CTRL_PRODUCT   = 0x02, // 产品信息
  CTRL_OTA       = 0x03, // OTA
  CTRL_BODY      = 0x80, // 人体存在/运动/方位
  CTRL_BREATH    = 0x81, // 呼吸
  CTRL_SLEEP     = 0x84, // 睡眠
  CTRL_HEART     = 0x85, // 心率
};

/* ========= 子命令（cmd, 0x00-0x7F 基本；0x80-0xFF 常作查询/应答） ========= */
/* 0x80 人体存在 */
enum : uint8_t {
  BODY_CMD_ENABLE         = 0x00,
  BODY_CMD_EXIST          = 0x01, // 00无人 / 01有人
  BODY_CMD_MOTION         = 0x02, // 01静止 / 02活跃
  BODY_CMD_BODY_PARAM     = 0x03, // 0-100
  BODY_CMD_DISTANCE       = 0x04, // 2B cm
  BODY_CMD_DIRECTION      = 0x05, // 6B x,y,z（符号+14位幅值）
};

/* 0x81 呼吸 */
enum : uint8_t {
  BREATH_CMD_ENABLE       = 0x00,
  BREATH_CMD_INFO         = 0x01, // 01正常 / 02过高 / 03过低 / 04无
  BREATH_CMD_VALUE        = 0x02, // 0-35
  BREATH_CMD_WAVE         = 0x05, // 5B，0-255，中轴128
  BREATH_CMD_LOW_SLOW_SET = 0x0B, // 10~20
};

/* 0x85 心率 */
enum : uint8_t {
  HEART_CMD_ENABLE        = 0x00,
  HEART_CMD_VALUE         = 0x02, // 60-120
  HEART_CMD_WAVE          = 0x05, // 5B
};

/* 0x84 睡眠（核心项，按需可继续扩展） */
enum : uint8_t {
  SLEEP_CMD_ENABLE          = 0x00,
  SLEEP_CMD_BED_STATE       = 0x01, // 0离床/1入床/2无
  SLEEP_CMD_STATE           = 0x02, // 0深/1浅/2清醒/3无
  SLEEP_CMD_AWAKE_MIN       = 0x03, // 2B
  SLEEP_CMD_LIGHT_MIN       = 0x04, // 2B
  SLEEP_CMD_DEEP_MIN        = 0x05, // 2B
  SLEEP_CMD_SCORE           = 0x06, // 1B 0-100
  SLEEP_CMD_COMPOSITE10M    = 0x0C, // 8B
  SLEEP_CMD_NIGHT_SUMMARY   = 0x0D, // 12B
  SLEEP_CMD_ABNORMAL        = 0x0E, // 1B
  SLEEP_CMD_MODE_SET        = 0x0F, // 0实时/1睡眠状态
  SLEEP_CMD_RATING          = 0x10, // 0无/1良/2般/3差
  SLEEP_CMD_STRUGGLE        = 0x11, // 0无/1常/2异常（状态）
  SLEEP_CMD_NOPERSON_TIM    = 0x12, // 0无/1常/2异常（状态）
  SLEEP_CMD_STRUGGLE_SWITCH = 0x13, // 0关/1开
  SLEEP_CMD_NOPERS_SWITCH   = 0x14, // 0关/1开
  SLEEP_CMD_NOPERS_MIN      = 0x15, // 5~120
  SLEEP_CMD_CUTOFF_MIN      = 0x16, // 30~180 步长10
  SLEEP_CMD_STRUGGLE_SENS   = 0x1A, // 0低/1中/2高
};

/* 产品信息子命令（注意：查询应答为 0xA1~0xA4） */
enum : uint8_t {
  PRODUCT_CMD_MODEL   = 0x01,
  PRODUCT_CMD_ID      = 0x02,
  PRODUCT_CMD_HW      = 0x03,
  PRODUCT_CMD_FW      = 0x04,
};

/* ===== 查询命令工具 ===== */
static inline uint8_t RADAR_QUERY(uint8_t subcmd) {
  return uint8_t(subcmd | 0x80);
}
static inline uint8_t RADAR_SUBCMD(uint8_t cmd)   {
  return uint8_t(cmd & 0x7F);
}

/* ========= 数据结构 ========= */
typedef struct {
  uint8_t  bed_state;         // 0离床/1入床/2无
  uint8_t  state;             // 0深/1浅/2清醒/3无
  uint16_t awake_min;         // 清醒时长 min
  uint16_t light_min;         // 浅睡时长 min
  uint16_t deep_min;          // 深睡时长 min
  uint8_t  score;             // 评分 0-100

  // 10分钟综合
  struct {
    uint8_t exist;            // 1有人/0无人
    uint8_t state;            // 同上
    uint8_t avg_breath;       // 平均呼吸 0-25
    uint8_t avg_heart;        // 平均心率 0-100
    uint8_t turn_cnt;         // 翻身次数
    uint8_t big_move_pct;     // 大动作占比
    uint8_t small_move_pct;   // 小动作占比
    uint8_t apnea_cnt;        // 呼吸暂停次数
  } composite10m;

  // 整夜总结
  struct {
    uint8_t  score;         // 整夜评分 0-100（来自 0x0D/0x8F 的首字节）
    uint16_t total_min;     // 总时长 min
    uint8_t  awake_pct;     // 清醒占比 %
    uint8_t  light_pct;     // 浅睡占比 %
    uint8_t  deep_pct;      // 深睡占比 %
    uint8_t  outbed_min;    // 离床时长 (0~255)
    uint8_t  outbed_cnt;    // 离床次数
    uint8_t  turn_cnt;      // 翻身次数
    uint8_t  avg_breath;    // 平均呼吸 (0~25)
    uint8_t  avg_heart;     // 平均心跳 (0~100)
    uint8_t  apnea_cnt;     // 呼吸暂停次数 (0~10)
    uint8_t  grade;         // 质量评级 0无/1良/2般/3差（来自 0x10/0x90）
  } night;

  uint8_t abnormal;           // 异常标志（0x0E/0x8E）

  // —— 新增：纯“状态”结果（与“开关”分离）——
  uint8_t struggle_state;     // 0无/1正常/2异常   （0x11/0x91）
  uint8_t noperson_state;     // 0无/1正常/2异常   （0x12/0x92）

  // —— 配置/开关类 ——（仅对应 0x13/0x93、0x14/0x94 等）
  struct {
    uint8_t mode;             // 0实时/1睡眠状态               （0x0F/0x8C）
    uint8_t struggle_enable;  // 0关/1开  挣扎开关             （0x13/0x93）
    uint8_t noperson_enable;  // 0关/1开  无人计时开关         （0x14/0x94）
    uint8_t noperson_min;     // 无人报警时间 5~120 min        （0x15/0x95）
    uint8_t cutoff_min;       // 截止时间 30~180 步长10        （0x16/0x96）
    uint8_t struggle_sens;    // 挣扎灵敏度 0低/1中/2高        （0x1A/0x9A）
  } cfg;

  // —— 新增：开关查询结果承载 ——（0x00/0x80）
  uint8_t enabled;            // 0关/1开
} sleep_t;

/* 人体存在 (0x80) */
typedef struct {
  uint8_t  exist;        // 是否有人 0无/1有
  uint8_t  motion;       // 运动状态 0无/1静止/2活跃
  uint8_t  body_param;   // 活动强度参数 0-100
  uint16_t distance_cm;  // 距离 cm
  int16_t  pos_xyz[3];   // 方位坐标 (解码符号+14位幅值)
  uint8_t  enabled;      // 开关查询结果 0关/1开
} body_exist_t;

/* 呼吸 (0x81) */
typedef struct {
  uint8_t info;            // 01正常/02过高/03过低/04无
  uint8_t value;           // 呼吸率 0-35
  uint8_t wave[5];         // 波形，5点
  uint8_t low_slow_thresh; // 低缓呼吸阈值
  uint8_t enabled;         // 开关查询结果 0关/1开
} breath_t;

/* 心率 (0x85) */
typedef struct {
  uint8_t value;           // 心率 60-120
  uint8_t wave[5];         // 波形
  uint8_t enabled;         // 开关查询结果 0关/1开
} heart_t;

/* 系统信息 */
typedef struct {
  uint8_t heartbeat;      // 最近心跳 (0x01)
  uint8_t init_done;      // 初始化完成 (0x05)
  uint8_t pos_outbound;   // 位置越界状态 (0x07, 0范围外 / 1范围内)
} system_t;

/* 产品信息 */
typedef struct {
  uint8_t model[32];      uint8_t model_len;
  uint8_t product_id[32]; uint8_t product_id_len;
  uint8_t hw_model[32];   uint8_t hw_model_len;
  uint8_t fw_version[32]; uint8_t fw_version_len;
} product_t;

/* ========= 回调 ========= */
typedef void (*RadarRecvFunc)(uint8_t ctrl, uint8_t cmd, const uint8_t *data, uint16_t len);

/* ========= 全局对象 ========= */
extern body_exist_t g_body;
extern breath_t     g_breath;
extern heart_t      g_heart;
extern sleep_t      g_sleep;
extern system_t     g_sys;
extern product_t    g_product;

/* ========= API ========= */
void radar_init(uint8_t rxPin = 7, uint8_t txPin = 6, uint32_t baud = 115200);
void radar_process_rx();

bool radar_send_frame(uint8_t ctrl, uint8_t cmd, const uint8_t* data, uint16_t len);
bool radar_send_query(uint8_t ctrl, uint8_t subcmd);
bool radar_send_set_u8(uint8_t ctrl, uint8_t subcmd, uint8_t value);

bool radar_register_handler(uint8_t ctrl, uint8_t cmd, RadarRecvFunc cb);
bool radar_unregister_handler(uint8_t ctrl, uint8_t cmd);

/* 默认回调：未注册命令时调用；若未设置则打印十六进制原始帧 */
bool radar_set_default_handler(RadarRecvFunc cb);

/* ======= 常用封装 ======= */
inline bool radar_query_exist()         {
  return radar_send_query(CTRL_BODY,  BODY_CMD_EXIST);
}
inline bool radar_query_motion()        {
  return radar_send_query(CTRL_BODY,  BODY_CMD_MOTION);
}
inline bool radar_query_body_param()    {
  return radar_send_query(CTRL_BODY,  BODY_CMD_BODY_PARAM);
}
inline bool radar_query_distance()      {
  return radar_send_query(CTRL_BODY,  BODY_CMD_DISTANCE);
}
inline bool radar_query_direction()     {
  return radar_send_query(CTRL_BODY,  BODY_CMD_DIRECTION);
}
inline bool radar_set_body_enable(bool en) {
  return radar_send_set_u8(CTRL_BODY, BODY_CMD_ENABLE, en ? 1 : 0);
}

inline bool radar_set_breath_enable(bool en) {
  return radar_send_set_u8(CTRL_BREATH, BREATH_CMD_ENABLE, en ? 1 : 0);
}
inline bool radar_query_breath_info()   {
  return radar_send_query(CTRL_BREATH, BREATH_CMD_INFO);
}
inline bool radar_query_breath_value()  {
  return radar_send_query(CTRL_BREATH, BREATH_CMD_VALUE);
}
inline bool radar_query_breath_wave()   {
  return radar_send_query(CTRL_BREATH, BREATH_CMD_WAVE);
}
inline bool radar_set_breath_low_slow(uint8_t v) {
  return radar_send_set_u8(CTRL_BREATH, BREATH_CMD_LOW_SLOW_SET, v);
}

inline bool radar_set_heart_enable(bool en) {
  return radar_send_set_u8(CTRL_HEART, HEART_CMD_ENABLE, en ? 1 : 0);
}
inline bool radar_query_heart_value()  {
  return radar_send_query(CTRL_HEART, HEART_CMD_VALUE);
}
inline bool radar_query_heart_wave()   {
  return radar_send_query(CTRL_HEART, HEART_CMD_WAVE);
}

// ---- 睡眠设置 ----
inline bool radar_set_sleep_enable(bool en) {
  return radar_send_set_u8(CTRL_SLEEP, SLEEP_CMD_ENABLE, en ? 1 : 0);
}
inline bool radar_set_sleep_mode(uint8_t mode/*0实时/1睡眠*/) {
  return radar_send_set_u8(CTRL_SLEEP, SLEEP_CMD_MODE_SET, mode ? 1 : 0);
}
inline bool radar_set_sleep_struggle_enable(bool en) {
  return radar_send_set_u8(CTRL_SLEEP, SLEEP_CMD_STRUGGLE_SWITCH, en ? 1 : 0);
}
inline bool radar_set_sleep_noperson_enable(bool en) {
  return radar_send_set_u8(CTRL_SLEEP, SLEEP_CMD_NOPERS_SWITCH,   en ? 1 : 0);
}
inline bool radar_set_sleep_noperson_minutes(uint8_t m) {
  return radar_send_set_u8(CTRL_SLEEP, SLEEP_CMD_NOPERS_MIN,  m);  // 5~120
}
inline bool radar_set_sleep_cutoff_minutes(uint8_t m)   {
  return radar_send_set_u8(CTRL_SLEEP, SLEEP_CMD_CUTOFF_MIN,  m);  // 30~180, step 10
}
inline bool radar_set_sleep_struggle_sens(uint8_t lvl)   {
  return radar_send_set_u8(CTRL_SLEEP, SLEEP_CMD_STRUGGLE_SENS, lvl);  // 0/1/2
}

// ---- 睡眠查询（显式 opcode）----
inline bool radar_query_mode()              {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x8C, &d, 1);
}
inline bool radar_query_composite10m()      {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x8D, &d, 1);
}
inline bool radar_query_night_summary()     {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x8F, &d, 1);
}
inline bool radar_query_abnormal()          {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x8E, &d, 1);
}

inline bool radar_query_rating()            {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x90, &d, 1);
}
inline bool radar_query_struggle_state()    {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x91, &d, 1);
}
inline bool radar_query_noperson_state()    {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x92, &d, 1);
}

inline bool radar_query_struggle_switch()   {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x93, &d, 1);
}
inline bool radar_query_noperson_switch()   {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x94, &d, 1);
}
inline bool radar_query_noperson_minutes()  {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x95, &d, 1);
}
inline bool radar_query_cutoff_minutes()    {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x96, &d, 1);
}
inline bool radar_query_struggle_sens()     {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_SLEEP, 0x9A, &d, 1);
}

// ---- 系统 ----
inline bool radar_query_heartbeat() {
  const uint8_t d = 0x0F;
  return radar_send_frame(0x01, 0x80, &d, 1);
}
inline bool radar_query_initdone()    {
  const uint8_t d = 0x0F;
  return radar_send_frame(0x05, 0x81, &d, 1);
}
inline bool radar_query_posout()      {
  const uint8_t d = 0x0F;
  return radar_send_frame(0x07, 0x87, &d, 1);
}

// ---- 呼吸补充 ----
inline bool radar_query_breath_lowslow() {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_BREATH, 0x8B, &d, 1);
}

// ---- 心率补充 ----
inline bool radar_query_heart_enable() {
  return radar_send_query(CTRL_HEART, HEART_CMD_ENABLE);
}

// ---- 睡眠缺口查询（按子命令查询）----
inline bool radar_query_sleep_bed_state() {
  return radar_send_query(CTRL_SLEEP, SLEEP_CMD_BED_STATE);
}
inline bool radar_query_sleep_state()     {
  return radar_send_query(CTRL_SLEEP, SLEEP_CMD_STATE);
}
inline bool radar_query_awake_min()       {
  return radar_send_query(CTRL_SLEEP, SLEEP_CMD_AWAKE_MIN);
}
inline bool radar_query_light_min()       {
  return radar_send_query(CTRL_SLEEP, SLEEP_CMD_LIGHT_MIN);
}
inline bool radar_query_deep_min()        {
  return radar_send_query(CTRL_SLEEP, SLEEP_CMD_DEEP_MIN);
}
inline bool radar_query_sleep_score()     {
  return radar_send_query(CTRL_SLEEP, SLEEP_CMD_SCORE);
}

// ---- 产品信息查询（A1~A4）----
inline bool radar_query_product_model()   {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_PRODUCT, 0xA1, &d, 1);
}
inline bool radar_query_product_id()      {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_PRODUCT, 0xA2, &d, 1);
}
inline bool radar_query_hw_model()        {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_PRODUCT, 0xA3, &d, 1);
}
inline bool radar_query_fw_version()      {
  const uint8_t d = 0x0F;
  return radar_send_frame(CTRL_PRODUCT, 0xA4, &d, 1);
}

// ---- 其他便捷查询 ----
inline bool radar_query_body_enable()   {
  return radar_send_query(CTRL_BODY,   BODY_CMD_ENABLE);
}
inline bool radar_query_breath_enable() {
  return radar_send_query(CTRL_BREATH, BREATH_CMD_ENABLE);
}
inline bool radar_query_sleep_enable()  {
  return radar_send_query(CTRL_SLEEP,  SLEEP_CMD_ENABLE);
}
inline bool radar_system_reset()        {
  const uint8_t d = 0x0F;
  return radar_send_frame(0x01, 0x02, &d, 1);
}

/* ======= OTA ======= */
bool radar_ota_begin(uint32_t total_size);
bool radar_ota_send_chunk(uint32_t frame_size, uint32_t offset, const uint8_t* chunk, uint16_t len);
bool radar_ota_end(uint8_t status);

#endif // __RADAR_PROTOCOL_H__
