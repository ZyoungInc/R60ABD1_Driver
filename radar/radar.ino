/* ========= radar_test.ino =========
 * 串口:  PC日志 -> Serial(115200)
 *        设备通信 -> Serial1(6:TX, 7:RX, 115200)
 * 覆盖功能：系统/产品/人体存在/呼吸/心率/睡眠/OTA（演示）
 */

#include "radar_protocol.h"

// ---------- 打印辅助 ----------
static void printHex2(uint8_t v){ if(v<16) Serial.print('0'); Serial.print(v, HEX); }
static void printHexBuf(const uint8_t* p, uint16_t n){
  for(uint16_t i=0;i<n;i++){ printHex2(p[i]); Serial.print(i+1<n?' ':' '); }
}
static void printAsciiLine(const uint8_t* p, uint16_t n){
  for(uint16_t i=0;i<n;i++){ char c = (p[i]>=32 && p[i]<=126)?(char)p[i]:'.'; Serial.print(c); }
}

// 未注册回调时的兜底打印
static void rx_default(uint8_t ctrl, uint8_t cmd, const uint8_t* d, uint16_t n){
  Serial.print(F("[RX default] ctrl=0x")); printHex2(ctrl);
  Serial.print(F(" cmd=0x")); printHex2(cmd);
  Serial.print(F(" len=")); Serial.print(n);
  Serial.print(F(" data: ")); printHexBuf(d,n);
  Serial.println();
}

// ---------- show 函数：把解析好的全局对象打印出来 ----------
static void show_system(){
  Serial.print(F("[SYSTEM] heartbeat=")); Serial.print(g_sys.heartbeat);
  Serial.print(F(" init_done=")); Serial.print(g_sys.init_done);
  Serial.print(F(" pos_outbound=")); Serial.println(g_sys.pos_outbound);
}

static void show_product(){
  Serial.print(F("[PRODUCT] model_len=")); Serial.print(g_product.model_len);
  Serial.print(F(" model_ascii=")); printAsciiLine(g_product.model, g_product.model_len);
  Serial.print(F("  hex=")); printHexBuf(g_product.model, g_product.model_len); Serial.println();

  Serial.print(F("          id_len=")); Serial.print(g_product.product_id_len);
  Serial.print(F(" id_ascii=")); printAsciiLine(g_product.product_id, g_product.product_id_len);
  Serial.print(F("  hex=")); printHexBuf(g_product.product_id, g_product.product_id_len); Serial.println();

  Serial.print(F("          hw_len=")); Serial.print(g_product.hw_model_len);
  Serial.print(F(" hw_ascii=")); printAsciiLine(g_product.hw_model, g_product.hw_model_len);
  Serial.print(F("  hex=")); printHexBuf(g_product.hw_model, g_product.hw_model_len); Serial.println();

  Serial.print(F("          fw_len=")); Serial.print(g_product.fw_version_len);
  Serial.print(F(" fw_ascii=")); printAsciiLine(g_product.fw_version, g_product.fw_version_len);
  Serial.print(F("  hex=")); printHexBuf(g_product.fw_version, g_product.fw_version_len); Serial.println();
}

static void show_body(){
  Serial.print(F("[BODY] en=")); Serial.print(g_body.enabled);
  Serial.print(F(" exist=")); Serial.print(g_body.exist);
  Serial.print(F(" motion=")); Serial.print(g_body.motion);
  Serial.print(F(" param=")); Serial.print(g_body.body_param);
  Serial.print(F(" dist_cm=")); Serial.print(g_body.distance_cm);
  Serial.print(F(" pos=(")); Serial.print(g_body.pos_xyz[0]); Serial.print(',');
  Serial.print(g_body.pos_xyz[1]); Serial.print(',');
  Serial.print(g_body.pos_xyz[2]); Serial.println(')');
}

static void show_breath(){
  Serial.print(F("[BREATH] en=")); Serial.print(g_breath.enabled);
  Serial.print(F(" info="));  Serial.print(g_breath.info);
  Serial.print(F(" value=")); Serial.print(g_breath.value);
  Serial.print(F(" low_slow=")); Serial.print(g_breath.low_slow_thresh);
  Serial.print(F(" wave=")); 
  for(int i=0;i<5;i++){ Serial.print(g_breath.wave[i]); Serial.print(i<4?',':' '); }
  Serial.println();
}

static void show_heart(){
  Serial.print(F("[HEART] en=")); Serial.print(g_heart.enabled);
  Serial.print(F(" value=")); Serial.print(g_heart.value);
  Serial.print(F(" wave=")); 
  for(int i=0;i<5;i++){ Serial.print(g_heart.wave[i]); Serial.print(i<4?',':' '); }
  Serial.println();
}

static void show_sleep_basic(){
  Serial.print(F("[SLEEP] en=")); Serial.print(g_sleep.enabled);
  Serial.print(F(" mode=")); Serial.print(g_sleep.cfg.mode);
  Serial.print(F(" bed="));  Serial.print(g_sleep.bed_state);
  Serial.print(F(" state="));Serial.print(g_sleep.state);
  Serial.print(F(" awake_min=")); Serial.print(g_sleep.awake_min);
  Serial.print(F(" light_min=")); Serial.print(g_sleep.light_min);
  Serial.print(F(" deep_min="));  Serial.print(g_sleep.deep_min);
  Serial.print(F(" score="));     Serial.println(g_sleep.score);
}

static void show_sleep_cfg(){
  Serial.print(F("[SLEEP.cfg] struggle_en=")); Serial.print(g_sleep.cfg.struggle_enable);
  Serial.print(F(" noperson_en=")); Serial.print(g_sleep.cfg.noperson_enable);
  Serial.print(F(" noperson_min=")); Serial.print(g_sleep.cfg.noperson_min);
  Serial.print(F(" cutoff_min="));   Serial.print(g_sleep.cfg.cutoff_min);
  Serial.print(F(" sens="));         Serial.println(g_sleep.cfg.struggle_sens);
}

static void show_sleep_10m(){
  Serial.print(F("[SLEEP.10m] exist=")); Serial.print(g_sleep.composite10m.exist);
  Serial.print(F(" state=")); Serial.print(g_sleep.composite10m.state);
  Serial.print(F(" avg_breath=")); Serial.print(g_sleep.composite10m.avg_breath);
  Serial.print(F(" avg_heart="));  Serial.print(g_sleep.composite10m.avg_heart);
  Serial.print(F(" turns="));      Serial.print(g_sleep.composite10m.turn_cnt);
  Serial.print(F(" big%="));       Serial.print(g_sleep.composite10m.big_move_pct);
  Serial.print(F(" small%="));     Serial.print(g_sleep.composite10m.small_move_pct);
  Serial.print(F(" apnea="));      Serial.println(g_sleep.composite10m.apnea_cnt);
}

static void show_sleep_night(){
  Serial.print(F("[SLEEP.night] total=")); Serial.print(g_sleep.night.total_min);
  Serial.print(F(" awake%=")); Serial.print(g_sleep.night.awake_pct);
  Serial.print(F(" light%=")); Serial.print(g_sleep.night.light_pct);
  Serial.print(F(" deep%="));  Serial.print(g_sleep.night.deep_pct);
  Serial.print(F(" outbed_min=")); Serial.print(g_sleep.night.outbed_min);
  Serial.print(F(" outbed_cnt=")); Serial.print(g_sleep.night.outbed_cnt);
  Serial.print(F(" turns="));      Serial.print(g_sleep.night.turn_cnt);
  Serial.print(F(" avg_breath=")); Serial.print(g_sleep.night.avg_breath);
  Serial.print(F(" avg_heart="));  Serial.print(g_sleep.night.avg_heart);
  Serial.print(F(" apnea="));      Serial.print(g_sleep.night.apnea_cnt);
  Serial.print(F(" grade="));     Serial.println(g_sleep.night.grade);
}

static void show_sleep_status_extra(){
  Serial.print(F("[SLEEP.extra] abnormal=")); Serial.print(g_sleep.abnormal);
  Serial.print(F(" struggle_state=")); Serial.print(g_sleep.struggle_state);
  Serial.print(F(" noperson_state=")); Serial.print(g_sleep.noperson_state);
  Serial.println();
}

// ---------- 测试步骤 ----------
typedef void (*Fn)();
struct Step {
  const char* name;
  Fn send;
  Fn show;
  uint32_t wait_ms;
};

// —— 每个阶段的发送动作 ——
// 系统
static void send_system(){
  radar_query_heartbeat();
  radar_query_initdone();
  radar_query_posout();
}
// 产品
static void send_product(){
  radar_query_product_model();
  radar_query_product_id();
  radar_query_hw_model();
  radar_query_fw_version();
}
// 人体存在
static void send_body(){
  radar_set_body_enable(true);
  radar_query_body_enable();
  radar_query_exist();
  radar_query_motion();
  radar_query_body_param();
  radar_query_distance();
  radar_query_direction();
}
// 呼吸
static void send_breath(){
  radar_set_breath_enable(true);
  radar_query_breath_enable();
  radar_set_breath_low_slow(12);
  radar_query_breath_lowslow();
  radar_query_breath_info();
  radar_query_breath_value();
  radar_query_breath_wave();
}
// 心率
static void send_heart(){
  radar_set_heart_enable(true);
  radar_query_heart_enable();
  radar_query_heart_value();
  radar_query_heart_wave();
}
// 睡眠（配置+查询）
static void send_sleep(){
  radar_set_sleep_enable(true);
  radar_set_sleep_mode(0);             // 实时
  radar_set_sleep_struggle_enable(true);
  radar_set_sleep_noperson_enable(true);
  radar_set_sleep_noperson_minutes(10);
  radar_set_sleep_cutoff_minutes(40);
  radar_set_sleep_struggle_sens(1);    // 中

  // 查询配置与状态
  radar_query_sleep_enable();
  radar_query_mode();
  radar_query_struggle_switch();
  radar_query_noperson_switch();
  radar_query_noperson_minutes();
  radar_query_cutoff_minutes();
  radar_query_struggle_sens();

  // 查询纯状态类
  radar_query_struggle_state();
  radar_query_noperson_state();
  radar_query_abnormal();
  radar_query_rating();

  // 查询10分钟综合、整夜统计
  radar_query_composite10m();
  radar_query_night_summary();

  // 查询缺口项
  radar_query_sleep_bed_state();
  radar_query_sleep_state();
  radar_query_awake_min();
  radar_query_light_min();
  radar_query_deep_min();
  radar_query_sleep_score();
}

// OTA（演示）
static void send_ota_demo(){
  const uint32_t total = 16;
  radar_ota_begin(total);
  uint8_t data[16];
  for (uint8_t i=0;i<16;i++) data[i] = i;
  // 假设设备先前通过 0x03 01 回应了每帧长度（此处直接演示发送一帧）
  radar_ota_send_chunk(total, 0, data, sizeof(data));
  radar_ota_end(0x01); // 01: 成功（以表为准）
}

// —— 阶段数组 ——（每步等待时间可视设备速度微调）
static Step kSteps[] = {
  {"system",  send_system,    show_system,        200},
  {"product", send_product,   show_product,       300},
  {"body",    send_body,      show_body,          250},
  {"breath",  send_breath,    show_breath,        300},
  {"heart",   send_heart,     show_heart,         250},
  {"sleepA",  send_sleep,     show_sleep_basic,   400},
  {"sleepB",  nullptr,        show_sleep_cfg,     10},
  {"sleepC",  nullptr,        show_sleep_10m,     10},
  {"sleepD",  nullptr,        show_sleep_night,   10},
  {"sleepE",  nullptr,        show_sleep_status_extra, 10},
  {"otaDemo", send_ota_demo,  nullptr,            150},
};

static const size_t kNumSteps = sizeof(kSteps)/sizeof(kSteps[0]);

// ---------- 运行时状态 ----------
static size_t   g_step = 0;
static uint32_t g_stepStart = 0;

// ---------- Arduino ----------
void setup() {
  Serial.begin(115200);
  while(!Serial){/* 等待串口准备好（某些板子需要） */}
  Serial.println(F("\n=== Radar Protocol All-Feature Test ==="));

  radar_init(D7, D6, 115200);          // Serial1: RX=7, TX=6
  radar_set_default_handler(rx_default);

  g_step = 0;
  g_stepStart = millis();
}

void loop() {
  // 处理接收
  radar_process_rx();

  // 阶段调度
  if (g_step < kNumSteps) {
    Step &st = kSteps[g_step];
    // 首次进入此阶段：发命令
    if (millis() - g_stepStart < 5) { // 进入阶段的第一个循环
      Serial.print(F("\n-- step ")); Serial.print(g_step); Serial.print(F(": "));
      Serial.print(st.name); Serial.println(F(" --"));
      if (st.send) st.send();
    }

    // 等待应答时间到 -> 打印结果 & 下一步
    if (millis() - g_stepStart >= st.wait_ms) {
      if (st.show) st.show();
      g_step++;
      g_stepStart = millis();
    }
  } else {
    // 全部跑完后，每 3s 复巡一次关键查询作为“心跳演示”
    static uint32_t t = 0;
    if (millis() - t > 3000) {
      t = millis();
      radar_query_exist();
      radar_query_breath_value();
      radar_query_heart_value();
      radar_query_sleep_state();
      Serial.println(F("[loop] poll exist/breath/heart/sleep_state..."));
    }
  }
}
