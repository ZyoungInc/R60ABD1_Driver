#include "datahandle.h"
#include "oled.h"
#include "esp_task_wdt.h"
#define Pin_key4 9
#define Pin_key3 10
#define Pin_key2 3
#define Pin_key1 8

unsigned long key_time = 0;

void setup()
{
  hardware_init();
  Serial.begin(115200);
  datahandle_init();
  display_init();
  //delay(500);
}

int8_t j = 1;
void loop()
{
  // CheckDataChanged();
  if ((digitalRead(Pin_key1) == LOW) && (((millis() - key_time) > 300)))
  {
    key_time = millis();
    j++;
    j = (j >= 6) ? 0 : j;
    if (j == 0)
    {
      disp_DataMenu();
    }
  }

  if ((digitalRead(Pin_key4) == LOW) && (((millis() - key_time) > 300)))
  {
    key_time = millis();
    j--;
    j = (j < 0) ? 5 : j;
    if (j == 0)
    {
      disp_DataMenu();
    }
  }

  if (j == 0)
  {
    SendRawData();
  }
  else
  {
    ProcessRx();
    display_update(j);
  }
  esp_task_wdt_reset();;
}



//硬件初始化
void hardware_init()
{
  pinMode(Pin_key1, INPUT_PULLUP);
  pinMode(Pin_key4, INPUT_PULLUP);

  // 定义看门狗配置
  esp_task_wdt_config_t wdt_config = {
    .timeout_ms = 5000,      // 超时时间 5 秒
    .idle_core_mask = (1 << portNUM_PROCESSORS) - 1, // 监控所有 CPU core
    .trigger_panic = true    // 超时时触发 panic (复位)
  };
  // 初始化任务看门狗
  esp_task_wdt_init(&wdt_config);
  esp_task_wdt_add(NULL);       // 当前任务
}
