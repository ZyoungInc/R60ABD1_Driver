#include <Arduino.h>
#include "datahandle.h"

body_exist_detect_t body_exist_detect = {};
breath_detect_t breath_detect = {};
sleep_detect_t sleep_detect = {};
heart_detect_t heart_detect = {};
data_change_t data_change = {};

/* 定义数据包接收状态的变量,并初始化为空闲状态 */
rx_datagram_state_t rx_datagram_state = IDLE;
control_mode_t control_mode = MODE_IDLE;

uint8_t rx_flag = 0;

void datahandle_init(void)
{
  Serial1.begin(115200, SERIAL_8N1, D7, D6);
}

/* 协议数据处理函数 */
void ProcessRx()
{
  // ⭐ 初始化所有局部变量，避免未定义行为
  uint8_t receivedbyte = 0;
  uint8_t rx_sum = 0;
  uint8_t command = 0;

  //数据存储数组
  uint8_t rx_data[10] = {0};

  uint16_t rx_dategram_len = 0;
  while (Serial1.available()) //缓存中有数据
  {
    switch (rx_datagram_state)
    {
      case IDLE: //在空闲时,判断是否读取帧头1
        {
          receivedbyte = Serial1.read();
          if (HEADER1 == receivedbyte)
          {
            rx_sum = 0;
            rx_datagram_state = SEEN_HEADER1;
            rx_sum += receivedbyte;
          }
        }
        break;

      case SEEN_HEADER1: //读取第一帧之后,判断是否读取帧头2
        {
          receivedbyte = Serial1.read();
          rx_datagram_state = (HEADER2 != receivedbyte) ? IDLE : SEEN_HEADER2;
          rx_sum += receivedbyte;
        }
        break;

      case SEEN_HEADER2: //读取第二帧后,根据控制字判断数据模式
        {
          uint8_t ctrl_mode = Serial1.read(); //控制字
          rx_sum += ctrl_mode;

          if (ctrl_mode == CMD_TICK)
            control_mode = MODE_SEND_TICK;
          else if (ctrl_mode == CMD_PRODUCT_INFO)
            control_mode = MODE_SEND_PRODUCT_INFO;
          else if (ctrl_mode == CMD_OTA)
            control_mode = MODE_SEND_OTA;
          else if (ctrl_mode == CMD_WORK_STATE)
            control_mode = MODE_SEND_WORK_STATE;
          else if (ctrl_mode == CMD_RADAR_DETECT_RANGE)
            control_mode = MODE_SEND_RADAR_DETECT_RANGE;
          else if (ctrl_mode == CMD_BODY_EXIST_DETECT)
            control_mode = MODE_SEND_BODY_EXIST_DETECT;
          else if (ctrl_mode == CMD_BREATH_DETECT)
            control_mode = MODE_SEND_BREATH_DETECT;
          else if (ctrl_mode == CMD_SLEEP_DETECT)
            control_mode = MODE_SEND_SLEEP_DETECT;
          else if (ctrl_mode == CMD_HEART_DETECT)
            control_mode = MODE_SEND_HEART_DETECT;
          else
            control_mode = MODE_IDLE;

          rx_datagram_state = SEEN_CONTROL;
          // DEBUG("SEEN_CONTROL:");
          // DEBUGLN_HEX(ctrl_mode);
        }
        break;

      case SEEN_CONTROL: //读取控制字后,判断命令字
        {
          command = Serial1.read(); //命令字
          rx_sum += command;

          // ctrl_mode = Serial1.read(); //控制字
          rx_datagram_state = SEEN_COMMAND;
          // DEBUG("SEEN_COMMAND:");
          // DEBUGLN_HEX(command);
        }
        break;

      case SEEN_COMMAND: //读取命令字后,识别数据长度
        {
          uint8_t len_temp[2];
          len_temp[0] = Serial1.read();
          rx_sum += len_temp[0];
          len_temp[1] = Serial1.read();
          rx_sum += len_temp[1];

          rx_dategram_len = (len_temp[0] << 8) | len_temp[1];
          rx_datagram_state = SEEN_LENGTH;
        }
        break;

      case SEEN_LENGTH: //读取数据长度后,保存数据
        {
          if (Serial1.available() < (int)rx_dategram_len) //判断数据包是否完整
          {
            rx_datagram_state = IDLE;
            return;
          }
          uint8_t readlen = rx_dategram_len; //数据包长度
          uint8_t tmp[rx_dategram_len];      //数据包缓存
          uint8_t *ptmp = tmp;               //数据包缓存指针
          while (readlen--)
          {
            receivedbyte = Serial1.read();
            *ptmp++ = receivedbyte; //将数据存入缓存
            rx_sum += receivedbyte; //校验和
          }
          // TODO 使用上面操作,可以直接操作rx_data指针,不用拷贝数据到rx_data数组中
          for (uint8_t i = 0; i < rx_dategram_len; i++) //将数据存储到数组中
          {
            rx_data[i] = tmp[i];
          }

          rx_datagram_state = SEEN_DATA;
        }
        break;

      case SEEN_DATA: //读取数据后,判断校验和
        {
          uint8_t getsum = Serial1.read();

          if (getsum != rx_sum)
          {
            rx_datagram_state = IDLE;
            return;
          }
          else
          {
            // ⭐ 这里就安全了，command 已经有初值，编译器不会再警告
            switch (control_mode)
            {
              case MODE_SEND_TICK: //心跳包
                {
                  // DEBUGLN("MODE_SEND_TICK");
                }
                break;

              case MODE_SEND_PRODUCT_INFO: //产品信息
                {
                  // DEBUGLN("MODE_SEND_PRODUCT_INFO");
                }
                break;

              case MODE_SEND_OTA: // OTA升级
                {
                  // DEBUGLN("MODE_SEND_OTA");
                }
                break;

              case MODE_SEND_WORK_STATE: //工作状态
                {
                  // DEBUGLN("MODE_SEND_WORK_STATE");
                }
                break;

              case MODE_SEND_RADAR_DETECT_RANGE: //雷达检测范围
                {
                  // DEBUGLN("MODE_SEND_RADAR_DETECT_RANGE");
                }
                break;

              case MODE_SEND_BODY_EXIST_DETECT: //人体存在检测
                {
                  data_change.data_change_body_exist = 1;
                  switch (command)
                  {
                    case 1:
                      { //检测人体存在
                        body_exist_detect.body_exist_flag = (0 != rx_data[0]) ? 1 : 0;
                      }
                      break;

                    case 2:
                      { //运动状态
                        body_exist_detect.work_state = rx_data[0];
                      }
                      break;

                    case 3:
                      { //运动值
                        // display_switch(1);
                        body_exist_detect.body_move_param = rx_data[0];
                      }
                      break;

                    case 4:
                      { //人体距离
                        body_exist_detect.body_distance = (rx_data[0] << 8) | rx_data[1];
                      }
                      break;

                    case 5:
                      { //人体方位
                        body_exist_detect.body_direction[0] = (rx_data[0] << 8) | rx_data[1];
                        body_exist_detect.body_direction[1] = (rx_data[2] << 8) | rx_data[3];
                        body_exist_detect.body_direction[2] = (rx_data[4] << 8) | rx_data[5];
                      }
                      break;
                  }
                }
                break;

              case MODE_SEND_BREATH_DETECT: //呼吸检测
                {
                  data_change.data_change_breath = 1;
                  switch (command)
                  {
                    case 1:
                      {
                        //呼吸检测状态
                        breath_detect.breath_detect_state = rx_data[0];

                        // if (breath_detect.breath_detect_state == 1)
                        //   DEBUGLN("Breath Normal"); //呼吸正常
                        // else if (breath_detect.breath_detect_state == 2)
                        //   DEBUGLN("Breath Fast!");
                        // else if (breath_detect.breath_detect_state == 3)
                        //   DEBUGLN("Breath Slow!");
                        // else if (breath_detect.breath_detect_state == 4)
                        //   DEBUGLN("Breath Search!");
                      }
                      break;

                    case 2:
                      {
                        //呼吸值
                        breath_detect.breath_detect_value = rx_data[0];
                        // DEBUG("Breath:");
                        // DEBUGLN(rx_data[0]);
                      }
                  }
                }
                break;

              case MODE_SEND_SLEEP_DETECT: //睡眠检测
                {
                  data_change.data_change_sleep = 1;
                  //判断命令字
                  switch (command)
                  {
                    case 1:
                      { //入床/离床状态
                        sleep_detect.sleep_bed_state = rx_data[0];
                      }
                      break;

                    case 2:
                      { //睡眠状态
                        sleep_detect.sleep_detect_state = rx_data[0];
                      }
                      break;

                    case 3:
                      { //清醒时长
                        sleep_detect.sleep_wake_hour = (rx_data[0] << 8) | rx_data[1];
                      }
                      break;

                    case 4:
                      { //浅睡时长
                        sleep_detect.sleep_light_hour = (rx_data[0] << 8) | rx_data[1];
                      }
                      break;

                    case 5:
                      { //深睡时长
                        sleep_detect.sleep_deep_hour = (rx_data[0] << 8) | rx_data[1];
                      }
                      break;

                    case 0x06:
                      { //睡眠质量评分
                        sleep_detect.sleep_score = rx_data[0];
                      }
                      break;

                    case 0x0c:
                      { //睡眠检测结果
                        sleep_detect.sleep_score_detail.sleep_detail_exist = rx_data[0];
                        sleep_detect.sleep_score_detail.sleep_detail_state = rx_data[1];
                        sleep_detect.sleep_score_detail.sleep_detail_avg_breath = rx_data[2];
                        sleep_detect.sleep_score_detail.sleep_detail_avg_heart = rx_data[3];
                        sleep_detect.sleep_score_detail.sleep_detail_turn_over_times = rx_data[4];
                        sleep_detect.sleep_score_detail.sleep_detail_turn_over_L = rx_data[5];
                        sleep_detect.sleep_score_detail.sleep_detail_turn_over_S = rx_data[6];
                        sleep_detect.sleep_score_detail.sleep_detail_breath_stoptimes = rx_data[7];
                      }
                      break;

                    case 0x0d:
                      { //睡眠详情
                        sleep_detect.sleep_score_detail_1.sleep_detail_score = rx_data[0];
                        sleep_detect.sleep_score_detail_1.sleep_detail_time = (rx_data[1] << 8) | rx_data[2];
                        sleep_detect.sleep_score_detail_1.sleep_detail_awake = rx_data[3];
                        sleep_detect.sleep_score_detail_1.sleep_detail_light = rx_data[4];
                        sleep_detect.sleep_score_detail_1.sleep_detail_away = rx_data[5];
                        sleep_detect.sleep_score_detail_1.sleep_detail_away_times = rx_data[6];
                        sleep_detect.sleep_score_detail_1.sleep_detail_avg_breath = rx_data[7];
                        sleep_detect.sleep_score_detail_1.sleep_detail_avg_heart = rx_data[8];
                        sleep_detect.sleep_score_detail_1.sleep_detail_breath_stoptimes = rx_data[9];
                      }
                      break;

                    case 0x0e:
                      { //异常检测
                        sleep_detect.sleep_score_detail_err = rx_data[0];
                      }
                      break;
                  }
                }
                break;

              case MODE_SEND_HEART_DETECT: //心率检测
                {
                  data_change.data_change_heart = 1;
                  switch (command)
                  {
                    case 0:
                      { //开关心率检测
                      }
                      break;

                    case 2:
                      { //心率值
                        heart_detect.heart_detect_value = rx_data[0];
                      }
                      break;

                    case 5:
                      { //心率波形
                        heart_detect.heart_wave_data[0] = rx_data[0];
                        heart_detect.heart_wave_data[1] = rx_data[1];
                        heart_detect.heart_wave_data[2] = rx_data[2];
                        heart_detect.heart_wave_data[3] = rx_data[3];
                        heart_detect.heart_wave_data[4] = rx_data[4];
                      }
                      break;
                  }
                }
                break;

              case MODE_IDLE: //空闲
                {
                }
                break;
            }
          }
          rx_datagram_state = SEEN_SUM;
        }
        break;

      case SEEN_SUM: //读取校验后,判断帧尾1
        {
          rx_datagram_state = (0x54 == Serial1.read()) ? SEEN_END1 : IDLE;
        }
        break;

      case SEEN_END1: //读取帧尾1后,判断帧尾2
        {
          rx_datagram_state = (0x43 == Serial1.read()) ? SEEN_END2 : IDLE;
        }
        break;

      case SEEN_END2: //判断帧尾2后,设置接收完成标志
        {
          rx_flag = 1;
          rx_datagram_state = IDLE;
        }
        break;

      default:
        {
          receivedbyte = 0;

          rx_datagram_state = IDLE;
        }
        break;
    }
  }
}

/*转发原始数据*/
void SendRawData()
{
  while (Serial1.available())
  {
    Serial.write(Serial1.read());
  }
}

void CheckDataChanged()
{
  //人体存在
  if (data_change.data_change_body_exist)
  {
    data_change.data_change_body_exist = 0;
    // print data
  }

  //呼吸检测
  if (data_change.data_change_breath)
  {
    data_change.data_change_breath = 0;
    // print data
  }

  //睡眠检测
  if (data_change.data_change_sleep)
  {
    data_change.data_change_sleep = 0;
    // print data
  }

  //心率检测
  if (data_change.data_change_heart)
  {
    data_change.data_change_heart = 0;
    // print data
  }
}
