#ifndef __DATAHANDLE_H__
#define __DATAHANDLE_H__

#include "stdint.h"

#define DEBUG(...) Serial.print(__VA_ARGS__)
#define DEBUGLN(...) Serial.println(__VA_ARGS__)
#define DEBUGLN_HEX(...) Serial.println(__VA_ARGS__, HEX)

/* 定义包头及指令信息 */
//帧头
#define HEADER1 0x53
#define HEADER2 0x59
//控制字
#define CMD_TICK 0x01               //心跳包
#define CMD_PRODUCT_INFO 0x02       //产品信息
#define CMD_OTA 0x03                // OTA升级
#define CMD_WORK_STATE 0x05         //工作状态
#define CMD_RADAR_DETECT_RANGE 0x07 //雷达探测范围
#define CMD_BODY_EXIST_DETECT 0x80  //人体存在检测
#define CMD_BREATH_DETECT 0x81      //呼吸检测
#define CMD_SLEEP_DETECT 0x84       //睡眠检测
#define CMD_HEART_DETECT 0x85       //心率检测
//帧尾
#define END1 0x54
#define END2 0x43

/* 枚举读取数据报文的状态 */
typedef enum
{
    IDLE,
    SEEN_HEADER1,
    SEEN_HEADER2,
    SEEN_CONTROL,
    SEEN_COMMAND,
    SEEN_LENGTH,
    SEEN_DATA,
    SEEN_SUM,
    SEEN_END1,
    SEEN_END2
} rx_datagram_state_t;

/* 枚举控制模式 */
typedef enum
{
    MODE_IDLE,
    MODE_SEND_TICK,
    MODE_SEND_PRODUCT_INFO,
    MODE_SEND_OTA,
    MODE_SEND_WORK_STATE,
    MODE_SEND_RADAR_DETECT_RANGE,
    MODE_SEND_BODY_EXIST_DETECT,
    MODE_SEND_BREATH_DETECT,
    MODE_SEND_SLEEP_DETECT,
    MODE_SEND_HEART_DETECT
} control_mode_t;

/*******************************************************************************/
//人体存在功能
typedef struct
{
    uint8_t body_exist_flag;   //有人无人检测标志
    uint8_t work_state;        //运动状态
    uint8_t body_move_param;   //体动参数
    uint16_t body_distance;    //人体距离
    uint8_t body_direction[3]; //人体方位
} body_exist_detect_t;

//呼吸检测功能
typedef struct
{
    uint8_t breath_detect_switch; //开关呼吸功能 
    uint8_t breath_detect_state;  //呼吸检测状态
    uint8_t breath_detect_value;  //呼吸检测值
    uint8_t breath_wave_data[5];  //呼吸波形
} breath_detect_t;

//睡眠评分
typedef struct
{
    uint8_t sleep_detail_exist; //睡眠详细状态
    uint8_t sleep_detail_state; //睡眠详细评分

    uint8_t sleep_detail_score;            //睡眠评分
    uint16_t sleep_detail_time;            //睡眠时间
    uint8_t sleep_detail_awake;            //清醒时长占比
    uint8_t sleep_detail_light;            //浅睡时长占比
    uint8_t sleep_detail_deep;             //深睡时长占比
    uint8_t sleep_detail_away;             //离床时长占比
    uint8_t sleep_detail_away_times;       //离床次数
    uint8_t sleep_detail_turn_over_times;  //翻身次数
    uint8_t sleep_detail_avg_breath;       //平均呼吸
    uint8_t sleep_detail_avg_heart;        //平均心率
    uint8_t sleep_detail_breath_stoptimes; //呼吸停顿次数
    uint8_t sleep_detail_turn_over_L;      //大动作次数
    uint8_t sleep_detail_turn_over_S;      //小动作次数
} sleep_detail_t;

//睡眠检测功能
typedef struct
{
    uint8_t sleep_detect_switch; //开关睡眠功能
    uint8_t sleep_bed_state;     //入床/离床状态
    uint8_t sleep_detect_state;  //睡眠检测状态

    uint8_t sleep_wake_hour;  //清醒时间
    uint8_t sleep_light_hour; //浅睡时长
    uint8_t sleep_deep_hour;  //深睡时长

    uint8_t sleep_score;                 //睡眠质量评分
    sleep_detail_t sleep_score_detail;   //睡眠质量评分详情
    sleep_detail_t sleep_score_detail_1; //睡眠质量评分详情1
    uint8_t sleep_score_detail_err;      //睡眠质量评分详情2
} sleep_detect_t;

//心率检测功能
typedef struct
{
    uint8_t heart_detect_switch; //开关心率功能
    uint8_t heart_detect_value;  //心率检测值
    uint8_t heart_wave_data[5];  //心率波形
} heart_detect_t;

//数据变化标志
typedef struct
{
    unsigned data_change_body_exist:1;
    unsigned data_change_breath:1;
    unsigned data_change_sleep:1;
    unsigned data_change_heart:1;
} data_change_t;

void datahandle_init(void);
void ProcessRx();
void SendRawData();


extern body_exist_detect_t body_exist_detect;
extern breath_detect_t breath_detect;
extern sleep_detect_t sleep_detect;
extern heart_detect_t heart_detect;



#endif // __DATAHANDLE_H__
