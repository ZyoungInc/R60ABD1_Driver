#include "oled.h"
#include "datahandle.h"
#include <U8g2lib.h>
#include <Wire.h>

// 推荐用默认 I2C 引脚，如果你确认用 GPIO7=SCL, GPIO6=SDA 可以写成这样：
// U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE, 7, 6);
U8G2_SSD1306_128X64_NONAME_F_HW_I2C u8g2(U8G2_R0, U8X8_PIN_NONE);

control_mode_t display_mode = MODE_SEND_BODY_EXIST_DETECT;

unsigned long tick = 0;
unsigned long MODE_tick = 0;

void display_init()
{
    u8g2.begin();
    u8g2.clearBuffer();
    u8g2.sendBuffer();
}

void display_update(uint8_t mode)
{
    display_mode = (mode == 1)   ? MODE_SEND_BODY_EXIST_DETECT
                   : (mode == 2) ? MODE_SEND_BREATH_DETECT
                   : (mode == 3) ? MODE_SEND_SLEEP_DETECT
                   : (mode == 4) ? MODE_SEND_HEART_DETECT
                                 : MODE_IDLE;

    if (millis() - tick > 300)
    {
        tick = millis();
        u8g2.clearBuffer();
        
        switch (display_mode)
        {
        case MODE_SEND_BODY_EXIST_DETECT:
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(0, 12, "BODY");
            u8g2.setFont(u8g2_font_6x10_tf);
            u8g2.setCursor(0, 30);
            u8g2.print("Move Value:");
            u8g2.print(body_exist_detect.body_move_param);
            break;

        case MODE_SEND_BREATH_DETECT:
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(0, 12, "BREATH");
            u8g2.setFont(u8g2_font_6x10_tf);
            u8g2.setCursor(0, 30);
            u8g2.print("Breath Value:");
            u8g2.print(breath_detect.breath_detect_value);
            break;

        case MODE_SEND_SLEEP_DETECT:
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(0, 12, "SLEEP");
            u8g2.setFont(u8g2_font_6x10_tf);
            u8g2.setCursor(0, 30);
            u8g2.print("Sleep Score:");
            u8g2.print(sleep_detect.sleep_score);
            break;

        case MODE_SEND_HEART_DETECT:
            u8g2.setFont(u8g2_font_ncenB08_tr);
            u8g2.drawStr(0, 12, "HEART");
            u8g2.setFont(u8g2_font_6x10_tf);
            u8g2.setCursor(0, 30);
            u8g2.print("Heart Rate:");
            u8g2.print(heart_detect.heart_detect_value);
            break;

        case MODE_IDLE:
            u8g2.setFont(u8g2_font_6x10_tf);
            u8g2.setCursor(0, 12);
            u8g2.print("Move   Value:");
            u8g2.print(body_exist_detect.body_move_param);
            u8g2.setCursor(0, 24);
            u8g2.print("Breath Value:");
            u8g2.print(breath_detect.breath_detect_value);
            u8g2.setCursor(0, 36);
            u8g2.print("Sleep  Score:");
            u8g2.print(sleep_detect.sleep_score);
            u8g2.setCursor(0, 48);
            u8g2.print("Heart  Rate:");
            u8g2.print(heart_detect.heart_detect_value);
            break;

        default:
            // 兜底，避免编译警告
            u8g2.setFont(u8g2_font_6x10_tf);
            u8g2.drawStr(0, 12, "Unknown Mode");
            break;
        }

        u8g2.sendBuffer();
    }
}

void disp_DataMenu()
{
    u8g2.clearBuffer();
    u8g2.setFont(u8g2_font_ncenB08_tr);
    u8g2.setCursor(0, 16);
    u8g2.print("SEND DATA");

    u8g2.setFont(u8g2_font_6x10_tf);
    u8g2.setCursor(0, 32);
    u8g2.print("Send RawData...");

    u8g2.sendBuffer();
}
