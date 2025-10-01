#ifndef __OLED_H__
#define __OLED_H__

#include <U8g2lib.h>
#include <Wire.h>

void display_init();

void display_update(uint8_t mode);

void disp_DataMenu();


#endif // __OLED_H__
