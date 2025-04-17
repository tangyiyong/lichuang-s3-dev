#include <stdio.h>
#include "esp32_s3_szp.h"
#include "logo_en_240x240_lcd.h"
#include "yingwu.h"
#include "demos/lv_demos.h"


void app_main(void)
{
    bsp_i2c_init();  // I2C初始化
    axp2101_init(); // AXP2101电源管理芯片初始化
    pca9557_init();  // IO扩展芯片初始化

    bsp_lvgl_start(); // 初始化液晶屏lvgl接口

    /* 下面5个demos 只打开1个运行 */
    // lv_demo_benchmark(); 
    // lv_demo_keypad_encoder(); 
    lv_demo_music(); 
    // lv_demo_stress(); 
    // lv_demo_widgets();
}
