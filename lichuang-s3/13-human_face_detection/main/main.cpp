#include <stdio.h>
#include "esp32_s3_szp.h"
#include "logo_en_240x240_lcd.h"
#include "yingwu.h"
#include "who_human_face_detection.hpp"


extern "C" void app_main(void)
{
    bsp_i2c_init();  // I2C初始化
    axp2101_init(); // AXP2101电源管理芯片初始化
    pca9557_init();  // IO扩展芯片初始化
    bsp_lcd_init();  // 液晶屏初始化
    lcd_draw_pictrue(0, 0, 320, 240, gImage_yingwu); // 显示3只鹦鹉图片
    vTaskDelay(500 / portTICK_PERIOD_MS);  // 延时500毫秒
    bsp_camera_init(); // 摄像头初始化
    app_camera_ai_lcd(); // 运行AI人脸检测任务
}
