#include <stdio.h>
#include "esp32_s3_szp.h"
#include "app_ui.h"
#include "app_sr.h"


void app_main(void)
{
    bsp_i2c_init();  // I2C初始化
    axp2101_init(); // AXP2101电源管理芯片初始化
    pca9557_init();  // IO扩展芯片初始化
    bsp_lvgl_start(); // 初始化液晶屏lvgl接口

    bsp_spiffs_mount(); // SPIFFS文件系统初始化
    bsp_codec_init(); // 音频初始化
    mp3_player_init(); // MP3播放器初始化

    app_sr_init();  // 语音识别初始化   
}
