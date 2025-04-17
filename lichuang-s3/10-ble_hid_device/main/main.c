#include <stdio.h>
#include "esp32_s3_szp.h"
#include "ble_hidd_demo.h"

void app_main(void)
{
    bsp_i2c_init();  // I2C初始化
    axp2101_init(); // AXP2101电源管理芯片初始化
    pca9557_init();  // IO扩展芯片初始化
    bsp_lvgl_start(); // 初始化液晶屏lvgl接口
    app_hid_ctrl(); // 运行蓝牙hid程序
}
