#include <stdio.h>
#include "esp32_s3_szp.h"
#include "app_ui.h"
#include "nvs_flash.h"


void app_main(void)
{
    // 初始化NVS
    esp_err_t ret = nvs_flash_init();
    if (ret == ESP_ERR_NVS_NO_FREE_PAGES || ret == ESP_ERR_NVS_NEW_VERSION_FOUND) {
        ESP_ERROR_CHECK(nvs_flash_erase());
        ret = nvs_flash_init();
    }
    ESP_ERROR_CHECK( ret );

    bsp_i2c_init();  // I2C初始化
    axp2101_init(); // AXP2101电源管理芯片初始化
    pca9557_init();  // IO扩展芯片初始化
    bsp_lvgl_start(); // 初始化液晶屏lvgl接口
    app_wifi_connect(); // 运行wifi连接程序
}
