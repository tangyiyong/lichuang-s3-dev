/*
 * SPDX-FileCopyrightText: 2022 Espressif Systems (Shanghai) CO LTD
 *
 * SPDX-License-Identifier: Unlicense OR CC0-1.0
 */
#include <inttypes.h>
#include <string.h>
#include "sdkconfig.h"
#include "esp_check.h"
#include "esp_vfs_fat.h"
#include "driver/i2s_tdm.h"
#include "driver/i2s_std.h"
#include "driver/i2c.h"
#include "es7210.h"
#include "format_wav.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "esp32_s3_szp.h"

/* I2C port and GPIOs */
#define EXAMPLE_I2C_NUM            (0)
#define EXAMPLE_I2C_SDA_IO         (1)
#define EXAMPLE_I2C_SCL_IO         (2)

/* I2S port and GPIOs */
#define EXAMPLE_I2S_NUM            (0)
#define EXAMPLE_I2S_MCK_IO         (38)
#define EXAMPLE_I2S_BCK_IO         (14)
#define EXAMPLE_I2S_WS_IO          (13)
#define EXAMPLE_I2S_DI_IO          (12)

/* SD card GPIOs */
#define EXAMPLE_SD_CMD_IO      (48) 
#define EXAMPLE_SD_CLK_IO      (47)
#define EXAMPLE_SD_DAT0_IO     (21)

/* I2S configurations */
#define EXAMPLE_I2S_TDM_FORMAT     (ES7210_I2S_FMT_I2S)
#define EXAMPLE_I2S_CHAN_NUM       (2)
#define EXAMPLE_I2S_SAMPLE_RATE    (48000)
#define EXAMPLE_I2S_MCLK_MULTIPLE  (I2S_MCLK_MULTIPLE_256)
#define EXAMPLE_I2S_SAMPLE_BITS    (I2S_DATA_BIT_WIDTH_16BIT)
#define EXAMPLE_I2S_TDM_SLOT_MASK  (I2S_TDM_SLOT0 | I2S_TDM_SLOT1)

/* ES7210 configurations */
#define EXAMPLE_ES7210_I2C_ADDR    (0x41)
#define EXAMPLE_ES7210_I2C_CLK     (100000)
#define EXAMPLE_ES7210_MIC_GAIN    (ES7210_MIC_GAIN_30DB)
#define EXAMPLE_ES7210_MIC_BIAS    (ES7210_MIC_BIAS_2V87)
#define EXAMPLE_ES7210_ADC_VOLUME  (0)

/* SD card & recording configurations */
#define EXAMPLE_RECORD_TIME_SEC    (10)
#define EXAMPLE_SD_MOUNT_POINT     "/sdcard"
#define EXAMPLE_RECORD_FILE_PATH   "/RECORD.WAV"

static const char *TAG = "example";


static i2s_chan_handle_t es7210_i2s_init(void)
{
    i2s_chan_handle_t i2s_rx_chan = NULL;  // 定义接收通道句柄
    ESP_LOGI(TAG, "Create I2S receive channel");
    i2s_chan_config_t i2s_rx_conf = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER); // 配置接收通道
    ESP_ERROR_CHECK(i2s_new_channel(&i2s_rx_conf, NULL, &i2s_rx_chan)); // 创建i2s通道

    ESP_LOGI(TAG, "Configure I2S receive channel to TDM mode");
    // 定义接收通道为I2S TDM模式 并配置
    i2s_tdm_config_t i2s_tdm_rx_conf = {  
        .slot_cfg = I2S_TDM_PHILIPS_SLOT_DEFAULT_CONFIG(EXAMPLE_I2S_SAMPLE_BITS, I2S_SLOT_MODE_STEREO, EXAMPLE_I2S_TDM_SLOT_MASK),
        .clk_cfg  = {
            .clk_src = I2S_CLK_SRC_DEFAULT,
            .sample_rate_hz = EXAMPLE_I2S_SAMPLE_RATE,
            .mclk_multiple = EXAMPLE_I2S_MCLK_MULTIPLE
        },
        .gpio_cfg = {
            .mclk = EXAMPLE_I2S_MCK_IO,
            .bclk = EXAMPLE_I2S_BCK_IO,
            .ws   = EXAMPLE_I2S_WS_IO,
            .dout = -1, // ES7210 only has ADC capability
            .din  = EXAMPLE_I2S_DI_IO
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_tdm_mode(i2s_rx_chan, &i2s_tdm_rx_conf)); // 初始化I2S通道为TDM模式

    return i2s_rx_chan;
}

sdmmc_card_t * mount_sdcard(void)
{
    sdmmc_card_t *sdmmc_card = NULL;

    ESP_LOGI(TAG, "Mounting SD card");
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = true,
        .max_files = 2,
        .allocation_unit_size = 8 * 1024
    };

    ESP_LOGI(TAG, "Initializing SD card");
    ESP_LOGI(TAG, "Using SDMMC peripheral");

    sdmmc_host_t sdmmc_host = SDMMC_HOST_DEFAULT(); // SDMMC主机接口配置
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT(); // SDMMC插槽配置
    slot_config.width = 1;  // 设置为1线SD模式
    slot_config.clk = EXAMPLE_SD_CLK_IO; 
    slot_config.cmd = EXAMPLE_SD_CMD_IO;
    slot_config.d0 = EXAMPLE_SD_DAT0_IO;
    slot_config.flags |= SDMMC_SLOT_FLAG_INTERNAL_PULLUP; // 打开内部上拉电阻

    ESP_LOGI(TAG, "Mounting filesystem");

    esp_err_t ret;
    while (1) {
        ret = esp_vfs_fat_sdmmc_mount(EXAMPLE_SD_MOUNT_POINT, &sdmmc_host, &slot_config, &mount_config, &sdmmc_card);
        if (ret == ESP_OK) {
            break;
        } else if (ret == ESP_FAIL) {
            ESP_LOGE(TAG, "Failed to mount filesystem.");
        } else {
            ESP_LOGE(TAG, "Failed to initialize the card (%s). ", esp_err_to_name(ret));
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }

    ESP_LOGI(TAG, "Card size: %lluMB, speed: %ldMHz",
            (((uint64_t)sdmmc_card->csd.capacity) * sdmmc_card->csd.sector_size) >> 20,
            sdmmc_card->max_freq_khz / 1000);

    return sdmmc_card;
}

static void es7210_codec_init(void)
{
    // 初始化I2C接口
    // ESP_LOGI(TAG, "Init I2C used to configure ES7210");
    // i2c_config_t i2c_conf = {
    //     .sda_io_num = EXAMPLE_I2C_SDA_IO,
    //     .scl_io_num = EXAMPLE_I2C_SCL_IO,
    //     .mode = I2C_MODE_MASTER,
    //     .sda_pullup_en = GPIO_PULLUP_ENABLE,
    //     .scl_pullup_en = GPIO_PULLUP_ENABLE,
    //     .master.clk_speed = EXAMPLE_ES7210_I2C_CLK,
    // };
    // ESP_ERROR_CHECK(i2c_param_config(EXAMPLE_I2C_NUM, &i2c_conf));
    // ESP_ERROR_CHECK(i2c_driver_install(EXAMPLE_I2C_NUM, i2c_conf.mode, 0, 0, 0));

    // 创建es7210器件句柄
    es7210_dev_handle_t es7210_handle = NULL;
    es7210_i2c_config_t es7210_i2c_conf = {
        .i2c_port = EXAMPLE_I2C_NUM,
        .i2c_addr = EXAMPLE_ES7210_I2C_ADDR
    };
    ESP_ERROR_CHECK(es7210_new_codec(&es7210_i2c_conf, &es7210_handle));

    // 初始化es7210芯片
    ESP_LOGI(TAG, "Configure ES7210 codec parameters");
    es7210_codec_config_t codec_conf = {
        .i2s_format = EXAMPLE_I2S_TDM_FORMAT,
        .mclk_ratio = EXAMPLE_I2S_MCLK_MULTIPLE,
        .sample_rate_hz = EXAMPLE_I2S_SAMPLE_RATE,
        .bit_width = (es7210_i2s_bits_t)EXAMPLE_I2S_SAMPLE_BITS,
        .mic_bias = EXAMPLE_ES7210_MIC_BIAS,
        .mic_gain = EXAMPLE_ES7210_MIC_GAIN,
        .flags.tdm_enable = true
    };
    ESP_ERROR_CHECK(es7210_config_codec(es7210_handle, &codec_conf));
    ESP_ERROR_CHECK(es7210_config_volume(es7210_handle, EXAMPLE_ES7210_ADC_VOLUME));
}

static esp_err_t record_wav(i2s_chan_handle_t i2s_rx_chan)
{
    ESP_RETURN_ON_FALSE(i2s_rx_chan, ESP_FAIL, TAG, "invalid i2s channel handle pointer");
    esp_err_t ret = ESP_OK;

    uint32_t byte_rate = EXAMPLE_I2S_SAMPLE_RATE * EXAMPLE_I2S_CHAN_NUM * EXAMPLE_I2S_SAMPLE_BITS / 8;
    uint32_t wav_size = byte_rate * EXAMPLE_RECORD_TIME_SEC;

    const wav_header_t wav_header =
        WAV_HEADER_PCM_DEFAULT(wav_size, EXAMPLE_I2S_SAMPLE_BITS, EXAMPLE_I2S_SAMPLE_RATE, EXAMPLE_I2S_CHAN_NUM);

    ESP_LOGI(TAG, "Opening file %s", EXAMPLE_RECORD_FILE_PATH);
    FILE *f = fopen(EXAMPLE_SD_MOUNT_POINT EXAMPLE_RECORD_FILE_PATH, "w");
    ESP_RETURN_ON_FALSE(f, ESP_FAIL, TAG, "error while opening wav file");

    /* Write wav header */
    ESP_GOTO_ON_FALSE(fwrite(&wav_header, sizeof(wav_header_t), 1, f), ESP_FAIL, err,
                      TAG, "error while writing wav header");

    /* Start recording */
    size_t wav_written = 0;
    static int16_t i2s_readraw_buff[4096];
    ESP_GOTO_ON_ERROR(i2s_channel_enable(i2s_rx_chan), err, TAG, "error while starting i2s rx channel");
    while (wav_written < wav_size) {
        if(wav_written % byte_rate < sizeof(i2s_readraw_buff)) {
            ESP_LOGI(TAG, "Recording: %"PRIu32"/%ds", wav_written/byte_rate + 1, EXAMPLE_RECORD_TIME_SEC);
        }
        size_t bytes_read = 0;
        /* Read RAW samples from ES7210 */
        ESP_GOTO_ON_ERROR(i2s_channel_read(i2s_rx_chan, i2s_readraw_buff, sizeof(i2s_readraw_buff), &bytes_read,
                          pdMS_TO_TICKS(1000)), err, TAG, "error while reading samples from i2s");
        /* Write the samples to the WAV file */
        ESP_GOTO_ON_FALSE(fwrite(i2s_readraw_buff, bytes_read, 1, f), ESP_FAIL, err,
                          TAG, "error while writing samples to wav file");
        wav_written += bytes_read;
    }

err:
    i2s_channel_disable(i2s_rx_chan);
    ESP_LOGI(TAG, "Recording done! Flushing file buffer");
    fclose(f);

    return ret;
}

void app_main(void)
{
    bsp_i2c_init();  // I2C初始化
    axp2101_init(); // AXP2101电源管理芯片初始化
    pca9557_init();  // IO扩展芯片初始化
    bsp_lcd_init();  // 液晶屏初始化
    
    /* 初始化I2S接口 */
    i2s_chan_handle_t i2s_rx_chan = es7210_i2s_init();  
    /* 初始化es7210芯片 */
    es7210_codec_init();
    /* 挂载SD卡 */
    sdmmc_card_t *sdmmc_card = mount_sdcard();
    /* 录音 */
    esp_err_t err = record_wav(i2s_rx_chan);
    /* 弹出SD卡 */
    esp_vfs_fat_sdcard_unmount(EXAMPLE_SD_MOUNT_POINT, sdmmc_card);
    if(err == ESP_OK) {
        ESP_LOGI(TAG, "Audio was successfully recorded into "EXAMPLE_RECORD_FILE_PATH
                      ". You can now remove the SD card safely");
    } else {
        ESP_LOGE(TAG, "Record failed, "EXAMPLE_RECORD_FILE_PATH" on SD card may not be playable.");
    }
}
