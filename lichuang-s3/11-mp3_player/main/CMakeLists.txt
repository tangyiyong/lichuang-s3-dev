idf_component_register(SRCS "esp32_s3_szp.c" "main.c" "app_ui.c"
                    INCLUDE_DIRS ".")

spiffs_create_partition_image(storage ../spiffs FLASH_IN_PROJECT)
