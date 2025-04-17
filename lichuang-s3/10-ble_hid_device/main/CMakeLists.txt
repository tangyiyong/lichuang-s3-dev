idf_component_register(SRCS "esp32_s3_szp.c" 
                            "main.c"
                            "ble_hidd_demo.c"
                            "esp_hidd_prf_api.c"
                            "hid_dev.c"
                            "hid_device_le_prf.c"
                    INCLUDE_DIRS ".")

target_compile_options(${COMPONENT_LIB} PRIVATE -Wno-unused-const-variable)
