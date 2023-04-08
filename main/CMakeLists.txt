set(srcs "main.c"
         "ble_hid/esp_hid_gap.c"
         "ble_hid/hid_host_gamepad.c"
         "oled/oled_drv.c"
         "oled/oled_main.c"
)
set(include_dirs "."
                 "ble_hid"
                 "oled"
)

idf_component_register(SRCS "${srcs}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       REQUIRES driver
                       PRIV_REQUIRES nvs_flash)
                       
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")