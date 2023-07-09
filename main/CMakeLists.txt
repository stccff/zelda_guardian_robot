set(srcs "main.c"
         "ble_hid/esp_hid_gap.c"
         "ble_hid/hid_host_gamepad.c"
         "oled/oled_drv.c"
         "oled/oled_main.c"
         "servo/servo_control.c"
         "sound/i2s_std_sound.c"
         "pwr_ctrl/pwr_ctrl.c"
)
set(include_dirs "."
                 "ble_hid"
                 "oled"
                 "servo"
                 "sound"
                 "pwr_ctrl"
)

set(embed_files "sound/sight_search.pcm")

idf_component_register(SRCS "${srcs}"
                       EMBED_FILES "${embed_files}"
                       INCLUDE_DIRS "${include_dirs}"
                       REQUIRES esp_hid
                       REQUIRES driver
                       PRIV_REQUIRES nvs_flash)
                       
target_compile_options(${COMPONENT_LIB} PRIVATE "-Wno-format")