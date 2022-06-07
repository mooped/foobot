idf_component_register(SRCS "foobot.c" "battery.c" "led.c" "motor.c" "mount.c" "wifi.c"
                    INCLUDE_DIRS ".")

spiffs_create_partition_image(www www FLASH_IN_PROJECT)
