idf_component_register(SRCS "publisher_main.c"
                    PRIV_REQUIRES esp_wifi nvs_flash
                    INCLUDE_DIRS "."
                    REQUIRES mpu6050_kalman i2cdev buzzer ssd1306_component cjson shared_data encoder_speed kinematics motor_control map)
