#include "camera_service.h"

#include <Arduino.h>
#include "esp_camera.h"

#include "app_config.h"


bool cameraServiceInit()
{
    camera_config_t config = {};

    /*
     * 使用 LEDC 给摄像头产生 XCLK。
     */
    config.ledc_channel = LEDC_CHANNEL_0;
    config.ledc_timer   = LEDC_TIMER_0;

    /*
     * 摄像头数据引脚。
     */
    config.pin_d0 = AppConfig::CameraPins::D0;
    config.pin_d1 = AppConfig::CameraPins::D1;
    config.pin_d2 = AppConfig::CameraPins::D2;
    config.pin_d3 = AppConfig::CameraPins::D3;
    config.pin_d4 = AppConfig::CameraPins::D4;
    config.pin_d5 = AppConfig::CameraPins::D5;
    config.pin_d6 = AppConfig::CameraPins::D6;
    config.pin_d7 = AppConfig::CameraPins::D7;

    /*
     * 时钟和同步信号。
     */
    config.pin_xclk  = AppConfig::CameraPins::XCLK;
    config.pin_pclk  = AppConfig::CameraPins::PCLK;
    config.pin_vsync = AppConfig::CameraPins::VSYNC;
    config.pin_href  = AppConfig::CameraPins::HREF;

    /*
     * SCCB 配置总线。
     */
    config.pin_sccb_sda = AppConfig::CameraPins::SIOD;
    config.pin_sccb_scl = AppConfig::CameraPins::SIOC;

    config.pin_pwdn  = AppConfig::CameraPins::PWDN;
    config.pin_reset = AppConfig::CameraPins::RESET;

    /*
     * 摄像头时钟和输出格式。
     */
    config.xclk_freq_hz = 20000000;
    config.pixel_format = PIXFORMAT_JPEG;

    /*
     * 有 PSRAM 时，使用双缓冲和 VGA。
     */
    if (psramFound())
    {
        Serial.println("PSRAM found");

        config.frame_size   = FRAMESIZE_VGA;
        config.jpeg_quality = 12;
        config.fb_count     = 2;

        config.grab_mode =
            CAMERA_GRAB_LATEST;

        config.fb_location =
            CAMERA_FB_IN_PSRAM;
    }
    else
    {
        Serial.println("PSRAM not found");

        config.frame_size   = FRAMESIZE_QVGA;
        config.jpeg_quality = 15;
        config.fb_count     = 1;

        config.grab_mode =
            CAMERA_GRAB_WHEN_EMPTY;

        config.fb_location =
            CAMERA_FB_IN_DRAM;
    }

    /*
     * 初始化摄像头驱动。
     */
    esp_err_t error =
        esp_camera_init(&config);

    if (error != ESP_OK)
    {
        Serial.printf(
            "Camera init failed: 0x%x\n",
            error
        );

        return false;
    }

    /*
     * 获取实际识别到的传感器。
     */
    sensor_t *sensor =
        esp_camera_sensor_get();

    if (sensor == nullptr)
    {
        Serial.println(
            "Cannot get camera sensor"
        );

        esp_camera_deinit();

        return false;
    }

    Serial.printf(
        "Camera sensor PID: 0x%04X\n",
        static_cast<unsigned int>(
            sensor->id.PID
        )
    );

    /*
     * OV3660 画面修正。
     */
    if (sensor->id.PID == OV3660_PID)
    {
        Serial.println("OV3660 detected");

        sensor->set_vflip(
            sensor,
            1
        );

        sensor->set_brightness(
            sensor,
            1
        );

        sensor->set_saturation(
            sensor,
            -2
        );
    }

    Serial.println("Camera service started");

    return true;
}