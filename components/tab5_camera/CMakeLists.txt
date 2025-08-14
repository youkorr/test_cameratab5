# CMakeLists.txt pour le composant Tab5 Camera
idf_component_register(
    SRCS "tab5_camera_v4l2.cpp"
    INCLUDE_DIRS "."
    REQUIRES 
        "esp_video_device"
        "esp_video_init" 
        "bsp"
        "driver"
        "esp_common"
        "freertos"
        "heap"
        "log"
        "esp_system"
        "esp_hw_support"
        "soc"
        "hal"
        "esp_driver_i2c"
        "esp_driver_gpio"
        "esp_driver_ppa"
        "esp_mm"
        "vfs"
        "esp_timer"
    PRIV_REQUIRES
        "esp_psram"
)
