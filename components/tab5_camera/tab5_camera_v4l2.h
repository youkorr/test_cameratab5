#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/i2c/i2c.h"

#ifndef TAB5_CAMERA_USE_ESP32
#define TAB5_CAMERA_USE_ESP32 1
#endif

#ifdef TAB5_CAMERA_USE_ESP32

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "esp_heap_caps.h"

// Includes pour ESP32 Camera (pas V4L2)
#include "esp_camera.h"

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"



namespace esphome {
namespace tab5_camera {

// Formats d'image supportés par le Tab5
typedef enum {
    TAB5_PIXFORMAT_RGB565,    // 2BPP/RGB565
    TAB5_PIXFORMAT_YUV422,    // 2BPP/YUV422
    TAB5_PIXFORMAT_GRAYSCALE, // 1BPP/GRAYSCALE
    TAB5_PIXFORMAT_JPEG,      // JPEG/Compressed
    TAB5_PIXFORMAT_RGB888,    // 3BPP/RGB888
    TAB5_PIXFORMAT_RAW,       // RAW
} tab5_pixformat_t;

// Tailles de frame supportées
typedef enum {
    TAB5_FRAMESIZE_96X96,    // 96x96
    TAB5_FRAMESIZE_QQVGA,    // 160x120
    TAB5_FRAMESIZE_QCIF,     // 176x144
    TAB5_FRAMESIZE_HQVGA,    // 240x176
    TAB5_FRAMESIZE_240X240,  // 240x240
    TAB5_FRAMESIZE_QVGA,     // 320x240
    TAB5_FRAMESIZE_CIF,      // 400x296
    TAB5_FRAMESIZE_HVGA,     // 480x320
    TAB5_FRAMESIZE_VGA,      // 640x480
    TAB5_FRAMESIZE_SVGA,     // 800x600
    TAB5_FRAMESIZE_XGA,      // 1024x768
    TAB5_FRAMESIZE_HD,       // 1280x720
    TAB5_FRAMESIZE_SXGA,     // 1280x1024
    TAB5_FRAMESIZE_UXGA,     // 1600x1200
} tab5_framesize_t;

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  Tab5Camera() = default;
  ~Tab5Camera();

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Configuration
  void set_name(const std::string &name) { this->name_ = name; }
  void set_sensor_address(uint8_t address) { this->set_i2c_address(address); }
  void set_resolution(tab5_framesize_t framesize) { this->framesize_ = framesize; }
  void set_pixel_format(tab5_pixformat_t format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = std::clamp((int)quality, 10, 100); }
  void set_vertical_flip(bool flip) { this->vertical_flip_ = flip; }
  void set_horizontal_mirror(bool mirror) { this->horizontal_mirror_ = mirror; }
  void set_brightness(int brightness) { this->brightness_ = std::clamp(brightness, -2, 2); }
  void set_contrast(int contrast) { this->contrast_ = std::clamp(contrast, -2, 2); }
  void set_saturation(int saturation) { this->saturation_ = std::clamp(saturation, -2, 2); }
  
  // Fonctions principales
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return this->streaming_active_; }
  
  // Getters
  const std::string &get_name() const { return this->name_; }
  tab5_framesize_t get_framesize() const { return this->framesize_; }
  tab5_pixformat_t get_pixel_format() const { return this->pixel_format_; }
  uint8_t* get_frame_buffer() const { return this->current_frame_buffer_; }
  size_t get_frame_buffer_size() const { return this->current_frame_size_; }

  // Callback pour les frames
  void add_on_frame_callback(std::function<void(uint8_t*, size_t)> &&callback) {
    this->on_frame_callbacks_.add(std::move(callback));
  }

  // Gestion des erreurs
  bool has_error() const { return this->error_state_; }
  const std::string &get_last_error() const { return this->last_error_; }

 protected:
  bool init_camera_();
  bool configure_camera_();
  void cleanup_resources_();
  
  static void streaming_task_(void *parameter);
  void streaming_loop_();
  
  bool camera_initialized_{false};
  bool streaming_active_{false};
  
  uint8_t* current_frame_buffer_{nullptr};
  size_t current_frame_size_{0};
  
  TaskHandle_t streaming_task_handle_{nullptr};
  QueueHandle_t control_queue_{nullptr};
  SemaphoreHandle_t frame_mutex_{nullptr};
  
  static constexpr uint32_t STREAM_TASK_STACK_SIZE = 8192;
  static constexpr uint32_t STREAM_TASK_PRIORITY = 5;
  static constexpr uint32_t STREAM_FRAME_RATE_MS = 33; // ~30 FPS

 private:
  std::string name_{"Tab5 Camera"};
  tab5_framesize_t framesize_{TAB5_FRAMESIZE_VGA};
  tab5_pixformat_t pixel_format_{TAB5_PIXFORMAT_JPEG};
  uint8_t jpeg_quality_{12};
  bool vertical_flip_{false};
  bool horizontal_mirror_{false};
  int brightness_{0};
  int contrast_{0};
  int saturation_{0};
  
  bool error_state_{false};
  std::string last_error_{""};
  void set_error_(const std::string &error);
  void clear_error_();
  
  CallbackManager<void(uint8_t*, size_t)> on_frame_callbacks_;
  
  // Fonctions utilitaires
  camera_config_t get_camera_config_();
  framesize_t convert_framesize_(tab5_framesize_t framesize);
  pixformat_t convert_pixel_format_(tab5_pixformat_t format);
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // TAB5_CAMERA_USE_ESP32



