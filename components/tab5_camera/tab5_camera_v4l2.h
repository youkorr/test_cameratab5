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

// Includes pour Tab5 V4L2 et BSP
#include "bsp/esp-bsp.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "components/tab5_camera/linux/videodev2.h"

#include "driver/ppa.h"

// Includes système nécessaires pour V4L2
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <sys/param.h>

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

namespace esphome {
namespace tab5_camera {

// Formats d'image supportés par le Tab5 (basé sur V4L2)
typedef enum {
    TAB5_PIXFORMAT_RGB565 = V4L2_PIX_FMT_RGB565,
    TAB5_PIXFORMAT_RGB888 = V4L2_PIX_FMT_RGB24,
    TAB5_PIXFORMAT_YUV422 = V4L2_PIX_FMT_YUV422P,
    TAB5_PIXFORMAT_YUV420 = V4L2_PIX_FMT_YUV420,
    TAB5_PIXFORMAT_GRAYSCALE = V4L2_PIX_FMT_GREY,
    TAB5_PIXFORMAT_RAW8 = V4L2_PIX_FMT_SBGGR8,
    TAB5_PIXFORMAT_RAW10 = V4L2_PIX_FMT_SBGGR10,
} tab5_pixformat_t;

// Tailles de frame supportées par le Tab5
typedef enum {
    TAB5_FRAMESIZE_QVGA,     // 320x240
    TAB5_FRAMESIZE_VGA,      // 640x480
    TAB5_FRAMESIZE_HD,       // 1280x720
    TAB5_FRAMESIZE_FULL_HD,  // 1920x1080
} tab5_framesize_t;

struct tab5_camera_config_t {
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint8_t buffer_count;
};

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  Tab5Camera() = default;
  ~Tab5Camera();

  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

  // Configuration
  void set_name(const std::string &name) { this->name_ = name; }
  void set_resolution(tab5_framesize_t framesize) { this->framesize_ = framesize; }
  void set_pixel_format(tab5_pixformat_t format) { this->pixel_format_ = format; }
  void set_vertical_flip(bool flip) { this->vertical_flip_ = flip; }
  void set_horizontal_mirror(bool mirror) { this->horizontal_mirror_ = mirror; }
  void set_camera_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }
  
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
  int open_video_device_(tab5_pixformat_t format);
  bool setup_camera_buffers_();
  
  static void streaming_task_(void *parameter);
  void streaming_loop_();
  
  bool camera_initialized_{false};
  bool streaming_active_{false};
  
  int video_fd_{-1};
  uint8_t* frame_buffers_[2]{nullptr, nullptr};  // Double buffer
  uint8_t* current_frame_buffer_{nullptr};
  size_t current_frame_size_{0};
  
  TaskHandle_t streaming_task_handle_{nullptr};
  QueueHandle_t control_queue_{nullptr};
  SemaphoreHandle_t frame_mutex_{nullptr};
  
  // Configuration PPA pour rotation/scaling
  ppa_client_handle_t ppa_handle_{nullptr};
  
  static constexpr uint32_t STREAM_TASK_STACK_SIZE = 8192;
  static constexpr uint32_t STREAM_TASK_PRIORITY = 5;
  static constexpr uint32_t STREAM_FRAME_RATE_MS = 33; // ~30 FPS
  static constexpr uint8_t BUFFER_COUNT = 2;

 private:
  std::string name_{"Tab5 Camera"};
  tab5_framesize_t framesize_{TAB5_FRAMESIZE_HD};
  tab5_pixformat_t pixel_format_{TAB5_PIXFORMAT_RGB565};
  bool vertical_flip_{false};
  bool horizontal_mirror_{true};  // Tab5 utilise mirror_x par défaut
  
  GPIOPin *reset_pin_{nullptr};
  
  bool error_state_{false};
  std::string last_error_{""};
  void set_error_(const std::string &error);
  void clear_error_();
  
  CallbackManager<void(uint8_t*, size_t)> on_frame_callbacks_;
  
  // Fonctions utilitaires
  tab5_camera_config_t get_camera_config_();
  void get_frame_dimensions_(uint32_t *width, uint32_t *height);
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // TAB5_CAMERA_USE_ESP32



