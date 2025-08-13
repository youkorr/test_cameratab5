#pragma once

#include "esphome/core/component.h"
#include "esphome/core/gpio.h"
#include "esphome/components/i2c/i2c.h"

// Définition spécifique pour ce composant
#ifndef TAB5_CAMERA_USE_ESP32
#define TAB5_CAMERA_USE_ESP32 1
#endif

#ifdef TAB5_CAMERA_USE_ESP32

#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include "driver/i2c_master.h"

// Includes pour l'API V4L2
#include <fcntl.h>
#include <sys/ioctl.h>
#include "esp_heap_caps.h"
#include <sys/errno.h>
#include <unistd.h>
#include <string.h>

// V4L2 headers
extern "C" {
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "esp_video_device.h"
#include "driver/ppa.h"
}

#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "freertos/semphr.h"

namespace esphome {
namespace tab5_camera {

// Formats d'image
typedef enum {
    TAB5_VIDEO_FMT_RAW8   = V4L2_PIX_FMT_SBGGR8,
    TAB5_VIDEO_FMT_RAW10  = V4L2_PIX_FMT_SBGGR10,
    TAB5_VIDEO_FMT_GREY   = V4L2_PIX_FMT_GREY,
    TAB5_VIDEO_FMT_RGB565 = V4L2_PIX_FMT_RGB565,
    TAB5_VIDEO_FMT_RGB888 = V4L2_PIX_FMT_RGB24,
    TAB5_VIDEO_FMT_YUV422 = V4L2_PIX_FMT_YUV422P,
    TAB5_VIDEO_FMT_YUV420 = V4L2_PIX_FMT_YUV420,
} tab5_fmt_t;

// Structure caméra
typedef struct {
    int fd;
    uint32_t width;
    uint32_t height;
    uint32_t pixel_format;
    uint8_t* buffer[2];
    size_t buffer_size;
} tab5_cam_t;

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
  void set_resolution(uint16_t width, uint16_t height) { 
    this->frame_width_ = width; 
    this->frame_height_ = height; 
  }
  
  // Fonctions principales
  bool take_snapshot();
  bool start_streaming();
  bool stop_streaming();
  bool is_streaming() const { return this->streaming_active_; }
  
  // Getters
  const std::string &get_name() const { return this->name_; }
  uint16_t get_frame_width() const { return this->frame_width_; }
  uint16_t get_frame_height() const { return this->frame_height_; }
  uint8_t* get_frame_buffer() const { 
    return this->camera_ ? this->camera_->buffer[0] : nullptr; 
  }
  size_t get_frame_buffer_size() const { 
    return this->camera_ ? this->camera_->buffer_size : 0; 
  }

  // Callback pour les frames
  void add_on_frame_callback(std::function<void(uint8_t*, size_t)> &&callback) {
    this->on_frame_callbacks_.add(std::move(callback));
  }

  // Gestion des erreurs
  bool has_error() const { return this->error_state_; }
  const std::string &get_last_error() const { return this->last_error_; }

 protected:
  bool init_video_system_();
  bool init_video_system_fallback_();
  bool open_video_device_(const char* dev_path, tab5_fmt_t format);
  bool setup_camera_buffers_();
  bool init_ppa_processor_();
  void cleanup_resources_();
  
  static void streaming_task_(void *parameter);
  void streaming_loop_();
  
  tab5_cam_t* camera_{nullptr};
  bool camera_initialized_{false};
  bool streaming_active_{false};
  
  esp_video_init_config_t video_config_{};
  ppa_client_handle_t ppa_handle_{nullptr};
  i2c_master_bus_handle_t i2c_bus_handle_{nullptr};
  
  TaskHandle_t streaming_task_handle_{nullptr};
  QueueHandle_t control_queue_{nullptr};
  
  static constexpr const char* DEVICE_PATH = "/dev/video0";
  static constexpr size_t BUFFER_COUNT = 2;
  static constexpr uint32_t DEFAULT_WIDTH = 1280;
  static constexpr uint32_t DEFAULT_HEIGHT = 720;

 private:
  std::string name_{"Tab5 Camera"};
  uint16_t frame_width_{DEFAULT_WIDTH};
  uint16_t frame_height_{DEFAULT_HEIGHT};
  
  bool error_state_{false};
  std::string last_error_{""};
  void set_error_(const std::string &error);
  void clear_error_();
  
  CallbackManager<void(uint8_t*, size_t)> on_frame_callbacks_;
};

}  // namespace tab5_camera
}  // namespace esphome

#endif  // TAB5_CAMERA_USE_ESP32



