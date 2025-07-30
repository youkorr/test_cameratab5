// Modifications pour tab5_camera.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32
#include "driver/ledc.h"
#include "esp_err.h"
#include "esp_camera.h"  // Pour l'intégration avec esp32_camera_web_server
#endif

namespace esphome {
namespace tab5_camera {

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Setters pour la configuration
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(uint8_t pin) { this->clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->clock_freq_ = freq; }
  void set_resolution(uint16_t width, uint16_t height) { 
    this->width_ = width; 
    this->height_ = height; 
  }
  void set_pixel_format(const std::string &format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_framerate(uint8_t fps) { this->framerate_ = fps; }
  void set_reset_pin(GPIOPin *pin) { this->reset_pin_ = pin; }

#ifdef USE_ESP32
  // Méthodes pour l'intégration avec esp32_camera_web_server
  camera_fb_t* capture_frame_for_web();
  bool init_camera_for_web_server();
#endif

 protected:
  std::string name_;
  uint8_t clock_pin_{36};
  uint32_t clock_freq_{24000000};
  uint16_t width_{320};
  uint16_t height_{240};
  std::string pixel_format_{"RGB565"};
  uint8_t jpeg_quality_{10};
  uint8_t framerate_{15};
  GPIOPin *reset_pin_{nullptr};

#ifdef USE_ESP32
  esp_err_t init_camera_clock();
  camera_config_t get_camera_config();
#endif
  
  bool camera_initialized_{false};
  bool web_server_ready_{false};
};

}  // namespace tab5_camera
}  // namespace esphome

// Modifications pour tab5_camera.cpp

#ifdef USE_ESP32
camera_config_t Tab5Camera::get_camera_config() {
  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = -1;        // Vous devrez définir ces pins selon votre Tab5
  config.pin_d1 = -1;
  config.pin_d2 = -1;
  config.pin_d3 = -1;
  config.pin_d4 = -1;
  config.pin_d5 = -1;
  config.pin_d6 = -1;
  config.pin_d7 = -1;
  config.pin_xclk = this->clock_pin_;
  config.pin_pclk = -1;      // À définir selon votre matériel
  config.pin_vsync = -1;     // À définir selon votre matériel
  config.pin_href = -1;      // À définir selon votre matériel
  config.pin_sscb_sda = -1;  // Utilisera I2C existant
  config.pin_sscb_scl = -1;  // Utilisera I2C existant
  config.pin_pwdn = -1;
  config.pin_reset = (this->reset_pin_ != nullptr) ? this->reset_pin_->get_pin() : -1;
  config.xclk_freq_hz = this->clock_freq_;
  
  // Configuration du format selon les paramètres
  if (this->pixel_format_ == "JPEG") {
    config.pixel_format = PIXFORMAT_JPEG;
  } else if (this->pixel_format_ == "RGB565") {
    config.pixel_format = PIXFORMAT_RGB565;
  } else if (this->pixel_format_ == "YUV422") {
    config.pixel_format = PIXFORMAT_YUV422;
  } else {
    config.pixel_format = PIXFORMAT_RGB565; // Par défaut
  }
  
  // Configuration de la résolution
  if (this->width_ == 1920 && this->height_ == 1080) {
    config.frame_size = FRAMESIZE_HD;
  } else if (this->width_ == 1280 && this->height_ == 720) {
    config.frame_size = FRAMESIZE_HD;
  } else if (this->width_ == 640 && this->height_ == 480) {
    config.frame_size = FRAMESIZE_VGA;
  } else if (this->width_ == 320 && this->height_ == 240) {
    config.frame_size = FRAMESIZE_QVGA;
  } else {
    config.frame_size = FRAMESIZE_QVGA; // Par défaut
  }
  
  config.jpeg_quality = this->jpeg_quality_;
  config.fb_count = 2; // Double buffering
  
  return config;
}

bool Tab5Camera::init_camera_for_web_server() {
  camera_config_t config = this->get_camera_config();
  
  // Initialiser la caméra ESP32
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x", err);
    return false;
  }
  
  // Configuration du capteur (si nécessaire)
  sensor_t * s = esp_camera_sensor_get();
  if (s != nullptr) {
    // Ajustements spécifiques au capteur SC2356
    s->set_quality(s, this->jpeg_quality_);
    s->set_framesize(s, config.frame_size);
    // Autres ajustements...
  }
  
  this->web_server_ready_ = true;
  ESP_LOGI(TAG, "Camera ready for web server");
  return true;
}

camera_fb_t* Tab5Camera::capture_frame_for_web() {
  if (!this->web_server_ready_) {
    return nullptr;
  }
  
  return esp_camera_fb_get();
}
#endif
