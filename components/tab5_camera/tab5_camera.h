// Modifications pour tab5_camera.h
#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/gpio.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32
#include "driver/ledc.h"
#include "esp_err.h"
// Retiré: #include "esp_camera.h" - pas disponible sans esp32_camera component
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

  // Méthodes pour fournir des données à d'autres composants
  bool is_ready() { return this->camera_initialized_; }
  std::string get_stream_url();

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
#endif
  
  bool camera_initialized_{false};
};

}  // namespace tab5_camera
}  // namespace esphome

