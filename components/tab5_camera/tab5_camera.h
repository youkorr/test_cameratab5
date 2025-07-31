#pragma once

#include "esphome/core/component.h"
#include "esphome/components/i2c/i2c.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32_P4
#include "esp_camera.h"
#include "driver/ledc.h"
#include "soc/soc_caps.h"
#endif

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

// Registres SC2356 (exemples typiques)
static const uint16_t SC2356_REG_CHIP_ID_H = 0x3107;
static const uint16_t SC2356_REG_CHIP_ID_L = 0x3108;
static const uint8_t SC2356_CHIP_ID_H = 0x23;
static const uint8_t SC2356_CHIP_ID_L = 0x56;

class Tab5Camera : public Component, public i2c::I2CDevice {
 public:
  void setup() override;
  void loop() override;
  void dump_config() override;
  float get_setup_priority() const override { return setup_priority::DATA; }

  // Setters pour la configuration
  void set_name(const std::string &name) { this->name_ = name; }
  void set_external_clock_pin(int pin) { this->external_clock_pin_ = pin; }
  void set_external_clock_frequency(uint32_t freq) { this->external_clock_frequency_ = freq; }
  void set_csi_pins(int sda, int scl, int mclk) {
    this->sda_pin_ = sda;
    this->scl_pin_ = scl;
    this->mclk_pin_ = mclk;
  }
  void set_resolution(uint16_t width, uint16_t height) {
    this->width_ = width;
    this->height_ = height;
  }
  void set_pixel_format(const std::string &format) { this->pixel_format_ = format; }
  void set_jpeg_quality(uint8_t quality) { this->jpeg_quality_ = quality; }
  void set_framerate(uint8_t framerate) { this->framerate_ = framerate; }

  // Méthodes publiques pour l'utilisation
  bool is_ready() { return this->is_ready_; }
  
#ifdef USE_ESP32_P4
  camera_fb_t *get_frame();
  void return_frame(camera_fb_t *fb);
#endif

 protected:
  std::string name_;
  int external_clock_pin_{36};
  uint32_t external_clock_frequency_{24000000};
  int sda_pin_{31};
  int scl_pin_{32};
  int mclk_pin_{36};
  
  uint16_t width_{320};
  uint16_t height_{240};
  std::string pixel_format_{"RGB565"};
  uint8_t jpeg_quality_{63};
  uint8_t framerate_{30};
  
  bool is_ready_{false};
  
  // Méthodes privées
  bool setup_camera_();
  bool setup_clock_();
  bool detect_sc2356_();
  bool configure_sc2356_();
  
#ifdef USE_ESP32_P4
  framesize_t get_framesize_();
  pixformat_t get_pixformat_();
#endif
  
#ifdef USE_ESP32_P4
  camera_config_t camera_config_;
#endif
};

} // namespace tab5_camera
} // namespace esphome

