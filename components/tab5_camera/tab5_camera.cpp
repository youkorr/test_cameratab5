#include "tab5_camera.h"
#include "esphome/core/log.h"

namespace esphome {

namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");

  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delayMicroseconds(10000);
    this->reset_pin_->digital_write(true);
    delayMicroseconds(10000);
  }

#ifdef USE_ESP32
  esp_err_t err = this->init_camera_clock();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize camera clock: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
#endif

  this->camera_initialized_ = true;
  ESP_LOGCONFIG(TAG, "Tab5 Camera initialized successfully");
}

void Tab5Camera::loop() {
  // Stub pour intégration future
}

void Tab5Camera::capture() {
  if (!this->camera_initialized_) {
    this->publish_failed();
    return;
  }

  // Remplacer cette simulation par l'appel au driver CSI/ISP selon votre caméra
  this->frame_size_ = this->width_ * this->height_ * 2;
  if (this->frame_buffer_ == nullptr) {
    this->frame_buffer_ = new uint8_t[this->frame_size_];
  }

  memset(this->frame_buffer_, 127, this->frame_size_); // Valeur grise fixe
  this->publish_image(this->frame_buffer_, this->frame_size_, camera::CameraImageType::RAW);
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Clock Pin: %d", this->clock_pin_);
  ESP_LOGCONFIG(TAG, "  Clock Frequency: %d Hz", this->clock_freq_);
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->width_, this->height_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %d", this->jpeg_quality_);
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
}

#ifdef USE_ESP32
esp_err_t Tab5Camera::init_camera_clock() {
  ledc_timer_config_t timer_conf = {};
  timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
  timer_conf.freq_hz = this->clock_freq_;
  timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  timer_conf.timer_num = LEDC_TIMER_0;

  esp_err_t err = ledc_timer_config(&timer_conf);
  if (err != ESP_OK) return err;

  ledc_channel_config_t ch_conf = {};
  ch_conf.gpio_num = this->clock_pin_;
  ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  ch_conf.channel = LEDC_CHANNEL_0;
  ch_conf.intr_type = LEDC_INTR_DISABLE;
  ch_conf.timer_sel = LEDC_TIMER_0;
  ch_conf.duty = 1;
  ch_conf.hpoint = 0;
  ch_conf.flags.output_invert = 0;

  return ledc_channel_config(&ch_conf);
}
#endif

}  // namespace tab5_camera
}  // namespace esphome
