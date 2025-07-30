#include "tab5_camera.h"
#include "esphome/core/log.h"

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");
  
  // Initialiser le pin de reset si configuré
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(false);
    delay(10);
    this->reset_pin_->digital_write(true);
    delay(10);
  }
  
  // Initialiser l'horloge de la caméra
  esp_err_t err = this->init_camera_clock();
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize camera clock: %s", esp_err_to_name(err));
    this->mark_failed();
    return;
  }
  
  ESP_LOGCONFIG(TAG, "Tab5 Camera initialized successfully");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Clock Pin: %d", this->clock_pin_);
  ESP_LOGCONFIG(TAG, "  Clock Frequency: %d Hz", this->clock_freq_);
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->width_, this->height_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %d", this->jpeg_quality_);
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  
  this->camera_initialized_ = true;
}

void Tab5Camera::loop() {
  // Logique de boucle si nécessaire
  // Pour l'instant, juste maintenir l'horloge active
}

esp_err_t Tab5Camera::init_camera_clock() {
  // Configuration du timer LEDC
  ledc_timer_config_t timer_conf = {};
  timer_conf.duty_resolution = LEDC_TIMER_1_BIT;
  timer_conf.freq_hz = this->clock_freq_;
  timer_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  timer_conf.deconfigure = false;
  timer_conf.clk_cfg = LEDC_AUTO_CLK;
  timer_conf.timer_num = LEDC_TIMER_0;
  
  esp_err_t err = ledc_timer_config(&timer_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ledc_timer_config failed for freq %d, rc=%x", this->clock_freq_, err);
    return err;
  }

  // Configuration du canal LEDC
  ledc_channel_config_t ch_conf = {};
  ch_conf.gpio_num = this->clock_pin_;
  ch_conf.speed_mode = LEDC_LOW_SPEED_MODE;
  ch_conf.channel = LEDC_CHANNEL_0;
  ch_conf.intr_type = LEDC_INTR_DISABLE;
  ch_conf.timer_sel = LEDC_TIMER_0;
  ch_conf.duty = 1;  // 50% duty cycle
  ch_conf.hpoint = 0;
  ch_conf.sleep_mode = LEDC_SLEEP_MODE_KEEP_ALIVE;
  
  err = ledc_channel_config(&ch_conf);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "ledc_channel_config failed, rc=%x", err);
    return err;
  }

  ESP_LOGI(TAG, "Camera clock initialized on pin %d at %d Hz", this->clock_pin_, this->clock_freq_);
  return ESP_OK;
}

}  // namespace tab5_camera
}  // namespace esphome
