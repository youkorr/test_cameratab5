#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32_P4

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");
  
  // Vérifier la communication I2C
  if (!this->detect_sc2356_()) {
    ESP_LOGE(TAG, "Failed to detect SC2356 camera sensor");
    this->mark_failed();
    return;
  }
  
  // Configurer l'horloge MCLK
  if (!this->setup_clock_()) {
    ESP_LOGE(TAG, "Failed to setup camera clock");
    this->mark_failed();
    return;
  }
  
  // Configurer la caméra CSI
  if (!this->setup_camera_()) {
    ESP_LOGE(TAG, "Failed to setup camera");
    this->mark_failed();
    return;
  }
  
  // Configurer le capteur SC2356
  if (!this->configure_sc2356_()) {
    ESP_LOGE(TAG, "Failed to configure SC2356");
    this->mark_failed();
    return;
  }
  
  this->is_ready_ = true;
  ESP_LOGCONFIG(TAG, "Tab5 Camera setup completed successfully");
}

void Tab5Camera::loop() {
  // Vérifications périodiques si nécessaire
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->width_, this->height_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %d", this->jpeg_quality_);
  ESP_LOGCONFIG(TAG, "  Framerate: %d fps", this->framerate_);
  ESP_LOGCONFIG(TAG, "  MCLK Pin: %d", this->external_clock_pin_);
  ESP_LOGCONFIG(TAG, "  MCLK Frequency: %d Hz", this->external_clock_frequency_);
  
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Setup: FAILED");
  } else {
    ESP_LOGCONFIG(TAG, "  Setup: SUCCESS");
  }
}

bool Tab5Camera::detect_sc2356_() {
  ESP_LOGD(TAG, "Detecting SC2356 camera sensor...");
  
  // Test de communication I2C simple
  uint8_t data;
  if (!this->read_register(0x3107, &data, 1)) {
    ESP_LOGW(TAG, "Failed to read from SC2356, trying alternative addresses...");
    
    // Essayer différentes adresses I2C communes pour SC2356
    uint8_t addresses[] = {0x30, 0x3C, 0x36};
    for (uint8_t addr : addresses) {
      this->set_i2c_address(addr);
      if (this->read_register(0x3107, &data, 1)) {
        ESP_LOGI(TAG, "SC2356 detected at address 0x%02X", addr);
        return true;
      }
    }
    return false;
  }
  
  ESP_LOGI(TAG, "SC2356 communication established");
  return true;
}

bool Tab5Camera::setup_clock_() {
  ESP_LOGD(TAG, "Setting up camera clock on pin %d", this->external_clock_pin_);
  
  // Configuration LEDC pour générer l'horloge MCLK
  ledc_timer_config_t ledc_timer = {
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .duty_resolution = LEDC_TIMER_1_BIT,
    .timer_num = LEDC_TIMER_0,
    .freq_hz = this->external_clock_frequency_,
    .clk_cfg = LEDC_AUTO_CLK
  };
  
  esp_err_t ret = ledc_timer_config(&ledc_timer);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC timer: %s", esp_err_to_name(ret));
    return false;
  }
  
  ledc_channel_config_t ledc_channel = {
    .gpio_num = this->external_clock_pin_,
    .speed_mode = LEDC_LOW_SPEED_MODE,
    .channel = LEDC_CHANNEL_0,
    .timer_sel = LEDC_TIMER_0,
    .duty = 1,  // 50% duty cycle
    .hpoint = 0
  };
  
  ret = ledc_channel_config(&ledc_channel);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to configure LEDC channel: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "Camera clock configured successfully at %d Hz", this->external_clock_frequency_);
  return true;
}

bool Tab5Camera::setup_camera_() {
  ESP_LOGD(TAG, "Configuring CSI camera...");
  
  // Configuration spécifique pour ESP32-P4 avec CSI
  this->camera_config_.pin_pwdn = -1;   // Pas de power down
  this->camera_config_.pin_reset = -1;  // Reset géré différemment
  this->camera_config_.pin_xclk = this->external_clock_pin_;
  this->camera_config_.pin_sccb_sda = this->sda_pin_;
  this->camera_config_.pin_sccb_scl = this->scl_pin_;
  
  // Broches parallèles non utilisées en CSI
  this->camera_config_.pin_d7 = -1;
  this->camera_config_.pin_d6 = -1;
  this->camera_config_.pin_d5 = -1;
  this->camera_config_.pin_d4 = -1;
  this->camera_config_.pin_d3 = -1;
  this->camera_config_.pin_d2 = -1;
  this->camera_config_.pin_d1 = -1;
  this->camera_config_.pin_d0 = -1;
  this->camera_config_.pin_vsync = -1;
  this->camera_config_.pin_href = -1;
  this->camera_config_.pin_pclk = -1;
  
  // Configuration CSI
  this->camera_config_.xclk_freq_hz = this->external_clock_frequency_;
  this->camera_config_.ledc_timer = LEDC_TIMER_0;
  this->camera_config_.ledc_channel = LEDC_CHANNEL_0;
  this->camera_config_.pixel_format = this->get_pixformat_();
  this->camera_config_.frame_size = this->get_framesize_();
  this->camera_config_.jpeg_quality = this->jpeg_quality_;
  this->camera_config_.fb_count = 2;  // Double buffering
  this->camera_config_.fb_location = CAMERA_FB_IN_PSRAM;
  this->camera_config_.grab_mode = CAMERA_GRAB_LATEST;
  
  // Configuration CSI spécifique (si supportée par ESP-IDF)
  #ifdef CONFIG_CAMERA_CSI_ENABLED
  this->camera_config_.conv_mode = CAMERA_CONV_YUV422_TO_YUV420;
  this->camera_config_.csi_config.data_lanes = 2;  // SC2356 utilise 2 lanes
  this->camera_config_.csi_config.line_sync_en = false;
  this->camera_config_.csi_config.err_check_en = true;
  #endif
  
  // Initialiser la caméra
  esp_err_t ret = esp_camera_init(&this->camera_config_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x (%s)", ret, esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "CSI camera initialized successfully");
  return true;
}

bool Tab5Camera::configure_sc2356_() {
  ESP_LOGD(TAG, "Configuring SC2356 sensor...");
  
  // Configuration basique du SC2356
  // Ces registres sont des exemples, vous devrez les adapter selon la datasheet SC2356
  
  delay(10); // Attendre que le capteur soit prêt
  
  // Exemple de configuration (à adapter selon votre capteur)
  struct {
    uint16_t reg;
    uint8_t val;
  } sc2356_config[] = {
    // Configuration de base (exemples)
    {0x0103, 0x01}, // Software reset
    {0x0100, 0x00}, // Standby
    // Ajouter ici la configuration spécifique selon la datasheet
    {0x0100, 0x01}, // Start streaming
  };
  
  for (auto &config : sc2356_config) {
    if (!this->write_register(config.reg, &config.val, 1)) {
      ESP_LOGE(TAG, "Failed to write register 0x%04X", config.reg);
      return false;
    }
    delay(1);
  }
  
  ESP_LOGI(TAG, "SC2356 configured successfully");
  return true;
}

framesize_t Tab5Camera::get_framesize_() {
  if (this->width_ == 160 && this->height_ == 120) return FRAMESIZE_QQVGA;
  if (this->width_ == 320 && this->height_ == 240) return FRAMESIZE_QVGA;
  if (this->width_ == 640 && this->height_ == 480) return FRAMESIZE_VGA;
  if (this->width_ == 1280 && this->height_ == 720) return FRAMESIZE_HD;
  if (this->width_ == 1920 && this->height_ == 1080) return FRAMESIZE_FHD;
  return FRAMESIZE_QVGA; // Default
}

pixformat_t Tab5Camera::get_pixformat_() {
  if (this->pixel_format_ == "RGB565") return PIXFORMAT_RGB565;
  if (this->pixel_format_ == "YUV422") return PIXFORMAT_YUV422;
  if (this->pixel_format_ == "JPEG") return PIXFORMAT_JPEG;
  if (this->pixel_format_ == "RAW10") return PIXFORMAT_RAW;
  return PIXFORMAT_RGB565; // Default
}

camera_fb_t *Tab5Camera::get_frame() {
  if (!this->is_ready_) {
    return nullptr;
  }
  return esp_camera_fb_get();
}

void Tab5Camera::return_frame(camera_fb_t *fb) {
  if (fb != nullptr) {
    esp_camera_fb_return(fb);
  }
}

} // namespace tab5_camera
} // namespace esphome

#endif // USE_ESP32_P4
