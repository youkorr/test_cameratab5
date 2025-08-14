#include "tab5_camera_v4l2.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#ifndef TAB5_CAMERA_USE_ESP32
#define TAB5_CAMERA_USE_ESP32 1
#endif

#ifdef TAB5_CAMERA_USE_ESP32

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

#define TASK_CONTROL_PAUSE  0
#define TASK_CONTROL_RESUME 1
#define TASK_CONTROL_EXIT   2

Tab5Camera::~Tab5Camera() {
  this->cleanup_resources_();
}

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");
  this->clear_error_();

  // Créer les objets de synchronisation
  this->control_queue_ = xQueueCreate(10, sizeof(int));
  if (!this->control_queue_) {
    this->set_error_("Failed to create control queue");
    this->mark_failed();
    return;
  }

  this->frame_mutex_ = xSemaphoreCreateMutex();
  if (!this->frame_mutex_) {
    this->set_error_("Failed to create frame mutex");
    this->mark_failed();
    return;
  }

  // Initialiser le BSP de la caméra Tab5
  ESP_LOGI(TAG, "Initializing Tab5 camera BSP...");
  bsp_cam_osc_init();

  // Initialiser la caméra
  if (!this->init_camera_()) {
    this->set_error_("Failed to initialize camera");
    this->mark_failed();
    return;
  }

  if (!this->configure_camera_()) {
    this->set_error_("Failed to configure camera");
    this->mark_failed();
    return;
  }

  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Tab5 Camera initialized successfully");
}

bool Tab5Camera::init_camera_() {
  camera_config_t config = this->get_camera_config_();

  // Initialiser la caméra ESP32
  esp_err_t err = esp_camera_init(&config);
  if (err != ESP_OK) {
    ESP_LOGE(TAG, "Camera init failed with error 0x%x: %s", err, esp_err_to_name(err));
    return false;
  }

  return true;
}

bool Tab5Camera::configure_camera_() {
  sensor_t* sensor = esp_camera_sensor_get();
  if (!sensor) {
    ESP_LOGE(TAG, "Failed to get camera sensor");
    return false;
  }

  // Configurer les paramètres de la caméra
  sensor->set_pixformat(sensor, this->convert_pixel_format_(this->pixel_format_));
  sensor->set_framesize(sensor, this->convert_framesize_(this->framesize_));
  sensor->set_vflip(sensor, this->vertical_flip_);
  sensor->set_hmirror(sensor, this->horizontal_mirror_);
  sensor->set_brightness(sensor, this->brightness_);
  sensor->set_contrast(sensor, this->contrast_);
  sensor->set_saturation(sensor, this->saturation_);

  if (this->pixel_format_ == TAB5_PIXFORMAT_JPEG) {
    sensor->set_quality(sensor, this->jpeg_quality_);
  }

  ESP_LOGI(TAG, "Camera configured successfully");
  return true;
}

camera_config_t Tab5Camera::get_camera_config_() {
  camera_config_t config;
  
  // Configuration des pins pour Tab5 - à ajuster selon le schéma réel
  config.pin_pwdn = -1;  // Pas de pin power down
  config.pin_reset = -1; // Pas de pin reset
  config.pin_xclk = GPIO_NUM_48;  // Clock pin
  config.pin_sccb_sda = GPIO_NUM_31;  // I2C SDA (système I2C du Tab5)
  config.pin_sccb_scl = GPIO_NUM_32;  // I2C SCL (système I2C du Tab5)
  
  // Pins de données (à ajuster selon le schéma du Tab5)
  config.pin_d7 = GPIO_NUM_7;
  config.pin_d6 = GPIO_NUM_6;
  config.pin_d5 = GPIO_NUM_5;
  config.pin_d4 = GPIO_NUM_4;
  config.pin_d3 = GPIO_NUM_3;
  config.pin_d2 = GPIO_NUM_2;
  config.pin_d1 = GPIO_NUM_1;
  config.pin_d0 = GPIO_NUM_0;
  config.pin_vsync = GPIO_NUM_45;
  config.pin_href = GPIO_NUM_46;
  config.pin_pclk = GPIO_NUM_47;

  // Configuration XCLK
  config.xclk_freq_hz = 20000000;  // 20MHz
  config.ledc_timer = LEDC_TIMER_0;
  config.ledc_channel = LEDC_CHANNEL_0;

  // Format et taille
  config.pixel_format = this->convert_pixel_format_(this->pixel_format_);
  config.frame_size = this->convert_framesize_(this->framesize_);

  // Configuration des buffers
  config.jpeg_quality = this->jpeg_quality_;
  config.fb_count = 2;  // Double buffer
  config.fb_location = CAMERA_FB_IN_PSRAM;
  config.grab_mode = CAMERA_GRAB_WHEN_EMPTY;

  return config;
}

framesize_t Tab5Camera::convert_framesize_(tab5_framesize_t framesize) {
  switch (framesize) {
    case TAB5_FRAMESIZE_96X96: return FRAMESIZE_96X96;
    case TAB5_FRAMESIZE_QQVGA: return FRAMESIZE_QQVGA;
    case TAB5_FRAMESIZE_QCIF: return FRAMESIZE_QCIF;
    case TAB5_FRAMESIZE_HQVGA: return FRAMESIZE_HQVGA;
    case TAB5_FRAMESIZE_240X240: return FRAMESIZE_240X240;
    case TAB5_FRAMESIZE_QVGA: return FRAMESIZE_QVGA;
    case TAB5_FRAMESIZE_CIF: return FRAMESIZE_CIF;
    case TAB5_FRAMESIZE_HVGA: return FRAMESIZE_HVGA;
    case TAB5_FRAMESIZE_VGA: return FRAMESIZE_VGA;
    case TAB5_FRAMESIZE_SVGA: return FRAMESIZE_SVGA;
    case TAB5_FRAMESIZE_XGA: return FRAMESIZE_XGA;
    case TAB5_FRAMESIZE_HD: return FRAMESIZE_HD;
    case TAB5_FRAMESIZE_SXGA: return FRAMESIZE_SXGA;
    case TAB5_FRAMESIZE_UXGA: return FRAMESIZE_UXGA;
    default: return FRAMESIZE_VGA;
  }
}

pixformat_t Tab5Camera::convert_pixel_format_(tab5_pixformat_t format) {
  switch (format) {
    case TAB5_PIXFORMAT_RGB565: return PIXFORMAT_RGB565;
    case TAB5_PIXFORMAT_YUV422: return PIXFORMAT_YUV422;
    case TAB5_PIXFORMAT_GRAYSCALE: return PIXFORMAT_GRAYSCALE;
    case TAB5_PIXFORMAT_JPEG: return PIXFORMAT_JPEG;
    case TAB5_PIXFORMAT_RGB888: return PIXFORMAT_RGB888;
    case TAB5_PIXFORMAT_RAW: return PIXFORMAT_RAW;
    default: return PIXFORMAT_JPEG;
  }
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  camera_fb_t* fb = esp_camera_fb_get();
  if (!fb) {
    ESP_LOGE(TAG, "Camera capture failed");
    return false;
  }

  // Protéger l'accès au buffer de frame
  if (xSemaphoreTake(this->frame_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
    this->current_frame_buffer_ = fb->buf;
    this->current_frame_size_ = fb->len;
    
    // Appeler les callbacks
    this->on_frame_callbacks_.call(fb->buf, fb->len);
    
    xSemaphoreGive(this->frame_mutex_);
  }

  // Libérer le frame buffer
  esp_camera_fb_return(fb);
  
  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_ || this->streaming_active_) {
    return false;
  }

  ESP_LOGI(TAG, "Starting camera streaming...");
  
  BaseType_t result = xTaskCreate(
    streaming_task_,
    "tab5_camera_stream",
    STREAM_TASK_STACK_SIZE,
    this,
    STREAM_TASK_PRIORITY,
    &this->streaming_task_handle_
  );
  
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create streaming task");
    return false;
  }

  this->streaming_active_ = true;
  ESP_LOGI(TAG, "Camera streaming started");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return false;
  }

  ESP_LOGI(TAG, "Stopping camera streaming...");
  
  int control_cmd = TASK_CONTROL_EXIT;
  xQueueSend(this->control_queue_, &control_cmd, portMAX_DELAY);

  // Attendre que la tâche se termine
  if (this->streaming_task_handle_) {
    vTaskDelete(this->streaming_task_handle_);
    this->streaming_task_handle_ = nullptr;
  }

  this->streaming_active_ = false;
  ESP_LOGI(TAG, "Camera streaming stopped");
  return true;
}

void Tab5Camera::streaming_task_(void *parameter) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(parameter);
  camera->streaming_loop_();
  vTaskDelete(nullptr);
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGI(TAG, "Camera streaming loop started");
  
  int control_cmd;
  TickType_t last_wake_time = xTaskGetTickCount();
  
  while (this->streaming_active_) {
    // Vérifier les commandes de contrôle
    if (xQueueReceive(this->control_queue_, &control_cmd, 0) == pdTRUE) {
      if (control_cmd == TASK_CONTROL_EXIT) {
        ESP_LOGI(TAG, "Received exit command");
        break;
      }
    }

    // Capturer une frame
    camera_fb_t* fb = esp_camera_fb_get();
    if (fb) {
      // Protéger l'accès au buffer
      if (xSemaphoreTake(this->frame_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        this->current_frame_buffer_ = fb->buf;
        this->current_frame_size_ = fb->len;
        
        // Appeler les callbacks
        this->on_frame_callbacks_.call(fb->buf, fb->len);
        
        xSemaphoreGive(this->frame_mutex_);
      }
      
      // Libérer le frame buffer
      esp_camera_fb_return(fb);
    } else {
      ESP_LOGW(TAG, "Camera capture failed in streaming");
    }

    // Respecter le taux de frames
    vTaskDelayUntil(&last_wake_time, pdMS_TO_TICKS(STREAM_FRAME_RATE_MS));
  }

  ESP_LOGI(TAG, "Camera streaming loop ended");
}

void Tab5Camera::cleanup_resources_() {
  if (this->streaming_active_) {
    this->stop_streaming();
  }

  if (this->camera_initialized_) {
    esp_camera_deinit();
    this->camera_initialized_ = false;
  }

  if (this->control_queue_) {
    vQueueDelete(this->control_queue_);
    this->control_queue_ = nullptr;
  }

  if (this->frame_mutex_) {
    vSemaphoreDelete(this->frame_mutex_);
    this->frame_mutex_ = nullptr;
  }

  this->current_frame_buffer_ = nullptr;
  this->current_frame_size_ = 0;
}

void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  ESP_LOGE(TAG, "Error: %s", error.c_str());
}

void Tab5Camera::clear_error_() {
  this->error_state_ = false;
  this->last_error_.clear();
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera:");
  ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Frame Size: %d", this->framesize_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: %d", this->pixel_format_);
  ESP_LOGCONFIG(TAG, "  JPEG Quality: %d", this->jpeg_quality_);
  ESP_LOGCONFIG(TAG, "  Vertical Flip: %s", this->vertical_flip_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Horizontal Mirror: %s", this->horizontal_mirror_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Status: %s", this->camera_initialized_ ? "Initialized" : "Not initialized");
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_active_ ? "Active" : "Inactive");
  
  if (this->has_error()) {
    ESP_LOGCONFIG(TAG, "  Error: %s", this->get_last_error().c_str());
  }
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

}  // namespace tab5_camera
}  // namespace esphome

#endif  // TAB5_CAMERA_USE_ESP32






