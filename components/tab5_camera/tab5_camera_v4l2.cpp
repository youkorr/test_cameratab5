#include "tab5_camera_v4l2.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

// CORRECTION : S'assurer que USE_ESP32 est défini avec une valeur
#ifndef USE_ESP32
#define USE_ESP32 1
#endif

#ifdef USE_ESP32

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

// Constantes de contrôle de tâche (de votre code)
#define TASK_CONTROL_PAUSE  0
#define TASK_CONTROL_RESUME 1
#define TASK_CONTROL_EXIT   2

Tab5Camera::~Tab5Camera() {
  this->cleanup_resources_();
}

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera (V4L2 API)...");
  
  this->clear_error_();
  
  // Créer la queue de contrôle (comme dans votre code)
  this->control_queue_ = xQueueCreate(10, sizeof(int));
  if (!this->control_queue_) {
    this->set_error_("Failed to create control queue");
    this->mark_failed();
    return;
  }
  
  // Initialiser le système vidéo avec ESPHome I2C
  if (!this->init_video_system_()) {
    this->set_error_("Video system initialization failed");
    this->mark_failed();
    return;
  }
  
  // Ouvrir le device vidéo
  if (!this->open_video_device_(DEVICE_PATH, TAB5_VIDEO_FMT_RGB565)) {
    this->set_error_("Failed to open video device");
    this->mark_failed();
    return;
  }
  
  // Configurer les buffers
  if (!this->setup_camera_buffers_()) {
    this->set_error_("Failed to setup camera buffers");
    this->mark_failed();
    return;
  }
  
  // Initialiser le PPA (Pixel Processing Accelerator)
  if (!this->init_ppa_processor_()) {
    this->set_error_("Failed to initialize PPA processor");
    this->mark_failed();
    return;
  }
  
  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Tab5 Camera initialized successfully");
}

bool Tab5Camera::init_video_system_() {
  ESP_LOGI(TAG, "Initializing Tab5 video system...");
  
  // Configuration CSI utilisant l'I2C d'ESPHome
  static esp_video_init_csi_config_t csi_config = {
    .sccb_config = {
      .init_sccb = true,  // Laisser le système initialiser l'I2C
      .i2c_handle = nullptr,  // Sera configuré automatiquement
      .freq = 400000,         // Même fréquence que votre config ESPHome
    },
    .reset_pin = -1,  // Pas de pin de reset
    .pwdn_pin = -1,   // Pas de pin de power down
  };
  
  // Configuration globale
  this->video_config_.csi = &csi_config;
  this->video_config_.dvp = nullptr;
  this->video_config_.jpeg = nullptr;
  
  // Initialiser le système vidéo ESP32-P4
  esp_err_t ret = esp_video_init(&this->video_config_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(ret));
    
    // Try alternative initialization without BSP dependency
    return this->init_video_system_fallback_();
  }
  
  ESP_LOGI(TAG, "Video system initialized successfully");
  return true;
}

bool Tab5Camera::init_video_system_fallback_() {
  ESP_LOGI(TAG, "Trying fallback video system initialization...");
  
  // Configuration CSI simplifiée
  static esp_video_init_csi_config_t csi_config = {
    .sccb_config = {
      .init_sccb = true,
      .i2c_handle = nullptr,
      .freq = 400000,
    },
    .reset_pin = -1,
    .pwdn_pin = -1,
  };
  
  // Créer un handle I2C manuel si nécessaire
  i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_31,  // Même que votre config ESPHome
    .scl_io_num = GPIO_NUM_32,  // Même que votre config ESPHome
    .clk_source = I2C_CLK_SRC_DEFAULT,
    .glitch_ignore_cnt = 7,
    .intr_priority = 0,
    .trans_queue_depth = 0,
    .flags = {
      .enable_internal_pullup = true,
    }
  };
  
  esp_err_t ret = i2c_new_master_bus(&bus_config, &this->i2c_bus_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to create I2C master bus: %s", esp_err_to_name(ret));
    return false;
  }
  
  csi_config.sccb_config.i2c_handle = this->i2c_bus_handle_;
  csi_config.sccb_config.init_sccb = false;  // Nous gérons l'I2C nous-mêmes
  
  this->video_config_.csi = &csi_config;
  this->video_config_.dvp = nullptr;
  this->video_config_.jpeg = nullptr;
  
  ret = esp_video_init(&this->video_config_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Fallback esp_video_init failed: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "Fallback video system initialized successfully");
  return true;
}

bool Tab5Camera::open_video_device_(const char* dev_path, tab5_fmt_t format) {
  ESP_LOGI(TAG, "Opening video device: %s", dev_path);
  
  // Allouer la structure caméra
  this->camera_ = (tab5_cam_t*)calloc(1, sizeof(tab5_cam_t));
  if (!this->camera_) {
    ESP_LOGE(TAG, "Failed to allocate camera structure");
    return false;
  }
  
  // Ouvrir le device (exactement comme dans votre code)
  this->camera_->fd = open(dev_path, O_RDONLY);
  if (this->camera_->fd < 0) {
    ESP_LOGE(TAG, "Failed to open video device %s: %s", dev_path, strerror(errno));
    return false;
  }
  
  // Vérifier les capacités
  struct v4l2_capability capability;
  if (ioctl(this->camera_->fd, VIDIOC_QUERYCAP, &capability) != 0) {
    ESP_LOGE(TAG, "Failed to get device capabilities: %s", strerror(errno));
    return false;
  }
  
  ESP_LOGI(TAG, "Video device info:");
  ESP_LOGI(TAG, "  Driver: %s", capability.driver);
  ESP_LOGI(TAG, "  Card: %s", capability.card);
  ESP_LOGI(TAG, "  Version: %d.%d.%d", 
           (capability.version >> 16) & 0xFF,
           (capability.version >> 8) & 0xFF, 
           capability.version & 0xFF);
  
  // Obtenir le format par défaut
  struct v4l2_format default_format = {};
  default_format.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  
  if (ioctl(this->camera_->fd, VIDIOC_G_FMT, &default_format) != 0) {
    ESP_LOGE(TAG, "Failed to get default format: %s", strerror(errno));
    return false;
  }
  
  ESP_LOGI(TAG, "Default format: %dx%d", 
           default_format.fmt.pix.width, 
           default_format.fmt.pix.height);
  
  // Définir le format souhaité
  struct v4l2_format new_format = {
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .fmt = {
      .pix = {
        .width = this->frame_width_,   // Utiliser la résolution configurée
        .height = this->frame_height_, // Utiliser la résolution configurée
        .pixelformat = format,
        .field = V4L2_FIELD_NONE,
      }
    }
  };
  
  if (ioctl(this->camera_->fd, VIDIOC_S_FMT, &new_format) != 0) {
    ESP_LOGW(TAG, "Failed to set video format, using default: %s", strerror(errno));
    // Utiliser le format par défaut
    new_format = default_format;
  }
  
  // Stocker les paramètres de format
  this->camera_->width = new_format.fmt.pix.width;
  this->camera_->height = new_format.fmt.pix.height;
  this->camera_->pixel_format = new_format.fmt.pix.pixelformat;
  
  ESP_LOGI(TAG, "Video device opened: %dx%d, format=0x%08X", 
           this->camera_->width, this->camera_->height, this->camera_->pixel_format);
  
  return true;
}

bool Tab5Camera::setup_camera_buffers_() {
  ESP_LOGI(TAG, "Setting up camera buffers...");
  
  // Demander des buffers (exactement comme dans votre code)
  struct v4l2_requestbuffers req = {};
  req.count = BUFFER_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;
  
  if (ioctl(this->camera_->fd, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "Failed to request buffers: %s", strerror(errno));
    return false;
  }
  
  // Mapper les buffers en mémoire
  for (size_t i = 0; i < BUFFER_COUNT; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    
    if (ioctl(this->camera_->fd, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to query buffer %zu: %s", i, strerror(errno));
      return false;
    }
    
    // Mapper le buffer (exactement comme dans votre code)
    this->camera_->buffer[i] = (uint8_t*)mmap(nullptr, buf.length, 
                                               PROT_READ | PROT_WRITE, 
                                               MAP_SHARED, 
                                               this->camera_->fd, buf.m.offset);
    
    if (this->camera_->buffer[i] == MAP_FAILED) {
      ESP_LOGE(TAG, "Failed to map buffer %zu: %s", i, strerror(errno));
      return false;
    }
    
    // Enqueuer le buffer pour utilisation
    if (ioctl(this->camera_->fd, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to queue buffer %zu: %s", i, strerror(errno));
      return false;
    }
    
    // Stocker la taille du buffer
    this->camera_->buffer_size = buf.length;
  }
  
  ESP_LOGI(TAG, "Camera buffers setup complete (%zu buffers, %zu bytes each)", 
           BUFFER_COUNT, this->camera_->buffer_size);
  
  return true;
}

bool Tab5Camera::init_ppa_processor_() {
  ESP_LOGI(TAG, "Initializing PPA processor...");
  
  // Configuration PPA (adaptée de votre code)
  ppa_client_config_t ppa_config = {
    .oper_type = PPA_OPERATION_SRM,  // Scale, Rotate, Mirror
    .max_pending_trans_num = 1,
  };
  
  esp_err_t ret = ppa_register_client(&ppa_config, &this->ppa_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register PPA client: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "PPA processor initialized successfully");
  return true;
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  // Implementation pour prendre une photo
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;

  // Démarrer le streaming si pas déjà fait
  int stream_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->camera_->fd, VIDIOC_STREAMON, &stream_type) != 0) {
    ESP_LOGE(TAG, "Failed to start streaming: %s", strerror(errno));
    return false;
  }

  // Capturer un frame
  if (ioctl(this->camera_->fd, VIDIOC_DQBUF, &buf) != 0) {
    ESP_LOGE(TAG, "Failed to dequeue buffer: %s", strerror(errno));
    return false;
  }

  // Traiter le frame via les callbacks
  this->on_frame_callbacks_.call(this->camera_->buffer[buf.index], buf.bytesused);

  // Remettre le buffer en queue
  if (ioctl(this->camera_->fd, VIDIOC_QBUF, &buf) != 0) {
    ESP_LOGE(TAG, "Failed to requeue buffer: %s", strerror(errno));
  }

  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_ || this->streaming_active_) {
    return false;
  }

  // Créer la tâche de streaming
  xTaskCreate(streaming_task_, "camera_stream", 4096, this, 5, &this->streaming_task_handle_);
  this->streaming_active_ = true;
  
  ESP_LOGI(TAG, "Camera streaming started");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return false;
  }

  // Envoyer signal d'arrêt
  int control_cmd = TASK_CONTROL_EXIT;
  xQueueSend(this->control_queue_, &control_cmd, 0);
  
  // Attendre l'arrêt de la tâche
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
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGI(TAG, "Starting streaming loop");
  
  // Démarrer le streaming V4L2
  int stream_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->camera_->fd, VIDIOC_STREAMON, &stream_type) != 0) {
    ESP_LOGE(TAG, "Failed to start streaming: %s", strerror(errno));
    return;
  }

  struct v4l2_buffer buf = {};
  int control_cmd;

  while (this->streaming_active_) {
    // Vérifier les commandes de contrôle
    if (xQueueReceive(this->control_queue_, &control_cmd, 0) == pdTRUE) {
      if (control_cmd == TASK_CONTROL_EXIT) {
        break;
      }
    }

    // Capturer un frame
    buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(this->camera_->fd, VIDIOC_DQBUF, &buf) == 0) {
      // Traiter le frame
      this->on_frame_callbacks_.call(this->camera_->buffer[buf.index], buf.bytesused);
      
      // Remettre le buffer en queue
      ioctl(this->camera_->fd, VIDIOC_QBUF, &buf);
    }

    vTaskDelay(pdMS_TO_TICKS(33)); // ~30 FPS
  }

  // Arrêter le streaming
  ioctl(this->camera_->fd, VIDIOC_STREAMOFF, &stream_type);
  ESP_LOGI(TAG, "Streaming loop ended");
}

void Tab5Camera::cleanup_resources_() {
  // Arrêter le streaming
  if (this->streaming_active_) {
    this->stop_streaming();
  }
  
  // Nettoyer le PPA
  if (this->ppa_handle_) {
    ppa_unregister_client(this->ppa_handle_);
    this->ppa_handle_ = nullptr;
  }
  
  // Nettoyer la caméra
  if (this->camera_) {
    // Désmapper les buffers
    for (size_t i = 0; i < BUFFER_COUNT; i++) {
      if (this->camera_->buffer[i] && this->camera_->buffer[i] != MAP_FAILED) {
        munmap(this->camera_->buffer[i], this->camera_->buffer_size);
      }
    }
    
    // Fermer le device
    if (this->camera_->fd >= 0) {
      close(this->camera_->fd);
