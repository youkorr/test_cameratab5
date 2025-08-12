#include "tab5_camera_v4l2.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"
#include "bsp/esp-bsp.h"  // Pour bsp_i2c_get_handle()

#ifdef USE_ESP32

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
  
  // Initialiser le système vidéo
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
  
  // Configuration CSI (adaptée de votre code)
  static esp_video_init_csi_config_t csi_config = {
    .sccb_config = {
      .init_sccb = false,
      .i2c_handle = nullptr,  // Sera défini plus tard
      .freq = 400000,         // Fréquence I2C
    },
    .reset_pin = -1,  // Pas de pin de reset
    .pwdn_pin = -1,   // Pas de pin de power down
  };
  
  // Récupérer le handle I2C du BSP (comme dans votre code)
  csi_config.sccb_config.i2c_handle = bsp_i2c_get_handle();
  if (!csi_config.sccb_config.i2c_handle) {
    ESP_LOGE(TAG, "Failed to get BSP I2C handle");
    return false;
  }
  
  // Configuration globale (comme dans votre code)
  this->video_config_.csi = &csi_config;
  this->video_config_.dvp = nullptr;
  this->video_config_.jpeg = nullptr;
  
  // Initialiser le système vidéo ESP32-P4
  esp_err_t ret = esp_video_init(&this->video_config_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(ret));
    return false;
  }
  
  ESP_LOGI(TAG, "Video system initialized successfully");
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
    ESP_LOGE(TAG, "Failed to open video device %s", dev_path);
    return false;
  }
  
  // Vérifier les capacités
  struct v4l2_capability capability;
  if (ioctl(this->camera_->fd, VIDIOC_QUERYCAP, &capability) != 0) {
    ESP_LOGE(TAG, "Failed to get device capabilities");
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
    ESP_LOGE(TAG, "Failed to get default format");
    return false;
  }
  
  ESP_LOGI(TAG, "Default format: %dx%d", 
           default_format.fmt.pix.width, 
           default_format.fmt.pix.height);
  
  // Définir le format souhaité si différent
  if (default_format.fmt.pix.pixelformat != format) {
    struct v4l2_format new_format = {
      .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
      .fmt = {
        .pix = {
          .width = default_format.fmt.pix.width,
          .height = default_format.fmt.pix.height,
          .pixelformat = format,
        }
      }
    };
    
    if (ioctl(this->camera_->fd, VIDIOC_S_FMT, &new_format) != 0) {
      ESP_LOGE(TAG, "Failed to set video format");
      return false;
    }
  }
  
  // Stocker les paramètres de format
  this->camera_->width = default_format.fmt.pix.width;
  this->camera_->height = default_format.fmt.pix.height;
  this->camera_->pixel_format = format;
  
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
    ESP_LOGE(TAG, "Failed to request buffers");
    return false;
  }
  
  // Mapper les buffers en mémoire
  for (size_t i = 0; i < BUFFER_COUNT; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    
    if (ioctl(this->camera_->fd, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to query buffer %zu", i);
      return false;
    }
    
    // Mapper le buffer (exactement comme dans votre code)
    this->camera_->buffer[i] = (uint8_t*)mmap(nullptr, buf.length, 
                                               PROT_READ | PROT_WRITE, 
                                               MAP_SHARED, 
                                               this->camera_->fd, buf.m.offset);
    
    if (this->camera_->buffer[i] == MAP_FAILED) {
      ESP_LOGE(TAG, "Failed to map buffer %zu", i);
      return false;
    }
    
    // Enqueuer le buffer pour utilisation
    if (ioctl(this->camera_->fd, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to queue buffer %zu", i);
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

bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }
  
  if (this->streaming_active_) {
    ESP_LOGW(TAG, "Streaming already active");
    return true;
  }
  
  ESP_LOGI(TAG, "Starting video streaming...");
  
  // Démarrer le streaming V4L2 (comme dans votre code)
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->camera_->fd, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "Failed to start V4L2 streaming");
    return false;
  }
  
  // Créer la tâche de streaming (adaptée de votre app_camera_display)
  this->streaming_active_ = true;
  
  BaseType_t result = xTaskCreate(
    Tab5Camera::streaming_task_,
    "tab5_streaming",
    8192,  // Même stack size que votre code
    this,
    5,     // Même priorité que votre code
    &this->streaming_task_handle_
  );
  
  if (result != pdPASS) {
    ESP_LOGE(TAG, "Failed to create streaming task");
    this->streaming_active_ = false;
    
    // Arrêter le streaming V4L2
    ioctl(this->camera_->fd, VIDIOC_STREAMOFF, &type);
    return false;
  }
  
  ESP_LOGI(TAG, "Video streaming started successfully");
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return true;
  }
  
  ESP_LOGI(TAG, "Stopping video streaming...");
  
  // Envoyer signal d'arrêt (comme dans votre code)
  int control = TASK_CONTROL_EXIT;
  xQueueSend(this->control_queue_, &control, portMAX_DELAY);
  
  // Attendre l'arrêt de la tâche
  uint32_t timeout = 0;
  while (this->streaming_active_ && timeout < 50) {
    vTaskDelay(pdMS_TO_TICKS(100));
    timeout++;
  }
  
  // Forcer l'arrêt si nécessaire
  if (this->streaming_active_ && this->streaming_task_handle_) {
    vTaskDelete(this->streaming_task_handle_);
    this->streaming_active_ = false;
  }
  
  this->streaming_task_handle_ = nullptr;
  
  // Arrêter le streaming V4L2
  enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  ioctl(this->camera_->fd, VIDIOC_STREAMOFF, &type);
  
  ESP_LOGI(TAG, "Video streaming stopped");
  return true;
}

// Tâche de streaming (adaptée de votre app_camera_display)
void Tab5Camera::streaming_task_(void *parameter) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(parameter);
  camera->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGI(TAG, "Streaming loop started");
  
  int task_control = 0;
  struct v4l2_buffer buf = {};
  
  while (this->streaming_active_) {
    // Attendre une frame (comme dans votre code)
    buf = {};  // Reset du buffer
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    
    if (ioctl(this->camera_->fd, VIDIOC_DQBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to dequeue buffer");
      break;
    }
    
    // Traiter la frame avec PPA si nécessaire
    // (Simplifié par rapport à votre code original)
    uint8_t* frame_data = this->camera_->buffer[buf.index];
    size_t frame_size = buf.bytesused > 0 ? buf.bytesused : this->camera_->buffer_size;
    
    // Appeler les callbacks utilisateur
    this->on_frame_callbacks_.call(frame_data, frame_size);
    
    // Remettre le buffer dans la queue (comme dans votre code)
    if (ioctl(this->camera_->fd, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to requeue buffer");
    }
    
    // Vérifier les commandes de contrôle (comme dans votre code)
    if (xQueueReceive(this->control_queue_, &task_control, 0) == pdTRUE) {
      if (task_control == TASK_CONTROL_EXIT) {
        break;
      } else if (task_control == TASK_CONTROL_PAUSE) {
        ESP_LOGI(TAG, "Streaming paused");
        // Attendre la commande de reprise ou d'arrêt
        if (xQueueReceive(this->control_queue_, &task_control, portMAX_DELAY) == pdTRUE) {
          if (task_control == TASK_CONTROL_EXIT) {
            break;
          }
          ESP_LOGI(TAG, "Streaming resumed");
        }
      }
    }
    
    // Délai comme dans votre code
    vTaskDelay(pdMS_TO_TICKS(10));
  }
  
  this->streaming_active_ = false;
  ESP_LOGI(TAG, "Streaming loop ended");
  vTaskDelete(nullptr);
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
    }
    
    free(this->camera_);
    this->camera_ = nullptr;
  }
  
  // Nettoyer la queue
  if (this->control_queue_) {
    vQueueDelete(this->control_queue_);
    this->control_queue_ = nullptr;
  }
  
  this->camera_initialized_ = false;
}

void Tab5Camera::set_error_(const std::string &error) {
  this->error_state_ = true;
  this->last_error_ = error;
  ESP_LOGE(TAG, "Camera error: %s", error.c_str());
}

void Tab5Camera::clear_error_() {
  this->error_state_ = false;
  this->last_error_.clear();
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Tab5 Camera (V4L2 API):");
  ESP_LOGCONFIG(TAG, "  Name: '%s'", this->name_.c_str());
  ESP_LOGCONFIG(TAG, "  Device Path: %s", DEVICE_PATH);
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  
  if (this->camera_initialized_) {
    ESP_LOGCONFIG(TAG, "  Camera FD: %d", this->camera_->fd);
    ESP_LOGCONFIG(TAG, "  Buffer Size: %zu bytes", this->camera_->buffer_size);
    ESP_LOGCONFIG(TAG, "  Status: Initialized");
  } else {
    ESP_LOGCONFIG(TAG, "  Status: Not initialized");
  }
  
  if (this->has_error()) {
    ESP_LOGCONFIG(TAG, "  Error: %s", this->get_last_error().c_str());
  }
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

#endif  // USE_ESP32
