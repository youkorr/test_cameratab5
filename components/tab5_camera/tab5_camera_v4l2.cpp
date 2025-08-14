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

// Définitions du périphérique caméra Tab5
#define CAM_DEV_PATH ESP_VIDEO_MIPI_CSI_DEVICE_NAME
#define MEMORY_TYPE V4L2_MEMORY_MMAP

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

  // Configurer le pin de reset de la caméra
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_pin_->digital_write(true);  // Reset actif
    delay(10);
    this->reset_pin_->digital_write(false); // Libérer le reset
    delay(50);
  }

  // Initialiser la caméra Tab5
  ESP_LOGI(TAG, "Initializing Tab5 camera...");
  
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
  // Configuration CSI basée sur le code Tab5
  esp_video_init_csi_config_t csi_config = {
    .sccb_config = {
      .init_sccb = false,
      .i2c_handle = bsp_i2c_get_handle(),
      .freq = 400000,
    },
    .reset_pin = -1,  // Géré par GPIO expander
    .pwdn_pin = -1,
  };

  esp_video_init_config_t cam_config = {
    .csi = &csi_config,
    .dvp = nullptr,
    .jpeg = nullptr,
  };

  // Initialiser le système vidéo ESP
  esp_err_t ret = esp_video_init(&cam_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to initialize video system: %s", esp_err_to_name(ret));
    return false;
  }

  // Ouvrir le périphérique vidéo
  this->video_fd_ = this->open_video_device_(this->pixel_format_);
  if (this->video_fd_ < 0) {
    ESP_LOGE(TAG, "Failed to open video device");
    return false;
  }

  // Configurer les buffers
  if (!this->setup_camera_buffers_()) {
    ESP_LOGE(TAG, "Failed to setup camera buffers");
    return false;
  }

  // Initialiser PPA pour les transformations d'image
  ppa_client_config_t ppa_config = {
    .oper_type = PPA_OPERATION_SRM,
    .max_pending_trans_num = 1,
  };
  
  esp_err_t ppa_ret = ppa_register_client(&ppa_config, &this->ppa_handle_);
  if (ppa_ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register PPA client: %s", esp_err_to_name(ppa_ret));
    return false;
  }

  return true;
}

int Tab5Camera::open_video_device_(tab5_pixformat_t format) {
  struct v4l2_format default_format;
  struct v4l2_capability capability;
  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  int fd = open(CAM_DEV_PATH, O_RDONLY);
  if (fd < 0) {
    ESP_LOGE(TAG, "Failed to open video device");
    return -1;
  }

  // Vérifier les capacités
  if (ioctl(fd, VIDIOC_QUERYCAP, &capability) != 0) {
    ESP_LOGE(TAG, "Failed to get device capabilities");
    close(fd);
    return -1;
  }

  ESP_LOGI(TAG, "Camera driver: %s", capability.driver);
  ESP_LOGI(TAG, "Camera card: %s", capability.card);

  // Obtenir le format par défaut
  memset(&default_format, 0, sizeof(struct v4l2_format));
  default_format.type = type;
  if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
    ESP_LOGE(TAG, "Failed to get format");
    close(fd);
    return -1;
  }

  // Configurer le format désiré
  uint32_t width, height;
  this->get_frame_dimensions_(&width, &height);

  struct v4l2_format format_config = {
    .type = type,
    .fmt = {
      .pix = {
        .width = width,
        .height = height,
        .pixelformat = format
      }
    }
  };

  if (ioctl(fd, VIDIOC_S_FMT, &format_config) != 0) {
    ESP_LOGE(TAG, "Failed to set format");
    close(fd);
    return -1;
  }

  ESP_LOGI(TAG, "Camera format set: %dx%d", width, height);
  return fd;
}

bool Tab5Camera::setup_camera_buffers_() {
  struct v4l2_requestbuffers req;
  memset(&req, 0, sizeof(req));
  req.count = BUFFER_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = MEMORY_TYPE;

  if (ioctl(this->video_fd_, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "Failed to request buffers");
    return false;
  }

  // Mapper les buffers en mémoire
  for (int i = 0; i < BUFFER_COUNT; i++) {
    struct v4l2_buffer buf;
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = MEMORY_TYPE;
    buf.index = i;

    if (ioctl(this->video_fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to query buffer %d", i);
      return false;
    }

    this->frame_buffers_[i] = (uint8_t*)mmap(nullptr, buf.length, 
                                             PROT_READ | PROT_WRITE, 
                                             MAP_SHARED, 
                                             this->video_fd_, 
                                             buf.m.offset);
    if (!this->frame_buffers_[i]) {
      ESP_LOGE(TAG, "Failed to map buffer %d", i);
      return false;
    }

    // Mettre le buffer en queue
    if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to queue buffer %d", i);
      return false;
    }
  }

  // Démarrer le streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "Failed to start streaming");
    return false;
  }

  return true;
}

bool Tab5Camera::configure_camera_() {
  // Configuration spécifique au Tab5 (ajuster selon les besoins)
  ESP_LOGI(TAG, "Camera configured successfully");
  return true;
}

void Tab5Camera::get_frame_dimensions_(uint32_t *width, uint32_t *height) {
  switch (this->framesize_) {
    case TAB5_FRAMESIZE_QVGA:
      *width = 320; *height = 240;
      break;
    case TAB5_FRAMESIZE_VGA:
      *width = 640; *height = 480;
      break;
    case TAB5_FRAMESIZE_HD:
      *width = 1280; *height = 720;
      break;
    case TAB5_FRAMESIZE_FULL_HD:
      *width = 1920; *height = 1080;
      break;
    default:
      *width = 1280; *height = 720;
  }
}

tab5_camera_config_t Tab5Camera::get_camera_config_() {
  uint32_t width, height;
  this->get_frame_dimensions_(&width, &height);
  
  return {
    .width = width,
    .height = height,
    .pixel_format = this->pixel_format_,
    .buffer_count = BUFFER_COUNT
  };
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  struct v4l2_buffer buf;
  memset(&buf, 0, sizeof(buf));
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = MEMORY_TYPE;

  // Attendre une frame
  if (ioctl(this->video_fd_, VIDIOC_DQBUF, &buf) != 0) {
    ESP_LOGE(TAG, "Failed to dequeue buffer");
    return false;
  }

  // Protéger l'accès au buffer de frame
  if (xSemaphoreTake(this->frame_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
    this->current_frame_buffer_ = this->frame_buffers_[buf.index];
    this->current_frame_size_ = buf.bytesused;
    
    // Appeler les callbacks
    this->on_frame_callbacks_.call(this->current_frame_buffer_, this->current_frame_size_);
    
    xSemaphoreGive(this->frame_mutex_);
  }

  // Remettre le buffer en queue
  if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
    ESP_LOGE(TAG, "Failed to queue buffer");
  }
  
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
  struct v4l2_buffer buf;
  
  while (this->streaming_active_) {
    // Vérifier les commandes de contrôle
    if (xQueueReceive(this->control_queue_, &control_cmd, 0) == pdTRUE) {
      if (control_cmd == TASK_CONTROL_EXIT) {
        ESP_LOGI(TAG, "Received exit command");
        break;
      }
    }

    // Capturer une frame
    memset(&buf, 0, sizeof(buf));
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = MEMORY_TYPE;

    if (ioctl(this->video_fd_, VIDIOC_DQBUF, &buf) == 0) {
      // Protéger l'accès au buffer
      if (xSemaphoreTake(this->frame_mutex_, pdMS_TO_TICKS(10)) == pdTRUE) {
        this->current_frame_buffer_ = this->frame_buffers_[buf.index];
        this->current_frame_size_ = buf.bytesused;
        
        // Appeler les callbacks
        this->on_frame_callbacks_.call(this->current_frame_buffer_, this->current_frame_size_);
        
        xSemaphoreGive(this->frame_mutex_);
      }
      
      // Remettre le buffer en queue
      if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
        ESP_LOGW(TAG, "Failed to requeue buffer");
      }
    } else {
      ESP_LOGW(TAG, "Failed to capture frame");
    }

    // Respecter le taux de frames
    vTaskDelay(pdMS_TO_TICKS(STREAM_FRAME_RATE_MS));
  }

  ESP_LOGI(TAG, "Camera streaming loop ended");
}

void Tab5Camera::cleanup_resources_() {
  if (this->streaming_active_) {
    this->stop_streaming();
  }

  if (this->ppa_handle_) {
    ppa_unregister_client(this->ppa_handle_);
    this->ppa_handle_ = nullptr;
  }

  // Arrêter le streaming
  if (this->video_fd_ >= 0) {
    int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(this->video_fd_, VIDIOC_STREAMOFF, &type);
    
    // Démapper les buffers
    for (int i = 0; i < BUFFER_COUNT; i++) {
      if (this->frame_buffers_[i]) {
        munmap(this->frame_buffers_[i], 0); // Size will be handled by system
        this->frame_buffers_[i] = nullptr;
      }
    }
    
    close(this->video_fd_);
    this->video_fd_ = -1;
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
  this->camera_initialized_ = false;
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
  ESP_LOGCONFIG(TAG, "  Pixel Format: 0x%x", this->pixel_format_);
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






