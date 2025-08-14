#include "tab5_camera_v4l2.h"
#include "esphome/core/log.h"
#include "esphome/core/helpers.h"

#ifdef USE_ESP32

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

#define TASK_CONTROL_PAUSE  0
#define TASK_CONTROL_RESUME 1
#define TASK_CONTROL_EXIT   2

// Définitions du périphérique caméra Tab5
#define CAM_DEV_PATH "/dev/video0"  // Path V4L2 standard pour ESP32-P4
#define MEMORY_TYPE V4L2_MEMORY_MMAP

Tab5Camera::~Tab5Camera() {
  this->cleanup_resources_();
}

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");
  this->clear_error_();

  // Vérifier que l'I2C est disponible
  if (!this->parent_->is_ready()) {
    this->set_error_("I2C bus not ready");
    this->mark_failed();
    return;
  }

  // Initialiser le BSP Tab5 d'abord
  if (!this->init_tab5_bsp_()) {
    this->set_error_("Failed to initialize Tab5 BSP");
    this->mark_failed();
    return;
  }

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

  // Configurer le pin de reset de la caméra via GPIO ou IO expander
  if (this->reset_pin_ != nullptr) {
    this->reset_pin_->setup();
    this->reset_camera_via_io_expander_();
  } else {
    // Utiliser le reset via IO expander du BSP
    this->reset_camera_via_io_expander_();
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

bool Tab5Camera::init_tab5_bsp_() {
  ESP_LOGI(TAG, "Initializing Tab5 BSP...");
  
  // Initialiser l'I2C du BSP si pas déjà fait
  esp_err_t ret = bsp_i2c_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init BSP I2C: %s", esp_err_to_name(ret));
    return false;
  }

  // Initialiser les IO expanders
  i2c_master_bus_handle_t i2c_handle = bsp_i2c_get_handle();
  if (i2c_handle == nullptr) {
    ESP_LOGE(TAG, "Failed to get I2C handle");
    return false;
  }
  
  bsp_io_expander_pi4ioe_init(i2c_handle);
  
  // Initialiser l'oscillateur caméra
  ret = bsp_cam_osc_init();
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to init camera oscillator: %s", esp_err_to_name(ret));
    return false;
  }

  ESP_LOGI(TAG, "Tab5 BSP initialized successfully");
  return true;
}

bool Tab5Camera::reset_camera_via_io_expander_() {
  ESP_LOGI(TAG, "Resetting camera via IO expander");
  
  // Le reset de la caméra est géré par le PI4IOE (P6 de PI4IOE1)
  // Dans le BSP, c'est déjà configuré dans l'init
  // On fait juste un reset pulse
  
  vTaskDelay(pdMS_TO_TICKS(10));
  ESP_LOGI(TAG, "Camera reset completed");
  return true;
}

bool Tab5Camera::init_camera_() {
  ESP_LOGI(TAG, "Initializing camera subsystem...");
  
  // Configuration CSI basée sur le code Tab5
  esp_video_init_csi_config_t csi_config = {
    .sccb_config = {
      .init_sccb = false,  // Ne pas réinitialiser l'I2C
      .i2c_handle = bsp_i2c_get_handle(),
      .freq = 400000,
    },
    .reset_pin = -1,  // Géré par GPIO expander
    .pwdn_pin = -1,   // Pas utilisé sur Tab5
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

  // Attendre que le système soit prêt
  vTaskDelay(pdMS_TO_TICKS(100));

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
    ESP_LOGW(TAG, "Failed to register PPA client: %s", esp_err_to_name(ppa_ret));
    // Ce n'est pas critique, on peut continuer sans PPA
    this->ppa_handle_ = nullptr;
  }

  return true;
}

int Tab5Camera::open_video_device_(tab5_pixformat_t format) {
  ESP_LOGI(TAG, "Opening video device: %s", CAM_DEV_PATH);
  
  struct v4l2_format default_format;
  struct v4l2_capability capability;
  const int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;

  int fd = open(CAM_DEV_PATH, O_RDWR); // Utiliser O_RDWR au lieu de O_RDONLY
  if (fd < 0) {
    ESP_LOGE(TAG, "Failed to open video device %s: errno=%d", CAM_DEV_PATH, errno);
    return -1;
  }

  // Vérifier les capacités
  memset(&capability, 0, sizeof(capability));
  if (ioctl(fd, VIDIOC_QUERYCAP, &capability) != 0) {
    ESP_LOGE(TAG, "Failed to get device capabilities: errno=%d", errno);
    close(fd);
    return -1;
  }

  ESP_LOGI(TAG, "Camera driver: %s", capability.driver);
  ESP_LOGI(TAG, "Camera card: %s", capability.card);

  // Obtenir le format par défaut
  memset(&default_format, 0, sizeof(struct v4l2_format));
  default_format.type = type;
  if (ioctl(fd, VIDIOC_G_FMT, &default_format) != 0) {
    ESP_LOGE(TAG, "Failed to get format: errno=%d", errno);
    close(fd);
    return -1;
  }

  // Configurer le format désiré
  uint32_t width, height;
  this->get_frame_dimensions_(&width, &height);

  struct v4l2_format format_config = {0};
  format_config.type = type;
  format_config.fmt.pix.width = width;
  format_config.fmt.pix.height = height;
  format_config.fmt.pix.pixelformat = format;
  format_config.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(fd, VIDIOC_S_FMT, &format_config) != 0) {
    ESP_LOGE(TAG, "Failed to set format: errno=%d", errno);
    close(fd);
    return -1;
  }

  // Calculer la taille du buffer
  this->buffer_size_ = format_config.fmt.pix.sizeimage;
  if (this->buffer_size_ == 0) {
    // Calculer manuellement si pas fourni
    this->buffer_size_ = width * height * 2; // Assume RGB565
  }

  ESP_LOGI(TAG, "Camera format set: %dx%d, buffer size: %d", width, height, this->buffer_size_);
  return fd;
}

bool Tab5Camera::setup_camera_buffers_() {
  ESP_LOGI(TAG, "Setting up camera buffers...");
  
  struct v4l2_requestbuffers req = {0};
  req.count = BUFFER_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = MEMORY_TYPE;

  if (ioctl(this->video_fd_, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "Failed to request buffers: errno=%d", errno);
    return false;
  }

  ESP_LOGI(TAG, "Requested %d buffers, got %d", BUFFER_COUNT, req.count);

  // Mapper les buffers en mémoire
  for (uint32_t i = 0; i < req.count && i < BUFFER_COUNT; i++) {
    struct v4l2_buffer buf = {0};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = MEMORY_TYPE;
    buf.index = i;

    if (ioctl(this->video_fd_, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to query buffer %d: errno=%d", i, errno);
      return false;
    }

    this->frame_buffers_[i] = (uint8_t*)mmap(nullptr, buf.length, 
                                             PROT_READ | PROT_WRITE, 
                                             MAP_SHARED, 
                                             this->video_fd_, 
                                             buf.m.offset);
    if (this->frame_buffers_[i] == MAP_FAILED) {
      ESP_LOGE(TAG, "Failed to map buffer %d: errno=%d", i, errno);
      this->frame_buffers_[i] = nullptr;
      return false;
    }

    ESP_LOGD(TAG, "Buffer %d mapped at %p, size=%d", i, this->frame_buffers_[i], buf.length);

    // Mettre le buffer en queue
    if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to queue buffer %d: errno=%d", i, errno);
      return false;
    }
  }

  // Démarrer le streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->video_fd_, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "Failed to start streaming: errno=%d", errno);
    return false;
  }

  ESP_LOGI(TAG, "Camera streaming started");
  return true;
}

bool Tab5Camera::configure_camera_() {
  ESP_LOGI(TAG, "Configuring camera settings...");
  
  // Configuration des contrôles caméra si supportés
  struct v4l2_control ctrl = {0};
  
  // Flip vertical
  if (this->vertical_flip_) {
    ctrl.id = V4L2_CID_VFLIP;
    ctrl.value = 1;
    if (ioctl(this->video_fd_, VIDIOC_S_CTRL, &ctrl) != 0) {
      ESP_LOGW(TAG, "Failed to set vertical flip");
    }
  }
  
  // Mirror horizontal
  if (this->horizontal_mirror_) {
    ctrl.id = V4L2_CID_HFLIP;
    ctrl.value = 1;
    if (ioctl(this->video_fd_, VIDIOC_S_CTRL, &ctrl) != 0) {
      ESP_LOGW(TAG, "Failed to set horizontal mirror");
    }
  }

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
    .pixel_format = (uint32_t)this->pixel_format_,
    .buffer_count = BUFFER_COUNT
  };
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    ESP_LOGE(TAG, "Camera not initialized");
    return false;
  }

  struct v4l2_buffer buf = {0};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = MEMORY_TYPE;

  // Attendre une frame avec timeout
  fd_set fds;
  struct timeval timeout;
  FD_ZERO(&fds);
  FD_SET(this->video_fd_, &fds);
  timeout.tv_sec = 1;  // 1 seconde de timeout
  timeout.tv_usec = 0;

  int ret = select(this->video_fd_ + 1, &fds, nullptr, nullptr, &timeout);
  if (ret <= 0) {
    ESP_LOGE(TAG, "Timeout waiting for frame");
    return false;
  }

  // Récupérer le buffer
  if (ioctl(this->video_fd_, VIDIOC_DQBUF, &buf) != 0) {
    ESP_LOGE(TAG, "Failed to dequeue buffer: errno=%d", errno);
    return false;
  }

  // Protéger l'accès au buffer de frame
  if (xSemaphoreTake(this->frame_mutex_, pdMS_TO_TICKS(100)) == pdTRUE) {
    this->current_frame_buffer_ = this->frame_buffers_[buf.index];
    this->current_frame_size_ = buf.bytesused;
    
    ESP_LOGD(TAG, "Frame captured: buffer=%d, size=%d", buf.index, buf.bytesused);
    
    // Appeler les callbacks
    this->on_frame_callbacks_.call(this->current_frame_buffer_, this->current_frame_size_);
    
    xSemaphoreGive(this->frame_mutex_);
  }

  // Remettre le buffer en queue
  if (ioctl(this->video_fd_, VIDIOC_QBUF, &buf) != 0) {
    ESP_LOGE(TAG, "Failed to queue buffer: errno=%d", errno);
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
  fd_set fds;
  struct timeval timeout;
  
  while (this->streaming_active_) {
    // Vérifier les commandes de contrôle
    if (xQueueReceive(this->control_queue_, &control_cmd, 0) == pdTRUE) {
      if (control_cmd == TASK_CONTROL_EXIT) {
        ESP_LOGI(TAG, "Received exit command");
        break;
      }
    }

    // Préparer select pour attendre une frame
    FD_ZERO(&fds);
    FD_SET(this->video_fd_, &fds);
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000;  // 100ms timeout

    int ret = select(this->video_fd_ + 1, &fds, nullptr, nullptr, &timeout);
    if (ret > 0 && FD_ISSET(this->video_fd_, &fds)) {
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
          ESP_LOGW(TAG, "Failed to requeue buffer: errno=%d", errno);
        }
      } else {
        ESP_LOGW(TAG, "Failed to capture frame: errno=%d", errno);
      }
    }

    // Respecter le taux de frames
    vTaskDelay(pdMS_TO_TICKS(STREAM_FRAME_RATE_MS));
  }

  ESP_LOGI(TAG, "Camera streaming loop ended");
}

void Tab5Camera::cleanup_resources_() {
  ESP_LOGI(TAG, "Cleaning up camera resources...");
  
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
      if (this->frame_buffers_[i] && this->frame_buffers_[i] != MAP_FAILED) {
        munmap(this->frame_buffers_[i], this->buffer_size_);
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
  
  ESP_LOGI(TAG, "Camera resources cleaned up");
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
  ESP_LOGCONFIG(TAG, "  I2C Address: 0x%02X", this->address_);
  ESP_LOGCONFIG(TAG, "  Frame Size: %d", this->framesize_);
  ESP_LOGCONFIG(TAG, "  Pixel Format: 0x%x", this->pixel_format_);
  ESP_LOGCONFIG(TAG, "  Vertical Flip: %s", this->vertical_flip_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Horizontal Mirror: %s", this->horizontal_mirror_ ? "YES" : "NO");
  ESP_LOGCONFIG(TAG, "  Status: %s", this->camera_initialized_ ? "Initialized" : "Not initialized");
  ESP_LOGCONFIG(TAG, "  Streaming: %s", this->streaming_active_ ? "Active" : "Inactive");
  
  uint32_t width, height;
  this->get_frame_dimensions_(&width, &height);
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", width, height);
  ESP_LOGCONFIG(TAG, "  Buffer Size: %d bytes", this->buffer_size_);
  
  if (this->has_error()) {
    ESP_LOGCONFIG(TAG, "  Error: %s", this->get_last_error().c_str());
  }
  
  LOG_I2C_DEVICE(this);
}

float Tab5Camera::get_setup_priority() const {
  return setup_priority::HARDWARE - 1.0f;
}

}  // namespace tab5_camera
}  // namespace esphome

#endif  // USE_ESP32





