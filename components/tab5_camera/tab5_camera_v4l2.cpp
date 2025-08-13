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

// Memory mapping implementation for ESP32
static void* esp_mmap(void* addr, size_t length, int prot, int flags, int fd, off_t offset) {
  (void)addr; (void)prot; (void)flags; (void)fd; (void)offset;
  void* ptr = heap_caps_malloc(length, MALLOC_CAP_SPIRAM | MALLOC_CAP_8BIT);
  if (ptr == nullptr) {
    ESP_LOGE(TAG, "Failed to allocate memory for buffer");
    return MAP_FAILED;
  }
  return ptr;
}

static int esp_munmap(void* addr, size_t length) {
  (void)length;
  if (addr != MAP_FAILED) {
    heap_caps_free(addr);
  }
  return 0;
}

#define mmap esp_mmap
#define munmap esp_munmap
#define MAP_FAILED ((void*)-1)

Tab5Camera::~Tab5Camera() {
  this->cleanup_resources_();
}

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");
  this->clear_error_();

  this->control_queue_ = xQueueCreate(10, sizeof(int));
  if (!this->control_queue_) {
    this->set_error_("Failed to create control queue");
    this->mark_failed();
    return;
  }

  if (!this->init_video_system_()) {
    this->set_error_("Video system initialization failed");
    this->mark_failed();
    return;
  }

  if (!this->open_video_device_(DEVICE_PATH, TAB5_VIDEO_FMT_RGB565)) {
    this->set_error_("Failed to open video device");
    this->mark_failed();
    return;
  }

  if (!this->setup_camera_buffers_()) {
    this->set_error_("Failed to setup camera buffers");
    this->mark_failed();
    return;
  }

  if (!this->init_ppa_processor_()) {
    this->set_error_("Failed to initialize PPA processor");
    this->mark_failed();
    return;
  }

  this->camera_initialized_ = true;
  ESP_LOGI(TAG, "Tab5 Camera initialized successfully");
}

bool Tab5Camera::init_video_system_() {
  ESP_LOGI(TAG, "Initializing video system...");

  static esp_video_init_csi_config_t csi_config = {
    .sccb_config = {
      .init_sccb = true,
      .i2c_handle = nullptr,
      .freq = 400000,
    },
    .reset_pin = -1,
    .pwdn_pin = -1,
  };

  this->video_config_.csi = &csi_config;
  this->video_config_.dvp = nullptr;
  this->video_config_.jpeg = nullptr;

  esp_err_t ret = esp_video_init(&this->video_config_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init failed: %s", esp_err_to_name(ret));
    return this->init_video_system_fallback_();
  }

  return true;
}

bool Tab5Camera::init_video_system_fallback_() {
  ESP_LOGI(TAG, "Trying fallback initialization...");

  i2c_master_bus_config_t bus_config = {
    .i2c_port = I2C_NUM_0,
    .sda_io_num = GPIO_NUM_31,
    .scl_io_num = GPIO_NUM_32,
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
    ESP_LOGE(TAG, "Failed to create I2C bus: %s", esp_err_to_name(ret));
    return false;
  }

  static esp_video_init_csi_config_t csi_config = {
    .sccb_config = {
      .init_sccb = false,
      .i2c_handle = this->i2c_bus_handle_,
      .freq = 400000,
    },
    .reset_pin = -1,
    .pwdn_pin = -1,
  };

  this->video_config_.csi = &csi_config;
  ret = esp_video_init(&this->video_config_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Fallback esp_video_init failed: %s", esp_err_to_name(ret));
    return false;
  }

  return true;
}

bool Tab5Camera::open_video_device_(const char* dev_path, tab5_fmt_t format) {
  this->camera_ = (tab5_cam_t*)calloc(1, sizeof(tab5_cam_t));
  if (!this->camera_) {
    ESP_LOGE(TAG, "Failed to allocate camera structure");
    return false;
  }

  this->camera_->fd = open(dev_path, O_RDONLY);
  if (this->camera_->fd < 0) {
    ESP_LOGE(TAG, "Failed to open video device: %s", strerror(errno));
    return false;
  }

  struct v4l2_format fmt = {
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .fmt = {
      .pix = {
        .width = this->frame_width_,
        .height = this->frame_height_,
        .pixelformat = format,
        .field = V4L2_FIELD_NONE,
      }
    }
  };

  if (ioctl(this->camera_->fd, VIDIOC_S_FMT, &fmt) != 0) {
    ESP_LOGE(TAG, "Failed to set video format: %s", strerror(errno));
    return false;
  }

  this->camera_->width = fmt.fmt.pix.width;
  this->camera_->height = fmt.fmt.pix.height;
  this->camera_->pixel_format = fmt.fmt.pix.pixelformat;

  return true;
}

bool Tab5Camera::setup_camera_buffers_() {
  struct v4l2_requestbuffers req = {
    .count = BUFFER_COUNT,
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .memory = V4L2_MEMORY_MMAP
  };

  if (ioctl(this->camera_->fd, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "Failed to request buffers: %s", strerror(errno));
    return false;
  }

  for (size_t i = 0; i < BUFFER_COUNT; i++) {
    struct v4l2_buffer buf = {
      .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
      .memory = V4L2_MEMORY_MMAP,
      .index = i
    };

    if (ioctl(this->camera_->fd, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to query buffer: %s", strerror(errno));
      return false;
    }

    this->camera_->buffer[i] = (uint8_t*)mmap(nullptr, buf.length, 
                                             PROT_READ | PROT_WRITE,
                                             MAP_SHARED,
                                             this->camera_->fd, buf.m.offset);
    if (this->camera_->buffer[i] == MAP_FAILED) {
      ESP_LOGE(TAG, "Failed to map buffer");
      return false;
    }

    if (ioctl(this->camera_->fd, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "Failed to queue buffer: %s", strerror(errno));
      return false;
    }

    this->camera_->buffer_size = buf.length;
  }

  return true;
}

bool Tab5Camera::init_ppa_processor_() {
  ppa_client_config_t ppa_config = {
    .oper_type = PPA_OPERATION_SRM,
    .max_pending_trans_num = 1,
  };

  esp_err_t ret = ppa_register_client(&ppa_config, &this->ppa_handle_);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "Failed to register PPA client: %s", esp_err_to_name(ret));
    return false;
  }

  return true;
}

bool Tab5Camera::take_snapshot() {
  if (!this->camera_initialized_) {
    return false;
  }

  struct v4l2_buffer buf = {
    .type = V4L2_BUF_TYPE_VIDEO_CAPTURE,
    .memory = V4L2_MEMORY_MMAP
  };

  int stream_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->camera_->fd, VIDIOC_STREAMON, &stream_type) != 0) {
    ESP_LOGE(TAG, "Failed to start streaming: %s", strerror(errno));
    return false;
  }

  if (ioctl(this->camera_->fd, VIDIOC_DQBUF, &buf) != 0) {
    ESP_LOGE(TAG, "Failed to dequeue buffer: %s", strerror(errno));
    return false;
  }

  this->on_frame_callbacks_.call(this->camera_->buffer[buf.index], buf.bytesused);

  if (ioctl(this->camera_->fd, VIDIOC_QBUF, &buf) != 0) {
    ESP_LOGE(TAG, "Failed to requeue buffer: %s", strerror(errno));
  }

  return true;
}

bool Tab5Camera::start_streaming() {
  if (!this->camera_initialized_ || this->streaming_active_) {
    return false;
  }

  xTaskCreate(streaming_task_, "camera_stream", 4096, this, 5, &this->streaming_task_handle_);
  this->streaming_active_ = true;
  return true;
}

bool Tab5Camera::stop_streaming() {
  if (!this->streaming_active_) {
    return false;
  }

  int control_cmd = TASK_CONTROL_EXIT;
  xQueueSend(this->control_queue_, &control_cmd, 0);

  if (this->streaming_task_handle_) {
    vTaskDelete(this->streaming_task_handle_);
    this->streaming_task_handle_ = nullptr;
  }

  this->streaming_active_ = false;
  return true;
}

void Tab5Camera::streaming_task_(void *parameter) {
  Tab5Camera *camera = static_cast<Tab5Camera*>(parameter);
  camera->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
  int stream_type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->camera_->fd, VIDIOC_STREAMON, &stream_type) != 0) {
    ESP_LOGE(TAG, "Failed to start streaming: %s", strerror(errno));
    return;
  }

  struct v4l2_buffer buf = {};
  int control_cmd;

  while (this->streaming_active_) {
    if (xQueueReceive(this->control_queue_, &control_cmd, 0) == pdTRUE) {
      if (control_cmd == TASK_CONTROL_EXIT) {
        break;
      }
    }

    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;

    if (ioctl(this->camera_->fd, VIDIOC_DQBUF, &buf) == 0) {
      this->on_frame_callbacks_.call(this->camera_->buffer[buf.index], buf.bytesused);
      ioctl(this->camera_->fd, VIDIOC_QBUF, &buf);
    }

    vTaskDelay(pdMS_TO_TICKS(33));
  }

  ioctl(this->camera_->fd, VIDIOC_STREAMOFF, &stream_type);
}

void Tab5Camera::cleanup_resources_() {
  if (this->streaming_active_) {
    this->stop_streaming();
  }

  if (this->ppa_handle_) {
    ppa_unregister_client(this->ppa_handle_);
    this->ppa_handle_ = nullptr;
  }

  if (this->camera_) {
    for (size_t i = 0; i < BUFFER_COUNT; i++) {
      if (this->camera_->buffer[i] && this->camera_->buffer[i] != MAP_FAILED) {
        munmap(this->camera_->buffer[i], this->camera_->buffer_size);
      }
    }

    if (this->camera_->fd >= 0) {
      close(this->camera_->fd);
    }

    free(this->camera_);
    this->camera_ = nullptr;
  }

  if (this->i2c_bus_handle_) {
    i2c_del_master_bus(this->i2c_bus_handle_);
    this->i2c_bus_handle_ = nullptr;
  }

  if (this->control_queue_) {
    vQueueDelete(this->control_queue_);
    this->control_queue_ = nullptr;
  }

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
  ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
  ESP_LOGCONFIG(TAG, "  Status: %s", this->camera_initialized_ ? "Initialized" : "Not initialized");
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






