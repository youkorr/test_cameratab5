#include "tab5_camera.h"
#include "esphome/core/log.h"

#ifdef USE_ESP32

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

Tab5Camera::~Tab5Camera() {
  this->cleanup_resources();
}

void Tab5Camera::setup() {
  ESP_LOGCONFIG(TAG, "Mise en place de la caméra Tab5...");

  if (!this->init_video_system()) {
    ESP_LOGE(TAG, "Échec de l'initialisation du système vidéo.");
    this->mark_failed();
    return;
  }

  if (!this->open_video_device()) {
    ESP_LOGE(TAG, "Échec de l'ouverture du périphérique vidéo.");
    this->mark_failed();
    return;
  }

  if (!this->setup_camera_buffers()) {
    ESP_LOGE(TAG, "Échec de la configuration des tampons (buffers).");
    this->mark_failed();
    return;
  }

  this->initialized_ = true;
  ESP_LOGI(TAG, "Caméra Tab5 initialisée avec succès.");
}

void Tab5Camera::dump_config() {
  ESP_LOGCONFIG(TAG, "Configuration de la caméra Tab5 :");
  LOG_I2C_DEVICE(this);
  LOG_CAMERA(this);
  if (this->initialized_) {
    ESP_LOGCONFIG(TAG, "  Périphérique V4L2: %s (fd: %d)", DEVICE_PATH, this->cam_.fd);
    ESP_LOGCONFIG(TAG, "  Résolution actuelle: %d x %d", this->cam_.width, this->cam_.height);
  }
  if (this->is_failed()) {
    ESP_LOGCONFIG(TAG, "  Statut: ÉCHEC");
  }
}

float Tab5Camera::get_setup_priority() const { return setup_priority::HARDWARE; }

bool Tab5Camera::init_video_system() {
  ESP_LOGD(TAG, "Initialisation du système vidéo ESP-IDF...");
  // La configuration est laissée vide pour utiliser les valeurs par défaut du BSP
  esp_video_init_config_t video_config = ESP_VIDEO_INIT_CONFIG_DEFAULT();
  esp_err_t ret = esp_video_init(&video_config);
  if (ret != ESP_OK) {
    ESP_LOGE(TAG, "esp_video_init a échoué: %s", esp_err_to_name(ret));
    return false;
  }
  return true;
}

bool Tab5Camera::open_video_device() {
  ESP_LOGD(TAG, "Ouverture du périphérique vidéo: %s", DEVICE_PATH);
  this->cam_.fd = open(DEVICE_PATH, O_RDWR);
  if (this->cam_.fd < 0) {
    ESP_LOGE(TAG, "Échec de l'ouverture de %s: %s", DEVICE_PATH, strerror(errno));
    return false;
  }

  // Configuration du format de l'image
  struct v4l2_format fmt = {};
  fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  fmt.fmt.pix.width = this->width_;
  fmt.fmt.pix.height = this->height_;
  fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_JPEG; // Demander du JPEG directement si possible
  fmt.fmt.pix.field = V4L2_FIELD_NONE;

  if (ioctl(this->cam_.fd, VIDIOC_S_FMT, &fmt) != 0) {
    ESP_LOGW(TAG, "Impossible de définir le format JPEG, essai avec YUYV...: %s", strerror(errno));
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_YUYV;
    if (ioctl(this->cam_.fd, VIDIOC_S_FMT, &fmt) != 0) {
        ESP_LOGE(TAG, "Impossible de définir un format vidéo compatible: %s", strerror(errno));
        return false;
    }
  }

  // Stocker la configuration réelle appliquée par le pilote
  this->cam_.width = fmt.fmt.pix.width;
  this->cam_.height = fmt.fmt.pix.height;
  this->cam_.pixel_format = fmt.fmt.pix.pixelformat;

  ESP_LOGI(TAG, "Format vidéo configuré: %dx%d, format: %c%c%c%c",
           this->cam_.width, this->cam_.height, (this->cam_.pixel_format & 0xff),
           (this->cam_.pixel_format >> 8) & 0xff, (this->cam_.pixel_format >> 16) & 0xff,
           (this->cam_.pixel_format >> 24) & 0xff);

  return true;
}

bool Tab5Camera::setup_camera_buffers() {
  ESP_LOGD(TAG, "Configuration des tampons V4L2...");
  struct v4l2_requestbuffers req = {};
  req.count = BUFFER_COUNT;
  req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  req.memory = V4L2_MEMORY_MMAP;

  if (ioctl(this->cam_.fd, VIDIOC_REQBUFS, &req) != 0) {
    ESP_LOGE(TAG, "VIDIOC_REQBUFS a échoué: %s", strerror(errno));
    return false;
  }
  this->cam_.buffer_count = req.count;

  for (size_t i = 0; i < this->cam_.buffer_count; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;

    if (ioctl(this->cam_.fd, VIDIOC_QUERYBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QUERYBUF a échoué pour le tampon %d: %s", i, strerror(errno));
      return false;
    }

    this->cam_.buffer_len[i] = buf.length;
    this->cam_.buffer[i] = (uint8_t*) mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, this->cam_.fd, buf.m.offset);

    if (this->cam_.buffer[i] == MAP_FAILED) {
      ESP_LOGE(TAG, "mmap a échoué pour le tampon %d: %s", i, strerror(errno));
      return false;
    }
  }
  return true;
}

void Tab5Camera::take_picture(camera::CameraPicture &image) {
  if (!this->initialized_) {
    ESP_LOGE(TAG, "La caméra n'est pas initialisée.");
    image.set_error("Not initialized");
    return;
  }

  // --- Processus de capture V4L2 ---
  // 1. Mettre tous les tampons en file d'attente
  for (size_t i = 0; i < this->cam_.buffer_count; i++) {
    struct v4l2_buffer buf = {};
    buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    buf.memory = V4L2_MEMORY_MMAP;
    buf.index = i;
    if (ioctl(this->cam_.fd, VIDIOC_QBUF, &buf) != 0) {
      ESP_LOGE(TAG, "VIDIOC_QBUF a échoué pour le tampon %d: %s", i, strerror(errno));
      image.set_error("Queue buffer failed");
      return;
    }
  }

  // 2. Démarrer le streaming
  int type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  if (ioctl(this->cam_.fd, VIDIOC_STREAMON, &type) != 0) {
    ESP_LOGE(TAG, "VIDIOC_STREAMON a échoué: %s", strerror(errno));
    image.set_error("Stream on failed");
    return;
  }

  // 3. Retirer un tampon de la file (attendre une image)
  struct v4l2_buffer buf = {};
  buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
  buf.memory = V4L2_MEMORY_MMAP;
  if (ioctl(this->cam_.fd, VIDIOC_DQBUF, &buf) != 0) {
    ESP_LOGE(TAG, "VIDIOC_DQBUF a échoué: %s", strerror(errno));
    ioctl(this->cam_.fd, VIDIOC_STREAMOFF, &type); // Essayer d'arrêter le stream
    image.set_error("Dequeue buffer failed");
    return;
  }

  // 4. Arrêter le streaming
  ioctl(this->cam_.fd, VIDIOC_STREAMOFF, &type);

  ESP_LOGD(TAG, "Image capturée, taille: %d bytes", buf.bytesused);

  // 5. Copier les données de l'image
  // Note: Si le format n'est pas JPEG, il faudra le convertir ici.
  // Pour l'instant, on suppose que c'est du JPEG.
  image.set_data(this->cam_.buffer[buf.index], buf.bytesused);
}

void Tab5Camera::cleanup_resources() {
  if (this->cam_.fd >= 0) {
    for (size_t i = 0; i < this->cam_.buffer_count; i++) {
      if (this->cam_.buffer[i]) {
        munmap(this->cam_.buffer[i], this->cam_.buffer_len[i]);
      }
    }
    close(this->cam_.fd);
    this->cam_.fd = -1;
  }
  this->initialized_ = false;
}

}  // namespace tab5_camera
}  // namespace esphome

#endif // USE_ESP32




