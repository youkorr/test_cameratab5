#pragma once

#include "esphome/core/component.h"
#include "esphome/components/camera/camera.h"
#include "esphome/components/i2c/i2c.h"

// La vérification #ifdef USE_ESP32 est correcte, mais il ne faut JAMAIS
// définir USE_ESP32 manuellement. Le système de build d'ESPHome s'en charge.
#ifdef USE_ESP32

// Includes du framework ESP-IDF pour V4L2 et PPA
#include "esp_log.h"
#include "esp_err.h"
#include "esp_system.h"
#include <fcntl.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <cerrno>
#include <cstring>

extern "C" {
#include "linux/videodev2.h"
#include "esp_video_init.h"
#include "driver/ppa.h"
}

namespace esphome {
namespace tab5_camera {

// Structure interne pour gérer l'état de la caméra V4L2
struct V4L2Camera {
  int fd{-1};
  uint32_t width{0};
  uint32_t height{0};
  uint32_t pixel_format{0};
  uint8_t* buffer[2]{nullptr, nullptr};
  size_t buffer_len[2]{0, 0};
  size_t buffer_count{0};
};

// La classe hérite de camera::Camera pour s'intégrer à ESPHome
// et de i2c::I2CDevice pour la communication.
class Tab5Camera : public camera::Camera, public i2c::I2CDevice {
 public:
  Tab5Camera() = default;
  ~Tab5Camera();

  // Méthodes standards d'un composant ESPHome
  void setup() override;
  void dump_config() override;
  float get_setup_priority() const override;

 protected:
  // Implémentation de la prise de photo, appelée par ESPHome/Home Assistant
  void take_picture(camera::CameraPicture &image) override;

  // Fonctions d'initialisation internes
  bool init_video_system();
  bool open_video_device();
  bool setup_camera_buffers();
  void cleanup_resources();

  V4L2Camera cam_{};
  bool initialized_{false};

  // Constantes
  static constexpr const char* DEVICE_PATH = "/dev/video0";
  static constexpr size_t BUFFER_COUNT = 2;
};

}  // namespace tab5_camera
}  // namespace esphome

#endif // USE_ESP32



