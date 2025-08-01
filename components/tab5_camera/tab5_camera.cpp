#include "tab5_camera.h"
#include "esphome/core/log.h"
#include "esphome/core/hal.h"

#ifdef USE_ESP32

namespace esphome {
namespace tab5_camera {

static const char *const TAG = "tab5_camera";

#ifdef HAS_ESP32_P4_CAMERA

// Registres d'initialisation SC2356 - BASÉS SUR LES SPÉCIFICATIONS SC2356
// Ces valeurs sont des exemples typiques - vous devez les extraire du HAL M5Stack réel
const uint16_t Tab5Camera::SC2356_INIT_REGS[][2] = {
    // Software Reset
    {0x0103, 0x01},
    {0x0100, 0x00},  // Standby mode
    
    // PLL Configuration
    {0x302a, 0x00},
    {0x302b, 0x00},
    {0x302c, 0x02},
    {0x302d, 0x00},
    {0x302e, 0x02},
    {0x302f, 0x80},
    
    // Clock Configuration
    {0x3106, 0x11},
    {0x3107, 0x01},
    
    // Sensor Core Configuration
    {0x3108, 0x00},
    {0x3109, 0x00},
    {0x310a, 0x00},
    {0x310b, 0x00},
    
    // Frame Configuration (VGA par défaut)
    {0x3800, 0x00},  // X start high
    {0x3801, 0x00},  // X start low
    {0x3802, 0x00},  // Y start high
    {0x3803, 0x00},  // Y start low
    {0x3804, 0x02},  // X end high (640)
    {0x3805, 0x80},  // X end low
    {0x3806, 0x01},  // Y end high (480)
    {0x3807, 0xe0},  // Y end low
    
    // Output Size
    {0x3808, 0x02},  // Output width high
    {0x3809, 0x80},  // Output width low (640)
    {0x380a, 0x01},  // Output height high
    {0x380b, 0xe0},  // Output height low (480)
    
    // Timing Configuration
    {0x380c, 0x03},  // Total width high
    {0x380d, 0x20},  // Total width low
    {0x380e, 0x02},  // Total height high
    {0x380f, 0x00},  // Total height low
    
    // Format Configuration
    {0x3820, 0x00},  // Vertical flip
    {0x3821, 0x00},  // Horizontal mirror
    
    // ISP Configuration
    {0x5000, 0x06},  // ISP control
    {0x5001, 0x01},  // AWB enable
    
    // Exit sleep mode
    {0x0100, 0x01},
    
    // Fin de la table
    {0xFFFF, 0xFF}  // Marqueur de fin
};

#endif // HAS_ESP32_P4_CAMERA

Tab5Camera::~Tab5Camera() {
#ifdef HAS_ESP32_P4_CAMERA
    cleanup_resources();
#endif
}

void Tab5Camera::setup() {
    ESP_LOGCONFIG(TAG, "Setting up Tab5 Camera...");
    
    this->clear_error_();
    
#ifdef HAS_ESP32_P4_CAMERA
    // Vérifier que nous sommes sur ESP32-P4
    if (!ESP_P4_CAMERA_SUPPORTED) {
        this->set_error_("ESP32-P4 with IDF 5.1+ required for camera support");
        this->mark_failed();
        return;
    }
    
    // Initialiser l'horloge externe
    if (!this->setup_external_clock_()) {
        this->set_error_("Failed to setup external clock");
        this->mark_failed();
        return;
    }
    
    // Initialiser LDO
    if (!this->init_ldo_()) {
        this->set_error_("Failed to initialize LDO regulators");
        this->mark_failed();
        return;
    }
    
    // Reset du capteur
    if (!this->reset_sensor_()) {
        this->set_error_("Failed to reset sensor");
        this->mark_failed();
        return;
    }
    
    // Initialiser le capteur I2C
    if (!this->init_sensor_()) {
        this->set_error_("Failed to initialize sensor");
        this->mark_failed();
        return;
    }
    
    // Initialiser le contrôleur de caméra ESP32-P4
    if (!this->init_camera_controller()) {
        this->set_error_("Failed to initialize camera controller");
        this->mark_failed();
        return;
    }
    
    // Initialiser ISP
    if (!this->init_isp_processor()) {
        this->set_error_("Failed to initialize ISP processor");
        this->mark_failed();
        return;
    }
    
#ifdef USE_LVGL
    // Configurer le canvas LVGL
    this->setup_lvgl_canvas_();
#endif
    
    ESP_LOGCONFIG(TAG, "Tab5 Camera initialized successfully");
    
#else
    this->set_error_("ESP32-P4 camera support not available");
    this->mark_failed();
#endif
}

void Tab5Camera::dump_config() {
    ESP_LOGCONFIG(TAG, "Tab5 Camera:");
    ESP_LOGCONFIG(TAG, "  Name: %s", this->name_.c_str());
    ESP_LOGCONFIG(TAG, "  Resolution: %dx%d", this->frame_width_, this->frame_height_);
    ESP_LOGCONFIG(TAG, "  Pixel Format: %s", this->pixel_format_.c_str());
    ESP_LOGCONFIG(TAG, "  Frame Rate: %d fps", this->framerate_);
    ESP_LOGCONFIG(TAG, "  External Clock: %d MHz", this->external_clock_frequency_ / 1000000);
    ESP_LOGCONFIG(TAG, "  Sensor Address: 0x%02X", this->sensor_address_);
    
#ifdef USE_LVGL
    if (this->canvas_configured_) {
        ESP_LOGCONFIG(TAG, "  LVGL Canvas: Configured");
    } else {
        ESP_LOGCONFIG(TAG, "  LVGL Canvas: Not configured");
    }
#endif
    
    if (this->has_error()) {
        ESP_LOGE(TAG, "  Error: %s", this->get_last_error().c_str());
    }
}

float Tab5Camera::get_setup_priority() const {
    // Priorité élevée pour s'initialiser avant les composants qui l'utilisent
    return setup_priority::HARDWARE - 1.0f;
}

#ifdef HAS_ESP32_P4_CAMERA

bool Tab5Camera::setup_external_clock_() {
    ESP_LOGD(TAG, "Setting up external clock on pin %d at %d Hz", 
             this->external_clock_pin_, this->external_clock_frequency_);
    
    // Configuration de l'horloge externe pour le capteur
    // Utiliser le générateur d'horloge ESP32-P4
    
    // TODO: Implémenter la configuration d'horloge spécifique ESP32-P4
    // Cette partie dépend de l'API IDF 5.1 pour ESP32-P4
    
    return true;
}

bool Tab5Camera::init_ldo_() {
    ESP_LOGD(TAG, "Initializing LDO regulators for MIPI");
    
    esp_ldo_channel_config_t ldo_mipi_phy_config = {
        .chan_id = 3,  // Channel ID pour MIPI PHY
        .voltage_mv = 2500,  // 2.5V pour MIPI PHY
    };
    
    esp_err_t ret = esp_ldo_acquire_channel(&ldo_mipi_phy_config, &this->ldo_mipi_phy_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to acquire LDO channel for MIPI PHY: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_ldo_channel_enable(this->ldo_mipi_phy_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable LDO channel for MIPI PHY: %s", esp_err_to_name(ret));
        return false;
    }
    
    this->ldo_initialized_ = true;
    ESP_LOGD(TAG, "LDO regulators initialized successfully");
    return true;
}

bool Tab5Camera::reset_sensor_() {
    ESP_LOGD(TAG, "Resetting sensor");
    
    if (this->reset_pin_ != nullptr) {
        this->reset_pin_->setup();
        this->reset_pin_->digital_write(false);
        delay(10);
        this->reset_pin_->digital_write(true);
        delay(20);
    }
    
    return true;
}

bool Tab5Camera::init_sensor_() {
    ESP_LOGD(TAG, "Initializing SC2356 sensor");
    
    // Test de communication I2C
    uint8_t chip_id;
    if (!this->read_sensor_register_(0x3107, &chip_id)) {
        ESP_LOGE(TAG, "Failed to read sensor chip ID - I2C communication failed");
        return false;
    }
    
    ESP_LOGD(TAG, "SC2356 Chip ID: 0x%02X", chip_id);
    
    // Vérifier l'ID du chip (SC2356 devrait retourner 0x23 ou similaire)
    if (chip_id != 0x23) {
        ESP_LOGW(TAG, "Unexpected chip ID: 0x%02X (expected 0x23)", chip_id);
    }
    
    // Initialiser les registres du capteur
    if (!this->init_sc2356_registers_()) {
        ESP_LOGE(TAG, "Failed to initialize SC2356 registers");
        return false;
    }
    
    this->sensor_initialized_ = true;
    ESP_LOGD(TAG, "SC2356 sensor initialized successfully");
    return true;
}

bool Tab5Camera::init_sc2356_registers_() {
    ESP_LOGD(TAG, "Writing SC2356 initialization registers");
    
    for (size_t i = 0; SC2356_INIT_REGS[i][0] != 0xFFFF; i++) {
        uint16_t reg = SC2356_INIT_REGS[i][0];
        uint8_t val = SC2356_INIT_REGS[i][1];
        
        if (!this->write_sensor_register_(reg, val)) {
            ESP_LOGE(TAG, "Failed to write register 0x%04X = 0x%02X", reg, val);
            return false;
        }
        
        // Petit délai entre les écritures
        delay(1);
    }
    
    ESP_LOGD(TAG, "SC2356 registers initialized successfully");
    return true;
}

bool Tab5Camera::init_camera_controller() {
    ESP_LOGD(TAG, "Initializing ESP32-P4 camera controller");
    
    // Configuration du bus MIPI CSI
    esp_cam_ctlr_csi_config_t csi_config = {
        .ctlr_id = 0,
        .h_res = this->frame_width_,
        .v_res = this->frame_height_,
        .lane_bit_rate_mbps = 400,  // 400 Mbps par lane
        .input_data_color_type = MIPI_CSI_COLOR_RGB565,  // Format d'entrée
        .output_data_color_type = MIPI_CSI_COLOR_RGB565,  // Format de sortie
        .data_lane_num = 2,  // 2 lanes MIPI
        .byte_swap_en = false,
        .queue_items = FRAME_QUEUE_SIZE,
    };
    
    esp_err_t ret = esp_cam_new_csi_ctlr(&csi_config, &this->cam_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create CSI controller: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Configuration des callbacks
    esp_cam_ctlr_evt_cbs_t cbs = {
        .on_get_new_vb = camera_get_new_vb_callback,
        .on_vb_done = camera_get_finished_trans_callback,
    };
    
    ret = esp_cam_ctlr_register_event_callbacks(this->cam_handle_, &cbs, this);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to register camera callbacks: %s", esp_err_to_name(ret));
        return false;
    }
    
    // Activer le contrôleur
    ret = esp_cam_ctlr_enable(this->cam_handle_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable camera controller: %s", esp_err_to_name(ret));
        return false;
    }
    
    this->camera_initialized_ = true;
    ESP_LOGD(TAG, "Camera controller initialized successfully");
    return true;
}

bool Tab5Camera::init_isp_processor() {
    ESP_LOGD(TAG, "Initializing ISP processor");
    
    isp_proc_config_t isp_config = {
        .input_data_source = ISP_INPUT_DATA_SOURCE_CSI,
        .input_data_color_type = ISP_COLOR_RGB565,
        .output_data_color_type = ISP_COLOR_RGB565,
        .h_res = this->frame_width_,
        .v_res = this->frame_height_,
        .has_line_start_packet = false,
        .has_line_end_packet = false,
    };
    
    esp_err_t ret = esp_isp_new_processor(&isp_config, &this->isp_proc_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to create ISP processor: %s", esp_err_to_name(ret));
        return false;
    }
    
    ret = esp_isp_enable(this->isp_proc_);
    if (ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to enable ISP processor: %s", esp_err_to_name(ret));
        return false;
    }
    
    ESP_LOGD(TAG, "ISP processor initialized successfully");
    return true;
}

bool Tab5Camera::start_streaming() {
    if (this->streaming_active_) {
        ESP_LOGW(TAG, "Streaming already active");
        return true;
    }
    
    ESP_LOGD(TAG, "Starting camera streaming");
    
    // Créer le sémaphore pour les frames
    this->frame_ready_semaphore_ = xSemaphoreCreateBinary();
    if (!this->frame_ready_semaphore_) {
        ESP_LOGE(TAG, "Failed to create frame semaphore");
        return false;
    }
    
    // Créer la queue pour les frames
    this->frame_queue_ = xQueueCreate(FRAME_QUEUE_SIZE, sizeof(FrameData));
    if (!this->frame_queue_) {
        ESP_LOGE(TAG, "Failed to create frame queue");
        return false;
    }
    
    // Allouer le buffer de frame
    size_t frame_size = this->calculate_frame_size_();
    this->frame_buffer_ = heap_caps_calloc(1, frame_size, MALLOC_CAP_SPIRAM);
    if (!this->frame_buffer_) {
        ESP_LOGE(TAG, "Failed to allocate frame buffer (%zu bytes)", frame_size);
        return false;
    }
    this->frame_buffer_size_ = frame_size;
    
    // Démarrer la tâche de streaming
    this->streaming_should_stop_ = false;
    BaseType_t ret = xTaskCreate(
        streaming_task,
        "camera_stream",
        STREAMING_TASK_STACK_SIZE,
        this,
        STREAMING_TASK_PRIORITY,
        &this->streaming_task_handle_
    );
    
    if (ret != pdPASS) {
        ESP_LOGE(TAG, "Failed to create streaming task");
        return false;
    }
    
    // Démarrer la capture
    esp_err_t esp_ret = esp_cam_ctlr_start(this->cam_handle_);
    if (esp_ret != ESP_OK) {
        ESP_LOGE(TAG, "Failed to start camera capture: %s", esp_err_to_name(esp_ret));
        return false;
    }
    
    this->streaming_active_ = true;
    ESP_LOGD(TAG, "Camera streaming started successfully");
    return true;
}

bool Tab5Camera::stop_streaming() {
    if (!this->streaming_active_) {
        return true;
    }
    
    ESP_LOGD(TAG, "Stopping camera streaming");
    
    // Arrêter la capture
    esp_cam_ctlr_stop(this->cam_handle_);
    
    // Arrêter la tâche de streaming
    this->streaming_should_stop_ = true;
    if (this->streaming_task_handle_) {
        vTaskDelete(this->streaming_task_handle_);
        this->streaming_task_handle_ = nullptr;
    }
    
    // Nettoyer les ressources
    if (this->frame_ready_semaphore_) {
        vSemaphoreDelete(this->frame_ready_semaphore_);
        this->frame_ready_semaphore_ = nullptr;
    }
    
    if (this->frame_queue_) {
        vQueueDelete(this->frame_queue_);
        this->frame_queue_ = nullptr;
    }
    
    if (this->frame_buffer_) {
        heap_caps_free(this->frame_buffer_);
        this->frame_buffer_ = nullptr;
        this->frame_buffer_size_ = 0;
    }
    
    this->streaming_active_ = false;
    ESP_LOGD(TAG, "Camera streaming stopped");
    return true;
}

void Tab5Camera::streaming_task(void *parameter) {
    Tab5Camera* camera = static_cast<Tab5Camera*>(parameter);
    camera->streaming_loop_();
}

void Tab5Camera::streaming_loop_() {
    ESP_LOGD(TAG, "Streaming task started");
    
    while (!this->streaming_should_stop_) {
        // Attendre qu'une frame soit prête
        if (xSemaphoreTake(this->frame_ready_semaphore_, pdMS_TO_TICKS(100)) == pdTRUE) {
            FrameData frame_data;
            if (xQueueReceive(this->frame_queue_, &frame_data, 0) == pdTRUE) {
                if (frame_data.valid) {
                    // Traiter la frame
#ifdef USE_LVGL
                    this->update_canvas_with_frame(
                        static_cast<uint8_t*>(frame_data.buffer), 
                        frame_data.size
                    );
#endif
                    // Appeler les callbacks
                    this->on_frame_callbacks_.call(
                        static_cast<uint8_t*>(frame_data.buffer), 
                        frame_data.size
                    );
                }
            }
        }
    }
    
    ESP_LOGD(TAG, "Streaming task ended");
}

// Callbacks statiques pour le contrôleur de caméra
bool IRAM_ATTR Tab5Camera::camera_get_new_vb_callback(
    esp_cam_ctlr_handle_t handle, 
    esp_cam_ctlr_trans_t *trans, 
    void *user_data) {
    
    Tab5Camera* camera = static_cast<Tab5Camera*>(user_data);
    
    // Fournir le buffer pour la prochaine frame
    trans->buffer = camera->frame_buffer_;
    trans->buflen = camera->frame_buffer_size_;
    
    return false;
}

bool IRAM_ATTR Tab5Camera::camera_get_finished_trans_callback(
    esp_cam_ctlr_handle_t handle, 
    esp_cam_ctlr_trans_t *trans, 
    void *user_data) {
    
    Tab5Camera* camera = static_cast<Tab5Camera*>(user_data);
    
    // Créer les données de frame
    FrameData frame_data = {
        .buffer = trans->buffer,
        .size = trans->received_size,
        .timestamp = esp_timer_get_time(),
        .valid = true
    };
    
    // Envoyer vers la queue (depuis l'ISR)
    BaseType_t higher_priority_task_woken = pdFALSE;
    xQueueSendFromISR(camera->frame_queue_, &frame_data, &higher_priority_task_woken);
    xSemaphoreGiveFromISR(camera->frame_ready_semaphore_, &higher_priority_task_woken);
    
    if (higher_priority_task_woken == pdTRUE) {
        portYIELD_FROM_ISR();
    }
    
    return false;
}

void Tab5Camera::cleanup_resources() {
    if (this->streaming_active_) {
        this->stop_streaming();
    }
    
    if (this->cam_handle_) {
        esp_cam_ctlr_disable(this->cam_handle_);
        esp_cam_del_ctlr(this->cam_handle_);
        this->cam_handle_ = nullptr;
    }
    
    if (this->isp_proc_) {
        esp_isp_disable(this->isp_proc_);
        esp_isp_del_processor(this->isp_proc_);
        this->isp_proc_ = nullptr;
    }
    
    if (this->ldo_mipi_phy_) {
        esp_ldo_channel_disable(this->ldo_mipi_phy_);
        esp_ldo_release_channel(this->ldo_mipi_phy_);
        this->ldo_mipi_phy_ = nullptr;
    }
}

#endif // HAS_ESP32_P4_CAMERA

#ifdef USE_LVGL

void Tab5Camera::set_lvgl_canvas(lv_obj_t* canvas) {
    this->canvas_ = canvas;
    if (canvas) {
        this->canvas_buffer_ = static_cast<lv_color_t*>(lv_canvas_get_buf(canvas));
        this->canvas_configured_ = true;
        ESP_LOGD(TAG, "LVGL Canvas configured");
    }
}

void Tab5Camera::setup_lvgl_canvas_() {
    if (!this->canvas_id_.empty()) {
        // Le canvas sera configuré par la suite via LVGL
        ESP_LOGD(TAG, "LVGL Canvas ID set: %s", this->canvas_id_.c_str());
    }
}

void Tab5Camera::update_canvas_with_frame(uint8_t* frame_data, size_t frame_size) {
    if (!this->canvas_ || !this->canvas_buffer_) {
        return;
    }
    
    this->convert_camera_frame_to_canvas_(frame_data, frame_size);
    this->invalidate_canvas_();
    
    // Appeler les callbacks de mise à jour canvas
    this->on_canvas_update_callbacks_.call();
}

void Tab5Camera::convert_camera_frame_to_canvas_(uint8_t* camera_data, size_t size) {
    if (this->pixel_format_ == "RGB565") {
        uint16_t* rgb565_data = reinterpret_cast<uint16_t*>(camera_data);
        size_t pixel_count = std::min(size / 2, 
                                     static_cast<size_t>(this->frame_width_ * this->frame_height_));
        this->convert_rgb565_to_lvgl_(rgb565_data, pixel_count, this->canvas_buffer_);
    } else if (this->pixel_format_ == "YUV422") {
        size_t pixel_count = std::min(size / 2, 
                                     static_cast<size_t>(this->frame_width_ * this->frame_height_));
        this->convert_yuv422_to_lvgl_(camera_data, pixel_count, this->canvas_buffer_);
    }
}

void Tab5Camera::convert_rgb565_to_lvgl_(uint16_t* rgb565_data, size_t pixel_count, lv_color_t* output) {
    for (size_t i = 0; i < pixel_count; i++) {
        uint16_t rgb565 = rgb565_data[i];
        
        // Extraire R, G, B depuis RGB565
        uint8_t r = (rgb565 >> 11) & 0x1F;
        uint8_t g = (rgb565 >> 5) & 0x3F;
        uint8_t b = rgb565 & 0x1F;
        
        // Convertir vers RGB888 pour LVGL
        r = (r * 255) / 31;
        g = (g * 255) / 63;
        b = (b * 255) / 31;
        
        // Stocker dans le buffer canvas LVGL
        output[i] = lv_color_make(r, g, b);
    }
}

void Tab5Camera::convert_yuv422_to_lvgl_(uint8_t* yuv_data, size_t pixel_count, lv_color_t* output) {
    // Conversion YUV422 vers RGB pour LVGL
    for (size_t i = 0; i < pixel_count; i += 2) {
        uint8_t y1 = yuv_data[i * 2];
        uint8_t u = yuv_data[i * 2 + 1];
        uint8_t y2 = yuv_data[i * 2 + 2];
        uint8_t v = yuv_data[i * 2 + 3];
        
        // Conversion YUV vers RGB (algorithme standard)
        int c1 = y1 - 16;
        int c2 = y2 - 16;
        int d = u - 128;
        int e = v - 128;
        
        // Pixel 1
        int r1 = (298 * c1 + 409 * e + 128) >> 8;
        int g1 = (298 * c1 - 100 * d - 208 * e + 128) >> 8;
        int b1 = (298 * c1 + 516 * d + 128) >> 8;
        
        // Pixel 2
        int r2 = (298 * c2 + 409 * e + 128) >> 8;
        int g2 = (298 * c2 - 100 * d - 208 * e + 128) >> 8;
        int b2 = (298 * c2 + 516 * d + 128) >> 8;
        
        // Clamping
        r1 = std::max(0, std::min(255, r1));
        g1 = std::max(0, std::min(255, g1));
        b1 = std::max(0, std::min(255, b1));
        r2 = std::max(0, std::min(255, r2));
        g2 = std::max(0, std::min(255, g2));
        b2 = std::max(0, std::min(255, b2));
        
        if (i < pixel_count) {
            output[i] = lv_color_make(r1, g1, b1);
        }
        if (i + 1 < pixel_count) {
            output[i + 1] = lv_color_make(r2, g2, b2);
        }
    }
}

void Tab5Camera::invalidate_canvas_() {
    if (this->canvas_) {
        lv_obj_invalidate(this->canvas_);
    }
}

#endif // USE_LVGL

// Communication I2C avec le capteur
bool Tab5Camera::read_sensor_register_(uint16_t reg, uint8_t *value) {
    return this->read_register_16(reg, value);
}

bool Tab5Camera::write_sensor_register_(uint16_t reg, uint8_t value) {
    return this->write_register_16(reg, value);
}

bool Tab5Camera::write_register_16(uint16_t reg, uint8_t val) {
    uint8_t data[3];
    data[0] = reg >> 8;     // Register high byte
    data[1] = reg & 0xFF;   // Register low byte
    data[2] = val;          // Value
    
    return this->write_bytes_raw(data, 3);
}

bool Tab5Camera::read_register_16(uint16_t reg, uint8_t *val) {
    uint8_t reg_data[2];
    reg_data[0] = reg >> 8;     // Register high byte
    reg_data[1] = reg & 0xFF;   // Register low byte
    
    if (!this->write_bytes_raw(reg_data, 2)) {
        return false;
    }
    
    return this->read_bytes_raw(val, 1);
}

bool Tab5Camera::take_snapshot() {
    ESP_LOGD(TAG, "Taking snapshot");
    
    if (!this->streaming_active_) {
        // Démarrer temporairement le streaming pour une capture
        if (!this->start_streaming()) {
            return false;
        }
        
        // Attendre une frame
        delay(100);
        
        this->stop_streaming();
    }
    
    return true;
}

void Tab5Camera::streaming_loop_() {
  ESP_LOGD(TAG, "Streaming loop started for camera '%s' (callback-based)", this->name_.c_str());

  // Au lieu d'appeler esp_cam_ctlr_receive() en boucle (qui sature la queue),
  // on utilise uniquement les callbacks pour récupérer les frames
  
  while (!this->streaming_should_stop_) {
    // Attendre qu'une frame soit disponible via le callback
    if (xSemaphoreTake(this->frame_ready_semaphore_, 100 / portTICK_PERIOD_MS) == pdTRUE) {
      
      // Récupérer les données de frame depuis notre queue applicative
      FrameData frame;
      if (xQueueReceive(this->frame_queue_, &frame, 0) == pdTRUE) {
        ESP_LOGV(TAG, "Frame received from callback, size: %zu bytes", frame.size);
        
        // Appel des callbacks avec les données reçues
        this->on_frame_callbacks_.call(static_cast<uint8_t*>(frame.buffer), frame.size);
      }
    }
    
    // Petite pause pour éviter de surcharger le système
    vTaskDelay(1 / portTICK_PERIOD_MS);
  }

  this->streaming_active_ = false;
  ESP_LOGD(TAG, "Streaming loop ended for camera '%s'", this->name_.c_str());
  vTaskDelete(nullptr);
}

#endif

}  // namespace tab5_camera
}  // namespace esphome
