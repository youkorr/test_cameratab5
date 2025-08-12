```
# Camera Tab5 avec API V4L2
tab5_camera:
  id: my_tab5_camera
  name: My Tab5 Camera
  address: 67
  i2c_id: bsp_bus
  resolution: HD
    
    # Callback pour traiter les frames
    on_frame:
      - lambda: |-
          ESP_LOGI("camera", "📸 Frame V4L2 reçue: %zu bytes", data_len);
          
          // Les données sont déjà en RGB565, prêtes à utiliser
          // Ici vous pourriez les envoyer via web_server, MQTT, etc.

# Contrôles simples
button:
  - platform: template
    name: "Start V4L2 Streaming"
    on_press:
      - lambda: |-
          if (id(main_camera).start_streaming()) {
            ESP_LOGI("camera", "✅ V4L2 streaming started");
          } else {
            ESP_LOGE("camera", "❌ V4L2 streaming failed");
          }

  - platform: template
    name: "Stop V4L2 Streaming" 
    on_press:
      - lambda: |-
          id(main_camera).stop_streaming();
          ESP_LOGI("camera", "🛑 V4L2 streaming stopped");

# Sensor de statut
sensor:
  - platform: template
    name: "Camera Streaming Status"
    lambda: |-
      return id(main_camera).is_streaming() ? 1 : 0;
    update_interval: 2s

# Test automatique au boot
script:
  - id: auto_test
    then:
      - delay: 3s
      - lambda: |-
          ESP_LOGI("test", "🚀 Starting V4L2 camera test...");
          
          if (id(main_camera).has_error()) {
            ESP_LOGE("test", "❌ Camera error: %s", 
                     id(main_camera).get_last_error().c_str());
            return;
          }
          
          ESP_LOGI("test", "✅ Camera initialized, starting streaming test...");
      
      - lambda: |-
          if (id(main_camera).start_streaming()) {
            ESP_LOGI("test", "✅ V4L2 streaming started successfully");
          } else {
            ESP_LOGE("test", "❌ Failed to start V4L2 streaming");
          }
      
      - delay: 10s  # Stream for 10 seconds
      
      - lambda: |-
          id(main_camera).stop_streaming();
          ESP_LOGI("test", "🛑 Test streaming stopped");
          ESP_LOGI("test", "✅ V4L2 camera test completed");

# Démarrer le test au boot
on_boot:
  - priority: -100
    then:
      - script.execute: auto_test

# Web server pour monitoring (optionnel)
web_server:
  port: 80
  
# WiFi pour accès distant (optionnel)  
wifi:
  ssid: "VotreWiFi"
  password: "VotreMotDePasse"
  
  ap:
    ssid: "Tab5-Camera"
    password: "12345678"
```
