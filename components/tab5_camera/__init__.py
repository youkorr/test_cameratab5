import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c, camera
from esphome.const import (
    CONF_ID,
    CONF_RESOLUTION,
)

# --- Dépendances et métadonnées ---
CODEOWNERS = ["@youkorr"]
# Le composant dépend des intégrations 'i2c' et 'camera' d'ESPHome
DEPENDENCIES = ["i2c", "camera"]
# Le composant n'est compatible qu'avec le framework espidf
ESP_IDF_REQUIRED = True

# --- Définition du composant ---
tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", camera.Camera, cg.Component, i2c.I2CDevice)

# --- Configuration ---
CAMERA_RESOLUTIONS = {
    "HD": (1280, 720),
    "VGA": (640, 480),
    "QVGA": (320, 240),
}

validate_resolution = cv.one_of(*CAMERA_RESOLUTIONS, upper=True)

# Le schéma de configuration hérite de celui de la caméra de base
# pour inclure automatiquement les options communes (nom, id, etc.)
CONFIG_SCHEMA = camera.CAMERA_SCHEMA.extend({
    cv.GenerateID(): cv.declare_id(Tab5Camera),
    cv.Optional(CONF_RESOLUTION, default="HD"): validate_resolution,
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x43))


async def to_code(config):
    """Génère le code C++ correspondant à la configuration YAML."""
    # Crée une nouvelle variable C++ pour notre caméra
    var = cg.new_Pvariable(config[CONF_ID])
    
    # Enregistre le composant auprès des systèmes de base d'ESPHome
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    await camera.register_camera(var, config)

    # Ajoute une définition unique pour pouvoir utiliser des #ifdef dans notre code C++
    cg.add_define("USE_TAB5_CAMERA")

    # Ajoute les bibliothèques nécessaires à la compilation.
    # 'espressif/esp32-camera' est la bibliothèque standard pour les caméras sur ESP32.
    # La bibliothèque 'esp-bsp' n'est pas ajoutée ici car le framework ESP-IDF
    # pour la carte P4 devrait déjà inclure le support matériel nécessaire.
    # L'ajouter manuellement peut créer des conflits.
    cg.add_library("espressif/esp32-camera", "2.0.4")

    # Il n'est PAS nécessaire d'ajouter manuellement des defines comme USE_ESP32
    # ou des build flags. Le framework s'en charge en fonction de la carte
    # sélectionnée dans votre fichier YAML (board: esp32-p4-evboard).
    # Forcer ces valeurs est la source principale de vos erreurs.


