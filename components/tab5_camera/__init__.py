import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_NAME

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

CONF_RESOLUTION = "resolution"

CAMERA_RESOLUTIONS = {
    "HD": (1280, 720),     # Résolution native Tab5
    "VGA": (640, 480),
    "QVGA": (320, 240),
}

def validate_resolution(value):
    if isinstance(value, str):
        value = cv.one_of(*CAMERA_RESOLUTIONS.keys(), upper=True)(value)
        return value
    return cv.invalid("Resolution must be one of: {}".format(", ".join(CAMERA_RESOLUTIONS.keys())))

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(Tab5Camera),
        cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
        cv.Optional(CONF_RESOLUTION, default="HD"): validate_resolution,
    })
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x43))  # Adresse par défaut du capteur caméra
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    cg.add(var.set_name(config[CONF_NAME]))
    
    # Configuration résolution
    resolution_str = config[CONF_RESOLUTION]
    width, height = CAMERA_RESOLUTIONS[resolution_str]
    cg.add(var.set_resolution(width, height))
    
    # CORRECTION : Définir USE_ESP32 avec une valeur pour éviter l'erreur '&&'
    cg.add_define("USE_ESP32", "1")  # Ajouter une valeur explicite
    cg.add_define("CONFIG_IDF_TARGET_ESP32P4", "1")
    
    # Flags de build pour ESP32-P4
    cg.add_build_flag("-DCONFIG_BSP_ERROR_CHECK")
    cg.add_build_flag("-DCONFIG_VIDEO_ENABLE")
    cg.add_build_flag("-DUSE_ESP32=1")  # Aussi dans les build flags
    
    # Composants ESP-IDF nécessaires
    cg.add_platformio_option("lib_deps", [
        "esp-bsp",
        "espressif/esp32-camera"
    ])
    
    # Chemins d'inclusion
    cg.add_build_flag("-I$PROJECT_DIR/components")
    cg.add_build_flag("-I$PROJECT_DIR/.esphome/build/tab5/config")
    
    # Flags spécifiques ESP32-P4
    cg.add_build_flag("-DCONFIG_ESP32P4_DEFAULT_CPU_FREQ_360=1")
    cg.add_build_flag("-DCONFIG_ESP32_SPIRAM_SUPPORT=1")


