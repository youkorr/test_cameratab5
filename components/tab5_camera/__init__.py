import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_NAME

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]
# Remove AUTO_LOAD = ["bsp"] - not needed

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
    
    # Ajouter les définitions nécessaires
    cg.add_define("USE_ESP32")
    cg.add_define("CONFIG_IDF_TARGET_ESP32P4")
    
    # Add build flags for ESP32-P4 and BSP support
    cg.add_build_flag("-DCONFIG_BSP_ERROR_CHECK")
    cg.add_build_flag("-DCONFIG_VIDEO_ENABLE")
    
    # Include necessary ESP-IDF components
    cg.add_platformio_option("lib_deps", "esp-bsp")
    
    # Add include paths if needed
    cg.add_build_flag("-I$PROJECT_DIR/components")
