import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_ADDRESS,
    CONF_FREQUENCY,
)
from esphome import pins

# Support conditionnel pour LVGL
try:
    from esphome.components import lvgl
    HAS_LVGL = True
except ImportError:
    HAS_LVGL = False

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]
AUTO_LOAD = []
MULTI_CONF = True

# Ajout conditionnel de LVGL aux dépendances
if HAS_LVGL:
    AUTO_LOAD.append("lvgl")

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Nouvelles constantes
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_FRAMERATE = "framerate"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_RESET_PIN = "reset_pin"
CONF_SENSOR_ADDRESS = "sensor_address"

# NOUVEAU : Support LVGL Canvas
CONF_CANVAS_ID = "canvas_id"
CONF_AUTO_DISPLAY = "auto_display"

# Résolutions supportées
CAMERA_RESOLUTIONS = {
    "1080P": (1920, 1080),
    "720P": (1280, 720),
    "VGA": (640, 480),
    "QVGA": (320, 240),
}

# Formats de pixel supportés
PIXEL_FORMATS = {
    "RAW8": "RAW8",
    "RAW10": "RAW10",
    "YUV422": "YUV422",
    "RGB565": "RGB565",  # CORRECTION : était "RBB565"
    "JPEG": "JPEG",
}

def validate_resolution(value):
    if isinstance(value, str):
        value = cv.one_of(*CAMERA_RESOLUTIONS.keys(), upper=True)(value)
        return value
    return cv.invalid("Resolution must be one of: {}".format(", ".join(CAMERA_RESOLUTIONS.keys())))

# Schema de configuration principal
base_schema = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(Tab5Camera),
        cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
        cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=0): cv.int_range(min=0, max=255),
        cv.Optional(CONF_FREQUENCY, default=24000000): cv.positive_int,
        cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_SENSOR_ADDRESS, default=0x36): cv.i2c_address,
        # Nouveaux paramètres
        cv.Optional(CONF_RESOLUTION, default="VGA"): validate_resolution,
        cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.one_of(*PIXEL_FORMATS.keys(), upper=True),
        cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
        cv.Optional(CONF_FRAMERATE, default=15): cv.int_range(min=1, max=60),
        cv.Optional(CONF_AUTO_DISPLAY, default=True): cv.boolean,
    }
)

# NOUVEAU : Ajout conditionnel du support LVGL Canvas
if HAS_LVGL:
    base_schema = base_schema.extend({
        cv.Optional(CONF_CANVAS_ID): cv.use_id(lvgl.LvObj),
    })

CONFIG_SCHEMA = cv.All(
    base_schema
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x36))
)

async def to_code(config):
    
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # Configuration de base
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_external_clock_pin(config[CONF_EXTERNAL_CLOCK_PIN]))
    cg.add(var.set_external_clock_frequency(config[CONF_FREQUENCY]))
    cg.add(var.set_sensor_address(config[CONF_SENSOR_ADDRESS]))
    
    # Nouveaux paramètres
    # Résolution
    resolution_str = config[CONF_RESOLUTION]
    width, height = CAMERA_RESOLUTIONS[resolution_str]
    cg.add(var.set_resolution(width, height))
    
    # Format pixel
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    
    # Qualité JPEG
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    
    # Framerate
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # Pin de reset (optionnel)
    if CONF_RESET_PIN in config:
        reset_pin = await cg.gpio_pin_expression(config[CONF_RESET_PIN])
        cg.add(var.set_reset_pin(reset_pin))
    
    # NOUVEAU : Configuration LVGL Canvas
    if HAS_LVGL and CONF_CANVAS_ID in config:
        canvas = await cg.get_variable(config[CONF_CANVAS_ID])
        cg.add(var.set_lvgl_canvas_id(config[CONF_CANVAS_ID]))
        # Le canvas sera configuré dans le setup() du composant
        
        # Ajouter les includes LVGL nécessaires
        cg.add_define("USE_LVGL")
        cg.add_library("lvgl", None)
    
    # NOUVEAU : Ajout des defines ESP32-P4 spécifiques
    cg.add_define("USE_ESP32")
    
    # Vérification ESP32-P4 spécifique
    cg.add_build_flag("-DCONFIG_IDF_TARGET_ESP32P4")
    
    # Libraries ESP32-P4 requises
    cg.add_platformio_option("lib_deps", [
        "espressif/esp32-camera",
        "https://github.com/espressif/esp-idf.git#v5.1.2"
    ])
    
    # Build flags spécifiques pour ESP32-P4
    cg.add_build_flag("-DESP_IDF_VERSION_VAL(5,1,0)")
    
    # NOUVEAU : Actions et triggers pour la caméra
    # Actions pour start/stop streaming
    cg.add_define("TAB5_CAMERA_STREAM_ACTION_SUPPORT")
    
    # Actions pour capture d'image
    cg.add_define("TAB5_CAMERA_SNAPSHOT_ACTION_SUPPORT")
