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

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]
MULTI_CONF = True

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Constantes pour la caméra
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_FRAMERATE = "framerate"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_RESET_PIN = "reset_pin"

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
    "RGB565": "RGB565",  # Correction de la typo "RBB565"
    "JPEG": "JPEG",
}

def validate_resolution(value):
    if isinstance(value, str):
        value = cv.one_of(*CAMERA_RESOLUTIONS.keys(), upper=True)(value)
        return value
    return cv.invalid("Resolution must be one of: {}".format(", ".join(CAMERA_RESOLUTIONS.keys())))

CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(Tab5Camera),
            cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
            cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=36): cv.int_range(min=0, max=255),  # Pin 36 comme dans votre code ESP-IDF
            cv.Optional(CONF_FREQUENCY, default=24000000): cv.positive_int,
            cv.Optional(CONF_RESET_PIN): pins.gpio_output_pin_schema,
            # Paramètres caméra
            cv.Optional(CONF_RESOLUTION, default="VGA"): validate_resolution,
            cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.one_of(*PIXEL_FORMATS.keys(), upper=True),  # Correction typo
            cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
            cv.Optional(CONF_FRAMERATE, default=15): cv.int_range(min=1, max=60),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x24))  # Gardez l'adresse I2C pour la communication avec le Tab5
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # Configuration de base
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_external_clock_pin(config[CONF_EXTERNAL_CLOCK_PIN]))
    cg.add(var.set_external_clock_frequency(config[CONF_FREQUENCY]))
    
    # Paramètres caméra
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

    # Les includes LEDC sont gérés dans le fichier .h/.cpp
