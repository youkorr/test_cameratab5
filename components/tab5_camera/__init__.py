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

# Constantes pour la caméra Tab5
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_FRAMERATE = "framerate"
CONF_EXTERNAL_CLOCK_PIN = "external_clock_pin"
CONF_RESET_PIN = "reset_pin"

# Configuration CSI pour ESP32-P4
CONF_CSI_DATAP1 = "csi_datap1"
CONF_CSI_DATAN1 = "csi_datan1"
CONF_CSI_CLKP = "csi_clkp"
CONF_CSI_CLKN = "csi_clkn"
CONF_CSI_DATAP0 = "csi_datap0"
CONF_CSI_DATAN0 = "csi_datan0"

# Configuration I2C pour PI4IOE5V6408
CONF_EXPANDER_ADDRESS = "expander_address"
CONF_CAM_POWER_PIN = "cam_power_pin"

# Résolutions supportées pour CSI
CAMERA_RESOLUTIONS = {
    "1080P": (1920, 1080),
    "720P": (1280, 720),
    "VGA": (640, 480),
    "QVGA": (320, 240),
    "QQVGA": (160, 120),
}

# Formats de pixel supportés pour CSI
PIXEL_FORMATS = {
    "RAW8": "RAW8",
    "RAW10": "RAW10", 
    "YUV422": "YUV422",
    "RGB565": "RGB565",
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
            
            # Configuration CSI pour ESP32-P4
            cv.Optional(CONF_CSI_DATAP1, default="CSI_DATAP1"): cv.string,
            cv.Optional(CONF_CSI_DATAN1, default="CSI_DATAN1"): cv.string,
            cv.Optional(CONF_CSI_CLKP, default="CSI_CLKP"): cv.string,
            cv.Optional(CONF_CSI_CLKN, default="CSI_CLKN"): cv.string,
            cv.Optional(CONF_CSI_DATAP0, default="CSI_DATAP0"): cv.string,
            cv.Optional(CONF_CSI_DATAN0, default="CSI_DATAN0"): cv.string,
            
            # Configuration I2C pour la caméra
            cv.Optional(CONF_EXTERNAL_CLOCK_PIN, default=36): cv.int_range(min=0, max=255),  # CAM_MCLK
            cv.Optional(CONF_FREQUENCY, default=24000000): cv.positive_int,
            
            # Configuration PI4IOE5V6408 pour le reset
            cv.Optional(CONF_EXPANDER_ADDRESS, default=0x43): cv.i2c_address,  # Adresse typique PI4IOE5V6408
            cv.Optional(CONF_CAM_POWER_PIN, default=6): cv.int_range(min=0, max=7),  # P6 sur PI4IOE5V6408
            
            # Paramètres caméra
            cv.Optional(CONF_RESOLUTION, default="VGA"): validate_resolution,
            cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.one_of(*PIXEL_FORMATS.keys(), upper=True),
            cv.Optional(CONF_JPEG_QUALITY, default=10): cv.int_range(min=1, max=63),
            cv.Optional(CONF_FRAMERATE, default=15): cv.int_range(min=1, max=60),
        }
    )
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x24))  # Adresse I2C principale pour Tab5
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # Configuration de base
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_external_clock_pin(config[CONF_EXTERNAL_CLOCK_PIN]))
    cg.add(var.set_external_clock_frequency(config[CONF_FREQUENCY]))
    
    # Configuration CSI
    cg.add(var.set_csi_config(
        config[CONF_CSI_DATAP1],
        config[CONF_CSI_DATAN1],
        config[CONF_CSI_CLKP],
        config[CONF_CSI_CLKN],
        config[CONF_CSI_DATAP0],
        config[CONF_CSI_DATAN0]
    ))
    
    # Configuration PI4IOE5V6408
    cg.add(var.set_expander_config(
        config[CONF_EXPANDER_ADDRESS],
        config[CONF_CAM_POWER_PIN]
    ))
    
    # Paramètres caméra
    resolution_str = config[CONF_RESOLUTION]
    width, height = CAMERA_RESOLUTIONS[resolution_str]
    cg.add(var.set_resolution(width, height))
    
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_framerate(config[CONF_FRAMERATE]))
    
    # Ajouter les includes nécessaires
    cg.add_library("ESP32-CSI", None)  # Si une librairie CSI existe
    cg.add_define("USE_ESP32_P4_CSI")
