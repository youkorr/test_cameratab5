import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_NAME

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Enums pour les formats et tailles
Tab5PixFormat = tab5_camera_ns.enum("tab5_pixformat_t")
PIXEL_FORMATS = {
    "RGB565": Tab5PixFormat.TAB5_PIXFORMAT_RGB565,
    "YUV422": Tab5PixFormat.TAB5_PIXFORMAT_YUV422,
    "GRAYSCALE": Tab5PixFormat.TAB5_PIXFORMAT_GRAYSCALE,
    "JPEG": Tab5PixFormat.TAB5_PIXFORMAT_JPEG,
    "RGB888": Tab5PixFormat.TAB5_PIXFORMAT_RGB888,
    "RAW": Tab5PixFormat.TAB5_PIXFORMAT_RAW,
}

Tab5FrameSize = tab5_camera_ns.enum("tab5_framesize_t")
FRAME_SIZES = {
    "96X96": Tab5FrameSize.TAB5_FRAMESIZE_96X96,
    "QQVGA": Tab5FrameSize.TAB5_FRAMESIZE_QQVGA,      # 160x120
    "QCIF": Tab5FrameSize.TAB5_FRAMESIZE_QCIF,        # 176x144
    "HQVGA": Tab5FrameSize.TAB5_FRAMESIZE_HQVGA,      # 240x176
    "240X240": Tab5FrameSize.TAB5_FRAMESIZE_240X240,
    "QVGA": Tab5FrameSize.TAB5_FRAMESIZE_QVGA,        # 320x240
    "CIF": Tab5FrameSize.TAB5_FRAMESIZE_CIF,          # 400x296
    "HVGA": Tab5FrameSize.TAB5_FRAMESIZE_HVGA,        # 480x320
    "VGA": Tab5FrameSize.TAB5_FRAMESIZE_VGA,          # 640x480
    "SVGA": Tab5FrameSize.TAB5_FRAMESIZE_SVGA,        # 800x600
    "XGA": Tab5FrameSize.TAB5_FRAMESIZE_XGA,          # 1024x768
    "HD": Tab5FrameSize.TAB5_FRAMESIZE_HD,            # 1280x720
    "SXGA": Tab5FrameSize.TAB5_FRAMESIZE_SXGA,        # 1280x1024
    "UXGA": Tab5FrameSize.TAB5_FRAMESIZE_UXGA,        # 1600x1200
}

# Configuration keys
CONF_RESOLUTION = "resolution"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_VERTICAL_FLIP = "vertical_flip"
CONF_HORIZONTAL_MIRROR = "horizontal_mirror"
CONF_BRIGHTNESS = "brightness"
CONF_CONTRAST = "contrast"
CONF_SATURATION = "saturation"

CONFIG_SCHEMA = cv.All(
    cv.Schema({
        cv.GenerateID(): cv.declare_id(Tab5Camera),
        cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
        cv.Optional(CONF_RESOLUTION, default="VGA"): cv.enum(FRAME_SIZES, upper=True),
        cv.Optional(CONF_PIXEL_FORMAT, default="JPEG"): cv.enum(PIXEL_FORMATS, upper=True),
        cv.Optional(CONF_JPEG_QUALITY, default=12): cv.int_range(min=10, max=63),
        cv.Optional(CONF_VERTICAL_FLIP, default=False): cv.boolean,
        cv.Optional(CONF_HORIZONTAL_MIRROR, default=False): cv.boolean,
        cv.Optional(CONF_BRIGHTNESS, default=0): cv.int_range(min=-2, max=2),
        cv.Optional(CONF_CONTRAST, default=0): cv.int_range(min=-2, max=2),
        cv.Optional(CONF_SATURATION, default=0): cv.int_range(min=-2, max=2),
    })
    .extend(cv.COMPONENT_SCHEMA)
    .extend(i2c.i2c_device_schema(0x43))
)

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    # Configuration de base
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_resolution(config[CONF_RESOLUTION]))
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    
    # Configuration optionnelle
    cg.add(var.set_jpeg_quality(config[CONF_JPEG_QUALITY]))
    cg.add(var.set_vertical_flip(config[CONF_VERTICAL_FLIP]))
    cg.add(var.set_horizontal_mirror(config[CONF_HORIZONTAL_MIRROR]))
    cg.add(var.set_brightness(config[CONF_BRIGHTNESS]))
    cg.add(var.set_contrast(config[CONF_CONTRAST]))
    cg.add(var.set_saturation(config[CONF_SATURATION]))
    
    # Définitions pour la compilation
    cg.add_define("USE_TAB5_CAMERA")
    cg.add_define("TAB5_CAMERA_USE_ESP32", 1)
    
    # Flags de build spécifiques au Tab5
    cg.add_build_flag("-DCONFIG_BSP_ERROR_CHECK")
    cg.add_build_flag("-DCONFIG_CAMERA_MODULE_ESP32_CAM")
    cg.add_build_flag("-DBOARD_HAS_PSRAM")
    
    # Dépendances platformio
    cg.add_platformio_option("lib_deps", [
        "espressif/esp32-camera@^2.0.4"
    ])


