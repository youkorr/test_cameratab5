import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_NAME, CONF_RESOLUTION
from esphome import pins

# Définir les constantes nécessaires
CONF_HORIZONTAL_MIRROR = "horizontal_mirror"
CONF_PIXEL_FORMAT = "pixel_format"
CONF_CAMERA_RESET_PIN = "camera_reset_pin"
CONF_JPEG_QUALITY = "jpeg_quality"
CONF_BRIGHTNESS = "brightness"
CONF_CONTRAST = "contrast"
CONF_SATURATION = "saturation"

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Formats de pixels
tab5_pixformat_t = tab5_camera_ns.enum("tab5_pixformat_t")
PIXEL_FORMATS = {
    "RGB565": tab5_pixformat_t.TAB5_PIXFORMAT_RGB565,
    "RGB888": tab5_pixformat_t.TAB5_PIXFORMAT_RGB888,
    "YUV422": tab5_pixformat_t.TAB5_PIXFORMAT_YUV422,
    "YUV420": tab5_pixformat_t.TAB5_PIXFORMAT_YUV420,
    "GRAYSCALE": tab5_pixformat_t.TAB5_PIXFORMAT_GRAYSCALE,
    "RAW8": tab5_pixformat_t.TAB5_PIXFORMAT_RAW8,
    "RAW10": tab5_pixformat_t.TAB5_PIXFORMAT_RAW10,
}

# Résolutions
tab5_framesize_t = tab5_camera_ns.enum("tab5_framesize_t")
FRAMESIZES = {
    "QVGA": tab5_framesize_t.TAB5_FRAMESIZE_QVGA,
    "VGA": tab5_framesize_t.TAB5_FRAMESIZE_VGA,
    "HD": tab5_framesize_t.TAB5_FRAMESIZE_HD,
    "FULL_HD": tab5_framesize_t.TAB5_FRAMESIZE_FULL_HD,
}

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Tab5Camera),
    cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
    cv.Optional(CONF_RESOLUTION, default="HD"): cv.enum(FRAMESIZES),
    cv.Optional(CONF_PIXEL_FORMAT, default="RGB565"): cv.enum(PIXEL_FORMATS),
    cv.Optional(CONF_HORIZONTAL_MIRROR, default=True): cv.boolean,
    cv.Optional(CONF_CAMERA_RESET_PIN): pins.gpio_output_pin_schema,
    cv.Optional(CONF_JPEG_QUALITY, default=12): cv.int_range(min=1, max=63),
    cv.Optional(CONF_BRIGHTNESS, default=0): cv.int_range(min=-2, max=2),
    cv.Optional(CONF_CONTRAST, default=0): cv.int_range(min=-2, max=2),
    cv.Optional(CONF_SATURATION, default=0): cv.int_range(min=-2, max=2),
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x43))

async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)
    
    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_resolution(config[CONF_RESOLUTION]))
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_horizontal_mirror(config[CONF_HORIZONTAL_MIRROR]))
    
    if CONF_CAMERA_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_CAMERA_RESET_PIN])
        cg.add(var.set_camera_reset_pin(pin))
    
    cg.add_library("esp_video_init", None)
    cg.add_library("driver/ppa", None)

