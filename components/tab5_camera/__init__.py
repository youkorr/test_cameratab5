import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import i2c
from esphome.const import (
    CONF_ID,
    CONF_NAME,
    CONF_RESOLUTION,
    CONF_VERTICAL_FLIP,
    CONF_HORIZONTAL_MIRROR,
)
from esphome import pins

CODEOWNERS = ["@youkorr"]
DEPENDENCIES = ["i2c"]

tab5_camera_ns = cg.esphome_ns.namespace("tab5_camera")
Tab5Camera = tab5_camera_ns.class_("Tab5Camera", cg.Component, i2c.I2CDevice)

# Formats de pixels
TAB5_PIXFORMAT_RGB565 = "RGB565"
TAB5_PIXFORMAT_RGB888 = "RGB888"
TAB5_PIXFORMAT_YUV422 = "YUV422"
TAB5_PIXFORMAT_YUV420 = "YUV420"
TAB5_PIXFORMAT_GRAYSCALE = "GRAYSCALE"
TAB5_PIXFORMAT_RAW8 = "RAW8"
TAB5_PIXFORMAT_RAW10 = "RAW10"

PIXEL_FORMATS = {
    TAB5_PIXFORMAT_RGB565: tab5_camera_ns.TAB5_PIXFORMAT_RGB565,
    TAB5_PIXFORMAT_RGB888: tab5_camera_ns.TAB5_PIXFORMAT_RGB888,
    TAB5_PIXFORMAT_YUV422: tab5_camera_ns.TAB5_PIXFORMAT_YUV422,
    TAB5_PIXFORMAT_YUV420: tab5_camera_ns.TAB5_PIXFORMAT_YUV420,
    TAB5_PIXFORMAT_GRAYSCALE: tab5_camera_ns.TAB5_PIXFORMAT_GRAYSCALE,
    TAB5_PIXFORMAT_RAW8: tab5_camera_ns.TAB5_PIXFORMAT_RAW8,
    TAB5_PIXFORMAT_RAW10: tab5_camera_ns.TAB5_PIXFORMAT_RAW10,
}

# Résolutions
TAB5_FRAMESIZE_QVGA = "QVGA"
TAB5_FRAMESIZE_VGA = "VGA"
TAB5_FRAMESIZE_HD = "HD"
TAB5_FRAMESIZE_FULL_HD = "FULL_HD"

FRAMESIZES = {
    TAB5_FRAMESIZE_QVGA: tab5_camera_ns.TAB5_FRAMESIZE_QVGA,
    TAB5_FRAMESIZE_VGA: tab5_camera_ns.TAB5_FRAMESIZE_VGA,
    TAB5_FRAMESIZE_HD: tab5_camera_ns.TAB5_FRAMESIZE_HD,
    TAB5_FRAMESIZE_FULL_HD: tab5_camera_ns.TAB5_FRAMESIZE_FULL_HD,
}

CONF_PIXEL_FORMAT = "pixel_format"
CONF_CAMERA_RESET_PIN = "camera_reset_pin"

CONFIG_SCHEMA = cv.Schema({
    cv.GenerateID(): cv.declare_id(Tab5Camera),
    cv.Optional(CONF_NAME, default="Tab5 Camera"): cv.string,
    cv.Optional(CONF_RESOLUTION, default=TAB5_FRAMESIZE_HD): cv.enum(FRAMESIZES),
    cv.Optional(CONF_PIXEL_FORMAT, default=TAB5_PIXFORMAT_RGB565): cv.enum(PIXEL_FORMATS),
    cv.Optional(CONF_VERTICAL_FLIP, default=False): cv.boolean,
    cv.Optional(CONF_HORIZONTAL_MIRROR, default=True): cv.boolean,
    cv.Optional(CONF_CAMERA_RESET_PIN): pins.gpio_output_pin_schema,
}).extend(cv.COMPONENT_SCHEMA).extend(i2c.i2c_device_schema(0x21))


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await i2c.register_i2c_device(var, config)

    cg.add(var.set_name(config[CONF_NAME]))
    cg.add(var.set_resolution(config[CONF_RESOLUTION]))
    cg.add(var.set_pixel_format(config[CONF_PIXEL_FORMAT]))
    cg.add(var.set_vertical_flip(config[CONF_VERTICAL_FLIP]))
    cg.add(var.set_horizontal_mirror(config[CONF_HORIZONTAL_MIRROR]))
    
    if CONF_CAMERA_RESET_PIN in config:
        pin = await cg.gpio_pin_expression(config[CONF_CAMERA_RESET_PIN])
        cg.add(var.set_camera_reset_pin(pin))

