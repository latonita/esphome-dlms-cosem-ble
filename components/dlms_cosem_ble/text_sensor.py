import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import text_sensor
from . import (
    DlmsCosemBle,
    dlms_cosem_ble_ns,
    obis_code,
    CONF_DLMS_COSEM_BLE_ID,
    CONF_OBIS_CODE,
    CONF_DONT_PUBLISH,
    CONF_CLASS,
    CONF_CP1251,
)

#AUTO_LOAD = ["dlms_cosem_ble"]

DlmsCosemBleTextSensor = dlms_cosem_ble_ns.class_(
    "DlmsCosemBleTextSensor", text_sensor.TextSensor
)

CONF_SHOW_AS_HEX = "show_as_hex"
# class can be 1,3,4,8

CONFIG_SCHEMA = cv.All(
    text_sensor.text_sensor_schema(
        DlmsCosemBleTextSensor,
    ).extend(
        {
            cv.GenerateID(CONF_DLMS_COSEM_BLE_ID): cv.use_id(DlmsCosemBle),
            cv.Required(CONF_OBIS_CODE): obis_code,
            cv.Optional(CONF_DONT_PUBLISH, default=False): cv.boolean,
            cv.Optional(CONF_CLASS, default=1): cv.one_of(1, 3, 4, 8, int=True),
            cv.Optional(CONF_CP1251): cv.boolean,
            cv.Optional(CONF_SHOW_AS_HEX, default=False): cv.boolean,
        }
    ),
    cv.has_exactly_one_key(CONF_OBIS_CODE),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_DLMS_COSEM_BLE_ID])
    var = await text_sensor.new_text_sensor(config)
    cg.add(var.set_obis_code(config[CONF_OBIS_CODE]))
    cg.add(var.set_dont_publish(config.get(CONF_DONT_PUBLISH)))
    cg.add(var.set_obis_class(config[CONF_CLASS]))
    cg.add(var.set_show_as_hex(config[CONF_SHOW_AS_HEX]))
    if conf := config.get(CONF_CP1251):
        cg.add(var.set_cp1251_conversion_required(config[CONF_CP1251]))
        
    cg.add(component.register_sensor(var))
