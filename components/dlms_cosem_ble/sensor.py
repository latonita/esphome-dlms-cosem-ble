import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import sensor
from . import (
    DlmsCosemBle,
    dlms_cosem_ble_ns,
    obis_code,
    CONF_DLMS_COSEM_BLE_ID,
    CONF_OBIS_CODE,
    CONF_DONT_PUBLISH,
    CONF_CLASS,
)

DlmsCosemBleSensor = dlms_cosem_ble_ns.class_("DlmsCosemBleSensor", sensor.Sensor)

CONF_MULTIPLIER = "multiplier"

CONFIG_SCHEMA = cv.All(
    sensor.sensor_schema(
        DlmsCosemBleSensor,
    ).extend(
        {
            cv.GenerateID(CONF_DLMS_COSEM_BLE_ID): cv.use_id(DlmsCosemBle),
            cv.Required(CONF_OBIS_CODE): obis_code,
            cv.Optional(CONF_DONT_PUBLISH, default=False): cv.boolean,
            cv.Optional(CONF_MULTIPLIER, default=1.0): cv.float_,
            cv.Optional(CONF_CLASS, default=3): cv.int_,
        }
    ),
    cv.has_exactly_one_key(CONF_OBIS_CODE),
)


async def to_code(config):
    component = await cg.get_variable(config[CONF_DLMS_COSEM_BLE_ID])
    var = await sensor.new_sensor(config)
    cg.add(var.set_obis_code(config[CONF_OBIS_CODE]))
    cg.add(var.set_dont_publish(config.get(CONF_DONT_PUBLISH)))
    cg.add(var.set_multiplier(config[CONF_MULTIPLIER]))
    cg.add(var.set_obis_class(config[CONF_CLASS]))
    cg.add(component.register_sensor(var))
