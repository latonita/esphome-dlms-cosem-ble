import re
from esphome import pins
import esphome.codegen as cg
import esphome.config_validation as cv
from esphome.components import ble_client, binary_sensor, time
from esphome.components import sensor as esphome_sensor
from esphome.const import (
    CONF_ID,
    CONF_AUTH,
    CONF_NAME,
    CONF_PASSWORD,
    CONF_PIN,
    CONF_SERVICE_UUID,
    CONF_SIGNAL_STRENGTH,
    CONF_TIME_ID,
    CONF_RECEIVE_TIMEOUT,
    CONF_UPDATE_INTERVAL,
    ENTITY_CATEGORY_DIAGNOSTIC,
    DEVICE_CLASS_SIGNAL_STRENGTH,
    STATE_CLASS_MEASUREMENT,
    UNIT_DECIBEL_MILLIWATT,
)

CODEOWNERS = ["@latonita"]

AUTO_LOAD = ["binary_sensor"]

DEPENDENCIES = ["ble_client", "sensor"]

MULTI_CONF = True

DEFAULTS_MAX_SENSOR_INDEX = 12
DEFAULTS_UPDATE_INTERVAL = "60s"
DEFAULTS_RECEIVE_TIMEOUT = "2000ms"

CONF_DLMS_COSEM_BLE_ID = "dlms_cosem_ble_id"
CONF_OBIS_CODE = "obis_code"
CONF_CLIENT_ADDRESS = "client_address"
CONF_SERVER_ADDRESS = "server_address"
CONF_LOGICAL_DEVICE = "logical_device"
CONF_PHYSICAL_DEVICE = "physical_device"
CONF_ADDRESS_LENGTH = "address_length"
CONF_DONT_PUBLISH = "dont_publish"
CONF_CLASS = "class"
CONF_CP1251 = "cp1251"


CONF_READ_UUID = "read_uuid"
CONF_WRITE_UUID = "write_uuid"

dlms_cosem_ble_ns = cg.esphome_ns.namespace("dlms_cosem_ble")
DlmsCosemBle = dlms_cosem_ble_ns.class_(
    "DlmsCosemBleComponent", cg.Component, ble_client.BLEClientNode
)

bt_uuid128_format = "XXXXXXXX-XXXX-XXXX-XXXX-XXXXXXXXXXXX"

def bt_uuid_128(value):
    in_value = cv.string_strict(value)
    value = in_value.upper()

    # if len(value) == len(bt_uuid16_format):
    #     pattern = re.compile("^[A-F|0-9]{4,}$")
    #     if not pattern.match(value):
    #         raise cv.Invalid(
    #             f"Invalid hexadecimal value for 16 bit UUID format: '{in_value}'"
    #         )
    #     return value
    # if len(value) == len(bt_uuid32_format):
    #     pattern = re.compile("^[A-F|0-9]{8,}$")
    #     if not pattern.match(value):
    #         raise cv.Invalid(
    #             f"Invalid hexadecimal value for 32 bit UUID format: '{in_value}'"
    #         )
    #     return value
    if len(value) == len(bt_uuid128_format):
        pattern = re.compile(
            "^[A-F|0-9]{8,}-[A-F|0-9]{4,}-[A-F|0-9]{4,}-[A-F|0-9]{4,}-[A-F|0-9]{12,}$"
        )
        if not pattern.match(value):
            raise cv.Invalid(
                f"Invalid hexadecimal value for 128 UUID format: '{in_value}'"
            )
        return value
    raise cv.Invalid(
#        f"Bluetooth UUID must be in 16 bit '{bt_uuid16_format}', 32 bit '{bt_uuid32_format}', or 128 bit '{bt_uuid128_format}' format"
        f"Bluetooth UUID must be in 128 bit '{bt_uuid128_format}' format"
    )

def obis_code(value):
    value = cv.string(value)
    # examples of valid OBIS codes that should pass:
    #   0.0.96.14.0.255
    #   1-0:32.7.0.255
    #   1-0:32.7.0*255

    # output will always be in format x.x.x.x.x.x

    # So we just accept dots, dashes, colons, and asterisks as separators
    match = re.match(r"^\d{1,3}[.\-:*]\d{1,3}[.\-:*]\d{1,3}[.\-:*]\d{1,3}[.\-:*]\d{1,3}[.\-:*]\d{1,3}$", value)
    if match is None:
        raise cv.Invalid(f"{value} is not a valid OBIS code. Use format A.B.C.D.E.F or A-B:C.D.E*F")
    
    # Normalize to dot-separated format
    normalized = re.sub(r'[.\-:*]', '.', value)
    return normalized


def validate_meter_address(value):
    if len(value) > 15:
        raise cv.Invalid("Meter address length must be no longer than 15 characters")
    return value


CONFIG_SCHEMA = cv.All(
    cv.Schema(
        {
            cv.GenerateID(): cv.declare_id(DlmsCosemBle),
            cv.Optional(
                CONF_UPDATE_INTERVAL, default=DEFAULTS_UPDATE_INTERVAL
            ): cv.update_interval,
            cv.Optional(CONF_SIGNAL_STRENGTH): cv.maybe_simple_value(
                esphome_sensor.sensor_schema(
                    unit_of_measurement=UNIT_DECIBEL_MILLIWATT,
                    accuracy_decimals=0,
                    device_class=DEVICE_CLASS_SIGNAL_STRENGTH,
                    state_class=STATE_CLASS_MEASUREMENT,
                    entity_category=ENTITY_CATEGORY_DIAGNOSTIC,
                ),
                key=CONF_NAME,
            ),
            cv.Optional(CONF_TIME_ID): cv.use_id(time.RealTimeClock),
            cv.Required(CONF_PIN): cv.int_range(min=0,max=999999),
            cv.Required(CONF_SERVICE_UUID): bt_uuid_128,
            cv.Required(CONF_READ_UUID): bt_uuid_128,
            cv.Required(CONF_WRITE_UUID): bt_uuid_128,
            cv.Optional(CONF_CLIENT_ADDRESS, default=16): cv.positive_int,
            cv.Optional(CONF_SERVER_ADDRESS, default=1): cv.Any(
                cv.positive_int,
                cv.Schema({
                    cv.Optional(CONF_LOGICAL_DEVICE, default=1): cv.positive_int,
                    cv.Required(CONF_PHYSICAL_DEVICE): cv.positive_int,
                    cv.Optional(CONF_ADDRESS_LENGTH, default=2): cv.one_of(1, 2, 4),
                })
            ),
            cv.Optional(CONF_AUTH, default=False): cv.boolean,
            cv.Optional(CONF_PASSWORD, default=""): cv.string,
            cv.Optional(
                CONF_RECEIVE_TIMEOUT, default=DEFAULTS_RECEIVE_TIMEOUT
            ): cv.positive_time_period_milliseconds,
        }
    )
    .extend(ble_client.BLE_CLIENT_SCHEMA)
    .extend(cv.polling_component_schema("60s"))
)


async def to_code(config):
    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)
    await ble_client.register_ble_node(var, config)

    if signal_strength_config := config.get(CONF_SIGNAL_STRENGTH):
        sens = await esphome_sensor.new_sensor(signal_strength_config)
        cg.add(var.set_signal_strength(sens))

    if CONF_TIME_ID in config:
        time_ = await cg.get_variable(config[CONF_TIME_ID])
        cg.add(var.set_time_source(time_))

    
    if CONF_SERVICE_UUID in config:
        cg.add(var.set_service_uuid(config[CONF_SERVICE_UUID]))
    
    if CONF_READ_UUID in config:
        cg.add(var.set_read_char_uuid(config[CONF_READ_UUID]))
    
    if CONF_WRITE_UUID in config:
        cg.add(var.set_write_char_uuid(config[CONF_WRITE_UUID]))

    if isinstance(config[CONF_SERVER_ADDRESS], int):
        cg.add(var.set_server_address(config[CONF_SERVER_ADDRESS]))
    else:
        cg.add(var.set_server_address(config[CONF_SERVER_ADDRESS][CONF_LOGICAL_DEVICE], 
                                      config[CONF_SERVER_ADDRESS][CONF_PHYSICAL_DEVICE], 
                                      config[CONF_SERVER_ADDRESS][CONF_ADDRESS_LENGTH]))    
     
    cg.add(var.set_client_address(config[CONF_CLIENT_ADDRESS]))
    cg.add(var.set_auth_required(config[CONF_AUTH]))
    cg.add(var.set_password(config[CONF_PASSWORD]))
    cg.add(var.set_receive_timeout_ms(config[CONF_RECEIVE_TIMEOUT]))

    cg.add(var.set_update_interval(config[CONF_UPDATE_INTERVAL]))

    cg.add(var.set_passkey(config[CONF_PIN]))
    cg.add_library("GuruxDLMS", None, "https://github.com/latonita/GuruxDLMS.c")
