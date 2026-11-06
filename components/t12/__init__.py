"""ESPHome component for the T12 soldering station.

The generated C++ owns the 100 Hz frame, the heater-off thermocouple interlock,
the ADS1115 read, the encoder/menu state machine and the OLED layout. The
control mathematics come from ``lib/control`` at the repository root, compiled
in place as a PlatformIO library so the native simulation and this firmware run
byte-identical scheduler and PID code.
"""
from pathlib import Path

import esphome.codegen as cg
import esphome.config_validation as cv
from esphome import pins
from esphome.components import i2c
from esphome.const import CONF_ID, CONF_I2C_ID, CONF_INVERTED

# display and font are compile-time dependencies: t12_component.h includes the
# display::Display and font::Font headers for renderDisplay().
DEPENDENCIES = ["i2c", "display", "font"]
MULTI_CONF = False

CONF_HEATER_PIN = "heater_pin"
CONF_SHAKE_PIN = "shake_pin"
CONF_BUZZER_PIN = "buzzer_pin"
CONF_ADS1115_ADDRESS = "ads1115_address"
CONF_FRONTEND_MODE = "frontend_mode"
CONF_CAL_ADC_200 = "cal_adc_200"
CONF_CAL_ADC_280 = "cal_adc_280"
CONF_CAL_ADC_360 = "cal_adc_360"
CONF_CAL_AMBIENT = "cal_ambient_c"
CONF_TC_UV_PER_C = "tc_uv_per_c"
CONF_CJC_TEMP = "cjc_temp_c"
CONF_EMA_ALPHA = "ema_alpha"
CONF_KP = "kp"
CONF_KI = "ki"
CONF_KD = "kd"
CONF_DEFAULT_SETPOINT = "default_setpoint"
CONF_TIP_NAME = "tip_name"
CONF_CSV_TELEMETRY = "csv_telemetry"

# FrontEndMode in lib/control/heater_timing.h
FRONTEND_MODES = {"OPAMP_3POINT": 0, "DIRECT_TC": 1}

t12_ns = cg.esphome_ns.namespace("t12")
T12Component = t12_ns.class_("T12Component", cg.Component, i2c.I2CDevice)


def validate_heater_pin(value):
    """Reject a non-inverted heater output.

    The 2N2222A / IRLZ44N low-side stage switches the heater on while the pin is
    high, so an un-inverted pin would drive the heater full on from the first
    boot frame, before the scheduler can clamp it.
    """
    if not value[CONF_INVERTED]:
        raise cv.Invalid(
            "heater_pin must set inverted: true. The 2N2222A / IRLZ44N stage "
            "switches the heater on while the pin is high, so a non-inverted "
            "output drives the heater full on at boot.",
            path=[CONF_INVERTED],
        )
    return value


CONFIG_SCHEMA = cv.Schema(
    {
        cv.GenerateID(): cv.declare_id(T12Component),
        cv.Required(CONF_HEATER_PIN): cv.All(
            pins.gpio_output_pin_schema, validate_heater_pin
        ),
        cv.GenerateID(CONF_I2C_ID): cv.use_id(i2c.I2CBus),
        cv.Optional(CONF_ADS1115_ADDRESS, default=0x48): cv.i2c_address,
        cv.Optional(CONF_SHAKE_PIN): pins.gpio_input_pin_schema,
        cv.Optional(CONF_BUZZER_PIN): pins.gpio_output_pin_schema,
        cv.Optional(CONF_FRONTEND_MODE, default="OPAMP_3POINT"): cv.enum(
            FRONTEND_MODES, upper=True
        ),
        cv.Optional(CONF_CAL_ADC_200, default=216.0): cv.float_,
        cv.Optional(CONF_CAL_ADC_280, default=308.0): cv.float_,
        cv.Optional(CONF_CAL_ADC_360, default=390.0): cv.float_,
        cv.Optional(CONF_CAL_AMBIENT, default=21.0): cv.float_,
        cv.Optional(CONF_TC_UV_PER_C, default=21.0): cv.float_,
        cv.Optional(CONF_CJC_TEMP, default=26.0): cv.float_,
        cv.Optional(CONF_EMA_ALPHA, default=1.0): cv.float_range(min=0.01, max=1.0),
        cv.Optional(CONF_KP, default=0.0048): cv.float_,
        cv.Optional(CONF_KI, default=0.0036): cv.float_,
        cv.Optional(CONF_KD, default=0.0): cv.float_,
        cv.Optional(CONF_DEFAULT_SETPOINT, default=320.0): cv.float_range(
            min=150.0, max=400.0
        ),
        cv.Optional(CONF_TIP_NAME, default="BC1.5"): cv.string,
        cv.Optional(CONF_CSV_TELEMETRY, default=True): cv.boolean,
    }
).extend(cv.COMPONENT_SCHEMA)


async def to_code(config):
    repo_root = Path(__file__).resolve().parents[2]
    control_dir = repo_root / "lib" / "control"
    manifest = control_dir / "library.json"

    # Fail here rather than inside PlatformIO: ESPHome 2026.10 turns a local
    # library directory into an IDF component, which needs library.json.
    if not manifest.is_file():
        raise cv.Invalid(
            f"library.json is missing: {manifest}. The t12 component compiles "
            "lib/control in place, so lib/control/library.json must exist."
        )

    # lib/control stays the single source of truth: put it on the include path
    # and hand the directory to the build as a local library, so pid.cpp and
    # heater_timing.cpp are compiled from where they live, not from a copy.
    cg.add_build_flag(f"-I{control_dir}")
    cg.add_library("control", None, control_dir.as_uri())

    var = cg.new_Pvariable(config[CONF_ID])
    await cg.register_component(var, config)

    heater_pin = await cg.gpio_pin_expression(config[CONF_HEATER_PIN])
    cg.add(var.set_heater_pin(heater_pin))
    if CONF_SHAKE_PIN in config:
        shake_pin = await cg.gpio_pin_expression(config[CONF_SHAKE_PIN])
        cg.add(var.set_shake_pin(shake_pin))
    if CONF_BUZZER_PIN in config:
        buzzer_pin = await cg.gpio_pin_expression(config[CONF_BUZZER_PIN])
        cg.add(var.set_buzzer_pin(buzzer_pin))

    parent = await cg.get_variable(config[CONF_I2C_ID])
    cg.add(var.set_i2c_bus(parent))
    cg.add(var.set_i2c_address(config[CONF_ADS1115_ADDRESS]))

    cg.add(var.set_frontend_mode(config[CONF_FRONTEND_MODE]))
    cg.add(var.set_cal_200(config[CONF_CAL_ADC_200]))
    cg.add(var.set_cal_280(config[CONF_CAL_ADC_280]))
    cg.add(var.set_cal_360(config[CONF_CAL_ADC_360]))
    cg.add(var.set_cal_ambient(config[CONF_CAL_AMBIENT]))
    cg.add(var.set_tc_uv_per_c(config[CONF_TC_UV_PER_C]))
    cg.add(var.set_cjc_temp(config[CONF_CJC_TEMP]))
    cg.add(var.set_ema_alpha(config[CONF_EMA_ALPHA]))
    cg.add(var.set_gains(config[CONF_KP], config[CONF_KI], config[CONF_KD]))
    cg.add(var.set_default_setpoint(config[CONF_DEFAULT_SETPOINT]))
    cg.add(var.set_tip_name(config[CONF_TIP_NAME]))
    cg.add(var.set_csv_telemetry(config[CONF_CSV_TELEMETRY]))
