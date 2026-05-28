#include "tmc5130.h"

#include <math.h>
#include <string.h>

#define TMC5130_WRITE_MASK 0x80u
#define TMC5130_SECONDS_PER_MINUTE 60.0
#define TMC5130_FCLK_HZ 12000000.0

#define REG_GCONF 0x00u
#define REG_IOIN 0x04u
#define REG_IHOLD_IRUN 0x10u
#define REG_TPOWERDOWN 0x11u
#define REG_TPWMTHRS 0x13u
#define REG_TCOOLTHRS 0x14u
#define REG_RAMPMODE 0x20u
#define REG_XACTUAL 0x21u
#define REG_VSTART 0x23u
#define REG_A1 0x24u
#define REG_V1 0x25u
#define REG_AMAX 0x26u
#define REG_VMAX 0x27u
#define REG_DMAX 0x28u
#define REG_D1 0x2Au
#define REG_VSTOP 0x2Bu
#define REG_TZEROWAIT 0x2Cu
#define REG_XTARGET 0x2Du
#define REG_CHOPCONF 0x6Cu
#define REG_COOLCONF 0x6Du
#define REG_PWMCONF 0x70u

static uint32_t field_mask(uint32_t width) {
    return width >= 32u ? 0xFFFFFFFFu : ((1u << width) - 1u);
}

static void set_bits(uint32_t *reg, uint32_t shift, uint32_t width, uint32_t value) {
    uint32_t mask = field_mask(width) << shift;
    *reg = (*reg & ~mask) | ((value & field_mask(width)) << shift);
}

static int write_register(tmc5130_t *driver, uint8_t address, uint32_t value) {
    uint8_t payload[5] = {
        (uint8_t)(address | TMC5130_WRITE_MASK),
        (uint8_t)(value >> 24),
        (uint8_t)(value >> 16),
        (uint8_t)(value >> 8),
        (uint8_t)(value)
    };
    if (driver == NULL || driver->spi.write == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    return driver->spi.write(driver->spi.context, payload, sizeof(payload)) == 0 ? TMC5130_OK : TMC5130_ERROR_SPI;
}

static int read_register(tmc5130_t *driver, uint8_t address, uint8_t *status, uint32_t *value) {
    uint8_t request[5] = {address, 0u, 0u, 0u, 0u};
    uint8_t response[5] = {0u, 0u, 0u, 0u, 0u};
    if (driver == NULL || driver->spi.transfer == NULL || status == NULL || value == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    if (driver->spi.transfer(driver->spi.context, request, response, sizeof(request)) != 0) {
        return TMC5130_ERROR_SPI;
    }
    if (driver->spi.transfer(driver->spi.context, request, response, sizeof(request)) != 0) {
        return TMC5130_ERROR_SPI;
    }
    *status = response[0];
    *value = ((uint32_t)response[1] << 24) | ((uint32_t)response[2] << 16) | ((uint32_t)response[3] << 8) | response[4];
    return TMC5130_OK;
}

static uint32_t clamp_current(double rsense, uint16_t current_mA) {
    double value;
    if (current_mA == 0u) {
        value = 0.0;
    } else {
        value = 32.0 * 1.41421 * ((double)current_mA / 1000.0) * (rsense + 0.02) / 0.325 - 1.0;
    }
    if (value < 0.0) {
        value = 0.0;
    }
    if (value > 31.0) {
        value = 31.0;
    }
    return (uint32_t)value;
}

static uint32_t ustep_t(double usteps_per_second) {
    double value = usteps_per_second / (TMC5130_FCLK_HZ / 2.0 / pow(2.0, 23.0));
    return value <= 0.0 ? 0u : (uint32_t)llround(value);
}

static uint32_t ustep_ta2(double usteps_per_second_squared) {
    double value = usteps_per_second_squared * ((512.0 * 256.0) * pow(2.0, 24.0) / pow(TMC5130_FCLK_HZ, 2.0));
    return value <= 0.0 ? 0u : (uint32_t)llround(value);
}

static int32_t from_units(const tmc5130_t *driver, double units) {
    double scale = (double)driver->config.motor.steps_per_rev * (double)driver->config.driver.microstep;
    return (int32_t)llround((units / driver->config.motor.units_per_rev) * scale);
}

static uint32_t from_rps(const tmc5130_t *driver, double rps) {
    double usteps_per_second = rps * (double)driver->config.driver.microstep * (double)driver->config.motor.steps_per_rev;
    return ustep_t(usteps_per_second);
}

static uint32_t from_rps2(const tmc5130_t *driver, double rps2) {
    double usteps_per_second_squared = rps2 * (double)driver->config.driver.microstep * (double)driver->config.motor.steps_per_rev;
    return ustep_ta2(usteps_per_second_squared);
}

static uint32_t from_units_per_second(const tmc5130_t *driver, double units) {
    return ustep_t((double)from_units(driver, units));
}

static uint32_t from_units_per_second_squared(const tmc5130_t *driver, double units) {
    return ustep_ta2((double)from_units(driver, units));
}

static double to_units(const tmc5130_t *driver, int32_t microsteps) {
    double scale = (double)driver->config.motor.steps_per_rev * (double)driver->config.driver.microstep * driver->config.motor.units_per_rev;
    return (double)microsteps / scale;
}

void tmc5130_default_config(tmc5130_config_t *config) {
    if (config == NULL) {
        return;
    }
    config->ramp_mode = 1u;
    config->rsense = 0.15;
    config->current_mA.hold_mA = 50u;
    config->current_mA.run_mA = 200u;
    config->driver.microstep = 256u;
    config->motion.velocity_rpm = 0.0;
    config->motion.acceleration_rpm = 60.0;
    config->motion.deceleration_rpm = 60.0;
    config->motor.steps_per_rev = 200u;
    config->motor.units_per_rev = 1.0;
}

int tmc5130_init_driver(tmc5130_t *driver, tmc5130_spi_t spi, const tmc5130_config_t *config) {
    if (driver == NULL || config == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    memset(driver, 0, sizeof(*driver));
    driver->spi = spi;
    driver->config = *config;
    return TMC5130_OK;
}

int tmc5130_init(tmc5130_t *driver) {
    uint32_t general_config = 0u;
    uint32_t chop_conf = 0u;
    uint32_t cool_conf = 0u;
    uint32_t pwm_conf = 0u;

    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }

    set_bits(&general_config, 4u, 1u, 0u);
    set_bits(&general_config, 7u, 1u, 0u);
    set_bits(&general_config, 12u, 1u, 0u);
    set_bits(&general_config, 2u, 1u, 1u);

    set_bits(&chop_conf, 0u, 4u, 0x03u);
    set_bits(&chop_conf, 4u, 3u, 0x04u);
    set_bits(&chop_conf, 7u, 4u, 0x01u);
    set_bits(&chop_conf, 14u, 1u, 0x01u);
    set_bits(&chop_conf, 15u, 2u, 0x02u);
    set_bits(&chop_conf, 18u, 1u, 0x01u);
    set_bits(&chop_conf, 19u, 1u, 0x01u);

    set_bits(&cool_conf, 16u, 7u, 0x00u);

    set_bits(&pwm_conf, 0u, 8u, 0xFFu);
    set_bits(&pwm_conf, 8u, 8u, 0x04u);
    set_bits(&pwm_conf, 18u, 1u, 1u);
    set_bits(&pwm_conf, 20u, 2u, 0x01u);

    if (write_register(driver, REG_GCONF, general_config) != TMC5130_OK ||
        write_register(driver, REG_CHOPCONF, chop_conf) != TMC5130_OK ||
        write_register(driver, REG_COOLCONF, cool_conf) != TMC5130_OK ||
        write_register(driver, REG_PWMCONF, pwm_conf) != TMC5130_OK ||
        write_register(driver, REG_TPWMTHRS, 500u) != TMC5130_OK ||
        write_register(driver, REG_TCOOLTHRS, 0u) != TMC5130_OK ||
        write_register(driver, REG_TPOWERDOWN, 0x10u) != TMC5130_OK ||
        write_register(driver, REG_RAMPMODE, driver->config.ramp_mode & 0x03u) != TMC5130_OK) {
        return TMC5130_ERROR_SPI;
    }

    if (tmc5130_set_current(driver, driver->config.current_mA.run_mA, driver->config.current_mA.hold_mA) != TMC5130_OK ||
        tmc5130_set_target_position(driver, 0.0) != TMC5130_OK ||
        tmc5130_set_current_position(driver, 0.0) != TMC5130_OK ||
        tmc5130_set_motion_rpm(driver, 0.0, 0.0, 0.0) != TMC5130_OK) {
        return TMC5130_ERROR_SPI;
    }

    return TMC5130_OK;
}

int tmc5130_set_current(tmc5130_t *driver, uint16_t run_mA, uint16_t hold_mA) {
    uint32_t reg = 0u;
    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    set_bits(&reg, 0u, 5u, clamp_current(driver->config.rsense, hold_mA));
    set_bits(&reg, 8u, 5u, clamp_current(driver->config.rsense, run_mA));
    set_bits(&reg, 16u, 4u, 3u);
    return write_register(driver, REG_IHOLD_IRUN, reg);
}

int tmc5130_set_motion_rpm(tmc5130_t *driver, double velocity_rpm, double acceleration_rpm, double deceleration_rpm) {
    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    return tmc5130_set_motion(
        driver,
        from_rps(driver, velocity_rpm / TMC5130_SECONDS_PER_MINUTE),
        from_rps2(driver, acceleration_rpm / TMC5130_SECONDS_PER_MINUTE),
        from_rps2(driver, deceleration_rpm / TMC5130_SECONDS_PER_MINUTE),
        1.0
    );
}

int tmc5130_set_motion_units(tmc5130_t *driver, double velocity_units, double acceleration_units, double deceleration_units) {
    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    return tmc5130_set_motion(
        driver,
        from_units_per_second(driver, velocity_units),
        from_units_per_second_squared(driver, acceleration_units),
        from_units_per_second_squared(driver, deceleration_units),
        1.0
    );
}

int tmc5130_set_motion(tmc5130_t *driver, uint32_t velocity, uint32_t acceleration, uint32_t deceleration, double stage_1_2) {
    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    if (write_register(driver, REG_AMAX, acceleration) != TMC5130_OK ||
        write_register(driver, REG_DMAX, deceleration) != TMC5130_OK ||
        write_register(driver, REG_VMAX, velocity) != TMC5130_OK ||
        write_register(driver, REG_A1, (uint32_t)llround((double)acceleration * stage_1_2)) != TMC5130_OK ||
        write_register(driver, REG_D1, (uint32_t)llround((double)deceleration * stage_1_2)) != TMC5130_OK ||
        write_register(driver, REG_V1, (uint32_t)llround((double)velocity * stage_1_2)) != TMC5130_OK ||
        write_register(driver, REG_VSTART, 0u) != TMC5130_OK ||
        write_register(driver, REG_VSTOP, 10u) != TMC5130_OK ||
        write_register(driver, REG_TZEROWAIT, 0u) != TMC5130_OK) {
        return TMC5130_ERROR_SPI;
    }
    return TMC5130_OK;
}

int tmc5130_set_velocity_rpm(tmc5130_t *driver, double velocity_rpm) {
    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    return write_register(driver, REG_VMAX, from_rps(driver, velocity_rpm / TMC5130_SECONDS_PER_MINUTE));
}

int tmc5130_read_version(tmc5130_t *driver, uint8_t *version) {
    uint8_t status = 0u;
    uint32_t value = 0u;
    if (version == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    if (read_register(driver, REG_IOIN, &status, &value) != TMC5130_OK) {
        return TMC5130_ERROR_SPI;
    }
    (void)status;
    *version = (uint8_t)((value >> 24) & 0xFFu);
    return TMC5130_OK;
}

int tmc5130_get_io_status(tmc5130_t *driver, tmc5130_io_status_t *status) {
    uint8_t read_status = 0u;
    uint32_t value = 0u;
    int rc;
    if (driver == NULL || status == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    rc = read_register(driver, REG_IOIN, &read_status, &value);
    if (rc != TMC5130_OK) {
        return rc;
    }
    (void)read_status;
    status->refl_step = (value & (1u << 0)) != 0u;
    status->refr_dir = (value & (1u << 1)) != 0u;
    status->encb_dcen_cfg4 = (value & (1u << 2)) != 0u;
    status->enca_dcin_cfg5 = (value & (1u << 3)) != 0u;
    status->drv_enn_cfg6 = (value & (1u << 4)) != 0u;
    status->enc_n_dco = (value & (1u << 5)) != 0u;
    status->sd_mode = (value & (1u << 6)) != 0u;
    status->swcomp_in = (value & (1u << 7)) != 0u;
    status->version = (uint8_t)((value >> 24) & 0xFFu);
    return TMC5130_OK;
}

int tmc5130_set_target_position(tmc5130_t *driver, double position_units) {
    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    return write_register(driver, REG_XTARGET, (uint32_t)from_units(driver, position_units));
}

int tmc5130_set_current_position(tmc5130_t *driver, double position_units) {
    int32_t position;
    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    position = from_units(driver, position_units);
    if (write_register(driver, REG_XACTUAL, (uint32_t)position) != TMC5130_OK) {
        return TMC5130_ERROR_SPI;
    }
    return write_register(driver, REG_XTARGET, (uint32_t)position);
}

int tmc5130_get_current_position(tmc5130_t *driver, double *position_units) {
    uint8_t status = 0u;
    uint32_t value = 0u;
    int rc;
    if (driver == NULL || position_units == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    rc = read_register(driver, REG_XACTUAL, &status, &value);
    if (rc != TMC5130_OK) {
        return rc;
    }
    (void)status;
    *position_units = to_units(driver, (int32_t)value);
    return TMC5130_OK;
}

int tmc5130_stop_motion(tmc5130_t *driver) {
    if (driver == NULL) {
        return TMC5130_ERROR_ARGUMENT;
    }
    if (write_register(driver, REG_VSTART, 0u) != TMC5130_OK) {
        return TMC5130_ERROR_SPI;
    }
    return write_register(driver, REG_VMAX, 0u);
}
