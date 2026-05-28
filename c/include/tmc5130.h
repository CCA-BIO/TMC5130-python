#ifndef TMC5130_H
#define TMC5130_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef int (*tmc5130_spi_write_fn)(void *context, const uint8_t *data, size_t length);
typedef int (*tmc5130_spi_transfer_fn)(void *context, const uint8_t *write_data, uint8_t *read_data, size_t length);

typedef struct {
    void *context;
    tmc5130_spi_write_fn write;
    tmc5130_spi_transfer_fn transfer;
} tmc5130_spi_t;

typedef struct {
    uint16_t hold_mA;
    uint16_t run_mA;
} tmc5130_current_config_t;

typedef struct {
    uint16_t microstep;
} tmc5130_driver_config_t;

typedef struct {
    double velocity_rpm;
    double acceleration_rpm;
    double deceleration_rpm;
} tmc5130_motion_config_t;

typedef struct {
    uint16_t steps_per_rev;
    double units_per_rev;
} tmc5130_motor_config_t;

typedef struct {
    uint8_t ramp_mode;
    double rsense;
    tmc5130_current_config_t current_mA;
    tmc5130_driver_config_t driver;
    tmc5130_motion_config_t motion;
    tmc5130_motor_config_t motor;
} tmc5130_config_t;

typedef struct {
    bool refl_step;
    bool refr_dir;
    bool enca_dcin_cfg5;
    bool encb_dcen_cfg4;
    bool drv_enn_cfg6;
    bool enc_n_dco;
    bool sd_mode;
    bool swcomp_in;
    uint8_t version;
} tmc5130_io_status_t;

typedef struct {
    tmc5130_spi_t spi;
    tmc5130_config_t config;
} tmc5130_t;

enum {
    TMC5130_OK = 0,
    TMC5130_ERROR_ARGUMENT = -1,
    TMC5130_ERROR_SPI = -2
};

void tmc5130_default_config(tmc5130_config_t *config);
int tmc5130_init_driver(tmc5130_t *driver, tmc5130_spi_t spi, const tmc5130_config_t *config);
int tmc5130_init(tmc5130_t *driver);
int tmc5130_set_current(tmc5130_t *driver, uint16_t run_mA, uint16_t hold_mA);
int tmc5130_set_motion_rpm(tmc5130_t *driver, double velocity_rpm, double acceleration_rpm, double deceleration_rpm);
int tmc5130_set_motion_units(tmc5130_t *driver, double velocity_units, double acceleration_units, double deceleration_units);
int tmc5130_set_motion(tmc5130_t *driver, uint32_t velocity, uint32_t acceleration, uint32_t deceleration, double stage_1_2);
int tmc5130_set_velocity_rpm(tmc5130_t *driver, double velocity_rpm);
int tmc5130_read_version(tmc5130_t *driver, uint8_t *version);
int tmc5130_get_io_status(tmc5130_t *driver, tmc5130_io_status_t *status);
int tmc5130_set_target_position(tmc5130_t *driver, double position_units);
int tmc5130_set_current_position(tmc5130_t *driver, double position_units);
int tmc5130_get_current_position(tmc5130_t *driver, double *position_units);
int tmc5130_stop_motion(tmc5130_t *driver);

#ifdef __cplusplus
}
#endif

#endif
