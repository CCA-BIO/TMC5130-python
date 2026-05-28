#include "tmc5130.hpp"

#include <cmath>

namespace tmc5130 {
namespace {
constexpr std::uint8_t kWriteMask = 0x80;
constexpr double kSecondsPerMinute = 60.0;
constexpr double kFclkHz = 12000000.0;

constexpr std::uint8_t kRegGconf = 0x00;
constexpr std::uint8_t kRegIoin = 0x04;
constexpr std::uint8_t kRegIholdIrun = 0x10;
constexpr std::uint8_t kRegTpowerdown = 0x11;
constexpr std::uint8_t kRegTpwmthrs = 0x13;
constexpr std::uint8_t kRegTcoolthrs = 0x14;
constexpr std::uint8_t kRegRampmode = 0x20;
constexpr std::uint8_t kRegXactual = 0x21;
constexpr std::uint8_t kRegVstart = 0x23;
constexpr std::uint8_t kRegA1 = 0x24;
constexpr std::uint8_t kRegV1 = 0x25;
constexpr std::uint8_t kRegAmax = 0x26;
constexpr std::uint8_t kRegVmax = 0x27;
constexpr std::uint8_t kRegDmax = 0x28;
constexpr std::uint8_t kRegD1 = 0x2A;
constexpr std::uint8_t kRegVstop = 0x2B;
constexpr std::uint8_t kRegTzerowait = 0x2C;
constexpr std::uint8_t kRegXtarget = 0x2D;
constexpr std::uint8_t kRegChopconf = 0x6C;
constexpr std::uint8_t kRegCoolconf = 0x6D;
constexpr std::uint8_t kRegPwmconf = 0x70;

std::uint32_t mask(std::uint32_t width) {
    return width >= 32 ? 0xFFFFFFFFu : ((1u << width) - 1u);
}

void setBits(std::uint32_t& reg, std::uint32_t shift, std::uint32_t width, std::uint32_t value) {
    const auto field_mask = mask(width) << shift;
    reg = (reg & ~field_mask) | ((value & mask(width)) << shift);
}

std::uint32_t clampCurrent(double rsense, std::uint16_t current_mA) {
    double value = current_mA == 0 ? 0.0 : 32.0 * 1.41421 * (static_cast<double>(current_mA) / 1000.0) * (rsense + 0.02) / 0.325 - 1.0;
    if (value < 0.0) value = 0.0;
    if (value > 31.0) value = 31.0;
    return static_cast<std::uint32_t>(value);
}

std::uint32_t ustepT(double usteps_per_second) {
    const auto value = usteps_per_second / (kFclkHz / 2.0 / std::pow(2.0, 23.0));
    return value <= 0.0 ? 0u : static_cast<std::uint32_t>(std::llround(value));
}

std::uint32_t ustepTa2(double usteps_per_second_squared) {
    const auto value = usteps_per_second_squared * ((512.0 * 256.0) * std::pow(2.0, 24.0) / std::pow(kFclkHz, 2.0));
    return value <= 0.0 ? 0u : static_cast<std::uint32_t>(std::llround(value));
}
}  // namespace

Driver::Driver(SpiDevice& spi, Config config) : spi_(spi), config_(config) {}

void Driver::init() {
    std::uint32_t general_config = 0;
    std::uint32_t chop_conf = 0;
    std::uint32_t cool_conf = 0;
    std::uint32_t pwm_conf = 0;

    setBits(general_config, 4, 1, 0);
    setBits(general_config, 7, 1, 0);
    setBits(general_config, 12, 1, 0);
    setBits(general_config, 2, 1, 1);

    setBits(chop_conf, 0, 4, 0x03);
    setBits(chop_conf, 4, 3, 0x04);
    setBits(chop_conf, 7, 4, 0x01);
    setBits(chop_conf, 14, 1, 0x01);
    setBits(chop_conf, 15, 2, 0x02);
    setBits(chop_conf, 18, 1, 0x01);
    setBits(chop_conf, 19, 1, 0x01);

    setBits(cool_conf, 16, 7, 0x00);

    setBits(pwm_conf, 0, 8, 0xFF);
    setBits(pwm_conf, 8, 8, 0x04);
    setBits(pwm_conf, 18, 1, 1);
    setBits(pwm_conf, 20, 2, 0x01);

    writeRegister(kRegGconf, general_config);
    writeRegister(kRegChopconf, chop_conf);
    writeRegister(kRegCoolconf, cool_conf);
    writeRegister(kRegPwmconf, pwm_conf);
    writeRegister(kRegTpwmthrs, 500);
    writeRegister(kRegTcoolthrs, 0);
    writeRegister(kRegTpowerdown, 0x10);
    writeRegister(kRegRampmode, config_.ramp_mode & 0x03u);
    setCurrent(config_.current_mA.run_mA, config_.current_mA.hold_mA);
    setTargetPosition(0.0);
    setCurrentPosition(0.0);
    setMotionRpm(0.0, 0.0, 0.0);
}

void Driver::setCurrent(std::uint16_t run_mA, std::uint16_t hold_mA) {
    std::uint32_t reg = 0;
    setBits(reg, 0, 5, clampCurrent(config_.rsense, hold_mA));
    setBits(reg, 8, 5, clampCurrent(config_.rsense, run_mA));
    setBits(reg, 16, 4, 3);
    writeRegister(kRegIholdIrun, reg);
}

void Driver::setMotionRpm(double velocity_rpm, double acceleration_rpm, double deceleration_rpm) {
    setMotion(
        fromRps(velocity_rpm / kSecondsPerMinute),
        fromRps2(acceleration_rpm / kSecondsPerMinute),
        fromRps2(deceleration_rpm / kSecondsPerMinute),
        1.0
    );
}

void Driver::setMotionUnits(double velocity_units, double acceleration_units, double deceleration_units) {
    setMotion(
        fromUnitsPerSecond(velocity_units),
        fromUnitsPerSecondSquared(acceleration_units),
        fromUnitsPerSecondSquared(deceleration_units),
        1.0
    );
}

void Driver::setMotion(std::uint32_t velocity, std::uint32_t acceleration, std::uint32_t deceleration, double stage_1_2) {
    writeRegister(kRegAmax, acceleration);
    writeRegister(kRegDmax, deceleration);
    writeRegister(kRegVmax, velocity);
    writeRegister(kRegA1, static_cast<std::uint32_t>(std::llround(static_cast<double>(acceleration) * stage_1_2)));
    writeRegister(kRegD1, static_cast<std::uint32_t>(std::llround(static_cast<double>(deceleration) * stage_1_2)));
    writeRegister(kRegV1, static_cast<std::uint32_t>(std::llround(static_cast<double>(velocity) * stage_1_2)));
    writeRegister(kRegVstart, 0);
    writeRegister(kRegVstop, 10);
    writeRegister(kRegTzerowait, 0);
}

void Driver::setVelocityRpm(double velocity_rpm) {
    writeRegister(kRegVmax, fromRps(velocity_rpm / kSecondsPerMinute));
}

std::uint8_t Driver::readVersion() {
    const auto [, value] = readRegister(kRegIoin);
    return static_cast<std::uint8_t>((value >> 24) & 0xFFu);
}

IoStatus Driver::getIoStatus() {
    const auto [, value] = readRegister(kRegIoin);
    return IoStatus{
        .refl_step = (value & (1u << 0)) != 0,
        .refr_dir = (value & (1u << 1)) != 0,
        .enca_dcin_cfg5 = (value & (1u << 3)) != 0,
        .encb_dcen_cfg4 = (value & (1u << 2)) != 0,
        .drv_enn_cfg6 = (value & (1u << 4)) != 0,
        .enc_n_dco = (value & (1u << 5)) != 0,
        .sd_mode = (value & (1u << 6)) != 0,
        .swcomp_in = (value & (1u << 7)) != 0,
        .version = static_cast<std::uint8_t>((value >> 24) & 0xFFu),
    };
}

void Driver::setTargetPosition(double position_units) {
    writeRegister(kRegXtarget, static_cast<std::uint32_t>(fromUnits(position_units)));
}

void Driver::setCurrentPosition(double position_units) {
    const auto position = static_cast<std::uint32_t>(fromUnits(position_units));
    writeRegister(kRegXactual, position);
    writeRegister(kRegXtarget, position);
}

double Driver::getCurrentPosition() {
    const auto [, value] = readRegister(kRegXactual);
    return toUnits(static_cast<std::int32_t>(value));
}

void Driver::stopMotion() {
    writeRegister(kRegVstart, 0);
    writeRegister(kRegVmax, 0);
}

std::map<std::string, std::string> Driver::configSummary() const {
    return {
        {"ramp_mode", std::to_string(config_.ramp_mode)},
        {"rsense", std::to_string(config_.rsense)},
        {"microstep", std::to_string(config_.driver.microstep)},
        {"steps_per_rev", std::to_string(config_.motor.steps_per_rev)},
        {"units_per_rev", std::to_string(config_.motor.units_per_rev)},
    };
}

void Driver::writeRegister(std::uint8_t address, std::uint32_t value) {
    std::array<std::uint8_t, 5> payload{{
        static_cast<std::uint8_t>(address | kWriteMask),
        static_cast<std::uint8_t>(value >> 24),
        static_cast<std::uint8_t>(value >> 16),
        static_cast<std::uint8_t>(value >> 8),
        static_cast<std::uint8_t>(value)
    }};
    spi_.write(payload);
}

std::pair<std::uint8_t, std::uint32_t> Driver::readRegister(std::uint8_t address) {
    std::array<std::uint8_t, 5> write_data{{address, 0, 0, 0, 0}};
    std::array<std::uint8_t, 5> read_data{{0, 0, 0, 0, 0}};
    spi_.transfer(write_data, read_data);
    spi_.transfer(write_data, read_data);
    const auto value = (static_cast<std::uint32_t>(read_data[1]) << 24) |
                       (static_cast<std::uint32_t>(read_data[2]) << 16) |
                       (static_cast<std::uint32_t>(read_data[3]) << 8) |
                       static_cast<std::uint32_t>(read_data[4]);
    return {read_data[0], value};
}

std::uint32_t Driver::fromRps(double rps) const {
    return ustepT(rps * static_cast<double>(config_.driver.microstep) * static_cast<double>(config_.motor.steps_per_rev));
}

std::uint32_t Driver::fromRps2(double rps2) const {
    return ustepTa2(rps2 * static_cast<double>(config_.driver.microstep) * static_cast<double>(config_.motor.steps_per_rev));
}

std::int32_t Driver::fromUnits(double units) const {
    const auto scale = static_cast<double>(config_.motor.steps_per_rev) * static_cast<double>(config_.driver.microstep) * config_.motor.units_per_rev;
    return static_cast<std::int32_t>(std::llround(units * scale));
}

std::uint32_t Driver::fromUnitsPerSecond(double units) const {
    return ustepT(static_cast<double>(fromUnits(units)));
}

std::uint32_t Driver::fromUnitsPerSecondSquared(double units) const {
    return ustepTa2(static_cast<double>(fromUnits(units)));
}

double Driver::toUnits(std::int32_t microsteps) const {
    const auto scale = static_cast<double>(config_.motor.steps_per_rev) * static_cast<double>(config_.driver.microstep) * config_.motor.units_per_rev;
    return static_cast<double>(microsteps) / scale;
}

}  // namespace tmc5130
