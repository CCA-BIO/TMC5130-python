#ifndef TMC5130_HPP
#define TMC5130_HPP

#include <array>
#include <cstdint>
#include <map>
#include <stdexcept>
#include <string>

namespace tmc5130 {

struct CurrentConfig {
    std::uint16_t hold_mA{50};
    std::uint16_t run_mA{200};
};

struct DriverConfig {
    std::uint16_t microstep{256};
};

struct MotionConfig {
    double velocity_rpm{0.0};
    double acceleration_rpm{60.0};
    double deceleration_rpm{60.0};
};

struct MotorConfig {
    std::uint16_t steps_per_rev{200};
    double units_per_rev{1.0};
};

struct Config {
    std::uint8_t ramp_mode{1};
    double rsense{0.15};
    CurrentConfig current_mA{};
    DriverConfig driver{};
    MotionConfig motion{};
    MotorConfig motor{};
};

struct IoStatus {
    bool refl_step{};
    bool refr_dir{};
    bool enca_dcin_cfg5{};
    bool encb_dcen_cfg4{};
    bool drv_enn_cfg6{};
    bool enc_n_dco{};
    bool sd_mode{};
    bool swcomp_in{};
    std::uint8_t version{};
};

class SpiDevice {
public:
    virtual ~SpiDevice() = default;
    virtual void write(const std::array<std::uint8_t, 5>& data) = 0;
    virtual void transfer(const std::array<std::uint8_t, 5>& write_data, std::array<std::uint8_t, 5>& read_data) = 0;
};

class Driver {
public:
    explicit Driver(SpiDevice& spi, Config config = {});

    void init();
    void setCurrent(std::uint16_t run_mA, std::uint16_t hold_mA);
    void setMotionRpm(double velocity_rpm, double acceleration_rpm, double deceleration_rpm);
    void setMotionUnits(double velocity_units, double acceleration_units, double deceleration_units);
    void setMotion(std::uint32_t velocity, std::uint32_t acceleration, std::uint32_t deceleration, double stage_1_2 = 1.0);
    void setVelocityRpm(double velocity_rpm);
    std::uint8_t readVersion();
    IoStatus getIoStatus();
    void setTargetPosition(double position_units);
    void setCurrentPosition(double position_units);
    double getCurrentPosition();
    void stopMotion();
    std::map<std::string, std::string> configSummary() const;

private:
    SpiDevice& spi_;
    Config config_;

    void writeRegister(std::uint8_t address, std::uint32_t value);
    std::pair<std::uint8_t, std::uint32_t> readRegister(std::uint8_t address);
    std::uint32_t fromRps(double rps) const;
    std::uint32_t fromRps2(double rps2) const;
    std::int32_t fromUnits(double units) const;
    std::uint32_t fromUnitsPerSecond(double units) const;
    std::uint32_t fromUnitsPerSecondSquared(double units) const;
    double toUnits(std::int32_t microsteps) const;
};

}  // namespace tmc5130

#endif
