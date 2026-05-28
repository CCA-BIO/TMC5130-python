use std::collections::BTreeMap;
use std::fmt;

const WRITE_MASK: u8 = 0x80;
const FCLK_HZ: f64 = 12_000_000.0;
const SECONDS_PER_MINUTE: f64 = 60.0;

const REG_GCONF: u8 = 0x00;
const REG_IOIN: u8 = 0x04;
const REG_IHOLD_IRUN: u8 = 0x10;
const REG_TPOWERDOWN: u8 = 0x11;
const REG_TPWMTHRS: u8 = 0x13;
const REG_TCOOLTHRS: u8 = 0x14;
const REG_RAMPMODE: u8 = 0x20;
const REG_XACTUAL: u8 = 0x21;
const REG_VSTART: u8 = 0x23;
const REG_A1: u8 = 0x24;
const REG_V1: u8 = 0x25;
const REG_AMAX: u8 = 0x26;
const REG_VMAX: u8 = 0x27;
const REG_DMAX: u8 = 0x28;
const REG_D1: u8 = 0x2A;
const REG_VSTOP: u8 = 0x2B;
const REG_TZEROWAIT: u8 = 0x2C;
const REG_XTARGET: u8 = 0x2D;
const REG_CHOPCONF: u8 = 0x6C;
const REG_COOLCONF: u8 = 0x6D;
const REG_PWMCONF: u8 = 0x70;

const VERSION_SHIFT: u32 = 24;
const VERSION_MASK: u32 = 0xFF;

fn mask(width: u32) -> u32 {
    if width >= 32 { u32::MAX } else { (1u32 << width) - 1 }
}

fn set_bits(register: &mut u32, shift: u32, width: u32, value: u32) {
    let field_mask = mask(width) << shift;
    *register = (*register & !field_mask) | ((value & mask(width)) << shift);
}

fn clamp_current(rsense: f64, current_ma: u16) -> u32 {
    let value = if current_ma == 0 {
        0.0
    } else {
        32.0 * 1.41421 * (current_ma as f64) / 1000.0 * (rsense + 0.02) / 0.325 - 1.0
    };
    value.clamp(0.0, 31.0) as u32
}

#[derive(Debug, Clone, PartialEq, Eq)]
pub enum Error {
    Spi(String),
    InvalidTransferLength(usize),
}

impl fmt::Display for Error {
    fn fmt(&self, f: &mut fmt::Formatter<'_>) -> fmt::Result {
        match self {
            Self::Spi(message) => write!(f, "SPI error: {message}"),
            Self::InvalidTransferLength(length) => write!(f, "invalid SPI transfer length: {length}"),
        }
    }
}

impl std::error::Error for Error {}

pub trait SpiDevice {
    type Error: fmt::Display;

    fn write(&mut self, data: &[u8]) -> Result<(), Self::Error>;
    fn transfer(&mut self, write_data: &[u8], read_data: &mut [u8]) -> Result<(), Self::Error>;
}

#[derive(Debug, Clone)]
pub struct CurrentConfig {
    pub hold_ma: u16,
    pub run_ma: u16,
}

impl Default for CurrentConfig {
    fn default() -> Self {
        Self { hold_ma: 50, run_ma: 200 }
    }
}

#[derive(Debug, Clone)]
pub struct DriverConfig {
    pub microstep: u16,
}

impl Default for DriverConfig {
    fn default() -> Self {
        Self { microstep: 256 }
    }
}

#[derive(Debug, Clone)]
pub struct MotionConfig {
    pub velocity_rpm: f64,
    pub acceleration_rpm: f64,
    pub deceleration_rpm: f64,
}

impl Default for MotionConfig {
    fn default() -> Self {
        Self { velocity_rpm: 0.0, acceleration_rpm: 60.0, deceleration_rpm: 60.0 }
    }
}

#[derive(Debug, Clone)]
pub struct MotorConfig {
    pub steps_per_rev: u16,
    pub units_per_rev: f64,
}

impl Default for MotorConfig {
    fn default() -> Self {
        Self { steps_per_rev: 200, units_per_rev: 1.0 }
    }
}

#[derive(Debug, Clone)]
pub struct Config {
    pub ramp_mode: u8,
    pub rsense: f64,
    pub current_ma: CurrentConfig,
    pub driver: DriverConfig,
    pub motion: MotionConfig,
    pub motor: MotorConfig,
}

impl Default for Config {
    fn default() -> Self {
        Self {
            ramp_mode: 1,
            rsense: 0.15,
            current_ma: CurrentConfig::default(),
            driver: DriverConfig::default(),
            motion: MotionConfig::default(),
            motor: MotorConfig::default(),
        }
    }
}

#[derive(Debug, Clone, Default)]
pub struct IoStatus {
    pub refl_step: bool,
    pub refr_dir: bool,
    pub enca_dcin_cfg5: bool,
    pub encb_dcen_cfg4: bool,
    pub drv_enn_cfg6: bool,
    pub enc_n_dco: bool,
    pub sd_mode: bool,
    pub swcomp_in: bool,
    pub version: u8,
}

pub struct Tmc5130<SPI> {
    spi: SPI,
    pub config: Config,
}

impl<SPI> Tmc5130<SPI>
where
    SPI: SpiDevice,
{
    pub fn new(spi: SPI, config: Config) -> Self {
        Self { spi, config }
    }

    pub fn into_inner(self) -> SPI {
        self.spi
    }

    pub fn spi_mut(&mut self) -> &mut SPI {
        &mut self.spi
    }

    pub fn init(&mut self) -> Result<(), Error> {
        let mut general_config = 0u32;
        set_bits(&mut general_config, 4, 1, 0);
        set_bits(&mut general_config, 7, 1, 0);
        set_bits(&mut general_config, 12, 1, 0);
        set_bits(&mut general_config, 2, 1, 1);

        let mut chop_conf = 0u32;
        set_bits(&mut chop_conf, 0, 4, 0x03);
        set_bits(&mut chop_conf, 4, 3, 0x04);
        set_bits(&mut chop_conf, 7, 4, 0x01);
        set_bits(&mut chop_conf, 15, 2, 0x02);
        set_bits(&mut chop_conf, 14, 1, 0x01);
        set_bits(&mut chop_conf, 18, 1, 0x01);
        set_bits(&mut chop_conf, 19, 1, 0x01);

        let mut cool_conf = 0u32;
        set_bits(&mut cool_conf, 16, 7, 0x00);

        let mut pwm_conf = 0u32;
        set_bits(&mut pwm_conf, 0, 8, 0xFF);
        set_bits(&mut pwm_conf, 8, 8, 0x04);
        set_bits(&mut pwm_conf, 18, 1, 1);
        set_bits(&mut pwm_conf, 20, 2, 0x01);

        self.write_register(REG_GCONF, general_config)?;
        self.write_register(REG_CHOPCONF, chop_conf)?;
        self.write_register(REG_COOLCONF, cool_conf)?;
        self.write_register(REG_PWMCONF, pwm_conf)?;
        self.write_register(REG_TPWMTHRS, 500)?;
        self.write_register(REG_TCOOLTHRS, 0)?;
        self.write_register(REG_TPOWERDOWN, 0x10)?;
        self.write_register(REG_RAMPMODE, (self.config.ramp_mode & 0x03) as u32)?;
        self.set_current(self.config.current_ma.run_ma, self.config.current_ma.hold_ma)?;
        self.set_target_position(0.0)?;
        self.set_current_position(0.0)?;
        self.set_motion_rpm(0.0, 0.0, 0.0)
    }

    pub fn set_current(&mut self, run_ma: u16, hold_ma: u16) -> Result<(), Error> {
        let mut register = 0u32;
        set_bits(&mut register, 0, 5, clamp_current(self.config.rsense, hold_ma));
        set_bits(&mut register, 8, 5, clamp_current(self.config.rsense, run_ma));
        set_bits(&mut register, 16, 4, 3);
        self.write_register(REG_IHOLD_IRUN, register)
    }

    pub fn set_motion_rpm(&mut self, velocity_rpm: f64, acceleration_rpm: f64, deceleration_rpm: f64) -> Result<(), Error> {
        let velocity = self.from_rps(velocity_rpm / SECONDS_PER_MINUTE);
        let acceleration = self.from_rps2(acceleration_rpm / SECONDS_PER_MINUTE);
        let deceleration = self.from_rps2(deceleration_rpm / SECONDS_PER_MINUTE);
        self.set_motion(velocity, acceleration, deceleration, 1.0)
    }

    pub fn set_motion_units(&mut self, velocity_units: f64, acceleration_units: f64, deceleration_units: f64) -> Result<(), Error> {
        let velocity = self.from_units_per_second(velocity_units);
        let acceleration = self.from_units_per_second_squared(acceleration_units);
        let deceleration = self.from_units_per_second_squared(deceleration_units);
        self.set_motion(velocity, acceleration, deceleration, 1.0)
    }

    pub fn set_motion(&mut self, velocity: u32, acceleration: u32, deceleration: u32, stage_1_2: f64) -> Result<(), Error> {
        self.write_register(REG_AMAX, acceleration)?;
        self.write_register(REG_DMAX, deceleration)?;
        self.write_register(REG_VMAX, velocity)?;
        self.write_register(REG_A1, ((acceleration as f64) * stage_1_2) as u32)?;
        self.write_register(REG_D1, ((deceleration as f64) * stage_1_2) as u32)?;
        self.write_register(REG_V1, ((velocity as f64) * stage_1_2) as u32)?;
        self.write_register(REG_VSTART, 0)?;
        self.write_register(REG_VSTOP, 10)?;
        self.write_register(REG_TZEROWAIT, 0)
    }

    pub fn set_velocity_rpm(&mut self, velocity_rpm: f64) -> Result<(), Error> {
        self.write_register(REG_VMAX, self.from_rps(velocity_rpm / SECONDS_PER_MINUTE))
    }

    pub fn read_version(&mut self) -> Result<u8, Error> {
        let (_, data) = self.read_register(REG_IOIN)?;
        Ok(((data >> VERSION_SHIFT) & VERSION_MASK) as u8)
    }

    pub fn get_io_status(&mut self) -> Result<IoStatus, Error> {
        let (_, data) = self.read_register(REG_IOIN)?;
        Ok(IoStatus {
            refl_step: data & (1 << 0) != 0,
            refr_dir: data & (1 << 1) != 0,
            encb_dcen_cfg4: data & (1 << 2) != 0,
            enca_dcin_cfg5: data & (1 << 3) != 0,
            drv_enn_cfg6: data & (1 << 4) != 0,
            enc_n_dco: data & (1 << 5) != 0,
            sd_mode: data & (1 << 6) != 0,
            swcomp_in: data & (1 << 7) != 0,
            version: ((data >> VERSION_SHIFT) & VERSION_MASK) as u8,
        })
    }

    pub fn set_target_position(&mut self, position_units: f64) -> Result<(), Error> {
        self.write_register(REG_XTARGET, self.from_units(position_units) as u32)
    }

    pub fn set_current_position(&mut self, position_units: f64) -> Result<(), Error> {
        let position = self.from_units(position_units) as u32;
        self.write_register(REG_XACTUAL, position)?;
        self.write_register(REG_XTARGET, position)
    }

    pub fn get_current_position(&mut self) -> Result<f64, Error> {
        let (_, data) = self.read_register(REG_XACTUAL)?;
        Ok(self.to_units(i32::from_be_bytes(data.to_be_bytes())))
    }

    pub fn stop_motion(&mut self) -> Result<(), Error> {
        self.write_register(REG_VSTART, 0)?;
        self.write_register(REG_VMAX, 0)
    }

    pub fn config_summary(&self) -> BTreeMap<&'static str, String> {
        BTreeMap::from([
            ("ramp_mode", self.config.ramp_mode.to_string()),
            ("rsense", self.config.rsense.to_string()),
            ("microstep", self.config.driver.microstep.to_string()),
            ("steps_per_rev", self.config.motor.steps_per_rev.to_string()),
            ("units_per_rev", self.config.motor.units_per_rev.to_string()),
        ])
    }

    fn write_register(&mut self, address: u8, value: u32) -> Result<(), Error> {
        let payload = [address | WRITE_MASK, (value >> 24) as u8, (value >> 16) as u8, (value >> 8) as u8, value as u8];
        self.spi.write(&payload).map_err(|error| Error::Spi(error.to_string()))
    }

    fn read_register(&mut self, address: u8) -> Result<(u8, u32), Error> {
        let write_data = [address, 0, 0, 0, 0];
        let mut read_data = [0u8; 5];
        self.spi.transfer(&write_data, &mut read_data).map_err(|error| Error::Spi(error.to_string()))?;
        self.spi.transfer(&write_data, &mut read_data).map_err(|error| Error::Spi(error.to_string()))?;
        let status = read_data[0];
        let value = u32::from_be_bytes([read_data[1], read_data[2], read_data[3], read_data[4]]);
        Ok((status, value))
    }

    fn ustep_t(&self, usteps_per_second: f64) -> u32 {
        (usteps_per_second / (FCLK_HZ / 2.0 / 2f64.powi(23))).round().max(0.0) as u32
    }

    fn ustep_ta2(&self, usteps_per_second_squared: f64) -> u32 {
        (usteps_per_second_squared * ((512.0 * 256.0) * 2f64.powi(24) / FCLK_HZ.powi(2))).round().max(0.0) as u32
    }

    fn ustep_s(&self, ustep_t: f64) -> f64 {
        ustep_t * (FCLK_HZ / 2.0 / 2f64.powi(23))
    }

    fn from_rps(&self, rps: f64) -> u32 {
        self.ustep_t(rps * self.config.driver.microstep as f64 * self.config.motor.steps_per_rev as f64)
    }

    fn from_rps2(&self, rps2: f64) -> u32 {
        self.ustep_ta2(rps2 * self.config.driver.microstep as f64 * self.config.motor.steps_per_rev as f64)
    }

    fn from_units(&self, units: f64) -> i32 {
        (units
            / self.config.motor.units_per_rev
            * self.config.motor.steps_per_rev as f64
            * self.config.driver.microstep as f64)
            .round() as i32
    }

    fn from_units_per_second(&self, units_per_second: f64) -> u32 {
        self.ustep_t(self.from_units(units_per_second) as f64)
    }

    fn from_units_per_second_squared(&self, units_per_second_squared: f64) -> u32 {
        self.ustep_ta2(self.from_units(units_per_second_squared) as f64)
    }

    fn to_units(&self, usteps: i32) -> f64 {
        usteps as f64
            / (self.config.motor.steps_per_rev as f64 * self.config.driver.microstep as f64)
            * self.config.motor.units_per_rev
    }

    #[allow(dead_code)]
    fn to_units_per_second(&self, ustep_t: u32) -> f64 {
        self.to_units(self.ustep_s(ustep_t as f64).round() as i32)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[derive(Default)]
    struct MockSpi {
        writes: Vec<Vec<u8>>,
        transfers: Vec<[u8; 5]>,
    }

    impl SpiDevice for MockSpi {
        type Error = &'static str;

        fn write(&mut self, data: &[u8]) -> Result<(), Self::Error> {
            self.writes.push(data.to_vec());
            Ok(())
        }

        fn transfer(&mut self, _write_data: &[u8], read_data: &mut [u8]) -> Result<(), Self::Error> {
            let value = self.transfers.first().copied().unwrap_or([0; 5]);
            read_data.copy_from_slice(&value);
            Ok(())
        }
    }

    #[test]
    fn packs_current_register() {
        let spi = MockSpi::default();
        let mut driver = Tmc5130::new(spi, Config::default());
        driver.set_current(200, 50).unwrap();
        assert_eq!(driver.into_inner().writes[0][0], REG_IHOLD_IRUN | WRITE_MASK);
    }

    #[test]
    fn reads_version_field() {
        let mut spi = MockSpi::default();
        spi.transfers.push([0xAA, 0x12, 0x34, 0x56, 0x78]);
        let mut driver = Tmc5130::new(spi, Config::default());
        assert_eq!(driver.read_version().unwrap(), 0x12);
    }
}
