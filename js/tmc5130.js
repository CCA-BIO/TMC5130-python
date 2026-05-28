const WRITE_MASK = 0x80;
const SECONDS_PER_MINUTE = 60;
const FCLK_HZ = 12_000_000;

const REG_GCONF = 0x00;
const REG_IOIN = 0x04;
const REG_IHOLD_IRUN = 0x10;
const REG_TPOWERDOWN = 0x11;
const REG_TPWMTHRS = 0x13;
const REG_TCOOLTHRS = 0x14;
const REG_RAMPMODE = 0x20;
const REG_XACTUAL = 0x21;
const REG_VSTART = 0x23;
const REG_A1 = 0x24;
const REG_V1 = 0x25;
const REG_AMAX = 0x26;
const REG_VMAX = 0x27;
const REG_DMAX = 0x28;
const REG_D1 = 0x2A;
const REG_VSTOP = 0x2B;
const REG_TZEROWAIT = 0x2C;
const REG_XTARGET = 0x2D;
const REG_CHOPCONF = 0x6C;
const REG_COOLCONF = 0x6D;
const REG_PWMCONF = 0x70;

function mask(width) {
  return width >= 32 ? 0xffffffff : (1 << width) - 1;
}

function setBits(register, shift, width, value) {
  const fieldMask = (mask(width) << shift) >>> 0;
  return ((register & ~fieldMask) | (((value & mask(width)) << shift) >>> 0)) >>> 0;
}

function clampCurrent(rsense, currentMa) {
  const raw = currentMa === 0 ? 0 : 32.0 * 1.41421 * (currentMa / 1000.0) * (rsense + 0.02) / 0.325 - 1;
  return Math.max(0, Math.min(31, Math.trunc(raw)));
}

class TMC5130 {
  constructor(spi, config = {}) {
    this.spi = spi;
    this.config = {
      ramp_mode: 1,
      rsense: 0.15,
      current_mA: { hold_mA: 50, run_mA: 200 },
      driver: { microstep: 256 },
      motion: { velocity_rpm: 0, acceleration_rpm: 60, deceleration_rpm: 60 },
      motor: { steps_per_rev: 200, units_per_rev: 1.0 },
      ...config,
      current_mA: { hold_mA: 50, run_mA: 200, ...(config.current_mA || {}) },
      driver: { microstep: 256, ...(config.driver || {}) },
      motion: { velocity_rpm: 0, acceleration_rpm: 60, deceleration_rpm: 60, ...(config.motion || {}) },
      motor: { steps_per_rev: 200, units_per_rev: 1.0, ...(config.motor || {}) },
    };
  }

  async init() {
    let generalConfig = 0;
    let chopConf = 0;
    let coolConf = 0;
    let pwmConf = 0;

    generalConfig = setBits(generalConfig, 4, 1, 0);
    generalConfig = setBits(generalConfig, 7, 1, 0);
    generalConfig = setBits(generalConfig, 12, 1, 0);
    generalConfig = setBits(generalConfig, 2, 1, 1);

    chopConf = setBits(chopConf, 0, 4, 0x03);
    chopConf = setBits(chopConf, 4, 3, 0x04);
    chopConf = setBits(chopConf, 7, 4, 0x01);
    chopConf = setBits(chopConf, 14, 1, 0x01);
    chopConf = setBits(chopConf, 15, 2, 0x02);
    chopConf = setBits(chopConf, 18, 1, 0x01);
    chopConf = setBits(chopConf, 19, 1, 0x01);

    coolConf = setBits(coolConf, 16, 7, 0x00);

    pwmConf = setBits(pwmConf, 0, 8, 0xff);
    pwmConf = setBits(pwmConf, 8, 8, 0x04);
    pwmConf = setBits(pwmConf, 18, 1, 1);
    pwmConf = setBits(pwmConf, 20, 2, 0x01);

    await this.writeRegister(REG_GCONF, generalConfig);
    await this.writeRegister(REG_CHOPCONF, chopConf);
    await this.writeRegister(REG_COOLCONF, coolConf);
    await this.writeRegister(REG_PWMCONF, pwmConf);
    await this.writeRegister(REG_TPWMTHRS, 500);
    await this.writeRegister(REG_TCOOLTHRS, 0);
    await this.writeRegister(REG_TPOWERDOWN, 0x10);
    await this.writeRegister(REG_RAMPMODE, this.config.ramp_mode & 0x03);
    await this.setCurrent(this.config.current_mA.run_mA, this.config.current_mA.hold_mA);
    await this.setTargetPosition(0);
    await this.setCurrentPosition(0);
    await this.setMotionRpm(0, 0, 0);
  }

  async setCurrent(runMa, holdMa) {
    let register = 0;
    register = setBits(register, 0, 5, clampCurrent(this.config.rsense, holdMa));
    register = setBits(register, 8, 5, clampCurrent(this.config.rsense, runMa));
    register = setBits(register, 16, 4, 3);
    await this.writeRegister(REG_IHOLD_IRUN, register);
  }

  async setMotionRpm(velocityRpm, accelerationRpm, decelerationRpm) {
    await this.setMotion(
      this.fromRps(velocityRpm / SECONDS_PER_MINUTE),
      this.fromRps2(accelerationRpm / SECONDS_PER_MINUTE),
      this.fromRps2(decelerationRpm / SECONDS_PER_MINUTE),
      1.0,
    );
  }

  async setMotionUnits(velocityUnits, accelerationUnits, decelerationUnits) {
    await this.setMotion(
      this.fromUnitsPerSecond(velocityUnits),
      this.fromUnitsPerSecondSquared(accelerationUnits),
      this.fromUnitsPerSecondSquared(decelerationUnits),
      1.0,
    );
  }

  async setMotion(velocity, acceleration, deceleration, stage12 = 1.0) {
    await this.writeRegister(REG_AMAX, acceleration);
    await this.writeRegister(REG_DMAX, deceleration);
    await this.writeRegister(REG_VMAX, velocity);
    await this.writeRegister(REG_A1, Math.round(acceleration * stage12));
    await this.writeRegister(REG_D1, Math.round(deceleration * stage12));
    await this.writeRegister(REG_V1, Math.round(velocity * stage12));
    await this.writeRegister(REG_VSTART, 0);
    await this.writeRegister(REG_VSTOP, 10);
    await this.writeRegister(REG_TZEROWAIT, 0);
  }

  async setVelocityRpm(velocityRpm) {
    await this.writeRegister(REG_VMAX, this.fromRps(velocityRpm / SECONDS_PER_MINUTE));
  }

  async readVersion() {
    const [, value] = await this.readRegister(REG_IOIN);
    return (value >>> 24) & 0xff;
  }

  async getIoStatus() {
    const [, value] = await this.readRegister(REG_IOIN);
    return {
      refl_step: Boolean(value & (1 << 0)),
      refr_dir: Boolean(value & (1 << 1)),
      encb_dcen_cfg4: Boolean(value & (1 << 2)),
      enca_dcin_cfg5: Boolean(value & (1 << 3)),
      drv_enn_cfg6: Boolean(value & (1 << 4)),
      enc_n_dco: Boolean(value & (1 << 5)),
      sd_mode: Boolean(value & (1 << 6)),
      swcomp_in: Boolean(value & (1 << 7)),
      version: (value >>> 24) & 0xff,
    };
  }

  async setTargetPosition(positionUnits) {
    await this.writeRegister(REG_XTARGET, this.fromUnits(positionUnits) >>> 0);
  }

  async setCurrentPosition(positionUnits) {
    const position = this.fromUnits(positionUnits) >>> 0;
    await this.writeRegister(REG_XACTUAL, position);
    await this.writeRegister(REG_XTARGET, position);
  }

  async getCurrentPosition() {
    const [, value] = await this.readRegister(REG_XACTUAL);
    return this.toUnits(value | 0);
  }

  async stopMotion() {
    await this.writeRegister(REG_VSTART, 0);
    await this.writeRegister(REG_VMAX, 0);
  }

  async writeRegister(address, value) {
    const data = Buffer.from([
      address | WRITE_MASK,
      (value >>> 24) & 0xff,
      (value >>> 16) & 0xff,
      (value >>> 8) & 0xff,
      value & 0xff,
    ]);
    await this.spi.write(data);
  }

  async readRegister(address) {
    const request = Buffer.from([address, 0, 0, 0, 0]);
    await this.spi.transfer(request);
    const response = await this.spi.transfer(request);
    const value = (((response[1] << 24) >>> 0) | (response[2] << 16) | (response[3] << 8) | response[4]) >>> 0;
    return [response[0], value];
  }

  fromRps(rps) {
    return this.ustepT(rps * this.config.driver.microstep * this.config.motor.steps_per_rev);
  }

  fromRps2(rps2) {
    return this.ustepTa2(rps2 * this.config.driver.microstep * this.config.motor.steps_per_rev);
  }

  fromUnits(units) {
    return Math.round((units / this.config.motor.units_per_rev) * this.config.motor.steps_per_rev * this.config.driver.microstep);
  }

  fromUnitsPerSecond(units) {
    return this.ustepT(this.fromUnits(units));
  }

  fromUnitsPerSecondSquared(units) {
    return this.ustepTa2(this.fromUnits(units));
  }

  toUnits(microsteps) {
    return (microsteps / (this.config.motor.steps_per_rev * this.config.driver.microstep)) * this.config.motor.units_per_rev;
  }

  ustepT(ustepsPerSecond) {
    return Math.max(0, Math.round(ustepsPerSecond / (FCLK_HZ / 2 / 2 ** 23)));
  }

  ustepTa2(ustepsPerSecondSquared) {
    return Math.max(0, Math.round(ustepsPerSecondSquared * ((512 * 256) * 2 ** 24 / FCLK_HZ ** 2)));
  }
}

module.exports = { TMC5130 };
