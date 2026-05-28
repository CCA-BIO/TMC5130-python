"""
Unit tests for the TMC5130 stepper motor driver.

This test module provides regression coverage for the TMC5130 driver without requiring
physical hardware. It uses mocked dependencies (board, digitalio, SPIDevice, DotWiz) to
verify core driver functionality including:

- Register serialization/deserialization and byte ordering
- SPI read/write framing and communication protocol
- Initialization sequence and register configuration
- Current clamping and hold-delay encoding
- RPM and motion unit conversions
- Motion profile register writes with staged acceleration
- Version detection and IO status decoding
- Position helpers and stop-motion behavior

The tests helped surface and fix two logic regressions:
1. Hold delay was being written to a non-existent field alias
2. set_motion_units() was using the wrong conversion helper for velocity
"""
import importlib.util
import sys
import types
import unittest
from pathlib import Path
from unittest import mock


REPO_ROOT = Path(__file__).resolve().parents[1]
MODULE_PATH = REPO_ROOT / "python" / "tmc5130.py"


class FakeDotWiz(dict):
    def __init__(self, value=None):
        super().__init__()
        for key, item in (value or {}).items():
            self[key] = self._wrap(item)

    def __getattr__(self, name):
        try:
            return self[name]
        except KeyError as exc:
            raise AttributeError(name) from exc

    def __setattr__(self, name, value):
        self[name] = self._wrap(value)

    @classmethod
    def _wrap(cls, value):
        if isinstance(value, dict):
            return cls(value)
        return value


class FakeDigitalInOut:
    def __init__(self, pin):
        self.pin = pin
        self.direction = None
        self.value = None


class FakeSPIDevice:
    instances = []

    def __init__(self, spi, cs, **kwargs):
        self.spi = spi
        self.cs = cs
        self.kwargs = kwargs
        self.write_calls = []
        self.write_readinto_calls = []
        self.read_responses = []
        self.__class__.instances.append(self)

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        return False

    def write(self, data):
        self.write_calls.append(bytes(data))

    def write_readinto(self, write_data, read_data):
        self.write_readinto_calls.append(bytes(write_data))
        response = self.read_responses.pop(0)
        read_data[:] = response

    @classmethod
    def reset(cls):
        cls.instances.clear()


def load_module():
    board = types.ModuleType("board")
    board.D5 = "D5"
    board.SPI = lambda: "fake-spi-bus"

    digitalio = types.ModuleType("digitalio")
    digitalio.Direction = types.SimpleNamespace(OUTPUT="output")
    digitalio.DigitalInOut = FakeDigitalInOut

    adafruit_bus_device = types.ModuleType("adafruit_bus_device")
    spi_device_module = types.ModuleType("adafruit_bus_device.spi_device")
    spi_device_module.SPIDevice = FakeSPIDevice

    pyjson5 = types.ModuleType("pyjson5")
    pyjson5.load = lambda stream: {}

    dotwiz = types.ModuleType("dotwiz")
    dotwiz.DotWiz = FakeDotWiz

    modules = {
        "board": board,
        "digitalio": digitalio,
        "adafruit_bus_device": adafruit_bus_device,
        "adafruit_bus_device.spi_device": spi_device_module,
        "pyjson5": pyjson5,
        "dotwiz": dotwiz,
    }

    FakeSPIDevice.reset()
    with mock.patch.dict(sys.modules, modules, clear=False):
        spec = importlib.util.spec_from_file_location("tmc5130_under_test", MODULE_PATH)
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        return module


TMC5130_MODULE = load_module()


class TMC5130Tests(unittest.TestCase):
    def setUp(self):
        FakeSPIDevice.reset()
        self.module = TMC5130_MODULE

    def make_driver(self, config=None):
        with mock.patch("builtins.print"):
            return self.module.TMC5130(motor_config=config)

    def decode_register(self, register_cls, msb_bytes):
        register = register_cls()
        register.set_register(bytearray(msb_bytes)[::-1])
        return register

    def test_register_serialization_round_trip_and_byte_order(self):
        register = self.module.XCompare()
        register.xcompare = 0x12345678

        self.assertEqual(register.get_register(), bytearray([0x12, 0x34, 0x56, 0x78]))
        self.assertEqual(register.get_register(msb=False), bytearray([0x78, 0x56, 0x34, 0x12]))

        loaded = self.module.XCompare()
        loaded.set_register(bytearray([0x78, 0x56, 0x34, 0x12]))
        self.assertEqual(loaded.xcompare, 0x12345678)

    def test_set_register_rejects_invalid_byte_count(self):
        with self.assertRaises(ValueError):
            self.module.XCompare().set_register(bytearray([0x01, 0x02, 0x03]))

    def test_write_applies_write_mask_and_sends_full_payload(self):
        driver = self.make_driver()

        driver._write(0x10, bytearray([0x01, 0x02, 0x03, 0x04]))

        self.assertEqual(driver.spi.write_calls, [b"\x90\x01\x02\x03\x04"])

    def test_read_uses_two_spi_transfers_and_reverses_data_bytes(self):
        driver = self.make_driver()
        driver.spi.read_responses = [
            bytes([0x00, 0x00, 0x00, 0x00, 0x00]),
            bytes([0xA5, 0x12, 0x34, 0x56, 0x78]),
        ]

        status, data = driver._read(self.module.IOStatus.address)

        expected_request = bytes([self.module.IOStatus.address, 0x00, 0x00, 0x00, 0x00])
        self.assertEqual(status, 0xA5)
        self.assertEqual(data, bytearray([0x78, 0x56, 0x34, 0x12]))
        self.assertEqual(driver.spi.write_readinto_calls, [expected_request, expected_request])

    def test_init_writes_expected_registers_and_delegates_follow_up_configuration(self):
        driver = self.make_driver(
            {
                "ramp_mode": 2,
                "current_mA": {"run_mA": 500, "hold_mA": 250},
                "driver": {"microstep": 256},
                "motor": {"steps_per_rev": 200, "units_per_rev": 1.0},
            }
        )

        with (
            mock.patch.object(driver, "_write") as write_mock,
            mock.patch.object(driver, "set_current") as set_current_mock,
            mock.patch.object(driver, "set_target_position") as set_target_position_mock,
            mock.patch.object(driver, "set_current_position") as set_current_position_mock,
            mock.patch.object(driver, "set_motion_rpm") as set_motion_rpm_mock,
        ):
            driver.init()

        self.assertEqual(
            [call.args[0] for call in write_mock.call_args_list],
            [
                self.module.GeneralConfig.address,
                self.module.ChopConf.address,
                self.module.CoolConf.address,
                self.module.PWMConf.address,
                self.module.TPWMThreshold.address,
                self.module.TCoolThreshold.address,
                self.module.TPowerDown.address,
                self.module.RampMode.address,
            ],
        )
        ramp_mode = self.decode_register(self.module.RampMode, write_mock.call_args_list[-1].args[1])
        self.assertEqual(ramp_mode.ramp_mode, 2)
        set_current_mock.assert_called_once_with(run_mA=500, hold_mA=250)
        set_target_position_mock.assert_called_once_with(0)
        set_current_position_mock.assert_called_once_with(0)
        set_motion_rpm_mock.assert_called_once_with(velocity_rpm=0, acceleration_rpm=0, deceleration_rpm=0)

    def test_set_current_clamps_values_and_sets_hold_delay_field(self):
        driver = self.make_driver()

        with mock.patch.object(driver, "_write") as write_mock:
            driver.set_current(run_mA=10_000, hold_mA=0)

        self.assertEqual(write_mock.call_count, 1)
        self.assertEqual(write_mock.call_args.args[0], self.module.IHoldIRun.address)
        register = self.decode_register(self.module.IHoldIRun, write_mock.call_args.args[1])
        self.assertEqual(register.irun, 31)
        self.assertEqual(register.ihold, 0)
        self.assertEqual(register.iholddelay, 3)

    def test_set_motion_rpm_converts_inputs_before_delegating(self):
        driver = self.make_driver()

        with mock.patch.object(driver, "set_motion") as set_motion_mock, mock.patch("builtins.print"):
            driver.set_motion_rpm(velocity_rpm=60, acceleration_rpm=120, deceleration_rpm=30)

        velocity, acceleration, deceleration = set_motion_mock.call_args.args
        self.assertAlmostEqual(velocity, driver._from_rps(1.0))
        self.assertAlmostEqual(acceleration, driver._from_rps2(2.0))
        self.assertAlmostEqual(deceleration, driver._from_rps2(0.5))

    def test_set_motion_units_uses_velocity_and_acceleration_unit_conversions(self):
        driver = self.make_driver()

        with mock.patch.object(driver, "set_motion") as set_motion_mock, mock.patch("builtins.print"):
            driver.set_motion_units(velocity_mms_degs=12.5, acceleration_mms_degs=3.5, deceleration_mms_degs=1.5)

        velocity, acceleration, deceleration = set_motion_mock.call_args.args
        self.assertAlmostEqual(velocity, driver._from_mms_degs(12.5))
        self.assertAlmostEqual(acceleration, driver._from_mms_degs(3.5))
        self.assertAlmostEqual(deceleration, driver._from_mms2_degs2(1.5))

    def test_set_motion_writes_all_motion_registers_with_stage_split(self):
        driver = self.make_driver()

        with mock.patch.object(driver, "_write") as write_mock:
            driver.set_motion(velocity=120, acceleration=80, deceleration=40, stage_1_2=0.25)

        self.assertEqual(
            [call.args[0] for call in write_mock.call_args_list],
            [
                self.module.AMax.address,
                self.module.DMax.address,
                self.module.VMax.address,
                self.module.A1.address,
                self.module.D1.address,
                self.module.V1.address,
                self.module.VStart.address,
                self.module.VStop.address,
                self.module.TZeroWait.address,
            ],
        )

        expectations = {
            self.module.AMax.address: (self.module.AMax, "amax", 80),
            self.module.DMax.address: (self.module.DMax, "dmax", 40),
            self.module.VMax.address: (self.module.VMax, "vmax", 120),
            self.module.A1.address: (self.module.A1, "a1", 20),
            self.module.D1.address: (self.module.D1, "d1", 10),
            self.module.V1.address: (self.module.V1, "v1", 30),
            self.module.VStart.address: (self.module.VStart, "vstart", 0),
            self.module.VStop.address: (self.module.VStop, "vstop", 10),
            self.module.TZeroWait.address: (self.module.TZeroWait, "tzerowait", 0),
        }

        for call in write_mock.call_args_list:
            register_cls, field_name, expected_value = expectations[call.args[0]]
            register = self.decode_register(register_cls, call.args[1])
            self.assertEqual(getattr(register, field_name), expected_value)

    def test_read_version_and_get_io_status_parse_register_fields(self):
        driver = self.make_driver()
        io_status = self.module.IOStatus()
        io_status.refl_step = 1
        io_status.refr_dir = 0
        io_status.enca_dcin_cfg5 = 1
        io_status.encb_dcen_cfg4 = 0
        io_status.drv_enn_cfg6 = 1
        io_status.enc_n_dco = 0
        io_status.sd_mode = 1
        io_status.swcomp_in = 1
        io_status.version = 0x42

        with mock.patch.object(driver, "_read", return_value=(0, io_status.get_register(msb=False))):
            self.assertEqual(driver.read_version(), 0x42)
            self.assertEqual(
                driver.get_io_status(),
                {
                    "refl_step": 1,
                    "refr_dir": 0,
                    "enca_dcin_cfg5": 1,
                    "encb_dcen_cfg4": 0,
                    "drv_enn_cfg6": 1,
                    "enc_n_dco": 0,
                    "sd_mode": 1,
                    "swcomp_in": 1,
                    "version": 0x42,
                },
            )

    def test_position_helpers_round_trip_and_keep_target_in_sync(self):
        driver = self.make_driver()

        with mock.patch.object(driver, "_write") as write_mock:
            driver.set_current_position(2.5)

        self.assertEqual(
            [call.args[0] for call in write_mock.call_args_list],
            [self.module.XActual.address, self.module.XTarget.address],
        )
        xactual = self.decode_register(self.module.XActual, write_mock.call_args_list[0].args[1])
        xtarget = self.decode_register(self.module.XTarget, write_mock.call_args_list[1].args[1])
        self.assertEqual(xactual.xactual, xtarget.xtarget)
        self.assertEqual(xactual.xactual, int(driver._from_mm_deg(2.5)))

        current_position = self.module.XActual()
        current_position.xactual = int(driver._from_mm_deg(2.5))
        with mock.patch.object(driver, "_read", return_value=(0, current_position.get_register(msb=False))):
            self.assertAlmostEqual(driver.get_current_position(), 2.5)

    def test_stop_motion_zeroes_vstart_and_vmax(self):
        driver = self.make_driver()

        with mock.patch.object(driver, "_write") as write_mock:
            driver.stop_motion()

        self.assertEqual(
            [call.args[0] for call in write_mock.call_args_list],
            [self.module.VStart.address, self.module.VMax.address],
        )
        vstart = self.decode_register(self.module.VStart, write_mock.call_args_list[0].args[1])
        vmax = self.decode_register(self.module.VMax, write_mock.call_args_list[1].args[1])
        self.assertEqual(vstart.vstart, 0)
        self.assertEqual(vmax.vmax, 0)


if __name__ == "__main__":
    unittest.main()
