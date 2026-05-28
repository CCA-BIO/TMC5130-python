import ctypes
import time
import board
import digitalio
from adafruit_bus_device.spi_device import SPIDevice
import pyjson5 as json5
from dotwiz import DotWiz
from copy import deepcopy


class HelperBase:
    _pack_ = 1

    def set_register(self, value: bytearray) -> None:
        """Set the register value from a 4-byte array (32 bits)"""
        # Check the bytearray is the correct size
        if len(value) != ctypes.sizeof(self):
            raise ValueError(f"Invalid bytearray size. Must be {ctypes.sizeof(self)} bytes.")

        # Copy the bytearray to the structure
        ctypes.memmove(ctypes.addressof(self), bytes(value), ctypes.sizeof(self))

    def get_register(self, msb: bool = True) -> bytearray:
        """Get the register value as a bytearray - 4 byte array : 32bits"""
        if msb:
            # Reverse the byte order
            return bytearray(ctypes.string_at(ctypes.addressof(self), ctypes.sizeof(self)))[::-1]
        else:
            return bytearray(ctypes.string_at(ctypes.addressof(self), ctypes.sizeof(self)))


class GeneralConfig(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x00
    _pack_ = 1
    _fields_ = [
        # Name, Type, Bit Count
        ("i_scale_analog", ctypes.c_uint32, 1),  # LSB - bit 0
        ("internal_rsense", ctypes.c_uint32, 1),
        ("en_pwm_mode", ctypes.c_uint32, 1),
        ("enc_commutation", ctypes.c_uint32, 1),
        ("shaft", ctypes.c_uint32, 1),
        ("diag0_error", ctypes.c_uint32, 1),
        ("diag0_otpw", ctypes.c_uint32, 1),
        ("diag0_stall", ctypes.c_uint32, 1),
        ("diag1_stall", ctypes.c_uint32, 1),
        ("diag1_index", ctypes.c_uint32, 1),
        ("diag1_onstate", ctypes.c_uint32, 1),
        ("diag1_steps_skipped", ctypes.c_uint32, 1),
        ("diag0_int_pushpull", ctypes.c_uint32, 1),
        ("diag1_poscomp_pushpull", ctypes.c_uint32, 1),
        ("small_hysteresis", ctypes.c_uint32, 1),
        ("stop_enable", ctypes.c_uint32, 1),
        ("direct_mode", ctypes.c_uint32, 1),
        ("test_mode", ctypes.c_uint32, 1),
        ("_padding", ctypes.c_uint32, 14),  # MSB -17th bit
    ]


class GeneralStatus(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x01
    _pack_ = 1
    _fields_ = [
        ("reset", ctypes.c_uint32, 1),
        ("drv_err", ctypes.c_uint32, 1),
        ("uv_cp", ctypes.c_uint32, 1),
        ("_padding", ctypes.c_uint32, 29),
    ]


class IFCounter(HelperBase, ctypes.LittleEndianStructure):
    """
    Interface transmission counter. This register becomes
    incremented with each successful UART interface write access.

    Disabled in SPI operation. The counter wraps around from 255 to 0.
    """

    address: int = 0x02
    _pack_ = 1
    _fields_ = [
        ("ifcnt", ctypes.c_uint32, 8),
        ("_padding", ctypes.c_uint32, 24),
    ]


class SlaveConf(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x03
    _pack_ = 1
    _fields_ = [
        ("slaveaddr", ctypes.c_uint32, 8),
        ("senddelay", ctypes.c_uint32, 4),
        ("_padding", ctypes.c_uint32, 20),
    ]


class IOStatus(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x04
    _pack_ = 1  # Pack the structure to 1 byte boundaries
    _fields_ = [
        # field_name, field_type, bit_count
        ("refl_step", ctypes.c_uint32, 1),
        ("refr_dir", ctypes.c_uint32, 1),
        ("encb_dcen_cfg4", ctypes.c_uint32, 1),
        ("enca_dcin_cfg5", ctypes.c_uint32, 1),
        ("drv_enn_cfg6", ctypes.c_uint32, 1),
        ("enc_n_dco", ctypes.c_uint32, 1),
        ("sd_mode", ctypes.c_uint32, 1),
        ("swcomp_in", ctypes.c_uint32, 1),
        ("_padding", ctypes.c_uint32, 16),
        ("version", ctypes.c_uint32, 8),
    ]


class XCompare(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x05
    _pack_ = 1
    _fields_ = [
        ("xcompare", ctypes.c_uint32, 32),
    ]


class IHoldIRun(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x10
    _pack_ = 1
    _fields_ = [
        ("ihold", ctypes.c_uint32, 5),  # 0-4 bits
        ("_padding_1", ctypes.c_uint32, 3),  # 5-7 bits
        ("irun", ctypes.c_uint32, 5),  # 8-12 bits
        ("_padding_2", ctypes.c_uint32, 3),  # 13-15 bits
        ("iholddelay", ctypes.c_uint32, 4),  # 16-19 bits
        ("_padding_3", ctypes.c_uint32, 12),  # 20-31 bits
    ]


class TPowerDown(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x11
    _pack_ = 1
    _fields_ = [
        ("tpowerdown", ctypes.c_uint32, 8),
        ("_padding", ctypes.c_uint32, 24),
    ]


class TStep(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x12
    _pack_ = 1
    _fields_ = [
        ("tstep", ctypes.c_uint32, 20),
        ("_padding", ctypes.c_uint32, 12),
    ]


class TPWMThreshold(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x13
    _pack_ = 1
    _fields_ = [
        ("tpwmthrs", ctypes.c_uint32, 20),
        ("_padding", ctypes.c_uint32, 12),
    ]


class TCoolThreshold(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x14
    _pack_ = 1
    _fields_ = [
        ("tcoolthrs", ctypes.c_uint32, 20),
        ("_padding", ctypes.c_uint32, 12),
    ]


class THigh(HelperBase, ctypes.LittleEndianStructure):
    address: int = 0x15
    _pack_ = 1
    _fields_ = [
        ("thigh", ctypes.c_uint32, 20),
        ("_padding", ctypes.c_uint32, 12),
    ]


class RampMode(HelperBase, ctypes.LittleEndianStructure):
    """
    RAMPMODE:
    0: Positioning mode (using all A, D and V
    parameters)
    1: Velocity mode to positive VMAX (using
    AMAX acceleration)
    2: Velocity mode to negative VMAX (using
    AMAX acceleration)
    3: Hold mode (velocity remains unchanged,
    unless stop event occurs)
    """

    address: int = 0x20
    _pack_ = 1
    _fields_ = [
        ("ramp_mode", ctypes.c_uint32, 2),
        ("_padding", ctypes.c_uint32, 30),
    ]


class XActual(HelperBase, ctypes.LittleEndianStructure):
    """
    Actual motor position (signed)
    Hint: This value normally should only be
    modified, when homing the drive. In
    positioning mode, modifying the register
    content will start a motion
    """

    address: int = 0x21
    _pack_ = 1
    _fields_ = [
        ("xactual", ctypes.c_uint32, 32),
    ]


class VActual(HelperBase, ctypes.LittleEndianStructure):
    """
    Actual motor velocity from ramp generator (signed)
    The sign matches the motion direction. A
    negative sign means motion to lower
    XACTUAL.
    """

    address: int = 0x22
    _pack_ = 1
    _fields_ = [
        ("vactual", ctypes.c_uint32, 24),
        ("_padding", ctypes.c_uint32, 8),
    ]


class VStart(HelperBase, ctypes.LittleEndianStructure):
    """
    Motor start velocity (unsigned)
    For universal use, set VSTOP ≥ VSTART. This is
    not required if the motion distance is sufficient
    to decelerate from VSTART to VSTOP.
    """

    address: int = 0x23
    _pack_ = 1
    _fields_ = [
        ("vstart", ctypes.c_uint32, 18),
        ("_padding", ctypes.c_uint32, 14),
    ]


class A1(HelperBase, ctypes.LittleEndianStructure):
    """
    First acceleration between VSTART and V1(unsigned)
    """

    address: int = 0x24
    _pack_ = 1
    _fields_ = [
        ("a1", ctypes.c_uint32, 16),
        ("_padding", ctypes.c_uint32, 16),
    ]


class V1(HelperBase, ctypes.LittleEndianStructure):
    """
    First acceleration / deceleration phase
    threshold velocity (unsigned)
    0: Disables A1 and D1 phase, use AMAX, DMAX
    only
    """

    address: int = 0x25
    _pack_ = 1
    _fields_ = [
        ("v1", ctypes.c_uint32, 20),
        ("_padding", ctypes.c_uint32, 12),
    ]


class AMax(HelperBase, ctypes.LittleEndianStructure):
    """
    Second acceleration between V1 and VMAX (unsigned)
    This is the acceleration and deceleration value
    for velocity mode.
    """

    address: int = 0x26
    _pack_ = 1
    _fields_ = [
        ("amax", ctypes.c_uint32, 16),
        ("_padding", ctypes.c_uint32, 16),
    ]


class VMax(HelperBase, ctypes.LittleEndianStructure):
    """
    Motion ramp target velocity (for positioning
    ensure VMAX ≥ VSTART) (unsigned)
    This is the target velocity in velocity mode. It
    can be changed any time during a motion.
    """

    address: int = 0x27
    _pack_ = 1
    _fields_ = [
        ("vmax", ctypes.c_uint32, 23),
        ("_padding", ctypes.c_uint32, 9),
    ]


class DMax(HelperBase, ctypes.LittleEndianStructure):
    """
    Deceleration between VMAX and V1 (unsigned)
    """

    address: int = 0x28
    _pack_ = 1
    _fields_ = [
        ("dmax", ctypes.c_uint32, 16),
        ("_padding", ctypes.c_uint32, 16),
    ]


class D1(HelperBase, ctypes.LittleEndianStructure):
    """
    Deceleration between V1 and VSTOP (unsigned)
    """

    address: int = 0x2A
    _pack_ = 1
    _fields_ = [
        ("d1", ctypes.c_uint32, 16),
        ("_padding", ctypes.c_uint32, 16),
    ]


class VStop(HelperBase, ctypes.LittleEndianStructure):
    """
    Motor stop velocity (unsigned)
    Hint: Set VSTOP ≥ VSTART to allow positioning for short distances
    Attention: Do not set 0 in positioning mode, minimum 10 recommend!
    """

    address: int = 0x2B
    _pack_ = 1
    _fields_ = [
        ("vstop", ctypes.c_uint32, 18),
        ("_padding", ctypes.c_uint32, 14),
    ]


class TZeroWait(HelperBase, ctypes.LittleEndianStructure):
    """
    Defines the waiting time after ramping down
    to zero velocity before next movement or
    direction inversion can start. Time range is
    about 0 to 2 seconds.
    This setting avoids excess acceleration e.g.
    from VSTOP to -VSTART.
    """

    address: int = 0x2C
    _pack_ = 1
    _fields_ = [
        ("tzerowait", ctypes.c_uint32, 20),
        ("_padding", ctypes.c_uint32, 12),
    ]


class XTarget(HelperBase, ctypes.LittleEndianStructure):
    """
    Target position for ramp mode (signed). Write
    a new target position to this register to
    activate the ramp generator positioning in
    RAMPMODE=0. Initialize all velocity,
    acceleration, and deceleration parameters before.
    Hint: The position is allowed to wrap around,
    thus, XTARGET value optionally can be treated as an unsigned number.
    Hint: The maximum possible displacement is +/-((2^31)-1).
    Hint: When increasing V1, D1 or DMAX during
    a motion, rewrite XTARGET afterwards to
    trigger a second acceleration phase, if desired.
    """

    address: int = 0x2D
    _pack_ = 1
    _fields_ = [
        ("xtarget", ctypes.c_uint32, 32),
    ]


class VDCMin(HelperBase, ctypes.LittleEndianStructure):
    """
    Automatic commutation DcStep becomes enabled above
    velocity VDCMIN (unsigned) (only when using internal ramp
    generator, not for STEP/DIR interface – in STEP/DIR mode,
    DcStep becomes enabled by the external signal DCEN)
    In this mode, the actual position is determined by the sensorless
    motor commutation and becomes fed back to XACTUAL.
    In case the motor becomes heavily loaded, VDCMIN also is used
    as the minimum step velocity. Activate stop on stall (sg_stop)
    to detect step loss.
    0: Disable, DcStep off
    |VACT| ≥ VDCMIN ≥ 256:
    - Triggers the same actions as exceeding THIGH setting.
    - Switches on automatic commutation DcStep
    Hint: Also set DCCTRL parameters to operate DcStep.
    (Only bits 22… 8 are used for value and for comparison)
    """

    address: int = 0x33
    _pack_ = 1
    _fields_ = [
        ("vdcmin", ctypes.c_uint32, 23),
        ("_padding", ctypes.c_uint32, 9),
    ]


class SWMode(HelperBase, ctypes.LittleEndianStructure):
    """
    Switch mode configuration
    """

    address: int = 0x34
    _pack_ = 1
    _fields_ = [
        # 1: Enables automatic motor stop during active left reference switch input
        # Hint: The motor restarts in case the stop switch becomes released.
        ("stop_l_enable", ctypes.c_uint32, 1),
        # 1: Enables automatic motor stop during active right reference switch input
        # Hint: The motor restarts in case the stop switch becomes released.
        ("stop_r_enable", ctypes.c_uint32, 1),
        # Sets the active polarity of the left reference switch input
        # 0=non-inverted, high active: a high level on REFL stops the motor
        # 1=inverted, low active: a low level on REFL stops the motor
        ("pol_stop_l", ctypes.c_uint32, 1),
        # Sets the active polarity of the right reference switch input
        # 0=non-inverted, high active: a high level on REFR stops the motor
        # 1=inverted, low active: a low level on REFR stops the motor
        ("pol_stop_r", ctypes.c_uint32, 1),
        # 1: Swap the left and the right reference switch input REFL and REFR
        ("swap_lr", ctypes.c_uint32, 1),
        # 1: Activates latching of the position to XLATCH upon an active going edge
        # on the left reference switch input REFL.
        # Hint: Activate latch_l_active to detect any spurious stop event by reading
        # status_latch_l
        ("latch_l_active", ctypes.c_uint32, 1),
        # 1: Activates latching of the position to XLATCH upon an inactive going
        # edge on the left reference switch input REFL. The active level is defined
        # by pol_stop_l.
        ("latch_l_inactive", ctypes.c_uint32, 1),
        # 1: Activates latching of the position to XLATCH upon an active going edge
        # on the right reference switch input REFR.
        # Hint: Activate latch_r_active to detect any spurious stop event by reading
        # status_latch_r
        ("latch_r_active", ctypes.c_uint32, 1),
        # 1: Activates latching of the position to XLATCH upon an inactive going
        # edge on the right reference switch input REFR. The active level is defined
        # by pol_stop_r.
        ("latch_r_inactive", ctypes.c_uint32, 1),
        # 1: Latch encoder position to ENC_LATCH upon reference switch event.
        ("en_latch_encoder", ctypes.c_uint32, 1),
        # 1: Enable stop by StallGuard2 (also available in DcStep mode). Disable to
        # release motor after stop event.
        # Attention: Do not enable during motor spin-up, wait until the motor
        # velocity exceeds a certain value, where StallGuard2 delivers a stable
        # result. This velocity threshold should be programmed using TCOOLTHRS.
        ("sg_stop", ctypes.c_uint32, 1),
        # 0: Hard stop
        # 1: Soft stop
        # The soft stop mode always uses the deceleration ramp settings DMAX, V1,
        # D1, VSTOP and TZEROWAIT for stopping the motor. A stop occurs when
        # the velocity sign matches the reference switch position (REFL for negative
        # velocities, REFR for positive velocities) and the respective switch stop
        # function is enabled.
        # A hard stop also uses TZEROWAIT before the motor becomes released.
        # Attention: Do not use soft stop in combination with StallGuard2.
        ("en_softstop", ctypes.c_uint32, 1),
        # Adds padding to the structure
        ("_padding", ctypes.c_uint32, 20),
    ]


class RampStat(HelperBase, ctypes.LittleEndianStructure):
    """
    Ramp status - Ramp and Reference Switch Status Register
    """

    address: int = 0x35
    _pack_ = 1
    _fields_ = [
        # Reference switch left status (1=active)
        ("status_stop_l", ctypes.c_uint32, 1),
        # Reference switch right status (1=active)
        ("status_stop_r", ctypes.c_uint32, 1),
        # 1: Latch left ready
        # (enable position latching using SW_MODE settings
        # latch_l_active or latch_l_inactive)
        # (Flag is cleared upon reading)
        ("status_latch_l", ctypes.c_uint32, 1),
        #  1: Latch right ready
        # (enable position latching using SW_MODE settings
        # latch_r_active or latch_r_inactive)
        # (Flag is cleared upon reading)
        ("status_latch_r", ctypes.c_uint32, 1),
        # 1: Signals an active stop left condition due to stop switch.
        # The stop condition and the interrupt condition can be removed by
        # setting RAMP_MODE to hold mode or by commanding a move to the
        # opposite direction. In soft_stop mode, the condition will remain
        # active until the motor has stopped motion into the direction of the
        # stop switch. Disabling the stop switch or the stop function also
        # clears the flag, but the motor will continue motion.
        # This bit is ORed to the interrupt output signal.
        ("event_stop_l", ctypes.c_uint32, 1),
        # 1: Signals an active stop right condition due to stop switch.
        # The stop condition and the interrupt condition can be removed by
        # setting RAMP_MODE to hold mode or by commanding a move to the
        # opposite direction. In soft_stop mode, the condition will remain
        # active until the motor has stopped motion into the direction of the
        # stop switch. Disabling the stop switch or the stop function also
        # clears the flag, but the motor will continue motion.
        # This bit is ORed to the interrupt output signal.
        ("event_stop_r", ctypes.c_uint32, 1),
        # 1: Signals an active StallGuard2 stop event.
        # Reading the register will clear the stall condition and the motor may
        # re-start motion unless the motion controller has been stopped.
        # (Flag and interrupt condition are cleared upon reading)
        # This bit is ORed to the interrupt output signal
        ("event_stop_sg", ctypes.c_uint32, 1),
        # 1: Signals, that the target position has been reached
        # (position_reached becoming active).
        # (Flag and interrupt condition are cleared upon reading)
        # This bit is ORed to the interrupt output signal.
        ("event_pos_reached", ctypes.c_uint32, 1),
        # 1: Signals, that the target velocity is reached.
        # This flag becomes set while VACTUAL and VMAX match.
        ("velocity_reached", ctypes.c_uint32, 1),
        # 1: Signals, that the target position is reached.
        # This flag becomes set while XACTUAL and XTARGET match.
        ("position_reached", ctypes.c_uint32, 1),
        # 1: Signals, that the actual velocity is 0.
        ("vzero", ctypes.c_uint32, 1),
        # 1: Signals, that TZEROWAIT is active after a motor stop. During this
        # time, the motor is in standstill.
        ("t_zerowait_active", ctypes.c_uint32, 1),
        # 1: Signals that the automatic ramp required moving back in the
        # opposite direction, e.g. due to on-the-fly parameter change
        # (Flag is cleared upon reading)
        ("second_move", ctypes.c_uint32, 1),
        # 1: Signals an active StallGuard2 input from the CoolStep driver or
        # from the DcStep unit, if enabled.
        # Hint: When polling this flag, stall events may be missed – activate
        # sg_stop to be sure not to miss the stall event.
        ("status_sg", ctypes.c_uint32, 1),
        # Adds padding to the structure
        ("_padding", ctypes.c_uint32, 18),
    ]


class XLatch(HelperBase, ctypes.LittleEndianStructure):
    """
    Ramp generator latch position, latches XACTUAL upon a
    programmable switch event (see SW_MODE).
    Hint: The encoder position can be latched to ENC_LATCH
    together with XLATCH to allow consistency checks.
    """

    address: int = 0x36
    _pack_ = 1
    _fields_ = [
        ("xlatch", ctypes.c_uint32, 32),
    ]


class EncMode(HelperBase, ctypes.LittleEndianStructure):
    """
    Encoder configuration and use of N channel
    """

    address: int = 0x38
    _pack_ = 1
    _fields_ = [
        # Required A polarity for an N channel event (0=neg., 1=pos.)
        ("pol_a", ctypes.c_uint32, 1),
        # Required B polarity for an N channel event (0=neg., 1=pos.)
        ("pol_b", ctypes.c_uint32, 1),
        # Defines active polarity of N (0=low active, 1=high active)
        ("pol_n", ctypes.c_uint32, 1),
        # 0 An N event occurs only when polarities given by
        # pol_N, pol_A and pol_B match.
        # 1 Ignore A and B polarity for N channel event
        ("ignore_ab", ctypes.c_uint32, 1),
        # 1: Always latch or latch and clear X_ENC upon an N event (once per
        # revolution, it is recommended to combine this setting with edge sensitive
        # N event)
        ("clr_cont", ctypes.c_uint32, 1),
        # 1: Latch or latch and clear X_ENC on the next N event following the write
        # access
        ("clr_once", ctypes.c_uint32, 1),
        # n p N channel event sensitivity
        # 0 0 N channel event is active during an active N event level
        # 0 1 N channel is valid upon active going N event
        # 1 0 N channel is valid upon inactive going N event
        # 1 1 N channel is valid upon active going and inactive going N event
        ("pos_edge_neg_edge", ctypes.c_uint32, 2),
        # 0 Upon N event, X_ENC becomes latched to ENC_LATCH only
        # 1 Latch and additionally clear encoder counter X_ENC at N-event
        ("clr_enc_x", ctypes.c_uint32, 1),
        # 1: Also latch XACTUAL position together with X_ENC.
        # Allows latching the ramp generator position upon an N channel event as
        # selected by pos_edge and neg_edge.
        ("latch_x_act", ctypes.c_uint32, 1),
        # 0 Encoder prescaler divisor binary mode:
        # Counts ENC_CONST (fractional part) /65536
        # 1 Encoder prescaler divisor decimal mode:
        # Counts in ENC_CONST (fractional part) /10000
        ("enc_sel_decimal", ctypes.c_uint32, 1),
        # Adds padding to the structure
        ("_padding", ctypes.c_uint32, 22),
    ]


class XEncoder(HelperBase, ctypes.LittleEndianStructure):
    """
    Actual encoder postion (signed)
    """

    address: int = 0x39
    _pack_ = 1
    _fields_ = [
        ("x_enc", ctypes.c_uint32, 32),
    ]


class EncoderConst(HelperBase, ctypes.LittleEndianStructure):
    """
    Accumulation constant (signed)
    16 bit integer part, 16 bit fractional part
    X_ENC accumulates
    +/- ENC_CONST / (2^16*X_ENC) (binary)
    or
    +/-ENC_CONST / (10^4*X_ENC) (decimal)
    ENCMODE bit enc_sel_decimal switches
    between decimal and binary setting.
    Use the sign, to match rotation direction!
    """

    address: int = 0x3A
    _pack_ = 1
    _fields_ = [
        ("enc_const", ctypes.c_uint32, 32),
    ]


class EncoderStatus(HelperBase, ctypes.LittleEndianStructure):
    """
    Encoder status
    bit 0: n_event
    1: Encoder N event detected. Status bit is
    cleared on read: Read (R) + clear (C)
    This bit is ORed to the interrupt output
    signal.
    """

    address: int = 0x3B
    _pack_ = 1
    _fields_ = [
        ("enc_status", ctypes.c_uint32, 1),
        ("_padding", ctypes.c_uint32, 31),
    ]


class EncoderLatch(HelperBase, ctypes.LittleEndianStructure):
    """
    Encoder position X_ENC latched on N event
    """

    address: int = 0x3C
    _pack_ = 1
    _fields_ = [
        ("enc_latch", ctypes.c_uint32, 32),
    ]


class MSlut0(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table 0

    Applys to all of the tables below
        Each bit gives the difference between entry x
        and entry x+1 when combined with the corresponding MSLUTSEL W bits:
        0: W= %00: -1
        %01: +0
        %10: +1
        %11: +2
        1: W= %00: +0
        %01: +1
        %10: +2
        %11: +3
        This is the differential coding for the first
        quarter of a wave. Start values for CUR_A and
        CUR_B are stored for MSCNT position 0 in
        START_SIN and START_SIN90.
        ofs31, ofs30, …, ofs01, ofs00
        …
        ofs255, ofs254, …, ofs225, ofs224
    """

    address: int = 0x60
    _pack_ = 1
    _fields_ = [
        ("mslut0", ctypes.c_uint32, 32),
    ]


class MSlut1(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table 1
    """

    address: int = 0x61
    _pack_ = 1
    _fields_ = [
        ("mslut1", ctypes.c_uint32, 32),
    ]


class MSlut2(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table 2
    """

    address: int = 0x62
    _pack_ = 1
    _fields_ = [
        ("mslut2", ctypes.c_uint32, 32),
    ]


class MSlut3(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table 3
    """

    address: int = 0x63
    _pack_ = 1
    _fields_ = [
        ("mslut3", ctypes.c_uint32, 32),
    ]


class MSlut4(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table 4
    """

    address: int = 0x64
    _pack_ = 1
    _fields_ = [
        ("mslut4", ctypes.c_uint32, 32),
    ]


class MSlut5(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table 5
    """

    address: int = 0x65
    _pack_ = 1
    _fields_ = [
        ("mslut5", ctypes.c_uint32, 32),
    ]


class MSlut6(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table 6
    """

    address: int = 0x66
    _pack_ = 1
    _fields_ = [
        ("mslut6", ctypes.c_uint32, 32),
    ]


class MSlut7(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table 7
    """

    address: int = 0x67
    _pack_ = 1
    _fields_ = [
        ("mslut7", ctypes.c_uint32, 32),
    ]


class MSlutSel(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep table selector
    """

    address: int = 0x68
    _pack_ = 1
    _fields_ = [
        # Width control bit coding W0…W3:
        # %00: MSLUT entry 0, 1 select: -1, +0
        # %01: MSLUT entry 0, 1 select: +0, +1
        # %10: MSLUT entry 0, 1 select: +1, +2
        # %11: MSLUT entry 0, 1 select: +2, +3
        ("WidthN", ctypes.c_uint32, 8),
        # The sine wave look-up table can be divided into up to
        # four segments using an individual step width control
        # entry Wx. The segment borders are selected by X1, X2
        # and X3.
        # Segment 0 goes from 0 to X1-1.
        # Segment 1 goes from X1 to X2-1.
        # Segment 2 goes from X2 to X3-1.
        # Segment 3 goes from X3 to 255.
        # For defined response the values shall satisfy:
        # 0<X1<X2<X3
        ("X1", ctypes.c_uint32, 8),
        ("X2", ctypes.c_uint32, 8),
        ("X3", ctypes.c_uint32, 8),
    ]


class MslutStart(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep start sine
    bit 7… 0: START_SIN
    bit 23… 16: START_SIN90
    START_SIN gives the absolute current at
    microstep table entry 0.
    START_SIN90 gives the absolute current for
    microstep table entry at positions 256.
    Start values are transferred to the microstep
    registers CUR_A and CUR_B, whenever the
    reference position MSCNT=0 is passed.
    """

    address: int = 0x69
    _pack_ = 1
    _fields_ = [
        ("start_sin", ctypes.c_uint32, 8),
        ("_padding_1", ctypes.c_uint32, 8),
        ("start_sin90", ctypes.c_uint32, 8),
        ("_padding_2", ctypes.c_uint32, 8),
    ]


class MicrostepCounter(HelperBase, ctypes.LittleEndianStructure):
    """
    Microstep counter. Indicates actual position
    in the microstep table for CUR_A. CUR_B uses
    an offset of 256 (2 phase motor).
    Hint: Move to a position where MSCNT is
    zero before re-initializing MSLUTSTART or
    MSLUT and MSLUTSEL.
    """

    address: int = 0x6A
    _pack_ = 1
    _fields_ = [
        ("mscnt", ctypes.c_uint32, 10),
        ("_padding", ctypes.c_uint32, 22),
    ]


class MicrostepCurrent(HelperBase, ctypes.LittleEndianStructure):
    """
    Actual microstep Current
    bit 8… 0: CUR_B (signed):
    Actual microstep current for
    motor phase B (sine wave) as
    read from MSLUT (not scaled by
    current)
    bit 24… 16: CUR_A (signed):
    Actual microstep current for
    motor phase A (co-sine wave) as
    read from MSLUT (not scaled by
    current)
    """

    address: int = 0x6B
    _pack_ = 1
    _fields_ = [
        ("mscuract", ctypes.c_uint32, 10),
        ("_padding", ctypes.c_uint32, 22),
    ]


class ChopConf(HelperBase, ctypes.LittleEndianStructure):
    """
    Chopper configuration for automatic current
    """

    address: int = 0x6C
    _pack_ = 1
    _fields_ = [
        ("toff", ctypes.c_uint32, 4),
        ("hstrt", ctypes.c_uint32, 3),
        ("hend", ctypes.c_uint32, 4),
        ("fd3", ctypes.c_uint32, 1),
        ("disfdcc", ctypes.c_uint32, 1),
        ("rndtf", ctypes.c_uint32, 1),
        ("chm", ctypes.c_uint32, 1),
        ("tbl", ctypes.c_uint32, 2),
        ("vsense", ctypes.c_uint32, 1),
        ("vhighfs", ctypes.c_uint32, 1),
        ("vhighchm", ctypes.c_uint32, 1),
        ("sync", ctypes.c_uint32, 4),
        ("mres", ctypes.c_uint32, 4),
        ("intpol", ctypes.c_uint32, 1),
        ("dedge", ctypes.c_uint32, 1),
        ("diss2g", ctypes.c_uint32, 1),
        ("padding_2", ctypes.c_uint32, 1),
    ]


class CoolConf(HelperBase, ctypes.LittleEndianStructure):
    """
    CoolStep configuration - smart current control
    """

    address: int = 0x6D
    _pack_ = 1
    _fields_ = [
        ("semin", ctypes.c_uint32, 4),
        ("padding_1", ctypes.c_uint32, 1),
        ("seup", ctypes.c_uint32, 2),
        ("padding_2", ctypes.c_uint32, 1),
        ("semax", ctypes.c_uint32, 4),
        ("padding_3", ctypes.c_uint32, 1),
        ("sedn", ctypes.c_uint32, 2),
        ("seimin", ctypes.c_uint32, 1),
        ("sgt", ctypes.c_uint32, 7),
        ("padding_4", ctypes.c_uint32, 1),
        ("sfilt", ctypes.c_uint32, 1),
        ("_padding", ctypes.c_uint32, 7),
    ]


class DCControl(HelperBase, ctypes.LittleEndianStructure):
    """
    DC motor control
    """

    address: int = 0x6E
    _pack_ = 1
    _fields_ = [
        ("dc_time", ctypes.c_uint32, 10),
        ("padding_1", ctypes.c_uint32, 6),
        ("dc_sg", ctypes.c_uint32, 8),
        ("padding_2", ctypes.c_uint32, 8),
    ]


class DriverStatus(HelperBase, ctypes.LittleEndianStructure):
    """
    DRV_STATUS - StallGuard2 Value and Driver Error Flags
    """

    address: int = 0x6F
    _pack_ = 1
    _fields_ = [
        ("sg_result", ctypes.c_uint32, 10),
        ("padding_1", ctypes.c_uint32, 5),
        ("fsactive", ctypes.c_uint32, 1),
        ("cs_actual", ctypes.c_uint32, 5),
        ("padding_2", ctypes.c_uint32, 3),
        ("stallguard", ctypes.c_uint32, 1),
        ("ot", ctypes.c_uint32, 1),
        ("otpw", ctypes.c_uint32, 1),
        ("s2ga", ctypes.c_uint32, 1),
        ("s2gb", ctypes.c_uint32, 1),
        ("ola", ctypes.c_uint32, 1),
        ("olb", ctypes.c_uint32, 1),
        ("stst", ctypes.c_uint32, 1),
    ]


class PWMConf(HelperBase, ctypes.LittleEndianStructure):
    """
    PWM configuration
    """

    address: int = 0x70
    _pack_ = 1
    _fields_ = [
        ("pwm_ampl", ctypes.c_uint32, 8),
        ("pwm_grad", ctypes.c_uint32, 8),
        ("pwm_freq", ctypes.c_uint32, 2),
        ("pwm_autoscale", ctypes.c_uint32, 1),
        ("pwm_symmetric", ctypes.c_uint32, 1),
        ("freewheel", ctypes.c_uint32, 2),
        ("_padding", ctypes.c_uint32, 10),
    ]


class PWMScale(HelperBase, ctypes.LittleEndianStructure):
    """
    PWM scale
    """

    address: int = 0x71
    _pack_ = 1
    _fields_ = [
        ("pwm_scale", ctypes.c_uint32, 8),
        ("_padding", ctypes.c_uint32, 24),
    ]


class EncoderModeConf(HelperBase, ctypes.LittleEndianStructure):
    """
    Encoder mode configuration for a special
    mode (enc_commutation), not for normal
    use.
    Bit 0: inv: Invert encoder inputs
    Bit 1: maxspeed: Ignore Step input. If
    set, the hold current IHOLD
    determines the motor current,
    unless a step source is activated.
    The direction in this mode is determined by
    the shaft bit in GCONF or by the inv bit.

    """

    address: int = 0x72
    _pack_ = 1
    _fields_ = [
        ("enc_m_ctrl", ctypes.c_uint32, 2),
        ("_padding", ctypes.c_uint32, 30),
    ]


class LostSteps(HelperBase, ctypes.LittleEndianStructure):
    """
    Lost steps
    """

    address: int = 0x73
    _pack_ = 1
    _fields_ = [
        ("lost_steps", ctypes.c_uint32, 20),
        ("_padding", ctypes.c_uint32, 12),
    ]


class TMC5130:
    WRITE_MASK = 0x80
    READ_MASK = 0x00
    SECONDS_PER_MINUTE = 60

    def __init__(
        self,
        spi=board.SPI(),
        cs: int = board.D5,
        freq_hz: int = 1_000_000,
        motor_config: dict | None = None,
    ):
        # Using SPIDevice to manage the SPI device
        self.cs = digitalio.DigitalInOut(cs)
        self.cs.direction = digitalio.Direction.OUTPUT
        self.cs.value = True  # Active low chip select
        # SPIMODE3 (CPOL=1, CPHA=1) is the default mode for TMC5130
        self._spi = spi
        self.spi = SPIDevice(
            self._spi,
            self.cs,
            cs_active_value=False,
            baudrate=freq_hz,
            polarity=1,
            phase=1,
        )

        # Motor Config
        self._config = motor_config or {
            # 0: Positioning Mode, 1: Velocity Mode +, 2: Velocity Mode -, 3: Hold Mode
            "ramp_mode": 1,
            "rsense": 0.15,  # Sense Resistor Value
            "current_mA": {"hold_mA": 50, "run_mA": 200},
            "driver": {
                "microstep": 256,
            },
            "motion": {
                "velocity_rpm": 0,
                "acceleration_rpm": 60,
                "deceleration_rpm": 60,
            },
            "motor": {
                "steps_per_rev": 200,  # Steps per revolution
                "units_per_rev": 1.0,  # How far linear carriage moves per revolution of the shaft
            },
        }

        # Easier access to config using dot notation
        # E.g self.config.motor.steps_per_rev
        self.config = DotWiz(self._config)

        print(self.config)

        # Realworld Unit Conversion
        # Page 75 https://www.analog.com/media/en/technical-documentation/data-sheets/TMC5130A_datasheet_rev1.20.pdf

        self._fclk: int = 12_000_000  # 12MHz Internal Clock

        # Where v is the velocity in ustep/t and t = 2^24 / fCLK
        # µstep/s velocity = ustep/t * (fCLK[Hz]/2/2^23)
        # self._ustep_s is used to convert tmc velocity register to real world velocity - read from register
        self._ustep_s = lambda ustep_t: (ustep_t * ((self._fclk / 2 / 2**23)))
        # self._ustep_t is used to convert real world velocity to tmc velocity register - write to register
        # ustep/t = v[ustep/s] / (fCLK[Hz]/2/2^23)
        self._ustep_t = lambda usteps_s: (usteps_s / (self._fclk / 2 / 2**23))

        # µstep acceleration ustep_s^2 = ustep/ta^2 * fCLK[Hz]^2 / (512*256) / 2^24
        # Where a is the acceleration in ustep/ta^2 and ta^2 = 2^41 / (fCLK)^2
        # self._ustep_s2 is used tmc accerleration register to real world acceleration - read from register
        self._ustep_s2 = lambda ustep_ta2: ustep_ta2 * (self._fclk**2 / (512 * 256) / 2**24)
        # The calculation is taking step/s^2 as input and converting it to ustep/ta^2
        # self._ustep_ta2 is used to convert real world acceleration to tmc acceleration register - write to register
        self._ustep_ta2 = lambda ustep_s2: ustep_s2 * ((512 * 256) * 2**24 / self._fclk**2)

        # rotations / s to ustep/s = rps * microstep * steps_per_rev
        # rotations / s v[rps] = v[µsteps/s] / Microstep / Full step per rotation
        # self._from_rps is used to rps input to tmc velocity register - write to register
        self._from_rps = lambda rps: self._ustep_t(rps * self.config.driver.microstep * self.config.motor.steps_per_rev)

        # rotations / s^2 a[rps^2] = a[µsteps/s^2] / Microstep / Full step per rotation
        # self._from_rps2 is used to rps^2 input to tmc acceleration register - write to register
        self._from_rps2 = lambda rps2: self._ustep_ta2(rps2 * self.config.driver.microstep * self.config.motor.steps_per_rev)

        # Ramp steps[µsteps]  = rs = (v[5130A])^2 / a[5130A] / 2^8
        # Microsteps during linear acceleration ramp (assuming acceleration from 0 to v)
        self._ramp_steps = lambda v, a: (self._ustep_s(v)) ** 2 / self._ustep_s2(a) / 2**8
        #  2^24 / (VACTUAL*256/USC)
        self._tstep = lambda vactual: 2**24 / (vactual * 256 / self.config.driver.microstep)

        # Actual update frequency
        self._ramp_rate_hz = self._fclk / 512

        # Custom Conversions
        # mm or deg -> ustep
        self._from_mm_deg = (
            lambda mm_deg: mm_deg * self.config.motor.steps_per_rev * self.config.driver.microstep * self.config.motor.units_per_rev
        )
        # mm/s or deg/s -> ustep/s -> ustep/t
        self._from_mms_degs = lambda mms_degs: self._ustep_t(self._from_mm_deg(mms_degs))
        # mm/s^2 or deg/s^2 -> ustep/s^2 -> ustep/ta^2
        self._from_mms2_degs2 = lambda mms2_degs2: self._ustep_ta2(self._from_mm_deg(mms2_degs2))

        # ustep -> mm or deg
        self._to_mm_deg = lambda ustep: ustep / (
            self.config.motor.steps_per_rev * self.config.driver.microstep * self.config.motor.units_per_rev
        )
        # ustep/t -> ustep/s -> mm/s or deg/s
        self._to_mms_degs = lambda ustep_t: self._to_mm_deg(self._ustep_s(ustep_t))
        # ustep/ta^2 -> ustep/s^2 -> mm/s^2 or deg/s^2
        self._to_mms_degs2 = lambda ustep_ta2: self._to_mm_deg(self._ustep_s2(ustep_ta2))

    def _write(self, address: int, value: bytearray):
        """
        Write a value to the given address to TMC5130 via SPI
        """
        address |= self.WRITE_MASK

        # First byte is address
        write_data = bytearray([address]) + value
        with self.spi as spi:
            spi.write(write_data)

    def _read(self, address: int) -> tuple[int, bytearray]:
        """
        Read a value from the given address from TMC5130 via SPI
        """
        address |= self.READ_MASK
        # First byte is address
        write_data = bytearray([address, 0x00, 0x00, 0x00, 0x00])
        # MSB First
        read_data = bytearray(5)
        with self.spi as spi:
            spi.write_readinto(write_data, read_data)
            spi.write_readinto(write_data, read_data)

        # SPI Data; 8bit stats | MSB - LSB 32 bit data
        status = read_data[0]
        # Get bytes 1 to 4 and reverse them so we can use the to create python classes
        data = read_data[1:][::-1]
        return status, data

    def init(self):
        """
        Configure the startup settings for TMC5130
        """
        # General Config
        general_config = GeneralConfig()
        chop_config = ChopConf()
        pwm_mode_chop = PWMConf()
        tpwm_threshold = TPWMThreshold()
        cool_threshold = TCoolThreshold()
        power_down = TPowerDown()  # Power down in standstill
        power_down.tpowerdown = 0x10
        ramp_mode = RampMode()
        cool_conf = CoolConf()

        # Set the general config
        general_config.shaft = 0
        general_config.diag0_stall = 0
        general_config.diag0_int_pushpull = 0
        general_config.en_pwm_mode = 1  # Does not work with slow speeds

        # Chopper Config
        chop_config.toff = 0x03  # 0010
        chop_config.hstrt = 0x04
        chop_config.hend = 0x1
        chop_config.tbl = 0x2  # Blank time
        chop_config.chm = 0x01  # chopper mode
        chop_config.vhighfs = 0x01  # Enable Fullstep at high velocity
        chop_config.vhighchm = 0x01  # High velocity chopper mode

        # Cool Step Config - Stallguard
        cool_conf.sgt = 0x00

        # PWM Conf
        pwm_mode_chop.pwm_autoscale = 1
        pwm_mode_chop.freewheel = 0x1  # 0 default, 1 free wheeling
        pwm_mode_chop.pwm_ampl = 0x255
        pwm_mode_chop.pwm_grad = 0x4

        tpwm_threshold.tpwmthrs = 500

        # Move type : ramp mode
        ramp_mode.ramp_mode = self.config.get("ramp_mode", 0)

        # Write to TMC Driver
        self._write(GeneralConfig.address, general_config.get_register())
        self._write(ChopConf.address, chop_config.get_register())
        self._write(CoolConf.address, cool_conf.get_register())
        self._write(PWMConf.address, pwm_mode_chop.get_register())
        self._write(TPWMThreshold.address, tpwm_threshold.get_register())
        self._write(TCoolThreshold.address, cool_threshold.get_register())
        self._write(TPowerDown.address, power_down.get_register())
        self._write(RampMode.address, ramp_mode.get_register())
        self.set_current(**self.config.get("current_mA", {}))
        self.set_target_position(0)
        self.set_current_position(0)
        self.set_motion_rpm(velocity_rpm=0, acceleration_rpm=0, deceleration_rpm=0)

    def set_current(self, run_mA: int, hold_mA: int):
        """
        Set the current for the TMC Driver
        """
        Rsense = self.config.rsense
        # Convert from mA to ihold and irun
        irun = 32.0 * 1.41421 * run_mA / 1000.0 * (Rsense + 0.02) / 0.325 - 1
        # Limit ihold to 31 and 0
        irun = min(31, max(0, irun))
        if hold_mA != 0:
            ihold = 32.0 * 1.41421 * hold_mA / 1000.0 * (Rsense + 0.02) / 0.325 - 1
        else:
            ihold = 0
        ihold = min(31, max(0, ihold))
        iholddelay = 3

        # Set the current
        _current = IHoldIRun()
        _current.ihold = int(ihold)
        _current.irun = int(irun)
        _current.iholddelay = int(iholddelay)

        self._write(IHoldIRun.address, _current.get_register())

    def set_motion_rpm(self, velocity_rpm: float, acceleration_rpm: float, deceleration_rpm: float):
        """
        Set the motion for the TMC Driver
        """
        velocity = self._from_rps(velocity_rpm / self.SECONDS_PER_MINUTE)
        acceleration = self._from_rps2(acceleration_rpm / self.SECONDS_PER_MINUTE)
        deceleration = self._from_rps2(deceleration_rpm / self.SECONDS_PER_MINUTE)
        print(f"Velocity: {velocity} Acceleration: {acceleration} Deceleration: {deceleration}")
        self.set_motion(velocity, acceleration, deceleration)

    def set_motion_units(
        self,
        velocity_mms_degs: float,
        acceleration_mms_degs: float,
        deceleration_mms_degs: float,
    ):
        """
        Set the motion for the TMC Driver
        """
        velocity = self._from_mms_degs(velocity_mms_degs)
        acceleration = self._from_mms2_degs2(acceleration_mms_degs)
        deceleration = self._from_mms2_degs2(deceleration_mms_degs)
        print(f"Velocity: {velocity} Acceleration: {acceleration} Deceleration: {deceleration}")
        self.set_motion(velocity, acceleration, deceleration)

    def set_motion(
        self,
        velocity: int | float,
        acceleration: int | float,
        deceleration: int | float,
        stage_1_2=1.0,
    ):
        """
        Set the motion for the TMC Driver
        velocity: Velocity in ustep/s
        acceleration: Acceleration in ustep/s^2
        deceleration: Deceleration in ustep/s^2
        stage_1_2: TMC5130 has two acceleration stages for 6 point of motion
        This ratio determines the split between the two stages.
        """
        # Set the motion
        _motion = AMax()
        _motion.amax = int(acceleration)
        self._write(AMax.address, _motion.get_register())

        _motion = DMax()
        _motion.dmax = int(deceleration)
        self._write(DMax.address, _motion.get_register())

        _motion = VMax()
        _motion.vmax = int(velocity)
        self._write(VMax.address, _motion.get_register())

        _motion = A1()
        _motion.a1 = int(acceleration * stage_1_2)
        self._write(A1.address, _motion.get_register())

        _motion = D1()
        _motion.d1 = int(deceleration * stage_1_2)
        self._write(D1.address, _motion.get_register())

        _motion = V1()
        _motion.v1 = int(velocity * stage_1_2)
        self._write(V1.address, _motion.get_register())

        _motion = VStart()
        _motion.vstart = 0
        self._write(VStart.address, _motion.get_register())

        _motion = VStop()
        _motion.vstop = 10
        self._write(VStop.address, _motion.get_register())

        _motion = TZeroWait()
        _motion.tzerowait = 0
        self._write(TZeroWait.address, _motion.get_register())

    def set_velocity_rpm(self, velocity_rpm: float):
        """
        Set the velocity of the motor
        """
        velocity = self._from_rps(velocity_rpm / self.SECONDS_PER_MINUTE)
        _vmax = VMax()
        _vmax.vmax = int(velocity)
        self._write(VMax.address, _vmax.get_register())

    def read_version(self):
        """
        Read the TMC5130 version
        """
        _, data = self._read(IOStatus.address)
        driver_info = IOStatus.from_buffer(data)
        return driver_info.version

    def get_io_status(self):
        """
        Read the TMC5130 IO Status
        """
        _, data = self._read(IOStatus.address)
        io_status = IOStatus.from_buffer(data)

        # Copy all of the values to a dictionary
        io_dict = {
            "refl_step": io_status.refl_step,
            "refr_dir": io_status.refr_dir,
            "enca_dcin_cfg5": io_status.enca_dcin_cfg5,
            "encb_dcen_cfg4": io_status.encb_dcen_cfg4,
            "drv_enn_cfg6": io_status.drv_enn_cfg6,
            "enc_n_dco": io_status.enc_n_dco,
            "sd_mode": io_status.sd_mode,
            "swcomp_in": io_status.swcomp_in,
            "version": io_status.version,
        }

        return deepcopy(io_dict)

    def set_target_position(self, position_mm_deg: float):
        """
        Set the target position
        """
        xtarget = XTarget()
        xtarget.xtarget = int(self._from_mm_deg(position_mm_deg))
        self._write(XTarget.address, xtarget.get_register())

    def set_current_position(self, position_mm_deg: float):
        """
        Set the current position
        """
        xactual = XActual()
        xactual.xactual = int(self._from_mm_deg(position_mm_deg))
        self._write(XActual.address, xactual.get_register())

        xtarget = XTarget()
        xtarget.xtarget = xactual.xactual
        self._write(XTarget.address, xtarget.get_register())

    def get_current_position(self):
        """
        Get the current position
        """
        _, data = self._read(XActual.address)
        xactual = XActual.from_buffer(data)
        return self._to_mm_deg(xactual.xactual)

    def stop_motion(self):
        """
        Stop the motion
        a) Switch to velocity mode, set VMAX=0 and AMAX to the desired deceleration value. This will stop
        the motor using a linear ramp.
        b) For a stop in positioning mode, set VSTART=0 and VMAX=0. VSTOP is not used in this case. The
        driver will use AMAX and A1 (as determined by V1) for going to zero velocity.
        c) For a stop using D1, DMAX and VSTOP, trigger the deceleration phase by copying XACTUAL to
        XTARGET. Set TZEROWAIT sufficiently to allow the CPU to interact during this time. The driver will
        decelerate and eventually come to a stop. Poll the actual velocity to terminate motion during
        TZEROWAIT time using option a) or b).
        d) Activate a stop switch. This can be done by means of the hardware input, e.g. using a wired 'OR'
        to the stop switch input. If you do not use the hardware input and have tied the REFL and REFR
        to a fixed level, enable the stop function (stop_l_enable, stop_r_enable) and use the inverting
        function (pol_stop_l, pol_stop_r) to simulate the switch activation.
        """

        _vstart = VStart()
        _vstart.vstart = 0
        self._write(VStart.address, _vstart.get_register())
        _vmax = VMax()
        _vmax.vmax = 0
        self._write(VMax.address, _vmax.get_register())


if __name__ == "__main__":
    # Load motor config
    with open("config.json5", "r") as f:
        # Use json5 converter to convert json5 to dict
        motor_config = json5.load(f)
    tmc = TMC5130(motor_config=motor_config)
    time.sleep(1)
    tmc.init()
    tmc.set_current(run_mA=100, hold_mA=50)
    tmc.set_motion_rpm(velocity_rpm=60, acceleration_rpm=60, deceleration_rpm=60)
    while True:
        time.sleep(10)
        tmc.stop_motion()
