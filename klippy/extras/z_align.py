# Copyright (C) 2026  36573259+Jacob10383@users.noreply.github.com
# This file may be distributed under the terms of the GNU GPLv3 license.
#
# MCU-assisted Z prep helper for integrated homing.
#
# Coordinates the MCU-side dual-Z photoelectric alignment before final host
# Z homing. XY homing can run while the MCU alignment is active; the post-align
# bed rise is a normal homing move.
#
# Each Z motor drops onto its own bottom switch, which levels the bed. Both
# sides then back off together and re-approach slowly twice; how far a side
# misses its trigger point is that switch's repeatability, which sets how
# accurately a power-loss resume finds Z again.
#
# Options and their defaults; the K2 config only sets the rise:
# [z_align]
# quick_speed: 30  # mm/s, first drop onto the switches.
# slow_speed: 10  # mm/s, the re-approaches and every lift.
# retract_dist: 2  # mm both sides lift before each re-approach; must be
#                  # more than the switch hysteresis.
# safe_dist: 40  # mm one side may keep dropping after the other side's
#                # switch triggered (a bed that tilted at power off).
# repeat_tolerance: 0.01  # mm a side may miss its trigger point before
#                         # the alignment is redone.
# repeat_max: 0.5  # mm miss still accepted, with a warning, once the
#                  # retries are used up.
# retries: 2  # Re-runs when a side misses by more than repeat_tolerance.
# endstop_pin_z: PA15, PA8  # Bottom switch pins, one per Z motor.
# zes_untrig: 1  # Level an untriggered switch reads.
# rise_distance: 340  # mm rise after alignment before the final Z home.
# rise_speed: 50  # mm/s for that rise.
# temp_rise_max_z_accel: 200  # Temporary accel cap for that rise.
# zmax: 350  # Logical Z of the bed sitting on the switches.
#
# The direction toward the switches comes from each stepper's dir_pin.
#
import json
import logging
import math
import os

import mcu

from .motor_control import MOTOR_COMMAND_TIMEOUT


def _klog(msg, *args, level=logging.info):
    level("z_align: " + msg, *args)


POLL_INTERVAL = 0.010
TEMP_RISE_MAX_Z_VELOCITY = 100.0
TEMP_RISE_MAX_Z_ACCEL = 200.0
TILT_BIAS_FILE = '/mnt/UDISK/printer_data/z_align_tilt_bias.json'
MAX_TILT_BIAS = 1.0
LIFT_MAX_DIST = 15.0  # mm lift allowed for a triggered switch to release
FAST_MAX_MARGIN = 20.0  # mm fast drop allowed beyond the Z range
FILTER_COUNT = 10  # Consecutive switch samples, one per halfstep
MCU_PASSES = 2  # Re-approaches per alignment
SETTLE_TIME = 0.150  # s at rest after each stop, before the next move
# Options of the earlier alignment, accepted so older configs load
_IGNORED_OPTIONS = (
    'zd_up', 'rise_max_dist', 'rise_clearance', 'fast_max_dist',
    'filter_cnt', 'timeout', 'retry_tolerance', 'rising_dist',
)
MCU_Z_ALIGN_ERROR_REASONS = {
    1: 'invalid MCU z-align configuration',
    2: 'a bottom switch did not release after lifting',
    3: 'a bottom switch did not trigger within the Z range',
    4: 'one Z side dropped more than safe_dist after the other side stopped',
    5: 'a bottom switch did not trigger again on a re-approach',
    6: 'MCU z-align was force-stopped',
}
MCU_Z_ALIGN_PHASES = {1: 'clear', 2: 'fast', 3: 'retract', 4: 'slow'}
QUERY_FORMAT = (
    "query_z_align oid=%c quick_ticks=%u slow_ticks=%u retract=%u"
    " safe=%u fast_max=%u lift_max=%u filter=%c settle_ticks=%u")
STATUS_FORMAT = (
    "z_align_status oid=%c flag=%c error=%c phase=%c start=%c end=%c"
    " lift=%u fast0=%u fast1=%u released0=%u released1=%u"
    " slow0_0=%u slow0_1=%u slow1_0=%u slow1_1=%u")


class ZAlignNullEndstop:
    def __init__(self, steppers, reactor):
        self._steppers = tuple(steppers)
        self._reactor = reactor

    def get_steppers(self):
        return list(self._steppers)

    def home_start(self, print_time, sample_time, sample_count,
                   rest_time, triggered=True):
        return self._reactor.completion()

    def home_wait(self, home_end_time):
        return 0.0

    def query_endstop(self, print_time):
        return False


class ZAlign:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        self.endstop_pin_z = config.getlist('endstop_pin_z', ('PA15', 'PA8'))
        self.zes_untrig = config.getint('zes_untrig', 1)
        for option in _IGNORED_OPTIONS:
            config.get(option, None)
        self.fast_drop_speed = config.getfloat('quick_speed', 30.0, above=0.0)
        self.settle_drop_speed = config.getfloat('slow_speed', 10.0, above=0.0)
        self.retract_dist = config.getfloat('retract_dist', 2.0, above=0.0)
        self.safe_dist = config.getfloat('safe_dist', 40.0, above=0.0)
        self.repeat_tolerance = config.getfloat(
            'repeat_tolerance', 0.01, above=0.0)
        self.repeat_max = config.getfloat(
            'repeat_max', 0.5, minval=self.repeat_tolerance)
        self.retries = config.getint('retries', 2, minval=0)
        self.rise_distance = config.getfloat('rise_distance', 340.0, above=0.0)
        self.rise_speed = config.getfloat('rise_speed', 50.0, above=0.0)
        self.temp_rise_max_z_accel = config.getfloat(
            'temp_rise_max_z_accel', TEMP_RISE_MAX_Z_ACCEL, above=0.0)
        zconfig = config.getsection('stepper_z')
        z_range = (
            zconfig.getfloat('position_max', 360.0, note_valid=False)
            - zconfig.getfloat('position_min', 0.0, note_valid=False))
        self.fast_max_dist = z_range + FAST_MAX_MARGIN
        # The fast drop, then the clear lift and every pass at slow speed
        slow_dist = LIFT_MAX_DIST + self.retract_dist * (1 + 3 * MCU_PASSES)
        settles = 2 + 2 * MCU_PASSES
        self.timeout = 1.5 * (
            self.fast_max_dist / self.fast_drop_speed
            + slow_dist / self.settle_drop_speed
            + settles * SETTLE_TIME) + 5.0
        self.zmax = config.getfloat('zmax', 350.0, above=0.0)
        self._toolhead = None
        self._z_align_query_cmd = None
        self._z_align_force_stop_cmd = None
        self._main_mcu = mcu.get_printer_mcu(self.printer, "mcu")
        self._oidz = self._main_mcu.create_oid()
        self._main_mcu.register_config_callback(self._build_config)
        self._step_distance = None
        self._z_align_status = {}
        self._orig_serial_handle_default = None
        self._state = 'idle'
        self._error = None
        self._phase_deadline = None
        self._prepared_zmax = None
        self._target_z = None
        self._logical_z_applied = False
        self._settle_attempt = 0
        self._last_tilt_mm = 0.0
        self._last_repeat_mm = [0.0, 0.0]
        self._z_tilt = None
        self._tilt_bias = 0.0
        self._learn_delta = None
        self._bottom_anchor = None
        self.endstop_pin_status = [0] * min(len(self.endstop_pin_z), 8)
        self.pin_len = min(len(self.endstop_pin_z), 8)
        self._timer = self.reactor.register_timer(
            self._handle_timer, self.reactor.NEVER)
        buttons = self.printer.load_object(config, 'buttons')
        buttons.register_buttons(self.endstop_pin_z, self._button_handler)
        self.printer.register_event_handler('klippy:connect', self._handle_connect)
        self.printer.register_event_handler('klippy:ready', self._handle_ready)
        self.printer.register_event_handler(
            'klippy:shutdown', self._handle_shutdown)
        self.printer.register_event_handler(
            'stepper_enable:motor_off', self._handle_motor_off)
        self.printer.register_event_handler(
            'gcode:request_restart', self._handle_request_restart)

    def _build_config(self):
        config_z_align = "config_z_align oid=%d" % (self._oidz,)
        self._main_mcu.add_config_cmd(config_z_align)
        for stepper_index, endstop_pin in enumerate(self.endstop_pin_z):
            section_name = 'stepper_z' if stepper_index == 0 else 'stepper_z%d' % (stepper_index,)
            section = self.config.getsection(section_name)
            step_pin = section.get('step_pin')
            dir_pin = section.get('dir_pin')
            # Away from the switches is toward Z min, which the host drives
            # with the DIR level equal to the pin's inversion
            zd_up = 1 if dir_pin.startswith('!') else 0
            dir_pin = dir_pin.lstrip('!')
            config_z_align_add = (
                "config_z_align_add oid=%d z_indx=%d zs_pin=%s zd_pin=%s "
                "zd_up=%d zes_pin=%s zes_untrig=%d"
                % (
                    self._oidz,
                    stepper_index,
                    step_pin,
                    dir_pin,
                    zd_up,
                    endstop_pin,
                    self.zes_untrig,
                ))
            self._main_mcu.add_config_cmd(config_z_align_add)
            _klog(
                "config_z_align_add oid=%d z_indx=%d zs_pin=%s "
                "zd_pin=%s zd_up=%d zes_pin=%s zes_untrig=%d",
                self._oidz, stepper_index, step_pin, dir_pin, zd_up,
                endstop_pin, self.zes_untrig)

    def get_switch_states(self, state):
        return [1 if (state & (1 << i)) == 0 else 0 for i in range(self.pin_len)]

    def _button_handler(self, _eventtime, state):
        self.endstop_pin_status = self.get_switch_states(state)

    def _handle_connect(self):
        self._toolhead = self.printer.lookup_object('toolhead')
        self._install_status_cache_hook()
        self._z_align_query_cmd = self._main_mcu.lookup_query_command(
            QUERY_FORMAT, STATUS_FORMAT, oid=self._oidz)
        self._z_align_force_stop_cmd = self._main_mcu.lookup_command(
            "z_align_force_stop oid=%c", cq=None)
        steppers = {
            stepper.get_name(): stepper
            for stepper in self._toolhead.get_kinematics().get_steppers()
        }
        stepper_z = steppers.get('stepper_z')
        if stepper_z is None:
            raise self.printer.config_error(
                'z_align requires the stepper_z object')
        self._step_distance = stepper_z.get_step_dist()
        self._reset_runtime()

    def _handle_ready(self):
        try:
            self._install_tilt_learning()
        except Exception:
            self._z_tilt = None
            _klog('learned Z-tilt setup failed; continuing without it',
                  level=logging.exception)

    def _install_tilt_learning(self):
        self._load_tilt_bias()
        z_tilt = self.printer.lookup_object('z_tilt', None)
        if z_tilt is None or len(z_tilt.z_helper.z_steppers) != 2:
            return
        original_finalize = z_tilt.probe_helper.finalize_callback
        original_adjust = z_tilt.z_helper.adjust_steppers

        def finalize(offsets, positions):
            try:
                result = original_finalize(offsets, positions)
            except Exception:
                self._learn_delta = None
                raise
            if self._learn_delta is not None and z_tilt.z_status.applied:
                learned = self._tilt_bias + self._learn_delta
                self._learn_delta = None
                self._save_tilt_bias(learned)
            return result

        def adjust(adjustments, speed):
            result = original_adjust(adjustments, speed)
            if self._learn_delta is not None:
                try:
                    self._learn_delta += (
                        float(adjustments[1]) - float(adjustments[0]))
                except Exception:
                    self._learn_delta = None
                    _klog('discarding invalid Z-tilt learning data',
                          level=logging.exception)
            return result

        z_tilt.probe_helper.finalize_callback = finalize
        z_tilt.z_helper.adjust_steppers = adjust
        self._z_tilt = z_tilt

    def _handle_motor_off(self, *_args):
        self._learn_delta = None
        self._bottom_anchor = None

    def _load_tilt_bias(self):
        self._tilt_bias = 0.0
        try:
            with open(TILT_BIAS_FILE) as stream:
                value = float(json.load(stream)['bias_mm'])
            if not math.isfinite(value) or abs(value) > MAX_TILT_BIAS:
                raise ValueError('bias outside +/-%.1fmm' % MAX_TILT_BIAS)
            self._tilt_bias = value
        except FileNotFoundError:
            pass
        except Exception:
            _klog('ignoring invalid tilt-bias file %s', TILT_BIAS_FILE,
                  level=logging.exception)

    def _save_tilt_bias(self, value):
        try:
            value = round(float(value), 6)
            if not math.isfinite(value) or abs(value) > MAX_TILT_BIAS:
                raise ValueError('learned bias outside +/-%.1fmm' % MAX_TILT_BIAS)
            os.makedirs(os.path.dirname(TILT_BIAS_FILE), exist_ok=True)
            temp = TILT_BIAS_FILE + '.tmp'
            with open(temp, 'w') as stream:
                json.dump({'version': 1, 'bias_mm': value}, stream)
                stream.write('\n')
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temp, TILT_BIAS_FILE)
            self._tilt_bias = value
            _klog('learned Z-tilt bias %.6fmm', value)
        except Exception:
            _klog('unable to save learned Z-tilt bias',
                  level=logging.exception)

    def _apply_tilt_bias(self):
        if self._z_tilt is None:
            return False
        if self._tilt_bias:
            self._z_tilt.z_helper.adjust_steppers(
                [0.0, self._tilt_bias], self.settle_drop_speed)
            self._toolhead.wait_moves()
        self._learn_delta = 0.0
        return bool(self._tilt_bias)

    def _handle_z_align_status(self, params, source="callback"):
        self._z_align_status = dict(params)

    def _install_status_cache_hook(self):
        serial = self._main_mcu._serial
        if self._orig_serial_handle_default is not None:
            return
        self._orig_serial_handle_default = serial.handle_default

        def wrapped_handle_default(params):
            if isinstance(params, dict) and params.get("#name") == "z_align_status":
                self._handle_z_align_status(params, source="default")
                return
            return self._orig_serial_handle_default(params)

        serial.handle_default = wrapped_handle_default
        _klog("installed serial default hook for z_align_status")

    def _handle_request_restart(self, _print_time):
        self._learn_delta = None
        self._bottom_anchor = None
        self._reset_runtime()

    def _handle_shutdown(self):
        self.reactor.update_timer(self._timer, self.reactor.NEVER)
        self._reset_runtime()

    def _reset_runtime(self):
        self._state = 'idle'
        self._error = None
        self._phase_deadline = None
        self._prepared_zmax = None
        self._target_z = None
        self._logical_z_applied = False
        self._settle_attempt = 0
        self._last_tilt_mm = 0.0
        self._last_repeat_mm = [0.0, 0.0]
        self._z_align_status = {}

    def _say(self, msg):
        _klog('%s', msg)
        try:
            self.gcode.respond_info('Z prep: %s' % (msg,))
        except Exception:
            _klog(
                'failed to echo console message',
                level=logging.exception)

    def _request_homing_abort(self, reason):
        homing = self.printer.lookup_object('homing')
        if not homing.has_active_homing_session():
            return
        homing.request_homing_abort(
            reason=reason,
            detail={'source': 'z_align'},
            abort_z_align=False)

    def _ensure_ready(self):
        if self._toolhead is None:
            raise self.printer.command_error('Z_ALIGN is not connected')
        if self._z_align_query_cmd is None or self._z_align_force_stop_cmd is None:
            raise self.printer.command_error('Z_ALIGN MCU path is not connected')

    def _is_toolhead_z_homed(self):
        if self._toolhead is None:
            return False
        try:
            eventtime = self.reactor.monotonic()
            homed_axes = self._toolhead.get_kinematics().get_status(
                eventtime).get('homed_axes', '')
        except Exception:
            _klog(
                'failed reading toolhead Z homed state',
                level=logging.exception)
            return False
        return 'z' in homed_axes

    def _homing_session_aborted(self):
        homing = self.printer.lookup_object('homing')
        return bool(homing.is_homing_session_aborted())

    def _get_homing_abort_reason(self):
        homing = self.printer.lookup_object('homing')
        return homing.motor_fault_abort_reason

    def _z_protection_active(self):
        motor_control = self.printer.lookup_object('motor_control')
        try:
            result = motor_control.query_kinematic_protection_status(
                data=11, timeout=MOTOR_COMMAND_TIMEOUT)
        except Exception as err:
            _klog(
                'failed querying Z protection before logical Z apply',
                level=logging.exception)
            raise self.printer.command_error(
                'Z prep could not verify Z motor protection state: %s'
                % (err,))
        return any(bool(result.get(axis, {}).get('has_error'))
                   for axis in ('z', 'z1'))

    def _calc_speed_ticks(self, speed):
        mcu_freq = self._main_mcu._serial.msgparser.get_constant_float('CLOCK_FREQ')
        return max(1, int((self._step_distance / speed) * mcu_freq / 2.0))

    def _calc_time_ticks(self, seconds):
        mcu_freq = self._main_mcu._serial.msgparser.get_constant_float('CLOCK_FREQ')
        return max(1, int(seconds * mcu_freq))

    def _calc_distance_steps(self, distance):
        return max(1, int(distance / self._step_distance) * 2)

    def _enable_z_align_steppers(self):
        # Returns the reactor time at which the enable pins take effect.
        stepper_enable = self.printer.lookup_object('stepper_enable', None)
        if stepper_enable is None or not hasattr(stepper_enable, 'set_motors_enable'):
            raise self.printer.command_error(
                'Z_ALIGN requires stepper_enable.set_motors_enable')
        did_enable = False
        for stepper_index in range(len(self.endstop_pin_z)):
            stepper_name = 'stepper_z' if stepper_index == 0 else 'stepper_z%d' % (
                stepper_index,)
            try:
                if stepper_enable.set_motors_enable([stepper_name], True):
                    did_enable = True
            except Exception as err:
                raise self.printer.command_error(
                    'Z_ALIGN failed enabling %s before MCU z_align: %s'
                    % (stepper_name, err))
        now = self.reactor.monotonic()
        if not did_enable:
            return now
        # Enable is scheduled on the toolhead timeline (~BUFFER_TIME_START
        # ahead when idle); the MCU z_align command runs immediately.
        enable_time = self._toolhead.get_last_move_time()
        lead = enable_time - self._main_mcu.estimated_print_time(now)
        return now + max(0., lead)

    def _force_stop_mcu_z_align(self):
        if self._z_align_force_stop_cmd is None:
            return
        try:
            self._z_align_force_stop_cmd.send([self._oidz])
        except Exception:
            _klog(
                'failed to force stop MCU z_align',
                level=logging.exception)

    def _start_mcu_z_align_attempt(self):
        if not self._settle_attempt:
            self._settle_attempt = 1
        self._z_align_status = {}
        params = self._z_align_query_cmd.send([
            self._oidz,
            self._calc_speed_ticks(self.fast_drop_speed),
            self._calc_speed_ticks(self.settle_drop_speed),
            self._calc_distance_steps(self.retract_dist),
            self._calc_distance_steps(self.safe_dist),
            self._calc_distance_steps(self.fast_max_dist),
            self._calc_distance_steps(LIFT_MAX_DIST),
            FILTER_COUNT,
            self._calc_time_ticks(SETTLE_TIME)])
        # A rejected start is answered with its final status
        if params and int(params.get('flag', 0) or 0):
            self._z_align_status = dict(params)
        self._phase_deadline = self.reactor.monotonic() + self.timeout
        self._state = 'mcu_wait'

    def _alignment_result(self, status):
        """Corrected tilt and each side's miss on the two re-approaches."""
        step = self._step_distance or 0.0
        expected = self._calc_distance_steps(self.retract_dist) // 2

        def steps(key):
            return int(status.get(key, 0) or 0)

        tilt = (steps('fast0') - steps('fast1')) * step
        misses = [
            [abs(steps('slow%d_%d' % (p, side)) - expected) * step
             for side in (0, 1)]
            for p in range(MCU_PASSES)]
        return tilt, misses

    def _mcu_z_align_trace(self, status):
        step = self._step_distance or 0.0

        def steps(key):
            return int(status.get(key, 0) or 0)

        phase = MCU_Z_ALIGN_PHASES.get(steps('phase'), 'start')
        return (
            'flag=%d error=%d phase=%s states=0x%x->0x%x lift=%.4fmm'
            ' fast=[%.4f,%.4f]mm released=[%.4f,%.4f]mm'
            ' slow=[%d,%d],[%d,%d] expected %d steps'
            % (steps('flag'), steps('error'), phase, steps('start'),
               steps('end'), steps('lift') * step / 2.0,
               steps('fast0') * step, steps('fast1') * step,
               steps('released0') * step / 2.0,
               steps('released1') * step / 2.0,
               steps('slow0_0'), steps('slow0_1'),
               steps('slow1_0'), steps('slow1_1'),
               self._calc_distance_steps(self.retract_dist) // 2))

    def _poll_mcu_z_align(self, eventtime):
        if self._phase_deadline is not None and eventtime > self._phase_deadline:
            _klog("timeout waiting status=%s", self._z_align_status)
            self._force_stop_mcu_z_align()
            self._fail(
                'timed out waiting for MCU z_align',
                request_homing_abort=True)
            return self.reactor.NEVER
        status = self._z_align_status
        flag = int(status.get('flag', 0) or 0)
        if flag == 0:
            return eventtime + POLL_INTERVAL
        attempts = self.retries + 1
        _klog('MCU z-align attempt %d/%d: %s',
              self._settle_attempt, attempts,
              self._mcu_z_align_trace(status))
        if flag != 1:
            # Mechanical or safety failures; a retry would repeat them
            reason = MCU_Z_ALIGN_ERROR_REASONS.get(
                int(status.get('error', 0) or 0),
                'MCU z-align reported an error')
            self._force_stop_mcu_z_align()
            self._fail('MCU z-align: %s' % (reason,),
                       request_homing_abort=True)
            return self.reactor.NEVER
        tilt, misses = self._alignment_result(status)
        repeat = misses[-1]
        worst = max(repeat)
        self._last_tilt_mm = tilt
        self._last_repeat_mm = list(repeat)
        if worst > self.repeat_tolerance and self._settle_attempt < attempts:
            self._settle_attempt += 1
            self._say(
                'alignment repeatability %.4fmm is over %.4fmm;'
                ' retrying (attempt %d/%d)'
                % (worst, self.repeat_tolerance, self._settle_attempt,
                   attempts))
            self._phase_deadline = None
            self._state = 'mcu_start'
            return self.reactor.NOW
        if worst > self.repeat_max:
            self._fail(
                'MCU z-align repeatability %.4fmm is over repeat_max %.4fmm'
                ' after %d attempts'
                % (worst, self.repeat_max, self._settle_attempt),
                request_homing_abort=True)
            return self.reactor.NEVER
        msg = ('tilt corrected %.2fmm, repeatability Z %.4f / Z1 %.4f mm'
               % (abs(tilt), repeat[0], repeat[1]))
        if worst > self.repeat_tolerance:
            msg += ('; over %.4fmm after %d attempts, so a power-loss'
                    ' resume may be less accurate'
                    % (self.repeat_tolerance, self._settle_attempt))
        self._say(msg)
        self._phase_deadline = None
        self._prepared_zmax = self.zmax
        self._target_z = max(0.0, self._prepared_zmax - self.rise_distance)
        self._state = 'prepared'
        return self.reactor.NEVER

    def _get_z_home_endstops(self):
        kin = self._toolhead.get_kinematics()
        rails = getattr(kin, 'rails', None)
        if rails is None or len(rails) < 3:
            raise self.printer.command_error(
                'Z prep requires Z rail endstops for safety-monitored rise')
        return rails[2].get_endstops()

    def _get_z_limit_snapshot(self):
        kin = self._toolhead.get_kinematics()
        if (not hasattr(kin, 'max_z_velocity')
                or not hasattr(kin, 'max_z_accel')):
            raise self.printer.command_error(
                'Z prep requires kinematics support for max_z_velocity/max_z_accel')
        return {
            'max_z_velocity': kin.max_z_velocity,
            'max_z_accel': kin.max_z_accel,
        }

    def _apply_temp_z_limits(self):
        kin = self._toolhead.get_kinematics()
        snapshot = self._get_z_limit_snapshot()
        kin.max_z_velocity = TEMP_RISE_MAX_Z_VELOCITY
        kin.max_z_accel = self.temp_rise_max_z_accel
        return snapshot

    def _restore_z_limits(self, snapshot):
        if snapshot is None:
            return
        kin = self._toolhead.get_kinematics()
        kin.max_z_velocity = snapshot['max_z_velocity']
        kin.max_z_accel = snapshot['max_z_accel']

    def _fail(self, msg, request_homing_abort=False):
        self._say(msg)
        self._error = msg
        self._state = 'error'
        self.invalidate_homing_state()
        if request_homing_abort:
            self._request_homing_abort(msg)

    def _handle_timer(self, eventtime):
        try:
            while True:
                if self._state == 'mcu_start':
                    self._start_mcu_z_align_attempt()
                    continue
                if self._state == 'mcu_wait':
                    return self._poll_mcu_z_align(eventtime)
                return self.reactor.NEVER
        except Exception:
            _klog('crashed', level=logging.exception)
            self._fail(
                'crashed; check klippy.log',
                request_homing_abort=True)
            return self.reactor.NEVER

    def is_active(self):
        return self._state in ('mcu_start', 'mcu_wait', 'prepared', 'blocking_rise')

    def needs_prep(self):
        return not self._is_toolhead_z_homed()

    def start_prepare(self):
        self._ensure_ready()
        if self._is_toolhead_z_homed():
            return False
        if self._state not in ('idle', 'error'):
            raise self.printer.command_error('Z prep is already running')
        self._learn_delta = None
        self._bottom_anchor = None
        self._reset_runtime()
        start_time = self._enable_z_align_steppers()
        self._state = 'mcu_start'
        self.reactor.update_timer(self._timer, start_time)
        return True

    def wait_prepare_complete(self):
        if self._is_toolhead_z_homed():
            return {
                'skipped': True,
                'z_known': self._is_toolhead_z_homed(),
            }
        self._ensure_ready()
        while True:
            if self._state == 'prepared':
                return {
                    'prepared_zmax': self._prepared_zmax,
                    'target_z': self._target_z,
                    'tilt_mm': self._last_tilt_mm,
                    'repeatability_mm': list(self._last_repeat_mm),
                    'skipped': False,
                    'z_known': self._is_toolhead_z_homed(),
                }
            if self._state == 'error':
                raise self.printer.command_error(
                    self._error or 'Z prep failed')
            if self._state == 'idle':
                raise self.printer.command_error(
                    'Z prep was not started before wait_prepare_complete')
            eventtime = self.reactor.monotonic()
            self.reactor.pause(eventtime + 0.100)

    def _z_steppers(self):
        return list(self._toolhead.get_kinematics().rails[2].get_steppers())

    def validate_reference_frame(self, frame):
        try:
            values = [float(value) for value in frame]
            steppers = self._z_steppers()
            z_min, z_max = self._toolhead.get_kinematics().rails[2].get_range()
            if len(values) != len(steppers) or len(values) != 2:
                raise ValueError('expected two Z positions')
            if not all(math.isfinite(value) for value in values):
                raise ValueError('non-finite Z position')
            if min(values) < z_min or max(values) > z_max:
                raise ValueError('Z position outside configured range')
            if max(values) - min(values) > (
                    MAX_TILT_BIAS + max(s.get_step_dist() for s in steppers)):
                raise ValueError('Z differential exceeds limit')
            return values
        except Exception as err:
            raise self.printer.command_error(
                'Invalid Z reference frame: %s' % err)

    def _record_bottom_anchor(self):
        try:
            self._bottom_anchor = [
                stepper.get_mcu_position() for stepper in self._z_steppers()]
        except Exception:
            self._bottom_anchor = None
            _klog('unable to capture PLR Z anchor', level=logging.exception)

    def capture_reference_frame(self):
        if self._bottom_anchor is None or not self._logical_z_applied:
            return None
        try:
            return self.validate_reference_frame([
                stepper.mcu_to_commanded_position(anchor)
                for stepper, anchor in zip(
                    self._z_steppers(), self._bottom_anchor)
            ])
        except Exception:
            return None

    def _apply_reference_frame(self, frame):
        values = self.validate_reference_frame(frame)
        common = max(values)
        adjustments = [common - value for value in values]
        adjusted = max(adjustments) > 1.0e-9
        if adjusted:
            if self._z_tilt is None:
                raise self.printer.command_error(
                    'Z reference restore requires z_tilt')
            self._z_tilt.z_helper.adjust_steppers(
                adjustments, self.settle_drop_speed)
            self._toolhead.wait_moves()
        restored = self.capture_reference_frame()
        if restored is None or any(
                abs(actual - expected) > stepper.get_step_dist() * 1.5
                for actual, expected, stepper in zip(
                    restored, values, self._z_steppers())):
            raise self.printer.command_error('Z reference restore did not verify')
        return adjusted

    def perform_blocking_rise(
            self, monitored=True, target_z=None, rise_speed=None,
            reference_frame=None):
        if self._is_toolhead_z_homed():
            return False
        self._ensure_ready()
        if self._state != 'prepared':
            self.wait_prepare_complete()
        if self._homing_session_aborted():
            self.invalidate_homing_state()
            raise self.printer.command_error(
                'Z prep refusing blocking rise after an aborted homing session')
        if monitored and self._z_protection_active():
            self.invalidate_homing_state()
            raise self.printer.command_error(
                'Z prep refusing blocking rise while a Z protection fault is active')
        zmax = self._prepared_zmax if self._prepared_zmax is not None else self.zmax
        frame = (
            self.validate_reference_frame(reference_frame)
            if reference_frame is not None else None)
        reference_z = max(frame) if frame is not None else zmax
        default_target = self._target_z if self._target_z is not None else max(
            0.0, zmax - self.rise_distance)
        target_z = default_target if target_z is None else float(target_z)
        move_speed = self.rise_speed if rise_speed is None else float(rise_speed)
        z_min, z_limit = self._toolhead.get_kinematics().rails[2].get_range()
        margin = (
            max(frame) - min(frame) if frame is not None
            else abs(self._tilt_bias) if self._z_tilt is not None else 0.0)
        if (not math.isfinite(target_z) or not math.isfinite(move_speed)
                or move_speed <= 0.0
                or target_z < z_min
                or target_z > min(zmax, z_limit - margin)):
            raise self.printer.command_error('Invalid targeted Z rise')
        z_limit_snapshot = None
        try:
            self._state = 'blocking_rise'
            self._toolhead.get_last_move_time()
            z_limit_snapshot = self._apply_temp_z_limits()
            from .homing import HomingMove
            if monitored:
                endstops = self._get_z_home_endstops()
            else:
                kin = self._toolhead.get_kinematics()
                z_steppers = [s for s in kin.get_steppers() if 'z' in s.get_name()]
                endstops = [(ZAlignNullEndstop(z_steppers, self.reactor),
                             'z_rise')]
            pos = list(self._toolhead.get_position())
            self._logical_z_applied = True
            self._target_z = target_z
            pos[2] = reference_z
            self._toolhead.set_position(pos, homing_axes="z")
            self._record_bottom_anchor()
            target = list(pos)
            target[2] = target_z
            hmove = HomingMove(self.printer, endstops, self._toolhead)
            hmove.homing_move(
                target, move_speed, triggered=monitored, check_triggered=False)
            if monitored and getattr(hmove, 'triggered_endstops', ()):
                raise self.printer.command_error(
                    'Z prep safety stop: Z endstop triggered during fast rise (%s)'
                    % (','.join(hmove.triggered_endstops),))
            if self._homing_session_aborted():
                raise self.printer.command_error(
                    self._get_homing_abort_reason()
                    or 'Z prep blocking rise aborted with the homing session')
            if monitored and self._z_protection_active():
                raise self.printer.command_error(
                    'Z prep refusing to finish the blocking rise while'
                    ' Z protection fault is active')
            if frame is None:
                adjusted = self._apply_tilt_bias()
            else:
                adjusted = self._apply_reference_frame(frame)
            if adjusted and self._homing_session_aborted():
                raise self.printer.command_error(
                    self._get_homing_abort_reason()
                    or 'Z prep blocking rise aborted with the homing session')
            if adjusted and monitored and self._z_protection_active():
                raise self.printer.command_error(
                    'Z prep refusing to finish after Z reference adjustment')
            self._state = 'idle'
            return True
        except Exception:
            self.invalidate_homing_state()
            raise
        finally:
            try:
                self._restore_z_limits(z_limit_snapshot)
            except Exception:
                _klog(
                    'failed to restore Z rise limits',
                    level=logging.exception)

    def perform_unmonitored_rise(self):
        """Rise through the null-endstop reference-frame path."""
        return self.perform_blocking_rise(monitored=False)

    def invalidate_homing_state(self):
        self._logical_z_applied = False
        self._prepared_zmax = None
        self._target_z = None
        self._learn_delta = None
        self._bottom_anchor = None

    def abort_internal(self, reason='Z prep aborted', motor_off=False,
                       restore_motor_mode=False, wait_until_safe=False):
        if self._state in ('idle', 'error'):
            return False
        self._force_stop_mcu_z_align()
        self._fail(reason, request_homing_abort=False)
        if motor_off:
            try:
                self.printer.lookup_object('stepper_enable').motor_off()
            except Exception:
                _klog(
                    'failed to motor_off during abort',
                    level=logging.exception)
        return True

    def get_status(self, _eventtime):
        return {
            'endstop_pin_status': list(self.endstop_pin_status),
            'state': self._state,
            'target_z': self._target_z,
            'prepared_zmax': self._prepared_zmax,
            'z_known': self._is_toolhead_z_homed(),
            'logical_z_applied': self._logical_z_applied,
            'last_tilt_mm': self._last_tilt_mm,
            'last_repeatability_mm': list(self._last_repeat_mm),
            'error': self._error or '',
        }


def load_config(config):
    return ZAlign(config)
