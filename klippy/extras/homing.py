# Helper code for implementing homing operations
#
# Copyright (C) 2016-2021  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging, math

try:
    from .motor_fault_decoder import format_fault_summary as _format_motor_fault_summary
except ImportError:
    from .motor_control import format_fault_summary as _format_motor_fault_summary


class HomingZProbeNotCalibrated(Exception):
    """Z rise completed but probe model not loaded — motors remain enabled."""
    pass
HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = .000015
ENDSTOP_SAMPLE_COUNT = 4
XY_HOMING_MAX_ACCEL = 1000.0
XY_STARTUP_PRIME_DIST = 0.1
XY_STARTUP_PRIME_SPEED = 20.0
XY_STARTUP_PRIME_ENDSTOP_MARGIN = 5.0
XY_POST_HOME_BACKOFF_MM = 5.0
XY_RETRIGGER_MISMATCH_TOLERANCE_MM = 1.0
Z_REHOME_CLEARANCE = 5.0
Z_REHOME_CLEARANCE_SPEED = 20.0

# Return a completion that completes when all completions in a list complete
def multi_complete(printer, completions):
    if len(completions) == 1:
        return completions[0]
    # Build completion that waits for all completions
    reactor = printer.get_reactor()
    cp = reactor.register_callback(lambda e: [c.wait() for c in completions])
    # If any completion indicates an error, then exit main completion early
    for c in completions:
        reactor.register_callback(
            lambda e, c=c: cp.complete(1) if c.wait() else 0)
    return cp

# Virtual endstop that never triggers — for HomingMove without physical endstop backing
class _NullEndstop:
    def __init__(self, steppers, reactor):
        self._steppers = steppers
        self._reactor = reactor
    def get_steppers(self):
        return list(self._steppers)
    def home_start(self, print_time, sample_time, sample_count,
                   rest_time, triggered=True):
        # Returns a completion that never fires so drip_move runs to end naturally
        return self._reactor.completion()
    def home_wait(self, home_end_time):
        return 0.0  # not triggered; clean with check_triggered=False
    def query_endstop(self, print_time):
        return False


# Tracking of stepper positions during a homing/probing move
class StepperPosition:
    def __init__(self, stepper, endstop_name):
        self.stepper = stepper
        self.endstop_name = endstop_name
        self.stepper_name = stepper.get_name()
        self.start_pos = stepper.get_mcu_position()
        self.halt_pos = self.trig_pos = None
    def note_home_end(self, trigger_time):
        self.halt_pos = self.stepper.get_mcu_position()
        self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)

# Implementation of homing/probing moves
class HomingMove:
    def __init__(self, printer, endstops, toolhead=None):
        self.printer = printer
        self.endstops = endstops
        if toolhead is None:
            toolhead = printer.lookup_object('toolhead')
        self.toolhead = toolhead
        self.stepper_positions = []
        self.force_stop_requested = False
        self.force_stop_reason = None
        self.trigger_times = {}
        self.triggered_endstops = ()
    def get_trigger_mm_for_stepper_names(self, stepper_names):
        trigger_positions = []
        stepper_names = set(stepper_names)
        for sp in self.stepper_positions:
            if sp.stepper_name not in stepper_names or sp.trig_pos is None:
                continue
            trigger_positions.append(sp.trig_pos * sp.stepper.get_step_dist())
        if not trigger_positions:
            return None
        return sum(trigger_positions) / len(trigger_positions)
    def get_mcu_endstops(self):
        return [es for es, name in self.endstops]
    def _calc_endstop_rate(self, mcu_endstop, movepos, speed):
        startpos = self.toolhead.get_position()
        axes_d = [mp - sp for mp, sp in zip(movepos, startpos)]
        move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
        move_t = move_d / speed
        max_steps = max([(abs(s.calc_position_from_coord(startpos)
                              - s.calc_position_from_coord(movepos))
                          / s.get_step_dist())
                         for s in mcu_endstop.get_steppers()])
        if max_steps <= 0.:
            return .001
        return move_t / max_steps
    def calc_toolhead_pos(self, kin_spos, offsets):
        kin_spos = dict(kin_spos)
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()
        thpos = self.toolhead.get_position()
        return list(kin.calc_position(kin_spos))[:3] + thpos[3:]
    def _lookup_motor_control(self):
        return self.printer.lookup_object('motor_control')
    def _set_homing_stall_mode(self, mode):
        return self._lookup_motor_control().set_homing_stall_mode(mode)
    def _query_core_protection(self, data=11, timeout=0.05):
        return self._lookup_motor_control().query_core_protection_status(
            data=data, timeout=timeout)
    def handle_force_stop(self, pause=1.0, cleanup=True, complete_drip=False):
        toolhead = self.printer.lookup_object('toolhead')
        if complete_drip and getattr(toolhead, 'special_queuing_state', '') == "Drip":
            drip_completion = getattr(toolhead, 'drip_completion', None)
            if drip_completion is not None:
                try:
                    drip_completion.complete(1)
                except Exception:
                    logging.exception("homing: failed completing drip homing stop")
        toolhead._handle_shutdown()
        if pause and toolhead.can_pause:
            toolhead.reactor.pause(toolhead.reactor.monotonic() + pause)
        if cleanup:
            self._query_core_protection(data=11, timeout=0.05)
            self._set_homing_stall_mode(2)
        toolhead.can_pause = True
    def request_external_force_stop(self, reason=None, immediate=False,
                                    cleanup=True):
        if self.force_stop_requested:
            return
        self.force_stop_requested = True
        self.force_stop_reason = reason or "Motor protection fault aborted homing"
        if immediate:
            self.handle_force_stop(
                pause=0.0, cleanup=cleanup, complete_drip=True)
            return
        self.handle_force_stop(cleanup=cleanup)
    def homing_move(self, movepos, speed, probe_pos=False,
                    triggered=True, check_triggered=True):
        phoming = self.printer.lookup_object('homing', None)
        if phoming is not None and hasattr(phoming, '_set_active_hmove'):
            phoming._set_active_hmove(self)
        # Notify start of homing/probing move
        try:
            self.printer.send_event("homing:homing_move_begin", self)
            # Note start location
            self.toolhead.flush_step_generation()
            kin = self.toolhead.get_kinematics()
            kin_spos = {s.get_name(): s.get_commanded_position()
                        for s in kin.get_steppers()}
            
            self.stepper_positions = [ StepperPosition(s, name)
                                       for es, name in self.endstops
                                       for s in es.get_steppers() ]

            # Start endstop checking
            print_time = self.toolhead.get_last_move_time()
            endstop_triggers = []
            for mcu_endstop, name in self.endstops:
                rest_time = self._calc_endstop_rate(mcu_endstop, movepos, speed)
                wait = mcu_endstop.home_start(print_time, ENDSTOP_SAMPLE_TIME,
                                              ENDSTOP_SAMPLE_COUNT, rest_time,
                                              triggered=triggered)
                endstop_triggers.append(wait)
            all_endstop_trigger = multi_complete(self.printer, endstop_triggers)
            self.toolhead.dwell(HOMING_START_DELAY)
            # Issue move
            error = None
            try:
                self.toolhead.drip_move(movepos, speed, all_endstop_trigger)
            except self.printer.command_error as e:
                if not self.force_stop_requested:
                    error = "Error during homing move: %s" % (str(e),)
                    logging.info("No trigger on %s after full movement, set MOTOR_STALL_MODE DATA=2"%name)
                    self.handle_force_stop(
                        pause=0.0, cleanup=False, complete_drip=True)
            # Wait for endstops to trigger
            trigger_times = {}
            move_end_print_time = self.toolhead.get_last_move_time()
            for mcu_endstop, name in self.endstops:
                trigger_time = mcu_endstop.home_wait(move_end_print_time)
                if trigger_time > 0.:
                    trigger_times[name] = trigger_time
                elif trigger_time < 0. and error is None and not self.force_stop_requested:
                    error = "Communication timeout during homing %s" % (name,)
                    logging.info("Communication timeout during homing %s, set MOTOR_STALL_MODE DATA=2"%name)
                    self.handle_force_stop(
                        pause=0.0, cleanup=False, complete_drip=True)
                elif check_triggered and error is None and not self.force_stop_requested:
                    error = "No trigger on %s after full movement" % (name,)
                    logging.info("No trigger on %s after full movement, set MOTOR_STALL_MODE DATA=2"%name)
                    self.handle_force_stop(
                        pause=0.0, cleanup=False, complete_drip=True)
            self.trigger_times = dict(trigger_times)
            self.triggered_endstops = tuple(sorted(trigger_times))
            # Determine stepper halt positions
            self.toolhead.flush_step_generation()
            for sp in self.stepper_positions:
                tt = trigger_times.get(sp.endstop_name, move_end_print_time)
                sp.note_home_end(tt)
            haltpos = trigpos = movepos
            if error is None and not self.force_stop_requested:
                if probe_pos:
                    halt_steps = {sp.stepper_name: sp.halt_pos - sp.start_pos
                                  for sp in self.stepper_positions}
                    trig_steps = {sp.stepper_name: sp.trig_pos - sp.start_pos
                                  for sp in self.stepper_positions}
                    haltpos = trigpos = self.calc_toolhead_pos(
                        kin_spos, trig_steps)
                    if trig_steps != halt_steps:
                        haltpos = self.calc_toolhead_pos(kin_spos, halt_steps)
                else:
                    over_steps = {sp.stepper_name: sp.halt_pos - sp.trig_pos
                                  for sp in self.stepper_positions}
                    if any(over_steps.values()):
                        self.toolhead.set_position(movepos)
                        halt_kin_spos = {
                            s.get_name(): s.get_commanded_position()
                            for s in kin.get_steppers()}
                        haltpos = self.calc_toolhead_pos(
                            halt_kin_spos, over_steps)
                self.toolhead.set_position(haltpos)
            # Signal homing/probing move complete
            try:
                self.printer.send_event("homing:homing_move_end", self)
            except self.printer.command_error as e:
                if error is None:
                    error = str(e)
            if self.force_stop_requested and error is None:
                reason = self.force_stop_reason or "Motor protection fault aborted homing"
                error = reason
            if error is not None:
                raise self.printer.command_error(error)
            return trigpos
        finally:
            if phoming is not None and hasattr(phoming, '_clear_active_hmove'):
                phoming._clear_active_hmove(self)
    def check_no_movement(self):
        if self.printer.get_start_args().get('debuginput') is not None:
            return None
        for sp in self.stepper_positions:
            if sp.start_pos == sp.trig_pos:
                return sp.endstop_name
        return None

# State tracking of homing requests
class Homing:
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object('toolhead')
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}
        self.out_z_all = 0
        self.homez_info = None

    def set_axes(self, axes):
        self.changed_axes = axes
    def get_axes(self):
        return self.changed_axes
    def get_trigger_position(self, stepper_name):
        return self.trigger_mcu_pos[stepper_name]
    def set_stepper_adjustment(self, stepper_name, adjustment):
        self.adjust_pos[stepper_name] = adjustment
    def _fill_coord(self, coord):
        # Fill in any None entries in 'coord' with current toolhead position
        thcoord = list(self.toolhead.get_position())
        for i in range(len(coord)):
            if coord[i] is not None:
                thcoord[i] = coord[i]
        return thcoord
    def set_homed_position(self, pos):
        self.toolhead.set_position(self._fill_coord(pos))

    def get_step(self, endstops):
        self.stepper_positions = [ StepperPosition(s, name)
                                   for es, name in endstops
                                   for s in es.get_steppers() ]
        for sp in self.stepper_positions:
            if sp.stepper_name == "stepper_z":
                kin = self.toolhead.get_kinematics()
                halt_step_dist = {s.get_name(): s.get_step_dist()
                                 for s in kin.get_steppers()}
                logging.info(
                    "start_pos:%s trig_pos:%s halt_step_dist:%s",
                    sp.start_pos, sp.trig_pos, halt_step_dist[sp.stepper_name])
                return [sp.start_pos, halt_step_dist[sp.stepper_name]]
        return None
    def _report_xy_retrigger_delta(self, rail, first_hmove, second_hmove):
        rail_name = rail.get_name()
        if rail_name not in ("stepper_x", "stepper_y"):
            return
        stepper_names = [s.get_name() for s in rail.get_steppers()]
        first_pos = first_hmove.get_trigger_mm_for_stepper_names(stepper_names)
        second_pos = second_hmove.get_trigger_mm_for_stepper_names(stepper_names)
        if first_pos is None or second_pos is None:
            return
        delta = abs(second_pos - first_pos)
        axis = "X" if rail_name == "stepper_x" else "Y"
        msg = (
            "%s delta between triggers=%.6f(%.6f,%.6f)"
            % (axis, delta, first_pos, second_pos))
        self.printer.lookup_object('gcode').respond_info(msg)
        logging.info(
            msg)
        if delta > XY_RETRIGGER_MISMATCH_TOLERANCE_MM:
            raise self.printer.command_error(
                "Trigger delta larger than %.6fmm"
                % (XY_RETRIGGER_MISMATCH_TOLERANCE_MM,))

    def home_rails(self, rails, forcepos, movepos):
        # Notify of upcoming homing operation
        self.printer.send_event("homing:home_rails_begin", self, rails)
        # Alter kinematics class to think printer is at forcepos
        homing_axes = [axis for axis in range(3) if forcepos[axis] is not None]
        startpos = self._fill_coord(forcepos)
        homepos = self._fill_coord(movepos)
        self.toolhead.set_position(startpos, homing_axes=homing_axes)
        # Perform first home
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = rails[0].get_homing_info()
        homez_info = self.get_step(endstops)
        if homez_info is not None:
            self.homez_info = homez_info
        hmove = HomingMove(self.printer, endstops)
        hmove.homing_move(homepos, hi.speed)
        first_hmove = hmove

        # Perform second home
        if hi.retract_dist:
            # Retract
            startpos = self._fill_coord(forcepos)
            homepos = self._fill_coord(movepos)
            axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
            move_d = math.sqrt(sum([d*d for d in axes_d[:3]]))
            retract_r = min(1., hi.retract_dist / move_d)
            retractpos = [hp - ad * retract_r
                          for hp, ad in zip(homepos, axes_d)]
            self.toolhead.move(retractpos, hi.retract_speed)
            # Home again
            startpos = [rp - ad * retract_r
                        for rp, ad in zip(retractpos, axes_d)]
            self.toolhead.set_position(startpos)
            hmove = HomingMove(self.printer, endstops)
            hmove.homing_move(homepos, hi.second_homing_speed)

            homez_info = self.get_step(endstops)
            if homez_info is not None:
                self.out_z_all = abs(self.homez_info[0] - homez_info[0]) * self.homez_info[1]
            if hmove.check_no_movement() is not None and rails[0].get_name() != "stepper_z":
                logging.info("hmove.check_no_movement %s, set MOTOR_STALL_MODE DATA=2" % hmove.check_no_movement())
                hmove.handle_force_stop(
                    pause=0.0, cleanup=False, complete_drip=True)
                raise self.printer.command_error(
                    "Endstop %s still triggered after retract"
                    % (hmove.check_no_movement(),))
            self._report_xy_retrigger_delta(rails[0], first_hmove, hmove)
        # Signal home operation complete
        self.toolhead.flush_step_generation()
        self.trigger_mcu_pos = {sp.stepper_name: sp.trig_pos
                                for sp in hmove.stepper_positions}
        self.adjust_pos = {}
        self.printer.send_event("homing:home_rails_end", self, rails)
        if any(self.adjust_pos.values()):
            # Apply any homing offsets
            kin = self.toolhead.get_kinematics()
            homepos = self.toolhead.get_position()
            kin_spos = {s.get_name(): (s.get_commanded_position()
                                       + self.adjust_pos.get(s.get_name(), 0.))
                        for s in kin.get_steppers()}
            newpos = kin.calc_position(kin_spos)
            for axis in homing_axes:
                homepos[axis] = newpos[axis]
            self.toolhead.set_position(homepos)

class PrinterHoming:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        # Register g-code commands
        gcode = self.printer.lookup_object('gcode')
        gcode.register_command('G28', self.cmd_G28)
        self.active_hmove = None
        self.motor_fault_abort_reason = None
        self._session_active = False
        self._session_axes = ()
        self._session_z_align = False
        self._session_aborted = False
        self._session_abort_result = None
        self._abort_in_progress = False
        self._xy_startup_prime_pending = True
        self._z_align_rise_prime_pending = True
        self.printer.register_event_handler('klippy:connect',
                                            self._handle_connect)
        self.printer.register_event_handler('gcode:request_restart',
                                            self._handle_request_restart)
    def _reset_startup_motion_primes(self):
        self._xy_startup_prime_pending = True
        self._z_align_rise_prime_pending = True
    def _handle_connect(self):
        self._reset_startup_motion_primes()
    def _handle_request_restart(self, _print_time):
        self._reset_startup_motion_primes()
    def consume_xy_startup_prime(self):
        if not self._xy_startup_prime_pending:
            return False
        self._xy_startup_prime_pending = False
        return True
    def consume_z_align_rise_startup_prime(self):
        if not self._z_align_rise_prime_pending:
            return False
        self._z_align_rise_prime_pending = False
        return True
    def _lookup_motor_control(self):
        return self.printer.lookup_object('motor_control')
    @staticmethod
    def _collect_active_faults(result):
        if not isinstance(result, dict):
            return {}
        return {
            axis: detail for axis, detail in result.items()
            if isinstance(detail, dict) and detail.get('active')
        }
    @staticmethod
    def _collect_active_errors(result):
        if not isinstance(result, dict):
            return {}
        return {
            axis: detail for axis, detail in result.items()
            if isinstance(detail, dict) and detail.get('has_error')
        }
    @staticmethod
    def _format_fault_summary(faults):
        if not faults:
            return "none"
        if _format_motor_fault_summary is not None:
            return _format_motor_fault_summary(
                faults, axis_labeler=lambda axis: str(axis).upper(), empty_text="none")
        parts = []
        for axis, detail in sorted(faults.items()):
            parts.append(
                "%s:err=%s warn=%s status=%s" % (
                    axis,
                    detail.get('error_code', 0),
                    detail.get('warning_code', 0),
                    detail.get('status_code', 0)))
        return ", ".join(parts)
    def _set_homing_stall_mode(self, mode):
        return self._lookup_motor_control().set_homing_stall_mode(mode)
    def _query_core_protection(self, data=11, timeout=0.05):
        return self._lookup_motor_control().query_core_protection_status(
            data=data, timeout=timeout)
    def _clear_core_fault_latches(self, data=5, timeout=0.05):
        return self._lookup_motor_control().clear_core_fault_latches(
            data=data, timeout=timeout)
    def _recover_core_faults_for_homing_start(self, query_data=11,
                                              clear_data=5, timeout=0.05):
        return self._lookup_motor_control().recover_core_faults_for_homing_start(
            query_data=query_data, clear_data=clear_data, timeout=timeout)
    def _set_active_hmove(self, hmove):
        self.active_hmove = hmove
    def _clear_active_hmove(self, hmove):
        if self.active_hmove is hmove:
            self.active_hmove = None
    def _normalize_session_axes(self, axes):
        if axes is None:
            return ()
        if isinstance(axes, (list, tuple)):
            raw = ''.join(str(axis) for axis in axes)
        else:
            raw = str(axes)
        seen = []
        for axis in raw.upper():
            if axis in ('X', 'Y', 'Z') and axis not in seen:
                seen.append(axis)
        return tuple(seen)
    def begin_homing_session(self, axes=None, z_align=False):
        axes = self._normalize_session_axes(axes)
        if self._session_active and self._session_aborted:
            self.end_homing_session()
        if self._session_active:
            merged = list(self._session_axes)
            for axis in axes:
                if axis not in merged:
                    merged.append(axis)
            self._session_axes = tuple(merged)
            self._session_z_align = self._session_z_align or bool(z_align)
            return False
        self._session_active = True
        self._session_axes = axes
        self._session_z_align = bool(z_align)
        self._session_aborted = False
        self._session_abort_result = None
        self._abort_in_progress = False
        self.motor_fault_abort_reason = None
        return True
    def end_homing_session(self):
        self._session_active = False
        self._session_axes = ()
        self._session_z_align = False
        self._session_aborted = False
        self._session_abort_result = None
        self._abort_in_progress = False
        self.motor_fault_abort_reason = None
    def has_active_homing_session(self):
        return self._session_active and not self._session_aborted \
            and not self._abort_in_progress
    def is_homing_session_aborted(self):
        return self._session_aborted
    def is_homing_abort_in_progress(self):
        return self._abort_in_progress
    def _is_probe_model_ready(self):
        scanner = self.printer.lookup_object('cartographer', None)
        if scanner is None:
            return True
        scan_mode = getattr(scanner, 'scan_mode', None)
        if scan_mode is None:
            return True
        if hasattr(scan_mode, 'is_ready'):
            return bool(scan_mode.is_ready)
        if hasattr(scan_mode, 'has_model'):
            return bool(scan_mode.has_model())
        return True
    def _check_scanner_connected(self):
        scanner = self.printer.lookup_object('cartographer', None)
        if scanner is not None:
            if hasattr(scanner, '_check_mcu_disconnected') and scanner._check_mcu_disconnected():
                raise self.printer.command_error(
                    "Scanner MCU is disconnected - cannot complete Z homing. "
                    "Photoelectric leveling is preserved. Reconnect scanner and retry G28 Z.")
    def _invalidate_core_homing_state(self):
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        if hasattr(kin, 'clear_homing_state'):
            kin.clear_homing_state([0, 1, 2])
            return
        if hasattr(kin, 'note_xy_not_homed'):
            kin.note_xy_not_homed()
        if hasattr(kin, 'note_z_not_homed'):
            kin.note_z_not_homed()
    def _mark_motor_control_not_homing(self):
        motor_control = self.printer.lookup_object('motor_control', None)
        if motor_control is not None and hasattr(motor_control, 'is_homing'):
            motor_control.is_homing = False
    def _should_abort_z_align(self):
        z_align = self.printer.lookup_object('z_align', None)
        if z_align is not None and hasattr(z_align, 'is_active'):
            try:
                if z_align.is_active():
                    return True
            except Exception:
                logging.exception('homing: failed checking z_align activity')
        return bool(self._session_z_align)
    def request_homing_abort(self, reason, detail=None, abort_z_align=True):
        if self._session_abort_result is not None:
            return dict(self._session_abort_result)
        if self._abort_in_progress:
            return {
                'reason': self.motor_fault_abort_reason or reason,
                'axes': tuple(self._session_axes),
                'active_hmove': self.active_hmove is not None,
                'z_align_aborted': False,
                'z_align_waited': 0.0,
                'persistent_faults': {},
            }
        self._abort_in_progress = True
        self._session_aborted = True
        self.motor_fault_abort_reason = reason
        z_align = self.printer.lookup_object('z_align', None)
        z_align_aborted = False
        protection_after_clear = {}
        persistent_faults = {}
        try:
            logging.warning('homing: request_homing_abort reason=%s detail=%s',
                            reason, detail)
            if abort_z_align and z_align is not None \
                    and hasattr(z_align, 'abort_internal'):
                z_align_aborted = bool(z_align.abort_internal(
                    reason=reason, motor_off=False, restore_motor_mode=False))
            if self.active_hmove is not None:
                self.active_hmove.request_external_force_stop(
                    reason=reason, immediate=True, cleanup=False)
            if z_align is not None \
                    and hasattr(z_align, 'invalidate_homing_state'):
                z_align.invalidate_homing_state()
            self._invalidate_core_homing_state()
            self._mark_motor_control_not_homing()
            self.printer.lookup_object('stepper_enable').motor_off()
            try:
                self._clear_core_fault_latches(data=5, timeout=0.05)
                protection_after_clear = self._query_core_protection(
                    data=11, timeout=0.05)
                persistent_faults = self._collect_active_faults(
                    protection_after_clear)
            except Exception:
                logging.exception('homing: failed clearing/rechecking core faults after abort')
            try:
                self._set_homing_stall_mode(2)
            except Exception:
                logging.exception('homing: failed restoring MOTOR_STALL_MODE DATA=2 after abort')
            try:
                # Re-assert motor_off after the coordinated cleanup so any
                # queued enable activity from the interrupted homing path does
                # not leave motors powered.
                self.printer.lookup_object('stepper_enable').motor_off()
            except Exception:
                logging.exception('homing: failed final motor_off after abort cleanup')
            if persistent_faults:
                reason = (
                    'Persistent motor protection fault after abort cleanup (%s)'
                    % (self._format_fault_summary(persistent_faults),))
                self.motor_fault_abort_reason = reason
            result = {
                'reason': self.motor_fault_abort_reason or reason,
                'axes': tuple(self._session_axes),
                'active_hmove': self.active_hmove is not None,
                'z_align_aborted': z_align_aborted,
                'z_align_waited': 0.0,
                'protection_after_clear': protection_after_clear,
                'persistent_faults': persistent_faults,
            }
            self._session_abort_result = dict(result)
            return result
        finally:
            self._abort_in_progress = False
    def request_motor_fault_abort(self, detail=None):
        axes = ()
        source = None
        fault_summary = None
        if isinstance(detail, dict):
            axes = tuple(sorted(detail.get('axes', {}).keys()))
            source = detail.get('source')
            active_faults = self._collect_active_faults(detail.get('axes', {}))
            if active_faults:
                fault_summary = self._format_fault_summary(active_faults)
        if fault_summary:
            reason = 'Motor protection fault aborted homing (%s)' % (
                fault_summary,)
        else:
            axis_text = ','.join(axes) if axes else (source or 'unknown')
            reason = 'Motor protection fault aborted homing (%s)' % (axis_text,)
        return self.request_homing_abort(
            reason=reason, detail=detail, abort_z_align=self._should_abort_z_align())
    def manual_home(self, toolhead, endstops, pos, speed,
                    triggered, check_triggered):
        hmove = HomingMove(self.printer, endstops, toolhead)
        try:
            hmove.homing_move(pos, speed, triggered=triggered,
                              check_triggered=check_triggered)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            raise
    def probing_move(self, mcu_probe, pos, speed):

        endstops = [(mcu_probe, "probe")]
        hmove = HomingMove(self.printer, endstops)
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Probing failed due to printer shutdown")
            raise
        return epos

    def _start_managed_homing_session(self, requested_axes, z_align=False):
        blocker = self._lookup_motor_control().get_homing_fault_blocker()
        if blocker:
            raise self.printer.command_error(blocker)
        session_created = self.begin_homing_session(
            axes=requested_axes, z_align=z_align)
        if not session_created:
            return False
        recovery = self._recover_core_faults_for_homing_start(
            query_data=11, clear_data=5, timeout=0.05)
        persistent = recovery.get('persistent_errors', {})
        if persistent:
            reason = (
                'Persistent motor protection fault before homing (%s)'
                % (self._format_fault_summary(persistent),))
            self.request_homing_abort(
                reason=reason,
                detail={'source': 'homing_session_start',
                        'recovery': recovery},
                abort_z_align=False)
            raise self.printer.command_error(
                self.motor_fault_abort_reason or reason)
        self._set_homing_stall_mode(1)
        return True

    def _finish_managed_homing_session(self):
        if not self._session_active:
            return
        if self._session_aborted or self._abort_in_progress:
            return
        protection = {}
        persistent = {}
        try:
            protection = self._query_core_protection(data=11, timeout=0.05)
            persistent = self._collect_active_errors(protection)
        finally:
            self._set_homing_stall_mode(2)
        if persistent:
            reason = (
                'Motor protection fault still active at homing end (%s)'
                % (self._format_fault_summary(persistent),))
            self.request_homing_abort(
                reason=reason,
                detail={'source': 'homing_session_finish',
                        'protection': protection},
                abort_z_align=False)
            raise self.printer.command_error(
                self.motor_fault_abort_reason or reason)
        self.end_homing_session()

    def _get_requested_axes(self, gcmd):
        axes = []
        for pos, axis in enumerate('XYZ'):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            return [0, 1, 2]
        return axes

    def _get_homed_axis_letters(self):
        toolhead = self.printer.lookup_object('toolhead')
        eventtime = self.printer.get_reactor().monotonic()
        homed_axes = toolhead.get_status(eventtime).get('homed_axes', '')
        return set(axis.lower() for axis in homed_axes)

    def _resolve_home_order(self, requested_axes, homed_axes):
        requested = set(requested_axes)
        order = []
        if 2 in requested:
            for axis_idx, axis_name in ((1, 'y'), (0, 'x')):
                if axis_idx in requested or axis_name not in homed_axes:
                    order.append(axis_idx)
            order.append(2)
            return order
        if requested == {0, 1}:
            return [1, 0]
        return list(requested_axes)

    def _get_axis_limits(self, axis_idx):
        axis_name = 'xyz'[axis_idx]
        section = self.config.getsection('stepper_%s' % (axis_name,))
        pos_min = section.getfloat('position_min', default=0.0)
        pos_max = section.getfloat('position_max')
        return pos_min, pos_max

    def _backoff_after_home(self, axis_idx):
        offsets = {0: -XY_POST_HOME_BACKOFF_MM, 1: XY_POST_HOME_BACKOFF_MM}
        if axis_idx not in offsets:
            return
        toolhead = self.printer.lookup_object('toolhead')
        pos = list(toolhead.get_position())
        pos_min, pos_max = self._get_axis_limits(axis_idx)
        new_pos = pos[axis_idx] + offsets[axis_idx]
        pos[axis_idx] = max(pos_min, min(pos_max, new_pos))
        toolhead.move(pos, 60.0)
        toolhead.wait_moves()

    def _set_toolhead_accel(self, accel):
        toolhead = self.printer.lookup_object('toolhead')
        if hasattr(toolhead, 'set_accel'):
            toolhead.set_accel(accel)
            return
        if hasattr(toolhead, 'set_max_accel'):
            toolhead.set_max_accel(accel)
            if hasattr(toolhead, '_calc_junction_deviation'):
                toolhead._calc_junction_deviation()

    def _prime_xy_home_once_after_restart(self, kin, axis_idx):
        if axis_idx not in (0, 1):
            return
        if not hasattr(kin, 'rails') or len(kin.rails) <= axis_idx:
            return
        rail = kin.rails[axis_idx]
        hi = rail.get_homing_info()
        pos_min, pos_max = rail.get_range()
        direction = 1.0 if getattr(hi, 'positive_dir', False) else -1.0
        margin = min(XY_STARTUP_PRIME_ENDSTOP_MARGIN, XY_STARTUP_PRIME_DIST)
        target_axis = hi.position_endstop - direction * margin
        target_axis = max(pos_min, min(pos_max, target_axis))
        start_axis = target_axis - direction * XY_STARTUP_PRIME_DIST
        start_axis = max(pos_min, min(pos_max, start_axis))
        prime_dist = abs(target_axis - start_axis)
        if prime_dist <= 0.0:
            return
        if not self.consume_xy_startup_prime():
            return
        toolhead = self.printer.lookup_object('toolhead')
        startpos = list(toolhead.get_position())
        startpos[axis_idx] = start_axis
        toolhead.set_position(startpos, homing_axes=(axis_idx,))
        primepos = list(startpos)
        primepos[axis_idx] = target_axis
        prime_speed = min(float(hi.speed), XY_STARTUP_PRIME_SPEED)
        logging.info(
            'homing: priming first %s home after restart dist=%.3f speed=%.3f',
            rail.get_name(), prime_dist, prime_speed)
        prime_hmove = HomingMove(self.printer, rail.get_endstops())
        prime_hmove.homing_move(
            primepos, prime_speed, triggered=True, check_triggered=False)

    def _compute_z_home_center(self):
        if self.config.has_section('bed_mesh'):
            bed_mesh = self.config.getsection('bed_mesh')
            mesh_min = bed_mesh.getfloatlist('mesh_min', count=2)
            mesh_max = bed_mesh.getfloatlist('mesh_max', count=2)
            return (
                mesh_min[0] + (mesh_max[0] - mesh_min[0]) / 2.0,
                mesh_min[1] + (mesh_max[1] - mesh_min[1]) / 2.0,
            )
        min_x, max_x = self._get_axis_limits(0)
        min_y, max_y = self._get_axis_limits(1)
        return (
            min_x + (max_x - min_x) / 2.0,
            min_y + (max_y - min_y) / 2.0,
        )

    def _move_to_z_home_center(self, speed=200.0):
        toolhead = self.printer.lookup_object('toolhead')
        pos = list(toolhead.get_position())
        center_x, center_y = self._compute_z_home_center()
        pos[0] = center_x
        pos[1] = center_y
        logging.info('homing: moving to Z-home center x=%.3f y=%.3f',
                     center_x, center_y)
        toolhead.move(pos, speed)
        toolhead.wait_moves()

    def _get_z_home_endstops(self, kin):
        rails = getattr(kin, 'rails', None)
        if rails is None or len(rails) < 3:
            raise self.printer.command_error(
                'Integrated G28 Z requires Z rail endstops for monitored pre-home motion')
        return rails[2].get_endstops()

    def _get_triggered_endstop_names(self, endstops, print_time):
        triggered = []
        for mcu_endstop, name in endstops:
            if not hasattr(mcu_endstop, 'query_endstop'):
                continue
            try:
                if mcu_endstop.query_endstop(print_time):
                    triggered.append(name)
            except Exception:
                logging.exception(
                    'homing: failed querying endstop state before monitored Z move')
        return tuple(sorted(set(triggered)))

    def _emit_raw_warning(self, msg):
        logging.warning('homing: %s', msg)
        gcode = self.printer.lookup_object('gcode')
        try:
            gcode.respond_raw(msg)
        except Exception:
            logging.exception('homing: failed emitting raw warning')

    def _move_known_z_to_clearance_before_home(self, kin):
        toolhead = self.printer.lookup_object('toolhead')
        pos = list(toolhead.get_position())
        current_z = pos[2]
        if current_z <= Z_REHOME_CLEARANCE:
            return False
        endstops = self._get_z_home_endstops(kin)
        start_triggered = self._get_triggered_endstop_names(
            endstops, toolhead.get_last_move_time())
        target = list(pos)
        target[2] = Z_REHOME_CLEARANCE
        logging.info(
            'homing: moving known Z %.3f -> %.3f before G28 Z at %.3fmm/s',
            current_z, Z_REHOME_CLEARANCE, Z_REHOME_CLEARANCE_SPEED)
        hmove = HomingMove(
            self.printer, endstops, toolhead)
        hmove.homing_move(
            target, Z_REHOME_CLEARANCE_SPEED,
            triggered=True, check_triggered=False)
        if getattr(hmove, 'triggered_endstops', ()):
            triggered_set = set(hmove.triggered_endstops)
            start_set = set(start_triggered)
            if start_set and triggered_set.issubset(start_set):
                msg = (
                    '!! G28 Z warning: Z endstop already triggered before pre-home '
                    'move at Z=%.3f (%s); continuing with normal G28 Z'
                    % (current_z, ','.join(sorted(triggered_set))))
            else:
                msg = (
                    '!! G28 Z warning: Z endstop triggered during pre-home move '
                    'from Z=%.3f to Z=%.3f (%s); continuing with normal G28 Z'
                    % (current_z, Z_REHOME_CLEARANCE,
                       ','.join(sorted(triggered_set))))
            self._emit_raw_warning(msg)
            return False
        return True

    def _home_single_axis(self, homing_state, kin, axis_idx):
        restore_accel = None
        if axis_idx in (0, 1):
            toolhead = self.printer.lookup_object('toolhead')
            current_accel = getattr(toolhead, 'max_accel', None)
            if current_accel is None and hasattr(toolhead, 'get_max_accel'):
                current_accel = toolhead.get_max_accel()
            if current_accel is not None and current_accel > XY_HOMING_MAX_ACCEL:
                restore_accel = current_accel
                logging.info(
                    'homing: lowering XY homing accel %.1f -> %.1f',
                    current_accel, XY_HOMING_MAX_ACCEL)
                self._set_toolhead_accel(XY_HOMING_MAX_ACCEL)
        try:
            if axis_idx in (0, 1):
                self._prime_xy_home_once_after_restart(kin, axis_idx)
            homing_state.set_axes([axis_idx])
            kin.home(homing_state)
            if axis_idx in (0, 1):
                self._backoff_after_home(axis_idx)
        finally:
            if restore_accel is not None:
                logging.info(
                    'homing: restoring accel %.1f after XY home',
                    restore_accel)
                self._set_toolhead_accel(restore_accel)

    def _integrated_home_z(self, homing_state, kin, z_was_homed=False):
        toolhead = self.printer.lookup_object('toolhead')
        z_align = self.printer.lookup_object('z_align', None)
        use_z_align = bool(
            z_align is not None
            and hasattr(z_align, 'needs_prep')
            and z_align.needs_prep())
        if use_z_align:
            z_align.wait_prepare_complete()
        self._move_to_z_home_center(speed=200.0)
        if use_z_align:
            if not self._is_probe_model_ready():
                z_align.perform_unmonitored_rise()
                raise HomingZProbeNotCalibrated("Scan model not loaded")
            z_align.perform_blocking_rise()
            if self._session_aborted:
                raise self.printer.command_error(
                    self.motor_fault_abort_reason or 'Homing session aborted')
        elif z_was_homed:
            self._move_known_z_to_clearance_before_home(kin)
        if not self._is_probe_model_ready():
            raise HomingZProbeNotCalibrated("Scan model not loaded")
        self._check_scanner_connected()
        homing_state.set_axes([2])
        kin.home(homing_state)
        pos = list(toolhead.get_position())
        pos[2] = 5.0
        toolhead.move(pos, 10.0)
        toolhead.wait_moves()
        gcode_move = self.printer.lookup_object('gcode_move', None)
        if gcode_move is not None and hasattr(gcode_move, 'reset_last_position'):
            gcode_move.reset_last_position()

    def cmd_G28(self, gcmd):
        axes = self._get_requested_axes(gcmd)
        homing_state = Homing(self.printer)
        toolhead = self.printer.lookup_object('toolhead')
        kin = toolhead.get_kinematics()
        if self._session_aborted:
            raise gcmd.error(
                self.motor_fault_abort_reason or 'Homing session aborted')
        homed_axes = self._get_homed_axis_letters()
        order = self._resolve_home_order(axes, homed_axes)
        z_align = self.printer.lookup_object('z_align', None)
        use_z_align = bool(
            2 in order
            and z_align is not None
            and hasattr(z_align, 'needs_prep')
            and z_align.needs_prep())
        session_owned = False
        if not self._session_active:
            requested_axes = ''.join('XYZ'[axis] for axis in order)
            self._start_managed_homing_session(
                requested_axes, z_align=use_z_align)
            session_owned = True
        try:
            if use_z_align and z_align is not None \
                    and hasattr(z_align, 'start_prepare'):
                z_align.start_prepare()
            for axis_idx in order:
                if axis_idx in (0, 1):
                    self._home_single_axis(homing_state, kin, axis_idx)
                else:
                    self._integrated_home_z(
                        homing_state, kin, z_was_homed=('z' in homed_axes))
            if session_owned:
                self._finish_managed_homing_session()
        except HomingZProbeNotCalibrated as err:
            # Rise completed cleanly — just restore state without killing motors
            logging.warning('homing: probe not calibrated after rise: %s', err)
            try:
                self._set_homing_stall_mode(2)
            except Exception:
                logging.exception(
                    'homing: failed restoring stall mode after probe-not-calibrated')
            try:
                self._mark_motor_control_not_homing()
            except Exception:
                logging.exception(
                    'homing: failed marking not-homing after probe-not-calibrated')
            self.end_homing_session()
            raise self.printer.command_error(str(err))
        except self.printer.command_error as err:
            logging.exception(err)
            abort_reason = self.motor_fault_abort_reason
            if not self._session_aborted:
                try:
                    self.request_homing_abort(
                        reason=self.motor_fault_abort_reason or str(err),
                        detail={'source': 'cmd_G28', 'error': str(err)},
                        abort_z_align=self._should_abort_z_align())
                except Exception:
                    logging.exception(
                        'homing: unified abort failed during cmd_G28 command_error')
            abort_reason = abort_reason or self.motor_fault_abort_reason
            self.end_homing_session()
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            if abort_reason is not None and str(err) != abort_reason:
                raise self.printer.command_error(abort_reason)
            raise
        except Exception as err:
            logging.exception(err)
            abort_reason = self.motor_fault_abort_reason
            if not self._session_aborted:
                try:
                    self.request_homing_abort(
                        reason=self.motor_fault_abort_reason or str(err),
                        detail={'source': 'cmd_G28', 'error': str(err)},
                        abort_z_align=self._should_abort_z_align())
                except Exception:
                    logging.exception(
                        'homing: unified abort failed during cmd_G28 exception')
            abort_reason = abort_reason or self.motor_fault_abort_reason
            self.end_homing_session()
            if abort_reason is not None and str(err) != abort_reason:
                raise self.printer.command_error(abort_reason)
            raise

def load_config(config):
    return PrinterHoming(config)
