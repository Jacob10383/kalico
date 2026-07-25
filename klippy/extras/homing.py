# Helper code for implementing homing operations
#
# Copyright (C) 2016-2024  Kevin O'Connor <kevin@koconnor.net>
#
# This file may be distributed under the terms of the GNU GPLv3 license.
import logging
import math

from .danger_options import get_danger_options
from .motor_control import (
    MOTOR_COMMAND_TIMEOUT,
    MOTOR_NO_ACK_TIMEOUT,
    format_fault_summary as _format_motor_fault_summary,
)


def _klog(msg, *args, level=logging.info):
    level("homing: " + msg, *args)


class HomingZProbeNotCalibrated(Exception):
    pass

HOMING_START_DELAY = 0.001
ENDSTOP_SAMPLE_TIME = 0.000015
ENDSTOP_SAMPLE_COUNT = 4
XY_STARTUP_PRIME_DIST = 0.1
XY_STARTUP_PRIME_SPEED = 20.0
XY_STARTUP_PRIME_ENDSTOP_MARGIN = 5.0
XY_RETRIGGER_MISMATCH_TOLERANCE_MM = 1.0
Z_REHOME_CLEARANCE = 5.0
Z_REHOME_CLEARANCE_SPEED = 20.0
Z_POST_HOME_LIFT = 3.0
Z_POST_HOME_LIFT_SPEED = 10.0


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
            lambda e, c=c: cp.complete(1) if c.wait() else 0
        )
    return cp


# Tracking of stepper positions during a homing/probing move
class StepperPosition:
    def __init__(self, stepper, endstop_name):
        self.stepper = stepper
        self.endstop_name = endstop_name
        self.stepper_name = stepper.get_name()
        self.start_pos = stepper.get_mcu_position()
        self.start_cmd_pos = stepper.mcu_to_commanded_position(self.start_pos)
        self.halt_pos = self.trig_pos = None

    def note_home_end(self, trigger_time):
        self.halt_pos = self.stepper.get_mcu_position()
        self.trig_pos = self.stepper.get_past_mcu_position(trigger_time)

    def verify_no_probe_skew(self, haltpos):
        new_start_pos = self.stepper.get_mcu_position(self.start_cmd_pos)
        if new_start_pos != self.start_pos:
            _klog(
                "Stepper '%s' position skew after probe: pos %d now %d",
                self.stepper.get_name(),
                self.start_pos,
                new_start_pos,
                level=logging.warning,
            )


# Implementation of homing/probing moves
class HomingMove:
    def __init__(self, printer, endstops, toolhead=None):
        self.printer = printer
        self.endstops = endstops
        if toolhead is None:
            toolhead = printer.lookup_object("toolhead")
        self.toolhead = toolhead
        self.stepper_positions = []
        self.distance_elapsed = []
        self.force_stop_requested = False
        self.force_stop_reason = None
        self.trigger_times = {}
        self.triggered_endstops = ()
        self.drip_completion = None

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
        move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
        move_t = move_d / speed
        max_steps = max(
            [
                (
                    abs(
                        s.calc_position_from_coord(startpos)
                        - s.calc_position_from_coord(movepos)
                    )
                    / s.get_step_dist()
                )
                for s in mcu_endstop.get_steppers()
            ]
        )
        if max_steps <= 0.0:
            return 0.001
        return move_t / max_steps

    def calc_toolhead_pos(self, kin_spos, offsets):
        kin_spos = dict(kin_spos)
        kin = self.toolhead.get_kinematics()
        for stepper in kin.get_steppers():
            sname = stepper.get_name()
            kin_spos[sname] += offsets.get(sname, 0) * stepper.get_step_dist()
        thpos = self.toolhead.get_position()
        return list(kin.calc_position(kin_spos))[:3] + thpos[3:]

    def _complete_drip_move(self):
        drip_completion = self.drip_completion
        if drip_completion is None:
            return
        try:
            drip_completion.complete(1)
        except Exception:
            _klog(
                "failed completing drip homing stop",
                level=logging.exception)

    def request_external_force_stop(self, reason=None, immediate=False):
        if self.force_stop_requested:
            return
        self.force_stop_requested = True
        self.force_stop_reason = (
            reason or "Motor protection fault aborted homing")
        self._complete_drip_move()

    def homing_move(
        self,
        movepos,
        speed,
        probe_pos=False,
        triggered=True,
        check_triggered=True,
    ):
        phoming = self.printer.lookup_object("homing")
        phoming._set_active_hmove(self)
        try:
            # Notify start of homing/probing move
            self.printer.send_event("homing:homing_move_begin", self)
            # Note start location
            self.toolhead.flush_step_generation()
            kin = self.toolhead.get_kinematics()
            kin_spos = {
                s.get_name(): s.get_commanded_position()
                for s in kin.get_steppers()
            }
            self.stepper_positions = [
                StepperPosition(s, name)
                for es, name in self.endstops
                for s in es.get_steppers()
            ]
            # Start endstop checking
            print_time = self.toolhead.get_last_move_time()
            endstop_triggers = []
            for mcu_endstop, name in self.endstops:
                rest_time = self._calc_endstop_rate(mcu_endstop, movepos, speed)
                wait = mcu_endstop.home_start(
                    print_time,
                    ENDSTOP_SAMPLE_TIME,
                    ENDSTOP_SAMPLE_COUNT,
                    rest_time,
                    triggered=triggered,
                )
                endstop_triggers.append(wait)
            all_endstop_trigger = multi_complete(self.printer, endstop_triggers)
            self.drip_completion = all_endstop_trigger

            self.toolhead.dwell(HOMING_START_DELAY)
            # Issue move
            error = None
            try:
                self.toolhead.drip_move(movepos, speed, all_endstop_trigger)
            except self.printer.command_error as e:
                if not self.force_stop_requested:
                    error = "Error during homing move: %s" % (str(e),)
            # Wait for endstops to trigger
            trigger_times = {}
            move_end_print_time = self.toolhead.get_last_move_time()
            for mcu_endstop, name in self.endstops:
                try:
                    trigger_time = mcu_endstop.home_wait(move_end_print_time)
                except self.printer.command_error as e:
                    if error is None and not self.force_stop_requested:
                        error = "Error during homing %s: %s" % (name, str(e))
                    continue
                if trigger_time > 0.0:
                    trigger_times[name] = trigger_time
                elif (
                    trigger_time < 0.0 and error is None
                    and not self.force_stop_requested
                ):
                    error = "Communication timeout during homing %s" % (name,)
                elif (
                    check_triggered and error is None
                    and not self.force_stop_requested
                ):
                    error = "No trigger on %s after full movement" % (name,)
            self.trigger_times = dict(trigger_times)
            self.triggered_endstops = tuple(sorted(trigger_times))
            # Determine stepper halt positions
            self.toolhead.flush_step_generation()
            for sp in self.stepper_positions:
                tt = trigger_times.get(sp.endstop_name, move_end_print_time)
                sp.note_home_end(tt)
            if error is None and not self.force_stop_requested:
                if probe_pos:
                    halt_steps = {
                        sp.stepper_name: sp.halt_pos - sp.start_pos
                        for sp in self.stepper_positions
                    }
                    trig_steps = {
                        sp.stepper_name: sp.trig_pos - sp.start_pos
                        for sp in self.stepper_positions
                    }
                    haltpos = trigpos = self.calc_toolhead_pos(
                        kin_spos, trig_steps)
                    if trig_steps != halt_steps:
                        haltpos = self.calc_toolhead_pos(kin_spos, halt_steps)
                    self.toolhead.set_position(haltpos)
                    for sp in self.stepper_positions:
                        sp.verify_no_probe_skew(haltpos)
                else:
                    haltpos = trigpos = movepos
                    over_steps = {
                        sp.stepper_name: sp.halt_pos - sp.trig_pos
                        for sp in self.stepper_positions
                    }
                    steps_moved = {
                        sp.stepper_name: (sp.halt_pos - sp.start_pos)
                        * sp.stepper.get_step_dist()
                        for sp in self.stepper_positions
                    }
                    filled_steps_moved = {
                        sname: steps_moved.get(sname, 0)
                        for sname in [s.get_name() for s in kin.get_steppers()]
                    }
                    self.distance_elapsed = kin.calc_position(filled_steps_moved)
                    if any(over_steps.values()):
                        self.toolhead.set_position(movepos)
                        halt_kin_spos = {
                            s.get_name(): s.get_commanded_position()
                            for s in kin.get_steppers()
                        }
                        haltpos = self.calc_toolhead_pos(
                            halt_kin_spos, over_steps)
                    self.toolhead.set_position(haltpos)
            else:
                trigpos = movepos
            # Signal homing/probing move complete
            try:
                self.printer.send_event("homing:homing_move_end", self)
            except self.printer.command_error as e:
                if error is None:
                    error = str(e)
            if self.force_stop_requested and error is None:
                error = (
                    self.force_stop_reason
                    or "Motor protection fault aborted homing")
            if error is not None:
                raise self.printer.command_error(error)
            return trigpos
        finally:
            self.drip_completion = None
            phoming._clear_active_hmove(self)

    def check_no_movement(self):
        if self.printer.get_start_args().get("debuginput") is not None:
            return None
        for sp in self.stepper_positions:
            if sp.start_pos == sp.trig_pos:
                return sp.endstop_name
        return None

    def moved_less_than_dist(self, min_dist, homing_axes):
        homing_axis_distances = [
            dist
            for i, dist in enumerate(self.distance_elapsed)
            if i in homing_axes
        ]
        distance_tolerance = (
            get_danger_options().homing_elapsed_distance_tolerance
        )
        if any(
            [
                abs(dist) < min_dist
                and min_dist - abs(dist) >= distance_tolerance
                for dist in homing_axis_distances
            ]
        ):
            return True
        return False


# State tracking of homing requests
class Homing:
    def __init__(self, printer):
        self.printer = printer
        self.toolhead = printer.lookup_object("toolhead")
        self.changed_axes = []
        self.trigger_mcu_pos = {}
        self.adjust_pos = {}

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

    def _set_homing_accel(self, accel, pre_homing):
        if accel is None:
            return
        if pre_homing:
            self.toolhead.set_accel(accel)
        else:
            self.toolhead.reset_accel()

    def _set_homing_current(self, homing_axes, pre_homing):
        print_time = self.toolhead.get_last_move_time()
        affected_rails = set()
        for axis in homing_axes:
            partial_rails = self.toolhead.get_active_rails_for_axis(axis)
            affected_rails = affected_rails | set(partial_rails)

        dwell_time = 0.0
        for rail in affected_rails:
            chs = rail.get_tmc_current_helpers()
            for ch in chs:
                if ch is not None:
                    current_dwell_time = ch.set_current_for_homing(
                        print_time, pre_homing
                    )
                    dwell_time = max(dwell_time, current_dwell_time)

        if dwell_time:
            self.toolhead.dwell(dwell_time)

    def _reset_endstop_states(self, endstops):
        # re-querying a tmc endstop seems to reset the state
        # otherwise it triggers almost immediately upon second home
        # this seems to be an adequate substitute for a 2 second dwell.
        print_time = self.toolhead.get_last_move_time()
        for endstop in endstops:
            endstop[0].query_endstop(print_time)

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
        msg = "%s delta between triggers=%.6f(%.6f,%.6f)" % (
            axis, delta, first_pos, second_pos)
        self.printer.lookup_object("gcode").respond_info(msg)
        _klog("%s", msg)
        if delta > XY_RETRIGGER_MISMATCH_TOLERANCE_MM:
            raise self.printer.command_error(
                "Trigger delta larger than %.6fmm"
                % (XY_RETRIGGER_MISMATCH_TOLERANCE_MM,))

    def home_rails(self, rails, forcepos, movepos):
        # Notify of upcoming homing operation
        self.printer.send_event("homing:home_rails_begin", self, rails)
        # Alter kinematics class to think printer is at forcepos
        force_axes = [axis for axis in range(3) if forcepos[axis] is not None]
        homing_axes = "".join(["xyz"[i] for i in force_axes])
        startpos = self._fill_coord(forcepos)
        homepos = self._fill_coord(movepos)
        self.toolhead.set_position(startpos, homing_axes=homing_axes)
        # Perform first home
        endstops = [es for rail in rails for es in rail.get_endstops()]
        hi = rails[0].get_homing_info()
        hmove = HomingMove(self.printer, endstops)

        try:
            self._set_homing_accel(hi.accel, pre_homing=True)
            self._set_homing_current(homing_axes, pre_homing=True)
            self._reset_endstop_states(endstops)
            hmove.homing_move(homepos, hi.speed)
        finally:
            self._set_homing_accel(hi.accel, pre_homing=False)

        first_hmove = hmove
        needs_rehome = False
        retract_dist = hi.retract_dist
        if hmove.moved_less_than_dist(hi.min_home_dist, force_axes):
            needs_rehome = True
            retract_dist = hi.min_home_dist

        # Perform second home
        if retract_dist:
            _klog("needs rehome: %s", needs_rehome)
            # Retract
            startpos = self._fill_coord(forcepos)
            homepos = self._fill_coord(movepos)
            axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
            move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
            retract_r = min(1.0, retract_dist / move_d)
            retractpos = [
                hp - ad * retract_r for hp, ad in zip(homepos, axes_d)
            ]
            self.toolhead.move(retractpos, hi.retract_speed)
            if not hi.use_sensorless_homing or needs_rehome:
                try:
                    # Home again
                    startpos = [
                        rp - ad * retract_r
                        for rp, ad in zip(retractpos, axes_d)
                    ]
                    self.toolhead.set_position(startpos)
                    self._reset_endstop_states(endstops)
                    self._set_homing_accel(hi.accel, pre_homing=True)

                    hmove = HomingMove(self.printer, endstops)
                    hmove.homing_move(homepos, hi.second_homing_speed)

                    if hmove.check_no_movement() is not None:
                        raise self.printer.command_error(
                            "Endstop %s still triggered after retract"
                            % (hmove.check_no_movement(),)
                        )
                    if (
                        hi.use_sensorless_homing
                        and needs_rehome
                        and hmove.moved_less_than_dist(
                            hi.min_home_dist, force_axes
                        )
                    ):
                        raise self.printer.command_error(
                            "Early homing trigger on second home!"
                        )
                    self._report_xy_retrigger_delta(
                        rails[0], first_hmove, hmove)
                finally:
                    self._set_homing_accel(hi.accel, pre_homing=False)
                    self._set_homing_current(homing_axes, pre_homing=False)

                if hi.retract_dist:
                    # Retract (again)
                    startpos = self._fill_coord(forcepos)
                    homepos = self._fill_coord(movepos)
                    axes_d = [hp - sp for hp, sp in zip(homepos, startpos)]
                    move_d = math.sqrt(sum([d * d for d in axes_d[:3]]))
                    retract_r = min(1.0, hi.retract_dist / move_d)
                    retractpos = [
                        hp - ad * retract_r for hp, ad in zip(homepos, axes_d)
                    ]
                    self.toolhead.move(retractpos, hi.retract_speed)

        self._set_homing_accel(hi.accel, pre_homing=False)
        self._set_homing_current(homing_axes, pre_homing=False)
        # Signal home operation complete
        self.toolhead.flush_step_generation()
        self.trigger_mcu_pos = {
            sp.stepper_name: sp.trig_pos for sp in hmove.stepper_positions
        }
        self.adjust_pos = {}
        self.printer.send_event("homing:home_rails_end", self, rails)
        if any(self.adjust_pos.values()):
            # Apply any homing offsets
            kin = self.toolhead.get_kinematics()
            homepos = self.toolhead.get_position()
            kin_spos = {
                s.get_name(): (
                    s.get_commanded_position()
                    + self.adjust_pos.get(s.get_name(), 0.0)
                )
                for s in kin.get_steppers()
            }
            newpos = kin.calc_position(kin_spos)
            for axis in force_axes:
                homepos[axis] = newpos[axis]
            self.toolhead.set_position(homepos)


class PrinterHoming:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        # Register g-code commands
        gcode = self.printer.lookup_object("gcode")
        gcode.register_command("G28", self.cmd_G28)
        self.active_hmove = None
        self.motor_fault_abort_reason = None
        self._session_active = False
        self._session_axes = ()
        self._session_z_align = False
        self._session_aborted = False
        self._session_abort_result = None
        self._abort_in_progress = False
        self._abort_cleanup_pending = False
        self._abort_had_active_hmove = False
        self._abort_z_align_aborted = False
        self._session_stall_mode_active = False
        self._xy_startup_prime_pending = True
        self._z_align_rise_prime_pending = True
        self.printer.register_event_handler(
            "klippy:connect", self._handle_connect)
        self.printer.register_event_handler(
            "gcode:request_restart", self._handle_request_restart)

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
        return self.printer.lookup_object("motor_control")

    @staticmethod
    def _collect_active_faults(result):
        if not isinstance(result, dict):
            return {}
        return {
            axis: detail for axis, detail in result.items()
            if isinstance(detail, dict) and detail.get("active")
        }

    @staticmethod
    def _collect_active_errors(result):
        if not isinstance(result, dict):
            return {}
        return {
            axis: detail for axis, detail in result.items()
            if isinstance(detail, dict) and detail.get("has_error")
        }

    @staticmethod
    def _format_fault_summary(faults):
        if not faults:
            return "none"
        return _format_motor_fault_summary(
            faults, axis_labeler=lambda axis: str(axis).upper(),
            empty_text="none")

    def _set_homing_stall_mode(self, mode):
        return self._lookup_motor_control().set_homing_stall_mode(mode)

    def _enter_homing_stall_mode(self):
        if self._session_stall_mode_active:
            return
        self._set_homing_stall_mode(1)
        self._session_stall_mode_active = True

    def _restore_homing_stall_mode(self):
        if not self._session_stall_mode_active:
            return
        self._set_homing_stall_mode(2)
        self._session_stall_mode_active = False

    def _query_kinematic_protection(
            self, data=11, timeout=MOTOR_COMMAND_TIMEOUT):
        return self._lookup_motor_control().query_kinematic_protection_status(
            data=data, timeout=timeout)

    def _clear_kinematic_fault_latches(
            self, data=5, timeout=MOTOR_NO_ACK_TIMEOUT):
        return self._lookup_motor_control().clear_kinematic_fault_latches(
            data=data, timeout=timeout)

    def _recover_kinematic_faults_for_homing_start(
            self, query_data=11, clear_data=5,
            timeout=MOTOR_COMMAND_TIMEOUT):
        return self._lookup_motor_control().recover_kinematic_faults_for_homing_start(
            query_data=query_data, clear_data=clear_data, timeout=timeout)

    def _set_active_hmove(self, hmove):
        self.active_hmove = hmove

    def _clear_active_hmove(self, hmove):
        if self.active_hmove is hmove:
            self.active_hmove = None
            if self._abort_cleanup_pending:
                self._abort_in_progress = True
                try:
                    self._finish_homing_abort()
                finally:
                    self._abort_in_progress = False

    def _normalize_session_axes(self, axes):
        if axes is None:
            return ()
        if isinstance(axes, (list, tuple)):
            raw = "".join(str(axis) for axis in axes)
        else:
            raw = str(axes)
        seen = []
        for axis in raw.upper():
            if axis in ("X", "Y", "Z") and axis not in seen:
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
        self._abort_cleanup_pending = False
        self._abort_had_active_hmove = False
        self._abort_z_align_aborted = False
        self._session_stall_mode_active = False
        self.motor_fault_abort_reason = None
        return True

    def end_homing_session(self):
        self._session_active = False
        self._session_axes = ()
        self._session_z_align = False
        self._session_aborted = False
        self._session_abort_result = None
        self._abort_in_progress = False
        self._abort_cleanup_pending = False
        self._abort_had_active_hmove = False
        self._abort_z_align_aborted = False
        self._session_stall_mode_active = False
        self.motor_fault_abort_reason = None

    def has_active_homing_session(self):
        return (
            self._session_active and not self._session_aborted
            and not self._abort_in_progress)

    def is_homing_session_aborted(self):
        return self._session_aborted

    def is_homing_abort_in_progress(self):
        return self._abort_in_progress or self._abort_cleanup_pending

    def _is_probe_model_ready(self):
        scanner = self.printer.lookup_object("cartographer", None)
        if scanner is None:
            return True
        scan_mode = getattr(scanner, "scan_mode", None)
        if scan_mode is None:
            return True
        if hasattr(scan_mode, "is_ready"):
            return bool(scan_mode.is_ready)
        if hasattr(scan_mode, "has_model"):
            return bool(scan_mode.has_model())
        return True

    def _check_scanner_connected(self):
        scanner = self.printer.lookup_object("cartographer", None)
        if (
            scanner is not None
            and hasattr(scanner, "_check_mcu_disconnected")
            and scanner._check_mcu_disconnected()
        ):
            raise self.printer.command_error(
                "Scanner MCU is disconnected - cannot complete Z homing. "
                "Photoelectric leveling is preserved. Reconnect scanner and "
                "retry G28 Z.")

    def _invalidate_kinematic_homing_state(self):
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        kin.clear_homing_state("xyz")

    def _mark_motor_control_not_homing(self):
        self.printer.lookup_object("motor_control").is_homing = False

    def _should_abort_z_align(self):
        z_align = self.printer.lookup_object("z_align", None)
        if z_align is not None and z_align.is_active():
            return True
        return bool(self._session_z_align)

    def _build_homing_abort_result(
            self, protection_after_clear=None, persistent_faults=None):
        return {
            "reason": self.motor_fault_abort_reason,
            "axes": tuple(self._session_axes),
            "active_hmove": self._abort_had_active_hmove,
            "z_align_aborted": self._abort_z_align_aborted,
            "cleanup_pending": self._abort_cleanup_pending,
            "protection_after_clear": protection_after_clear or {},
            "persistent_faults": persistent_faults or {},
        }

    def _finish_homing_abort(self):
        try:
            z_align = self.printer.lookup_object("z_align", None)
            if z_align is not None:
                z_align.invalidate_homing_state()
            self._invalidate_kinematic_homing_state()
            self._mark_motor_control_not_homing()
        finally:
            self.printer.lookup_object("stepper_enable").motor_off()
        protection_after_clear = {}
        persistent_faults = {}
        try:
            self._clear_kinematic_fault_latches(data=5)
            protection_after_clear = self._query_kinematic_protection(
                data=11, timeout=MOTOR_COMMAND_TIMEOUT)
            persistent_faults = self._collect_active_faults(
                protection_after_clear)
        except Exception:
            _klog(
                "failed clearing/rechecking kinematic faults "
                "after abort", level=logging.exception)
        try:
            self._restore_homing_stall_mode()
        except Exception:
            _klog(
                "failed restoring MOTOR_STALL_MODE DATA=2 "
                "after abort", level=logging.exception)
        if persistent_faults:
            self.motor_fault_abort_reason = (
                "Persistent motor protection fault after abort cleanup (%s)"
                % (self._format_fault_summary(persistent_faults),))
        result = self._build_homing_abort_result(
            protection_after_clear, persistent_faults)
        result["cleanup_pending"] = False
        self._session_abort_result = dict(result)
        self._abort_cleanup_pending = False
        return result

    def request_homing_abort(self, reason, detail=None, abort_z_align=True):
        if self._session_abort_result is not None:
            return dict(self._session_abort_result)
        if self._abort_in_progress:
            return self._build_homing_abort_result()
        if self._session_aborted:
            if self._abort_cleanup_pending and self.active_hmove is None:
                self._abort_in_progress = True
                try:
                    return self._finish_homing_abort()
                finally:
                    self._abort_in_progress = False
            return self._build_homing_abort_result()
        self._abort_in_progress = True
        self._session_aborted = True
        self.motor_fault_abort_reason = reason
        z_align = self.printer.lookup_object("z_align", None)
        try:
            _klog(
                "request_homing_abort reason=%s detail=%s",
                reason, detail, level=logging.warning)
            if abort_z_align and z_align is not None:
                self._abort_z_align_aborted = bool(z_align.abort_internal(
                    reason=reason, motor_off=False,
                    restore_motor_mode=False))
            self._abort_had_active_hmove = self.active_hmove is not None
            self._abort_cleanup_pending = True
            if self.active_hmove is not None:
                self.active_hmove.request_external_force_stop(
                    reason=reason, immediate=True)
                return self._build_homing_abort_result()
            return self._finish_homing_abort()
        finally:
            self._abort_in_progress = False

    def request_motor_fault_abort(self, detail=None):
        axes = ()
        source = None
        fault_summary = None
        if isinstance(detail, dict):
            axes = tuple(sorted(detail.get("axes", {}).keys()))
            source = detail.get("source")
            active_faults = self._collect_active_faults(detail.get("axes", {}))
            if active_faults:
                fault_summary = self._format_fault_summary(active_faults)
        if fault_summary:
            reason = "Motor protection fault aborted homing (%s)" % (
                fault_summary,)
        else:
            axis_text = ",".join(axes) if axes else (source or "unknown")
            reason = "Motor protection fault aborted homing (%s)" % (axis_text,)
        return self.request_homing_abort(
            reason=reason, detail=detail,
            abort_z_align=self._should_abort_z_align())

    def manual_home(
        self, toolhead, endstops, pos, speed, triggered, check_triggered
    ):
        hmove = HomingMove(self.printer, endstops, toolhead)
        try:
            hmove.homing_move(
                pos, speed, triggered=triggered, check_triggered=check_triggered
            )
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown"
                )
            raise

    def probing_move(self, mcu_probe, pos, speed):
        endstops = [(mcu_probe, "probe")]
        hmove = HomingMove(self.printer, endstops)
        try:
            epos = hmove.homing_move(pos, speed, probe_pos=True)
        except self.printer.command_error:
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Probing failed due to printer shutdown"
                )
            raise
        if hmove.check_no_movement() is not None:
            raise self.printer.command_error(
                "Probe triggered prior to movement"
            )
        return epos

    def _start_managed_homing_session(self, requested_axes, z_align=False):
        blocker = self._lookup_motor_control().get_homing_fault_blocker()
        if blocker:
            raise self.printer.command_error(blocker)
        session_created = self.begin_homing_session(
            axes=requested_axes, z_align=z_align)
        if not session_created:
            return False
        try:
            try:
                recovery = self._recover_kinematic_faults_for_homing_start(
                    query_data=11, clear_data=5,
                    timeout=MOTOR_COMMAND_TIMEOUT)
            except Exception as err:
                raise self.printer.command_error(
                    "Motor protection recovery failed before homing: %s"
                    % (err,))
            persistent = recovery.get("persistent_errors", {})
            if persistent:
                reason = (
                    "Persistent motor protection fault before homing (%s)"
                    % (self._format_fault_summary(persistent),))
                self.request_homing_abort(
                    reason=reason,
                    detail={
                        "source": "homing_session_start",
                        "recovery": recovery,
                    },
                    abort_z_align=False)
                raise self.printer.command_error(
                    self.motor_fault_abort_reason or reason)
            self._enter_homing_stall_mode()
            return True
        except Exception:
            if not self._session_aborted:
                self.end_homing_session()
            raise

    def _finish_managed_homing_session(self):
        if not self._session_active:
            return
        if self._session_aborted or self._abort_in_progress:
            return
        protection = {}
        persistent = {}
        try:
            protection = self._query_kinematic_protection(
                data=11, timeout=MOTOR_COMMAND_TIMEOUT)
            persistent = self._collect_active_errors(protection)
        finally:
            self._restore_homing_stall_mode()
        if persistent:
            reason = (
                "Motor protection fault still active at homing end (%s)"
                % (self._format_fault_summary(persistent),))
            self.request_homing_abort(
                reason=reason,
                detail={
                    "source": "homing_session_finish",
                    "protection": protection,
                },
                abort_z_align=False)
            raise self.printer.command_error(
                self.motor_fault_abort_reason or reason)
        self.end_homing_session()

    def _get_requested_axes(self, gcmd):
        axes = []
        for pos, axis in enumerate("XYZ"):
            if gcmd.get(axis, None) is not None:
                axes.append(pos)
        if not axes:
            return [0, 1, 2]
        return axes

    def _get_homed_axis_letters(self):
        toolhead = self.printer.lookup_object("toolhead")
        eventtime = self.printer.get_reactor().monotonic()
        homed_axes = toolhead.get_status(eventtime).get("homed_axes", "")
        return set(axis.lower() for axis in homed_axes)

    def _resolve_home_order(self, requested_axes, homed_axes):
        requested = set(requested_axes)
        order = []
        if 2 in requested:
            for axis_idx, axis_name in ((1, "y"), (0, "x")):
                if axis_idx in requested or axis_name not in homed_axes:
                    order.append(axis_idx)
            order.append(2)
            return order
        if requested == {0, 1}:
            return [1, 0]
        return list(requested_axes)

    def _get_axis_limits(self, axis_idx):
        axis_name = "xyz"[axis_idx]
        section = self.config.getsection("stepper_%s" % (axis_name,))
        pos_min = section.getfloat("position_min", default=0.0)
        pos_max = section.getfloat("position_max")
        return pos_min, pos_max

    def _prime_xy_home_once_after_restart(self, kin, axis_idx):
        if axis_idx not in (0, 1):
            return
        rail = kin.rails[axis_idx]
        hi = rail.get_homing_info()
        pos_min, pos_max = rail.get_range()
        direction = 1.0 if getattr(hi, "positive_dir", False) else -1.0
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
        toolhead = self.printer.lookup_object("toolhead")
        startpos = list(toolhead.get_position())
        startpos[axis_idx] = start_axis
        toolhead.set_position(startpos, homing_axes="xyz"[axis_idx])
        primepos = list(startpos)
        primepos[axis_idx] = target_axis
        prime_speed = min(float(hi.speed), XY_STARTUP_PRIME_SPEED)
        _klog(
            "priming first %s home after restart dist=%.3f speed=%.3f",
            rail.get_name(), prime_dist, prime_speed)
        prime_hmove = HomingMove(self.printer, rail.get_endstops())
        prime_hmove.homing_move(
            primepos, prime_speed, triggered=True, check_triggered=False)

    def _compute_z_home_center(self):
        if self.config.has_section("bed_mesh"):
            bed_mesh = self.config.getsection("bed_mesh")
            mesh_min = bed_mesh.getfloatlist("mesh_min", count=2)
            mesh_max = bed_mesh.getfloatlist("mesh_max", count=2)
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
        toolhead = self.printer.lookup_object("toolhead")
        pos = list(toolhead.get_position())
        center_x, center_y = self._compute_z_home_center()
        pos[0] = center_x
        pos[1] = center_y
        _klog(
            "moving to Z-home center x=%.3f y=%.3f",
            center_x, center_y)
        toolhead.move(pos, speed)
        toolhead.wait_moves()

    def _get_z_home_endstops(self, kin):
        rails = kin.rails
        return rails[2].get_endstops()

    def _get_triggered_endstop_names(self, endstops, print_time):
        triggered = []
        for mcu_endstop, name in endstops:
            if mcu_endstop.query_endstop(print_time):
                triggered.append(name)
        return tuple(sorted(set(triggered)))

    def _emit_raw_warning(self, msg):
        _klog("%s", msg, level=logging.warning)
        self.printer.lookup_object("gcode").respond_raw(msg)

    def _move_known_z_to_clearance_before_home(self, kin):
        toolhead = self.printer.lookup_object("toolhead")
        pos = list(toolhead.get_position())
        current_z = pos[2]
        if current_z <= Z_REHOME_CLEARANCE:
            return False
        endstops = self._get_z_home_endstops(kin)
        start_triggered = self._get_triggered_endstop_names(
            endstops, toolhead.get_last_move_time())
        target = list(pos)
        target[2] = Z_REHOME_CLEARANCE
        _klog(
            "moving known Z %.3f -> %.3f before G28 Z at %.3fmm/s",
            current_z, Z_REHOME_CLEARANCE, Z_REHOME_CLEARANCE_SPEED)
        hmove = HomingMove(self.printer, endstops, toolhead)
        hmove.homing_move(
            target, Z_REHOME_CLEARANCE_SPEED,
            triggered=True, check_triggered=False)
        if hmove.triggered_endstops:
            triggered_set = set(hmove.triggered_endstops)
            start_set = set(start_triggered)
            if start_set and triggered_set.issubset(start_set):
                msg = (
                    "!! G28 Z warning: Z endstop already triggered before "
                    "pre-home move at Z=%.3f (%s); continuing with normal G28 Z"
                    % (current_z, ",".join(sorted(triggered_set))))
            else:
                msg = (
                    "!! G28 Z warning: Z endstop triggered during pre-home "
                    "move from Z=%.3f to Z=%.3f (%s); continuing with normal "
                    "G28 Z"
                    % (
                        current_z, Z_REHOME_CLEARANCE,
                        ",".join(sorted(triggered_set))))
            self._emit_raw_warning(msg)
            return False
        return True

    def _home_single_axis(self, homing_state, kin, axis_idx):
        if axis_idx in (0, 1):
            self._prime_xy_home_once_after_restart(kin, axis_idx)
        homing_state.set_axes([axis_idx])
        kin.home(homing_state)

    def _integrated_home_z(self, homing_state, kin, z_was_homed=False):
        z_align = self.printer.lookup_object("z_align", None)
        use_z_align = bool(z_align is not None and z_align.needs_prep())
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
                    self.motor_fault_abort_reason or "Homing session aborted")
        elif z_was_homed:
            self._move_known_z_to_clearance_before_home(kin)
        if not self._is_probe_model_ready():
            raise HomingZProbeNotCalibrated("Scan model not loaded")
        self._check_scanner_connected()
        homing_state.set_axes([2])
        kin.home(homing_state)
        # Lift Z to a known clearance after a successful home so it is not left
        # at the trigger position; reset_last_position keeps gcode_move in sync
        # after the direct toolhead move.
        toolhead = self.printer.lookup_object("toolhead")
        pos = list(toolhead.get_position())
        pos[2] = Z_POST_HOME_LIFT
        toolhead.move(pos, Z_POST_HOME_LIFT_SPEED)
        toolhead.wait_moves()
        self.printer.lookup_object("gcode_move").reset_last_position()

    def cmd_G28(self, gcmd):
        axes = self._get_requested_axes(gcmd)
        homing_state = Homing(self.printer)
        toolhead = self.printer.lookup_object("toolhead")
        kin = toolhead.get_kinematics()
        if self._session_aborted:
            reason = self.motor_fault_abort_reason or "Homing session aborted"
            if self._abort_cleanup_pending and self.active_hmove is None:
                self.request_homing_abort(
                    reason, abort_z_align=False)
            self.end_homing_session()
            raise gcmd.error(reason)
        session_owned = False
        try:
            homed_axes = self._get_homed_axis_letters()
            order = self._resolve_home_order(axes, homed_axes)
            z_align = self.printer.lookup_object("z_align", None)
            use_z_align = bool(
                2 in order
                and z_align is not None
                and z_align.needs_prep())
            if not self._session_active:
                requested_axes = "".join("XYZ"[axis] for axis in order)
                # We own the session from the moment we request it: a fault
                # raised mid-start (e.g. persistent pre-homing fault) still
                # leaves teardown to the finally block below.
                session_owned = True
                self._start_managed_homing_session(
                    requested_axes, z_align=use_z_align)
            if use_z_align:
                z_align.start_prepare()
            for axis_idx in order:
                if axis_idx in (0, 1):
                    self._home_single_axis(homing_state, kin, axis_idx)
                else:
                    self._integrated_home_z(
                        homing_state, kin, z_was_homed=("z" in homed_axes))
            if session_owned:
                self._finish_managed_homing_session()
        except HomingZProbeNotCalibrated as err:
            # Rise completed cleanly; leave motors enabled. Stall mode and
            # session teardown are handled by the finally block below.
            _klog(
                "probe not calibrated after rise: %s",
                err, level=logging.warning)
            self._mark_motor_control_not_homing()
            raise self.printer.command_error(str(err))
        except Exception as err:
            _klog("%s", err, level=logging.exception)
            abort_reason = self.motor_fault_abort_reason
            if (
                self._session_active
                and (not self._session_aborted or self._abort_cleanup_pending)
            ):
                try:
                    self.request_homing_abort(
                        reason=self.motor_fault_abort_reason or str(err),
                        detail={"source": "cmd_G28", "error": str(err)},
                        abort_z_align=self._should_abort_z_align())
                except Exception as cleanup_err:
                    _klog(
                        "unified abort failed during cmd_G28",
                        level=logging.exception)
                    abort_reason = "Homing abort cleanup failed: %s" % (
                        cleanup_err,)
            abort_reason = abort_reason or self.motor_fault_abort_reason
            if self.printer.is_shutdown():
                raise self.printer.command_error(
                    "Homing failed due to printer shutdown")
            if abort_reason is not None and str(err) != abort_reason:
                raise self.printer.command_error(abort_reason)
            raise
        finally:
            if session_owned and self._session_active:
                try:
                    self._restore_homing_stall_mode()
                except Exception:
                    _klog(
                        "failed restoring stall mode during final cleanup",
                        level=logging.exception)
                if not self._abort_cleanup_pending:
                    self.end_homing_session()


def load_config(config):
    global HOMING_START_DELAY
    HOMING_START_DELAY = get_danger_options().homing_start_delay
    global ENDSTOP_SAMPLE_TIME
    ENDSTOP_SAMPLE_TIME = get_danger_options().endstop_sample_time
    global ENDSTOP_SAMPLE_COUNT
    ENDSTOP_SAMPLE_COUNT = get_danger_options().endstop_sample_count
    return PrinterHoming(config)
