# Copyright (C) 2026  36573259+Jacob10383@users.noreply.github.com
# This file may be distributed under the terms of the GNU GPLv3 license.
# prtouch - K2 nozzle load cell helpers on top of [load_cell_probe]
#
# The native nozzle MCU runs Kalico's load cell probe, which does the
# probing. This module adds the K2 commands built on it: a multi-sample Z
# home, the rear bed tab scrub, and Cartographer calibration with nozzle
# contact as Z=0.
import logging
import random
from statistics import median
from types import SimpleNamespace

_POST_HOME_LIFT = 3.0

# Options of the stock-firmware PRTouch detector, still accepted so older
# configs load. The load cell probe now owns probing and its settings.
_IGNORED_OPTIONS = (
    'register_as_probe', 'samples', 'samples_result', 'samples_tolerance',
    'samples_tolerance_retries', 'no_trigger_retries',
    'no_trigger_retry_max_distance', 'step_swap_pin', 'pres_swap_pin',
    'pres_cfg_regs', 'pres_tri_hold', 'pres_acq_tkms', 'pres_tri_fter',
    'pres_ded_tkms', 'pres_idle_swap_state', 'pres_release_timeout',
    'pres_ack_timeout', 'pres_rearm_delay',
) + tuple('pres_cs%d_pin' % (i,) for i in range(8))


def _sample_range(values):
    return max(values) - min(values)


def _best_subset(values, size):
    if len(values) < size:
        return None
    values = sorted(values)
    start = min(
        range(len(values) - size + 1),
        key=lambda i: values[i + size - 1] - values[i])
    return values[start:start + size]


def _thermal_z_comp(rate_mm_c, print_temp, touch_temp):
    """Z shift so a touch at touch_temp is correct when printing at print_temp.

    Hotter nozzle grows toward the bed. Label contact lower by rate*(T_print -
    T_touch) so that after heating, tip-bed contact lands at z_offset.
    """
    return rate_mm_c * (print_temp - touch_temp)


class PRTouch:
    def __init__(self, config):
        self.config = config
        self.printer = config.get_printer()
        self.reactor = self.printer.get_reactor()
        self.gcode = self.printer.lookup_object('gcode')
        for option in _IGNORED_OPTIONS:
            config.get(option, None)
        # z_offset now belongs to the load cell probe
        self._legacy_z_offset = config.getfloat('z_offset', 0.)
        self.probe_name = config.get('probe', 'load_cell_probe')
        # Default the probe's z_offset in memory rather than in the shipped
        # include, so SAVE_CONFIG can store it without an include conflict.
        # prtouch.cfg puts [prtouch] first so this runs before the probe.
        fileconfig = config.fileconfig
        if (fileconfig.has_section(self.probe_name)
                and not fileconfig.has_option(self.probe_name, 'z_offset')):
            fileconfig.set(self.probe_name, 'z_offset', '0')
        probe_config = config.getsection(self.probe_name)
        self.trigger_force = probe_config.getint(
            'trigger_force', 75, note_valid=False)
        self.speed = config.getfloat('speed', 5., above=0.)
        self.lift_speed = config.getfloat('lift_speed', self.speed, above=0.)
        self.sample_retract_dist = config.getfloat(
            'sample_retract_dist', 2., above=0.)

        # PRTOUCH_HOME: multi-sample Z home with the nozzle
        self.home_xy = config.getfloatlist('home_xy', (175., 175.), count=2)
        self.home_samples = config.getint('home_samples', 3, minval=1)
        self.home_max_samples = config.getint(
            'home_max_samples', max(10, self.home_samples),
            minval=self.home_samples)
        self.home_max_noisy = config.getint('home_max_noisy', 3, minval=0)
        self.home_sample_range = config.getfloat(
            'home_sample_range', 0.010, minval=0.)
        self.home_travel_speed = config.getfloat(
            'home_travel_speed', 200., above=0.)
        self.home_z_hop = config.getfloat('home_z_hop', 2., above=0.)
        # Nozzle/stack growth (mm/°C). Used only when PRTOUCH_HOME PRINT_TEMP= is set.
        self.thermal_expansion = config.getfloat(
            'thermal_expansion', 0., minval=0.)

        # PRTOUCH_SCRUB: detect the flexible rear tab, then scrub its surface.
        self.scrub_x_start = config.getfloat('scrub_x_start', 173.)
        self.scrub_x_end = config.getfloat('scrub_x_end', 223.)
        self.scrub_y_min = config.getfloat('scrub_y_min', 353.)
        self.scrub_y_max = config.getfloat('scrub_y_max', 356.)
        if self.scrub_y_min > self.scrub_y_max:
            raise config.error(
                "prtouch: scrub_y_min must not exceed scrub_y_max")
        # The tab deflects more under a harder tap than the bed does
        self.scrub_detect_hold_ratio = config.getfloat(
            'scrub_detect_hold_ratio', 2.5, above=1.)
        self.scrub_detect_deflection = config.getfloat(
            'scrub_detect_deflection', .035, above=0.)
        self.scrub_tab_depth = config.getfloat(
            'scrub_tab_depth', .15, minval=0., maxval=.20)
        self.scrub_no_tab_depth = config.getfloat(
            'scrub_no_tab_depth', .05, minval=0., maxval=.20)
        if self.scrub_no_tab_depth > self.scrub_tab_depth:
            raise config.error(
                "prtouch: scrub_no_tab_depth must not exceed scrub_tab_depth")
        self.scrub_speed = config.getfloat('scrub_speed', 10., above=0.)

        self._probe = None
        self.printer.register_event_handler(
            'klippy:connect', self._handle_connect)
        self.printer.register_event_handler(
            'klippy:ready', self._handle_ready)
        self.gcode.register_command(
            'PRTOUCH_HOME', self.cmd_PRTOUCH_HOME,
            desc=self.cmd_PRTOUCH_HOME_help)
        self.gcode.register_command(
            'PRTOUCH_SCAN_CALIBRATE', self.cmd_PRTOUCH_SCAN_CALIBRATE,
            desc=self.cmd_PRTOUCH_SCAN_CALIBRATE_help)
        self.gcode.register_command(
            'PRTOUCH_AXIS_TWIST_COMPENSATION',
            self.cmd_PRTOUCH_AXIS_TWIST_COMPENSATION,
            desc=self.cmd_PRTOUCH_AXIS_TWIST_COMPENSATION_help)
        self.gcode.register_command(
            'PRTOUCH_SCRUB', self.cmd_PRTOUCH_SCRUB,
            desc=self.cmd_PRTOUCH_SCRUB_help)

    def _handle_connect(self):
        probe_list = self.printer.lookup_object('probe_list', None)
        if probe_list is not None:
            self._probe = probe_list.get_probe(self.probe_name)

    def _missing_probe_msg(self):
        return ("prtouch: [%s] not found; update the K2 configs to use the "
                "nozzle load cell" % (self.probe_name,))

    def _require_probe(self):
        if self._probe is None:
            raise self.printer.command_error(self._missing_probe_msg())

    def _handle_ready(self):
        # An older config without the load cell still loads; the PRTOUCH
        # commands report the problem instead
        if self._probe is None:
            logging.warning(self._missing_probe_msg())
            self.gcode.respond_raw("!! " + self._missing_probe_msg())
        if self._legacy_z_offset:
            self.gcode.respond_raw(
                "!! prtouch: [prtouch] z_offset is no longer used. Set "
                "z_offset in [%s] instead." % (self.probe_name,))

    @property
    def position_endstop(self):
        self._require_probe()
        return self._probe.get_offsets()[2]

    def _probe_one(self, gcmd):
        """Single nozzle touch at the current XY, without twist correction."""
        self._require_probe()
        toolhead = self.printer.lookup_object('toolhead')
        try:
            epos = list(self._probe.run_probe(gcmd))
        except self.printer.command_error:
            # run_probe only closes its sessions on success; a stale retry
            # state would fail the next touch here without a sample
            try:
                self._probe.multi_probe_end()
            finally:
                self._probe.retry_session.end()
            raise
        # The probe adds axis twist correction, which is calibrated for the
        # offset scanner; a nozzle contact needs none. A single sample ends
        # where it touched, which a bad tap retry may have moved.
        twist = self.printer.lookup_object('axis_twist_compensation', None)
        if twist is not None:
            epos[2] -= twist.get_z_compensation_value(toolhead.get_position())
        return epos[:3]

    def _sample_gcmd(self, name, extra=None):
        params = {
            'SAMPLES': '1',
            'SAMPLES_RESULT': 'median',
            'SAMPLES_TOLERANCE': '999',
            'SAMPLES_TOLERANCE_RETRIES': '0',
        }
        params.update(extra or {})
        return self.gcode.create_gcode_command(name, name, params)

    def _multi_sample_probe_z(self, gcmd=None):
        samples = self.home_samples
        max_samples = max(self.home_max_samples, samples)
        max_noisy = self.home_max_noisy
        sample_range = self.home_sample_range
        extra = {}
        if gcmd is not None:
            samples = gcmd.get_int(
                'SAMPLES', samples, minval=1, maxval=50)
            max_samples = gcmd.get_int(
                'MAX_SAMPLES', max(max_samples, samples),
                minval=samples, maxval=100)
            max_noisy = gcmd.get_int(
                'MAX_NOISY', max_noisy, minval=0, maxval=100)
            sample_range = gcmd.get_float(
                'SAMPLE_RANGE', sample_range, minval=0., maxval=1.)
            for key in ('PROBE_SPEED', 'LIFT_SPEED', 'SAMPLE_RETRACT_DIST'):
                val = gcmd.get(key, None)
                if val is not None:
                    extra[key] = val
        window_size = samples + max_noisy
        sample_gcmd = self._sample_gcmd('PRTOUCH_SAMPLE', extra)
        collected = []
        best = None
        for attempt in range(1, max_samples + 1):
            epos = self._probe_one(sample_gcmd)
            collected.append(epos[2])
            window = collected[-window_size:]
            if len(window) >= samples:
                best = _best_subset(window, samples)
                if best is not None and _sample_range(best) <= sample_range:
                    break
            if attempt < max_samples:
                self._retract_home_sample(sample_gcmd)
        if best is None or _sample_range(best) > sample_range:
            raise self.printer.command_error(
                "prtouch: unable to find %d samples within %.4fmm "
                "after %d touches"
                % (samples, sample_range, max_samples))
        return float(median(best)), collected, best

    def _seed_z_homed_if_needed(self, toolhead):
        curtime = self.reactor.monotonic()
        if 'z' in toolhead.get_status(curtime)['homed_axes']:
            return False
        zconfig = self.config.getsection('stepper_z')
        z_max = zconfig.getfloat('position_max')
        pos = toolhead.get_position()
        pos[2] = z_max - 10.
        toolhead.set_position(pos, homing_axes='z')
        return True

    def _lookup_carto_macro(self, class_name, *need_attrs):
        carto = self.printer.lookup_object('cartographer', None)
        if carto is None:
            return None
        for reg in getattr(carto, 'macros', []):
            macro = getattr(reg, 'macro', None)
            if (
                    macro is not None
                    and type(macro).__name__ == class_name
                    and all(hasattr(macro, name) for name in need_attrs)):
                return macro
        return None

    def _establish_nozzle_z_zero(self, gcmd=None):
        toolhead = self.printer.lookup_object('toolhead')
        seeded = self._seed_z_homed_if_needed(toolhead)
        try:
            pos = toolhead.get_position()
            if pos[2] < self.sample_retract_dist:
                toolhead.manual_move(
                    [None, None, self.sample_retract_dist], self.lift_speed)
                toolhead.wait_moves()
            trigger_z, _collected, best = self._multi_sample_probe_z(gcmd)
            toolhead.get_last_move_time()  # sync print_time before rewriting Z
            pos = toolhead.get_position()
            pos[2] = pos[2] - trigger_z + self.position_endstop
        finally:
            if seeded:
                toolhead.get_kinematics().clear_homing_state('z')
        toolhead.set_position(pos, homing_axes='z')
        return trigger_z, best

    cmd_PRTOUCH_HOME_help = (
        "Multi-sample Z home with the nozzle load cell "
        "(more accurate than a single G28 Z touch). "
        "Optional PRINT_TEMP applies thermal_expansion compensation.")

    cmd_PRTOUCH_SCAN_CALIBRATE_help = (
        "Calibrate Cartographer scan model using the nozzle load cell for "
        "Z=0. Requires cartographer. Optional MODEL=default.")

    cmd_PRTOUCH_AXIS_TWIST_COMPENSATION_help = (
        "Calibrate axis twist using Cartographer scans and nozzle contacts.")

    def _get_extruder_temp(self):
        pheaters = self.printer.lookup_object('heaters')
        heater = pheaters.lookup_heater('extruder')
        temp, _target = heater.get_temp(self.reactor.monotonic())
        return float(temp)

    def cmd_PRTOUCH_SCAN_CALIBRATE(self, gcmd):
        self._require_probe()
        macro = self._lookup_carto_macro('ScanCalibrateMacro', '_calibrate')
        if macro is None:
            raise gcmd.error(
                "prtouch: cartographer ScanCalibrateMacro not loaded")
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.reactor.monotonic()
        homed = toolhead.get_status(curtime)['homed_axes']
        if 'x' not in homed or 'y' not in homed:
            raise gcmd.error(
                "prtouch: must home X/Y before PRTOUCH_SCAN_CALIBRATE")
        model = gcmd.get('MODEL', 'default').strip().lower()
        zrp = macro._config.bed_mesh.zero_reference_position
        travel = macro._config.general.travel_speed
        toolhead.manual_move([zrp[0], zrp[1], None], travel)
        toolhead.wait_moves()
        trigger_z, best = self._establish_nozzle_z_zero(gcmd)
        gcmd.respond_info(
            "prtouch: nozzle Z=%.4f (median=%.4f range=%.4f); "
            "running carto scan calibrate"
            % (self.position_endstop, trigger_z, _sample_range(best)))
        macro._calibrate(model)
        self.gcode.run_script_from_command('SAVE_CONFIG RESTART=0')

    def cmd_PRTOUCH_AXIS_TWIST_COMPENSATION(self, gcmd):
        self._require_probe()
        macro = self._lookup_carto_macro(
            'AxisTwistCompensationMacro', 'run', 'probe')
        if macro is None:
            raise gcmd.error(
                "prtouch: cartographer AxisTwistCompensationMacro not loaded")
        original_probe = macro.probe

        def nozzle_contact():
            trigger_z, _collected, _best = self._multi_sample_probe_z(gcmd)
            self._retract_home_sample(gcmd)
            return trigger_z - self.position_endstop

        macro.probe = SimpleNamespace(
            touch=original_probe.touch,
            perform_scan=original_probe.perform_scan,
            perform_touch=nozzle_contact)
        try:
            macro.run(gcmd)
        except (RuntimeError, ValueError) as error:
            raise gcmd.error(str(error)) from error
        finally:
            macro.probe = original_probe

    def cmd_PRTOUCH_HOME(self, gcmd):
        self._require_probe()
        toolhead = self.printer.lookup_object('toolhead')
        curtime = self.reactor.monotonic()
        homed = toolhead.get_status(curtime)['homed_axes']
        if 'x' not in homed or 'y' not in homed:
            raise gcmd.error("prtouch: must home X/Y before PRTOUCH_HOME")

        travel_speed = gcmd.get_float(
            'TRAVEL_SPEED', self.home_travel_speed, above=0.)
        z_hop = gcmd.get_float('Z_HOP', self.home_z_hop, above=0.)
        print_temp = gcmd.get_float('PRINT_TEMP', None, minval=0.)

        thermal_comp = 0.
        touch_temp = None
        if print_temp is not None:
            if self.thermal_expansion <= 0.:
                raise gcmd.error(
                    "prtouch: PRINT_TEMP requires thermal_expansion "
                    "in [prtouch] config")
            touch_temp = self._get_extruder_temp()
            thermal_comp = _thermal_z_comp(
                self.thermal_expansion, print_temp, touch_temp)

        z_was_unhomed = 'z' not in homed
        try:
            if z_was_unhomed:
                self._seed_z_homed_if_needed(toolhead)

            pos = toolhead.get_position()
            toolhead.manual_move(
                [None, None, pos[2] + z_hop], travel_speed)
            toolhead.manual_move(
                [self.home_xy[0], self.home_xy[1], None], travel_speed)
            toolhead.wait_moves()

            trigger_z, collected, best = self._multi_sample_probe_z(gcmd)
            toolhead.get_last_move_time()  # sync print_time before rewriting Z
            pos = toolhead.get_position()
            # Contact becomes z_offset - thermal_comp so hotter print tip
            # lands at z_offset after nozzle growth.
            pos[2] = (
                pos[2] - trigger_z + self.position_endstop - thermal_comp)
        finally:
            if z_was_unhomed:
                toolhead.get_kinematics().clear_homing_state('z')

        toolhead.set_position(pos, homing_axes='z')
        pos = toolhead.get_position()
        pos[2] = _POST_HOME_LIFT
        toolhead.manual_move([None, None, pos[2]], self.lift_speed)
        toolhead.wait_moves()
        gcode_move = self.printer.lookup_object('gcode_move', None)
        if gcode_move is not None:
            gcode_move.reset_last_position()
        msg = (
            "prtouch: Z home at (%.3f, %.3f) z=%.4f "
            "(median=%.4f range=%.4f attempts=%d)"
            % (pos[0], pos[1], self.position_endstop - thermal_comp,
               trigger_z, _sample_range(best), len(collected)))
        if print_temp is not None:
            msg += (
                " thermal=%.4fmm (touch=%.1fC print=%.1fC rate=%.6fmm/C)"
                % (thermal_comp, touch_temp, print_temp,
                   self.thermal_expansion))
        gcmd.respond_info(msg)

    def _retract_home_sample(self, gcmd):
        retract = gcmd.get_float(
            'SAMPLE_RETRACT_DIST', self.sample_retract_dist, above=0.)
        lift_speed = gcmd.get_float('LIFT_SPEED', self.lift_speed, above=0.)
        toolhead = self.printer.lookup_object('toolhead')
        pos = toolhead.get_position()
        toolhead.manual_move([None, None, pos[2] + retract], lift_speed)

    def _zone_manual_move(self, coord, speed):
        self.printer.lookup_object(
            'extended_zone_transform').manual_move(coord, speed)

    def _scrub_probe_line(self, toolhead, probe_gcmd, start, end):
        values = []
        for point in (start, end):
            self._zone_manual_move(
                [point[0], point[1], None], self.home_travel_speed)
            toolhead.wait_moves()
            values.append(self._probe_one(probe_gcmd)[2])
            self._retract_home_sample(probe_gcmd)
        if abs(values[1] - values[0]) > .5:
            raise self.printer.command_error(
                "prtouch: scrub endpoints differ by more than 0.500mm")
        return values

    def _scrub_round_trip(
            self, toolhead, start, end, z_start, z_center, z_end, depth):
        center = [
            .5 * (start[0] + end[0]),
            .5 * (start[1] + end[1]),
        ]
        clear_z = max(z_start, z_center, z_end) + self.sample_retract_dist
        self._zone_manual_move([None, None, clear_z], self.lift_speed)
        self._zone_manual_move(
            [start[0], start[1], None], self.home_travel_speed)
        self._zone_manual_move(
            [None, None, z_start - depth], self.lift_speed)
        toolhead.wait_moves()
        self._zone_manual_move(
            [center[0], center[1], z_center - depth], self.scrub_speed)
        self._zone_manual_move(
            [end[0], end[1], z_end - depth], self.scrub_speed)
        self._zone_manual_move(
            [center[0], center[1], z_center - depth], self.scrub_speed)
        self._zone_manual_move(
            [start[0], start[1], z_start - depth], self.scrub_speed)
        toolhead.wait_moves()
        self._zone_manual_move([None, None, clear_z], self.lift_speed)
        toolhead.wait_moves()

    def _scrub_lift(self, toolhead):
        pos = toolhead.get_position()
        z_max = self.config.getsection('stepper_z').getfloat('position_max')
        lift_z = min(pos[2] + self.sample_retract_dist, z_max)
        if lift_z > pos[2]:
            self._zone_manual_move([None, None, lift_z], self.lift_speed)
            toolhead.wait_moves()

    cmd_PRTOUCH_SCRUB_help = (
        "Detect the flexible rear bed tab and scrub the nozzle against it.")

    def cmd_PRTOUCH_SCRUB(self, gcmd):
        self._require_probe()
        self.gcode.run_script_from_command("HOME_IF_NEEDED AXIS=XYZ")
        toolhead = self.printer.lookup_object('toolhead')
        # Tap analysis reports where contact began, which is the same for
        # both forces on the tab. Scrub uses the trigger heights instead, as
        # the stock detector reported and the scrub thresholds were tuned on.
        raw = {'DISABLE_PULLBACK_MOVE': '1'}
        probe_gcmd = self._sample_gcmd('PRTOUCH_SCRUB_PROBE', raw)
        high_force = min(250, int(round(
            self.trigger_force * self.scrub_detect_hold_ratio)))
        hard_gcmd = self._sample_gcmd(
            'PRTOUCH_SCRUB_PROBE', dict(raw, TRIGGER_FORCE=str(high_force)))
        try:
            y = random.uniform(self.scrub_y_min, self.scrub_y_max)
            start = (self.scrub_x_start, y)
            end = (self.scrub_x_end, y)
            self._scrub_lift(toolhead)

            center = [
                .5 * (start[0] + end[0]),
                .5 * (start[1] + end[1]),
            ]
            self._zone_manual_move(
                [center[0], center[1], None], self.home_travel_speed)
            toolhead.wait_moves()
            deflections = []
            normal_zs = []
            for _sample in range(3):
                normal_z = self._probe_one(probe_gcmd)[2]
                normal_zs.append(normal_z)
                self._retract_home_sample(probe_gcmd)
                high_z = self._probe_one(hard_gcmd)[2]
                self._retract_home_sample(probe_gcmd)
                deflections.append(abs(normal_z - high_z))
            deflection = round(float(median(deflections)), 6)
            tab_detected = deflection > self.scrub_detect_deflection
            depth = (
                self.scrub_tab_depth if tab_detected
                else self.scrub_no_tab_depth)
            gcmd.respond_info(
                "prtouch: %s: deflection %.4fmm %s %.4fmm; "
                "wiping %.3fmm deep at %.1fmm/s"
                % ("tab detected" if tab_detected else "no tab",
                   deflection, ">" if tab_detected else "<=",
                   self.scrub_detect_deflection, depth, self.scrub_speed))

            z_start, z_end = self._scrub_probe_line(
                toolhead, probe_gcmd, start, end)
            z_center = float(median(normal_zs))
            center_offset = z_center - .5 * (z_start + z_end)
            passes = 0
            shortened = 0.
            for passes in range(1, 4):
                self._scrub_round_trip(
                    toolhead, start, end, z_start, z_center, z_end, depth)
                next_start, next_end = self._scrub_probe_line(
                    toolhead, probe_gcmd, start, end)
                shortened = (
                    .5 * (z_start + z_end)
                    - .5 * (next_start + next_end))
                z_start, z_end = next_start, next_end
                z_center = .5 * (z_start + z_end) + center_offset
                if shortened < .01:
                    break
            gcmd.respond_info(
                "prtouch: scrub tab=%s deflection=%.4fmm depth=%.3fmm "
                "passes=%d shorten=%.4fmm"
                % (tab_detected, deflection, depth, passes, shortened))
        except self.printer.command_error as error:
            homed = toolhead.get_status(
                self.reactor.monotonic())['homed_axes']
            if self.printer.is_shutdown() or not all(
                    axis in homed for axis in 'xyz'):
                raise
            gcmd.respond_info("prtouch: scrub stopped: %s" % (error,))
        finally:
            try:
                self._scrub_lift(toolhead)
            except Exception:
                logging.exception("prtouch: scrub cleanup lift")


def load_config(config):
    return PRTouch(config)
