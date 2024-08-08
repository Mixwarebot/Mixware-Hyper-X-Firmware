/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2020 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
 *
 * Based on Sprinter and grbl.
 * Copyright (c) 2011 Camiel Gubbels / Erik van der Zalm
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <https://www.gnu.org/licenses/>.
 *
 */

/**
 * G429.cpp -
 */

#include "../../inc/MarlinConfig.h"

#include "../gcode.h"

#include "../../module/endstops.h"
#include "../../module/planner.h"
#include "../../module/probe.h"

#if HAS_LEVELING
#include "../../feature/bedlevel/bedlevel.h"
#endif

#if HAS_MULTI_HOTEND
#include "../../module/tool_change.h"
#endif

const xyz_pos_t start_pos[XY] = {XYOC_START_POS_X, XYOC_START_POS_Y};
const xyz_pos_t end_pos[XY] = {
    {start_pos[0].x - XYOC_PROBE_CLEARANCE, start_pos[0].y, start_pos[0].z - XYOC_PROBE_HEIGHT},
    {start_pos[1].x, start_pos[1].y - XYOC_PROBE_CLEARANCE, start_pos[1].z - XYOC_PROBE_HEIGHT}
};

float xypoc_measure(const AxisEnum axis, bool retract) {

    do_blocking_move_to(start_pos[axis], feedRate_t(MMM_TO_MMS(XYOC_FEEDRATE_TRAVEL))); // Measure the starting point.
    do_blocking_move_to_z(end_pos[axis].z, feedRate_t(MMM_TO_MMS(XYOC_FEEDRATE_FAST)));
    endstops.enable_xypoc(true); // Enable endstops for next homing move

    // Move down until the probe is triggered
    if (axis == 0)
        do_blocking_move_to_x(end_pos[axis].x, feedRate_t(MMM_TO_MMS(XYOC_FEEDRATE_SLOW))); // Measure the end point.
    else
        do_blocking_move_to_y(end_pos[axis].y, feedRate_t(MMM_TO_MMS(XYOC_FEEDRATE_SLOW))); // Measure the end point.

    // Clear endstop flags
    endstops.hit_on_purpose();

    // Get Z where the steppers were interrupted
    set_current_from_steppers_for_axis(ALL_AXES_ENUM);

    // Tell the planner where we actually are
    sync_plan_position();

    // Return the single probe result
    const float measured = axis == 0 ? current_position.x : current_position.y;

    endstops.enable_xypoc(false);
    if (retract) {
        const uint8_t steps = 5;
        const xy_pos_t probe_after_pos[XY][steps] = {
            {
                {measured + XYOC_SAFETY_GAP, start_pos[axis].y},
                {measured + XYOC_SAFETY_GAP, start_pos[axis].y + XYOC_WIDTH},
                {measured - XYOC_NOZZLE_WIDTH, start_pos[axis].y + XYOC_WIDTH},
                {measured - XYOC_NOZZLE_WIDTH, start_pos[axis].y},
                {start_pos[axis].x - XYOC_WIDTH, start_pos[axis].y},
            },
            {
                {start_pos[axis].x, measured + XYOC_SAFETY_GAP},
                {start_pos[axis].x + XYOC_WIDTH, measured + XYOC_SAFETY_GAP},
                {start_pos[axis].x + XYOC_WIDTH, measured - XYOC_NOZZLE_WIDTH},
                {start_pos[axis].x, measured - XYOC_NOZZLE_WIDTH},
                {start_pos[axis].x, start_pos[axis].y - XYOC_WIDTH},
            }};

        for (uint8_t i = 0; i < steps; i++)
            do_blocking_move_to_xy(probe_after_pos[axis][i], feedRate_t(MMM_TO_MMS(XYOC_FEEDRATE_FAST)));
        do_blocking_move_to_z(start_pos[axis].z, feedRate_t(MMM_TO_MMS(XYOC_FEEDRATE_FAST)));
    }

    return measured;
}

void GcodeSuite::G429() {

    const bool seenX = parser.seen_test('X'),
               seenY = parser.seen_test('Y'),
               no_action = !(seenX || seenY);
    const bool seenC = parser.seenval('C');
    const int count = seenC ? parser.value_int() : 1;

    if (no_action) {
        SERIAL_ECHO_START();
        SERIAL_ECHO("XYPOC: Unspecified axis[X|Y].");
        SERIAL_EOL();
        return;
    }

    if (seenX || seenY) {

        // Keep powered steppers from timing out
        reset_stepper_timeout();

        #if ENABLED(DUAL_X_CARRIAGE)
            bool IDEX_saved_duplication_state = extruder_duplication_enabled;
            DualXMode IDEX_saved_mode = dual_x_carriage_mode;
        #endif
        // Send 'N' to force homing before G29 (internal only)
        if (parser.seen_test('N'))
            process_subcommands_now(F("G28L0"));

        // Don't allow auto-leveling without homing first
        if (homing_needed_error())
            return;

        planner.synchronize(); // Wait for planner moves to finish!
        TERN_(HAS_DUPLICATION_MODE, set_duplication_enabled(false));
        TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

        remember_feedrate_scaling_off();

        const AxisEnum axis = seenY ? Y_AXIS : X_AXIS;
        const xy_pos_t tool_change_pos = {0, start_pos[axis].y};

        if (active_extruder != 0) tool_change(0, true);
        float measured_left = xypoc_measure(axis, true);

        if (count == 2) {
            measured_left = (xypoc_measure(axis, true) * 2 + measured_left * 3) / 5.0;
        }
        else if (count > 2) {
            for (uint8_t i = 1; i < count; i++)
                measured_left += xypoc_measure(axis, true);
            measured_left /= count;
        }

        if (parser.seen_test('H')) {
            process_subcommands_now(F("G28"));
            planner.synchronize(); // Wait for planner moves to finish!
        }
        do_blocking_move_to_xy(tool_change_pos, feedRate_t(MMM_TO_MMS(XYOC_FEEDRATE_TRAVEL)));

        if (active_extruder != 1) tool_change(1, true);
        float measured_right = xypoc_measure(axis, true);

        if (count == 2) {
            measured_right = (xypoc_measure(axis, true) * 2 + measured_right * 3) / 5.0;
        }
        else if (count > 2) {
            for (uint8_t i = 1; i < count; i++)
                measured_right += xypoc_measure(axis, true);
            measured_right /= count;
        }

        // Restore state after probing
        restore_feedrate_and_scaling();

        #if ENABLED(DUAL_X_CARRIAGE)
            dual_x_carriage_mode = IDEX_saved_mode;
            set_duplication_enabled(IDEX_saved_duplication_state);
        #endif
        TERN_(HAS_LEVELING, set_bed_leveling_enabled());

        float measured_offset = measured_left - measured_right;

        SERIAL_ECHO_START();
        SERIAL_ECHOLNPGM(seenX ? "XYPOC measured X:" : "XYPOC measured Y:", measured_offset);
        SERIAL_EOL();
    }
}