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

void GcodeSuite::G429() {

  const bool seenX = parser.seen_test('X'),
             seenY = parser.seen_test('Y'),
             no_action = seenX || seenY;

  if (!no_action || seenX) {

    // Keep powered steppers from timing out
    reset_stepper_timeout();

    #if ENABLED(DUAL_X_CARRIAGE)
        bool IDEX_saved_duplication_state = extruder_duplication_enabled;
        DualXMode IDEX_saved_mode = dual_x_carriage_mode;
    #endif

    // Send 'N' to force homing before G29 (internal only)
    if (parser.seen_test('N'))
        process_subcommands_now(TERN(CAN_SET_LEVELING_AFTER_G28, F("G28L0"), FPSTR(G28_STR)));

    // Don't allow auto-leveling without homing first
    if (homing_needed_error()) return;

    planner.synchronize();          // Wait for planner moves to finish!

    if (active_extruder != 0) tool_change(0, true);
    TERN_(HAS_DUPLICATION_MODE, set_duplication_enabled(false));
    // TERN_(HAS_LEVELING, set_bed_leveling_enabled(false));

    remember_feedrate_scaling_off();

    xyz_pos_t start_pos[XY] = {XYPOC_START_X, XYPOC_START_Y};
    xyz_pos_t end_pos[XY] = {
        {start_pos[0].x - XYPOC_PROBE_CLEARANCE, start_pos[0].y, start_pos[0].z - XYPOC_PROBE_HEIGHT},
        {start_pos[1].x, start_pos[1].y - XYPOC_PROBE_CLEARANCE, start_pos[1].z - XYPOC_PROBE_HEIGHT}
    };

    do_blocking_move_to(start_pos[0], feedRate_t(MMM_TO_MMS(XYPOC_FEEDRATE_TRAVEL)));// Measure the starting point.
    endstops.enable(true); // Enable endstops for next homing move

    // Move down until the probe is triggered
    do_blocking_move_to_x(end_pos[0].x, feedRate_t(MMM_TO_MMS(XYPOC_FEEDRATE_SLOW)));// Measure the end point.

    // Clear endstop flags
    endstops.hit_on_purpose();

    // Get Z where the steppers were interrupted
    set_current_from_steppers_for_axis(ALL_AXES_ENUM);

    // Tell the planner where we actually are
    sync_plan_position();

    // Return the single probe result
    const float c_x = current_position.x,
                c_y = current_position.y;
    endstops.enable(false);

    xy_pos_t probe_after_pos[5] = {
        {c_x + 1, start_pos[0].y},
        {c_x + 1, start_pos[0].y + 30},
        {c_x - 15, start_pos[0].y + 30},
        {c_x - 15, start_pos[0].y},
        {start_pos[0].x - 25, start_pos[0].y},
    };
    do_blocking_move_to_xy(probe_after_pos[0], feedRate_t(MMM_TO_MMS(XYPOC_FEEDRATE_FAST)));
    do_blocking_move_to_xy(probe_after_pos[1], feedRate_t(MMM_TO_MMS(XYPOC_FEEDRATE_FAST)));
    do_blocking_move_to_xy(probe_after_pos[2], feedRate_t(MMM_TO_MMS(XYPOC_FEEDRATE_FAST)));
    do_blocking_move_to_xy(probe_after_pos[3], feedRate_t(MMM_TO_MMS(XYPOC_FEEDRATE_FAST)));
    do_blocking_move_to_xy(probe_after_pos[4], feedRate_t(MMM_TO_MMS(XYPOC_FEEDRATE_FAST)));
    do_blocking_move_to_z(start_pos[0].z, feedRate_t(MMM_TO_MMS(XYPOC_FEEDRATE_FAST)));

    // Restore state after probing
    restore_feedrate_and_scaling();
  }
}