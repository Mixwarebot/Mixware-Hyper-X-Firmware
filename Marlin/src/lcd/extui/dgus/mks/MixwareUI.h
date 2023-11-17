/**
 * Marlin 3D Printer Firmware
 * Copyright (c) 2022 MarlinFirmware [https://github.com/MarlinFirmware/Marlin]
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
#pragma once

#include "../../../../inc/MarlinConfig.h"
#include "../../mks_ui/mixware_ui/language/language.h"

#define MIX_DEFAULT_Kp_H     15.16
#define MIX_DEFAULT_Ki_H      0.91
#define MIX_DEFAULT_Kd_H     55.20

#define MTR_RELOADING_INF1_EN   "Heating, please wait..."
#define MTR_RELOADING_INF2_EN   "Loading, please wait..."
#define MTR_RELOADING_INF3_EN   "Unloading, please wait..."
#define MTR_RELOADING_INF4_EN   "Done!"

#define MTR_CANCEL_EN "Cancel"
#define MTR_CONFIRM_EN "Confirm"

#define MTR_RELOADING_INF1_CN   { 0xFDD5, 0xDAD4, 0xD3BC, 0xC8C8, 0x202C, 0xEBC7, 0xD4C9, 0xC8B5, 0x2000 }
#define MTR_RELOADING_INF2_CN   { 0xFDD5, 0xDAD4, 0xF8BD, 0xCFC1, 0x202C, 0xEBC7, 0xD4C9, 0xC8B5, 0x2000 }
#define MTR_RELOADING_INF3_CN   { 0xFDD5, 0xDAD4, 0xCBCD, 0xCFC1, 0x202C, 0xEBC7, 0xD4C9, 0xC8B5, 0x2000 }
#define MTR_RELOADING_INF4_CN   { 0xEACD, 0xC9B3, 0x2000 }

#define MTR_CANCEL_CN { 0xA1C8, 0xFBCF, 0x2000 }
#define MTR_CONFIRM_CN { 0xB7C8, 0xCFC8, 0x2000 }

typedef enum {
    LEVEL_STATE_NULL,
    LEVEL_STATE_HOMEING,
    LEVEL_STATE_LEVELING,
    LEVEL_STATE_LEVELERR,
    LEVEL_STATE_FINISHED
} MixwareUI_LevelState;

class MixwareUI_Level
{
public:
    MixwareUI_Level() { state = LEVEL_STATE_NULL; }

    inline void set_auto_leveling_state(MixwareUI_LevelState ls) { if (state != ls) state = ls; }
    inline MixwareUI_LevelState get_auto_leveling_state() { return state; }

// private:
    MixwareUI_LevelState state;
};

extern MixwareUI_Level mix_level;
