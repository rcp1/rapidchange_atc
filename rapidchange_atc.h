/*
  rapidchange_atc.h - Tool change routine to support Rapidchange magazine

  Part of grblHAL

  Copyright (c) 2024 rvalotta

  Grbl is free software: you can redistribute it and/or modify
  it under the terms of the GNU General Public License as published by
  the Free Software Foundation, either version 3 of the License, or
  (at your option) any later version.

  Grbl is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
  GNU General Public License for more details.

  You should have received a copy of the GNU General Public License
  along with Grbl.  If not, see <http://www.gnu.org/licenses/>.
*/

#ifndef _RAPIDCHANGE_ATC_H_
#define _RAPIDCHANGE_ATC_H_

// Used to disable some functionality while developing
#define DEBUG 0

// Hal settings API
static void atc_settings_save (void);
static void atc_settings_load (void);
static void atc_settings_restore (void);

// HAL plugin API
static void reset (void);
static void report_options (bool newopt);

// FluidNC port
static coord_data_t calculate_tool_pos (tool_id_t tool_id);
static coord_data_t get_manual_pos (void);
static coord_data_t get_tool_pos (tool_id_t tool_id);
static bool tool_has_pocket (tool_id_t tool_id);

static void spin_cw (float speed);
static void spin_ccw (float speed);
static void spin_stop (void);
static bool rapid_to_pocket_xy (tool_id_t tool_id);
static bool rapid_to_z (float position);
static bool linear_to_z (float position, float feedrate);
static void open_dust_cover (bool open);
static bool load_tool (tool_id_t tool_id);
static bool set_tool (void);
static void record_program_state (void);
static bool restore_program_state (void);
static void set_tool_change_state (void);
static bool unload_tool (void);

// HAL tool change API
static void tool_select (tool_data_t *tool, bool next);
static status_code_t tool_change (parser_state_t *parser_state);
void atc_init (void);
static void debug_output (char* message, coord_data_t *target, plan_line_data_t *pl_data);
#endif
