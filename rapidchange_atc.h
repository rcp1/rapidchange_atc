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
#define DEBUG 1

static void plugin_settings_save (void);
static void plugin_settings_restore (void);
static coord_data_t get_tool_location(tool_data_t tool);
static void plugin_settings_load (void);
static void reset (void);
static void tool_select (tool_data_t *tool, bool next);
static status_code_t tool_change (parser_state_t *parser_state);
static void report_options (bool newopt);
static void manualToolUnLoad ();
static void manualToolLoad ();
static bool laserBlocked();
static void debug_output(char* message, coord_data_t *target, plan_line_data_t *pl_data);
static bool is_setting_available (const setting_detail_t *setting);
#endif
