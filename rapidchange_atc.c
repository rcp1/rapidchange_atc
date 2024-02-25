/*
  rapidchange_atc.c - Tool change routine to support Rapidchange magazine

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

#include <string.h>
#include <stdio.h>

#include "grbl/hal.h"
#include "grbl/motion_control.h"
#include "grbl/protocol.h"
#include "grbl/nvs_buffer.h"
#include "grbl/nuts_bolts.h"

#include "rapidchange_atc.h"

typedef struct {
    char     alignment;
    char     direction;
    uint8_t  number_of_pockets;
    float    pocket_offset;
    float    pocket_1_x_pos;
    float    pocket_1_y_pos;
    float    pocket_z_start;
    float    pocket_z_retract;
    float    pocket_z_engage;
    float    pocket_z_traverse;
    float    pocket_z_safe_clearance;
    float    pocket_feed_rate;
    float    pocket_rpm;
    bool     tool_setter;
    bool     tool_recognition;
    bool     dust_cover;
} atc_settings_t;

static volatile bool execute_posted = false;
static volatile uint32_t spin_lock = 0;
static nvs_address_t nvs_address;
static atc_settings_t atc;
static tool_data_t current_tool, *next_tool = NULL;
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;

static const setting_group_detail_t atc_groups [] = {
    { Group_Root, Group_UserSettings, "RapidChange ATC"}
};

static const setting_detail_t atc_settings[] = {
    { 900, Group_UserSettings, "Alignment", "Axis", Format_RadioButtons, "X,Y", NULL, NULL, Setting_NonCore, &atc.alignment, NULL, NULL },
    { 901, Group_UserSettings, "Direction", NULL, Format_RadioButtons, "Positive,Negative", NULL, NULL, Setting_NonCore, &atc.direction, NULL, NULL },
    { 902, Group_UserSettings, "Number of tool pockets", NULL, Format_Int8, "#00", "0", "9999", Setting_NonCore, &atc.number_of_pockets, NULL, NULL },
    { 903, Group_UserSettings, "Pocket Offset", "mm", Format_Int16, "###0", "0",  "9999.999", Setting_NonCore, &atc.pocket_offset, NULL, NULL },
    { 904, Group_UserSettings, "Pocket 1 X Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_1_x_pos, NULL, NULL },
    { 905, Group_UserSettings, "Pocket 1 Y Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_1_y_pos, NULL, NULL },
    { 910, Group_UserSettings, "Pocket Z Start", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_z_start, NULL, NULL },
    { 911, Group_UserSettings, "Pocket Z Retract", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_z_retract, NULL, NULL },
    { 912, Group_UserSettings, "Pocket Z Engage", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_z_engage, NULL, NULL },
    { 913, Group_UserSettings, "Pocket Z Traverse", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_z_traverse, NULL, NULL },
    { 914, Group_UserSettings, "Pocket Z Safe Clearance", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.pocket_z_safe_clearance, NULL, NULL },
    { 920, Group_UserSettings, "Pocket Engage Feed Rate", "mm/min", Format_Int16, "###0", "0", "3000", Setting_NonCore, &atc.pocket_feed_rate, NULL, NULL },
    { 921, Group_UserSettings, "Pocket Engage Spindle RPM", "rpm", Format_Int16, "###0", "0", "24000", Setting_NonCore, &atc.pocket_rpm, NULL, NULL },
    { 930, Group_UserSettings, "Tool Setter", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.tool_setter, NULL, NULL },
    { 940, Group_UserSettings, "Tool Recognition", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.tool_recognition, NULL, NULL },
    { 950, Group_UserSettings, "Dust Cover", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.dust_cover, NULL, NULL },
};

#ifndef NO_SETTINGS_DESCRIPTIONS

static const setting_descr_t atc_descriptions[] = {
    { 900, "Value: X Axis or Y Axis\\n\\nThe axis along which the tool pockets of the magazine are aligned in the XY plane." },
    { 901, "Value: Positive or Negative\\n\\nThe direction of travel along the alignment axis from pocket 1 to pocket 2, either positive or negative." },
    { 902, "Value: Count\\n\\nThe total number of pockets in the magazine that may be occupied by a tool." },
    { 903, "Value: Distance (mm)\\n\\nThe distance from one pocket to the next when measuring from center to center." },
    { 904, "Value: X Machine Coordinate (mm)\\n\\nThe X axis position referencing the center of the first tool pocket." },
    { 905, "Value: Y Machine Coordinate (mm)\\n\\nThe Y axis position referencing the center of the first tool pocket." },
    { 910, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the spindle is started for (dis-)engagement." },
    { 911, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the spindle is retracted between engagement." },
    { 912, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position to which the spindle plunges when engaging the clamping nut." },
    { 913, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the spindle traverses the magazine between dropping off and picking up a tool." },
    { 914, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for safe clearances of all obstacles." },
    { 920, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the spindle moves when (dis-)engaging the clamping nut." },
    { 921, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle when loading or unloading a tool." },
    { 930, "Value: Enabled or Disabled\\n\\nAllows for enabling or disabling setting the tool offset during a tool change. This can be useful when configuring your magazine or performing diagnostics to shorten the tool change cycle." },
    { 940, "Value: Enabled or Disabled\\n\\nEnables or disables tool recognition as part of an automatic tool change. If tool recognition is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
    { 950, "Value: Enabled or Disabled\\n\\nEnables or disables the dust cover. If a dust cover is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
};

#endif

// Write settings to non volatile storage (NVS).
static void atc_settings_save (void)
{
    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

// Load settings from volatile storage (NVS)
static void atc_settings_load (void)
{
    if(hal.nvs.memcpy_from_nvs((uint8_t *)&atc, nvs_address, sizeof(atc_settings_t), true) != NVS_TransferResult_OK)
        atc_settings_restore();
}

// Restore default settings and write to non volatile storage (NVS).
static void atc_settings_restore (void)
{
    memset(&atc, 0, sizeof(atc_settings_t));
    atc.pocket_offset = 0.0f;
    atc.pocket_1_x_pos = 0.0f;
    atc.pocket_1_y_pos = 0.0f;
    atc.pocket_z_start = 0.0f;
    atc.pocket_z_retract = 0.0f;
    atc.pocket_z_engage = 0.0f;
    atc.pocket_z_traverse = 0.0f;
    atc.pocket_z_safe_clearance = 0.0f;
    atc.pocket_feed_rate = 0.0f;
    atc.pocket_rpm = 0.0f;

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

static setting_details_t setting_details = {
    .groups = atc_groups,
    .n_groups = sizeof(atc_groups) / sizeof(setting_group_detail_t),
    .settings = atc_settings,
    .n_settings = sizeof(atc_settings) / sizeof(setting_detail_t),
#ifndef NO_SETTINGS_DESCRIPTIONS
    .descriptions = atc_descriptions,
    .n_descriptions = sizeof(atc_descriptions) / sizeof(setting_descr_t),
#endif
    .save = atc_settings_save,
    .load = atc_settings_load,
    .restore = atc_settings_restore
};

// Return X,Y based on tool number
static coord_data_t get_tool_location(tool_data_t tool) {
    coord_data_t target = {0};
    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct
    target.x = atc.pocket_1_x_pos;
    target.y = atc.pocket_1_y_pos;

    int8_t multiplier = atc.direction ? -1 : 1;
    float tool_offset = (tool.tool_id - 1) * atc.pocket_offset * multiplier;

    if(atc.alignment == 0) { // X Axis
        target.x = atc.pocket_1_x_pos + tool_offset;
    } else {
        target.y = atc.pocket_1_y_pos + tool_offset;
    }

    return target;
}

//     protocol_buffer_synchronize();


// Reset claimed HAL entry points and restore previous tool if needed on soft restart.
// Called from EXEC_RESET and EXEC_STOP handlers (via HAL).
static void reset (void)
{
    if(next_tool) { //TODO: move to gc_xxx() function?
        // Restore previous tool if reset is during change

        if(current_tool.tool_id != next_tool->tool_id) {
            memcpy(next_tool, &current_tool, sizeof(tool_data_t));
            system_add_rt_report(Report_Tool);
        }

        gc_state.tool_pending = gc_state.tool->tool_id;
        next_tool = NULL;
    }

    driver_reset();
}

// Set next and/or current tool. Called by gcode.c on on a Tn or M61 command (via HAL).
static void tool_select (tool_data_t *tool, bool next)
{
    next_tool = tool;
    if(!next)
        memcpy(&current_tool, tool, sizeof(tool_data_t));
}

static status_code_t spindle(bool load) {

    debug_output(load ? "Loading" : "Unloading", NULL, NULL);
    coord_data_t target = {0}, current_pos;
    plan_line_data_t plan_data;

    if(current_tool.tool_id == 0 && !load) {
        debug_output("No tool to unload", NULL, NULL);
        return Status_OK;
    }

    if(next_tool->tool_id > atc.number_of_pockets) {
        debug_output("Tool number is larger than pocket. Manual Tool Change", NULL, NULL);
        if(load) {
            manualToolLoad();
        } else {
            manualToolUnLoad();
        }
        return Status_OK;
    }

    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct
    plan_data_init(&plan_data);

    // Lets stop the spindle and set the feed rate for all moves.
    plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){0}, 0.0f);
    plan_data.feed_rate = atc.pocket_feed_rate;
    plan_data.condition.rapid_motion = Off;

    system_convert_array_steps_to_mpos(current_pos.values, sys.position);
    debug_output("Getting Current POS", &current_pos, &plan_data);

    // Raise Z to safe clearance
    target = current_pos;
    target.z = atc.pocket_z_safe_clearance;
    debug_output("Raising Z to Clearance Height", NULL, &plan_data);
    mc_line(target.values, &plan_data);

    // Get X,Y for current tool and move to that position
    target = get_tool_location((load?*next_tool:current_tool));
    target.z = atc.pocket_z_safe_clearance;
    debug_output("Determine tool position and go there", &target, &plan_data);
    mc_line(target.values, &plan_data);

    target.z = atc.pocket_z_start;
    debug_output("Going to Spindle Start Height", &target, &plan_data);
    mc_line(target.values, &plan_data);

    // Turn on the spindle CCW
    if(load) {
        plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){ .on = On }, atc.pocket_rpm);
    } else {
        plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){ .on = On, .ccw = On }, atc.pocket_rpm);
    }
    hal.delay_ms(3000, NULL);

    // move to engagement height
    target.z = atc.pocket_z_engage;
    debug_output("Turning on spindle and moving to engagement height", &target, &plan_data);
    mc_line(target.values, &plan_data);

    // Bring Spindle up
    target.z = atc.pocket_z_safe_clearance;
    mc_line(target.values, &plan_data);

    // Turn Spindle off
    plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t)(spindle_state_t){ .on = Off }, 0.0f);
    debug_output("Stopping spindle and raising to clearance height", &target, &plan_data);
    mc_line(target.values, &plan_data);

    debug_output("Updating current tool", NULL, NULL);

    if(load) {
        memset(&current_tool, 0, sizeof(tool_data_t));
    } else {
        memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    }

    protocol_buffer_synchronize();

    return Status_OK;

}

static void manualToolLoad() {

}

static void manualToolUnLoad() {

}

static void measureTool() {
    coord_data_t current_pos;

    system_convert_array_steps_to_mpos(current_pos.values, sys.position);

}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static status_code_t tool_change (parser_state_t *parser_state)
{
    if(next_tool == NULL)
        return Status_GCodeToolError;

    if(current_tool.tool_id == next_tool->tool_id)
        return Status_OK;

#ifndef DEBUG
    uint8_t homed_req =  (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT);

    if((sys.homed.mask & homed_req) != homed_req)
        return Status_HomingRequired;
#endif

    // Save current position
    coord_data_t previous;
    system_convert_array_steps_to_mpos(previous.values, sys.position);

    debug_output("Turning off Coolant", NULL, NULL);

    // Stop spindle and coolant
    hal.coolant.set_state((coolant_state_t){0});

    debug_output("Check if we need to unload tool", NULL, NULL);
    spindle(false);
    debug_output("Check if we need to load a tool", NULL, NULL);
    spindle(true);
    debug_output("Check if we need to measure a tool", NULL, NULL);
    measureTool();


    return Status_OK;
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN: RapidChange ATC v0.01]" ASCII_EOL);
    }
}

// Claim HAL tool change entry points and clear current tool offsets.
void my_plugin_init (void)
{
    protocol_enqueue_foreground_task(report_info, "RapidChange ATC plugin trying to initialize!");
    hal.driver_cap.atc = On;

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    if(sys.tlo_reference_set.mask != 0) {
        sys.tlo_reference_set.mask = 0;
        system_add_rt_report(Report_TLOReference);
    }

    gc_set_tool_offset(ToolLengthOffset_Cancel, 0, 0.0f);

    hal.tool.select = tool_select;
    hal.tool.change = tool_change;

    if((nvs_address = nvs_alloc(sizeof(atc_settings_t)))) {
         settings_register(&setting_details);
    } else {
        protocol_enqueue_foreground_task(report_warning, "RapidChange ATC plugin failed to initialize, no NVS storage for settings!");
    }

    if(driver_reset == NULL) {
        driver_reset = hal.driver_reset;
        hal.driver_reset = reset;
    }
}

void debug_output(char* message, coord_data_t *target, plan_line_data_t *pl_data) {

#ifdef DEBUG
    hal.stream.write("[R-ATC]: ");
    hal.stream.write(message);
    hal.stream.write(ASCII_EOL);

    if(target != NULL) {
        hal.stream.write(ASCII_EOL);
        hal.stream.write("Target:" ASCII_EOL);
        hal.stream.write("X: ");
        hal.stream.write( ftoa(target->x, 3) );
        hal.stream.write(ASCII_EOL);
        hal.stream.write("y: ");
        hal.stream.write( ftoa(target->y, 3) );
        hal.stream.write(ASCII_EOL);
        hal.stream.write("z: ");
        hal.stream.write( ftoa(target->z, 3) );
        hal.stream.write(ASCII_EOL);
    }

    if(pl_data != NULL) {
        hal.stream.write(ASCII_EOL "Plan:" ASCII_EOL);
        hal.stream.write("Feed Rate:");
        hal.stream.write(ftoa(pl_data->feed_rate,3));
        hal.stream.write(ASCII_EOL);
        hal.stream.write("Spindle RPM:");
        hal.stream.write(ftoa(pl_data->spindle.rpm,3));
        hal.stream.write(ASCII_EOL);
        hal.stream.write("Spindle State:");
        char buffer[8U] = ""; /*the output buffer*/

        sprintf (buffer, "%d", pl_data->spindle.state.value);
        hal.stream.write(buffer);
        hal.stream.write(ASCII_EOL);

        hal.stream.write(ASCII_EOL);
    }
#endif
}
