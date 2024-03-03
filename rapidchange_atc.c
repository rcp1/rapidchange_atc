/*
  rapidchange_atc.c - Tool change routine to support Rapidchange magazine

  Part of grblHAL

  Copyright (c) 2024 rvalotta
  Copyright (c) 2024 rcp1

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

// Used to print debug statements in the normal stream
#define RAPIDCHANGE_DEBUG 1

#if RAPIDCHANGE_DEBUG
#define RAPIDCHANGE_DEBUG_PRINT(message) \
    hal.stream.write("[R-ATC]: "); \
    hal.stream.write(message); \
    hal.stream.write(ASCII_EOL);
#else
#define RAPIDCHANGE_DEBUG_PRINT(...)
#endif


static const char atc_tool_recognition_port_name[] = {
    "RapidChange Tool Recognition"
};

typedef struct {
    char     alignment;
    char     direction;
    uint8_t  number_of_pockets;
    float    pocket_offset;
    float    x_pocket_1;
    float    y_pocket_1;
    float    z_start;
    float    z_retract;
    float    z_engage;
    float    z_traverse;
    float    z_safe_clearance;
    float    engage_feed_rate;
    float    load_rpm;
    float    unload_rpm;
    uint16_t spindle_ramp_time;
    bool     tool_setter;
    float    tool_setter_x;
    float    tool_setter_y;
    float    tool_setter_z_seek_start;
    float    tool_setter_seek_feed_rate;
    float    tool_setter_set_feed_rate;
    float    tool_setter_max_travel;
    float    tool_setter_seek_retreat;
    bool     tool_recognition;
    uint8_t  tool_recognition_port;
    float    tool_recognition_z_zone_1;
    float    tool_recognition_z_zone_2;
    bool     dust_cover;
} atc_settings_t;

static volatile bool execute_posted = false;
static volatile uint32_t spin_lock = 0;
static nvs_address_t nvs_address;
static atc_settings_t atc;
static tool_data_t current_tool = {0}, *next_tool = NULL;
static driver_reset_ptr driver_reset = NULL;
static on_report_options_ptr on_report_options;
static coord_data_t target = {0}, previous;

static uint8_t n_ports;
static char max_port[4] = "0";

static const setting_group_detail_t atc_groups [] = {
    { Group_Root, Group_UserSettings, "RapidChange ATC"}
};

static bool is_setting_available (const setting_detail_t *setting)
{
    bool available = false;

    switch(setting->id) {
        case 941:
            available = atc.tool_recognition_port != 0xFF;;
            break;
        default:
            break;
    }

    return available;
}

static const setting_detail_t atc_settings[] = {
    { 900, Group_UserSettings, "Alignment", "Axis", Format_RadioButtons, "X,Y", NULL, NULL, Setting_NonCore, &atc.alignment, NULL, NULL },
    { 901, Group_UserSettings, "Direction", NULL, Format_RadioButtons, "Positive,Negative", NULL, NULL, Setting_NonCore, &atc.direction, NULL, NULL },
    { 902, Group_UserSettings, "Number of tool pockets", NULL, Format_Int8, "#00", "0", "9999", Setting_NonCore, &atc.number_of_pockets, NULL, NULL },
    { 903, Group_UserSettings, "Pocket Offset", "mm", Format_Decimal, "###0", "0",  "9999.999", Setting_NonCore, &atc.pocket_offset, NULL, NULL },
    { 904, Group_UserSettings, "Pocket 1 X Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.x_pocket_1, NULL, NULL },
    { 905, Group_UserSettings, "Pocket 1 Y Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.y_pocket_1, NULL, NULL },
    { 910, Group_UserSettings, "Pocket Z Start Offset", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_start, NULL, NULL },
    { 911, Group_UserSettings, "Pocket Z Retract Offset", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_retract, NULL, NULL },
    { 912, Group_UserSettings, "Pocket Z Engage", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_engage, NULL, NULL },
    { 913, Group_UserSettings, "Pocket Z Traverse", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_traverse, NULL, NULL },
    { 914, Group_UserSettings, "Pocket Z Safe Clearance", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.z_safe_clearance, NULL, NULL },
    { 920, Group_UserSettings, "Pocket Engage Feed Rate", "mm/min", Format_Decimal, "###0", "0", "10000", Setting_NonCore, &atc.engage_feed_rate, NULL, NULL },
    { 921, Group_UserSettings, "Pocket Load Spindle RPM", "rpm", Format_Decimal, "###0", "0", "10000", Setting_NonCore, &atc.load_rpm, NULL, NULL },
    { 922, Group_UserSettings, "Pocket Unload Spindle RPM", "rpm", Format_Decimal, "###0", "0", "10000", Setting_NonCore, &atc.unload_rpm, NULL, NULL },
    { 923, Group_UserSettings, "Spindle Ramp-up Wait Time", "ms", Format_Int16, "###0", "0", "60000", Setting_NonCore, &atc.spindle_ramp_time, NULL, NULL },
    { 930, Group_UserSettings, "Tool Setter", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.tool_setter, NULL, NULL },
    { 931, Group_UserSettings, "Tool Setter X Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_x, NULL, NULL },
    { 932, Group_UserSettings, "Tool Setter Y Position", "mm", Format_Decimal, "-###0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_y, NULL, NULL },
    { 933, Group_UserSettings, "Tool Setter Z Seek Start", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_z_seek_start, NULL, NULL },
    { 934, Group_UserSettings, "Tool Setter Seek Feed Rate", "mm/min", Format_Decimal, "###0", "0", "10000", Setting_NonCore, &atc.tool_setter_seek_feed_rate, NULL, NULL },
    { 935, Group_UserSettings, "Tool Setter Set Feed Rate", "mm/min", Format_Decimal, "###0", "0", "10000", Setting_NonCore, &atc.tool_setter_set_feed_rate, NULL, NULL },
    { 936, Group_UserSettings, "Tool Setter Max Travel", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_max_travel, NULL, NULL },
    { 937, Group_UserSettings, "Tool Setter Seek Retreat", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_setter_seek_retreat, NULL, NULL },
    { 940, Group_UserSettings, "Tool Recognition", NULL, Format_RadioButtons, "Disabled, Enabled", NULL, NULL, Setting_NonCore, &atc.tool_recognition, NULL, NULL },
    { 941, Group_AuxPorts, "Tool Recognition Port", NULL, Format_Int8, "#0", "0", max_port, Setting_NonCore, &atc.tool_recognition_port, NULL, is_setting_available, { .reboot_required = On } },
    { 942, Group_UserSettings, "Tool Recognition Z Zone 1", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_recognition_z_zone_1, NULL, NULL },
    { 943, Group_UserSettings, "Tool Recognition Z Zone 2", "mm", Format_Decimal, "-##0.000", "-9999.999", "9999.999", Setting_NonCore, &atc.tool_recognition_z_zone_2, NULL, NULL },
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
    { 910, "Value: Z Machine Coordinate Offset (mm)\\n\\nThe Z offset added to Z Engage at which the spindle is started for (dis-)engagement." },
    { 911, "Value: Z Machine Coordinate Offset (mm)\\n\\nThe Z offset added to Z Engage at which the spindle is retracted between engagement." },
    { 912, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position to which the spindle plunges when engaging the clamping nut." },
    { 913, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the spindle traverses the magazine between dropping off and picking up a tool." },
    { 914, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position for safe clearances of all obstacles." },
    { 920, "Value: Feed Rate (mm/min)\\n\\nThe feed rate at which the spindle moves when (dis-)engaging the clamping nut." },
    { 921, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle when loading a tool." },
    { 922, "Value: Spindle Speed (rpm)\\n\\nThe rpm at which to operate the spindle when unloading a tool." },
    { 923, "Value: Spindle Ramp-up Wait Time (ms)\\n\\nThe wait time till the spindle reaches the (un-)load speed." },
    { 930, "Value: Enabled or Disabled\\n\\nAllows for enabling or disabling setting the tool offset during a tool change. This can be useful when configuring your magazine or performing diagnostics to shorten the tool change cycle." },
    { 931, "Value: X Machine Coordinate (mm)\\n\\nThe X axis position referencing the center of the tool setter." },
    { 932, "Value: Y Machine Coordinate (mm)\\n\\nThe Y axis position referencing the center of the tool setter." },
    { 933, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position to which the spindle moves before starting the tool setting probe cycle." },
    { 934, "Value: Feed Rate (mm/min)\\n\\nThe feed rate to quickly find the tool change sensor before the slower locating phase." },
    { 935, "Value: Feed Rate (mm/min)\\n\\nThe feed rate to slowly engage tool change sensor to determine the tool offset accurately." },
    { 936, "Value: Distance (mm)\\n\\nThe maximum probing distance for tool setting." },
    { 937, "Value: Distance (mm)\\n\\nThe pull-off distance for the retract move before the slower locating phase." },
    { 940, "Value: Enabled or Disabled\\n\\nEnables or disables tool recognition as part of an automatic tool change. If tool recognition is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
    { 941, "Aux input port number to use for tool recognition IR sensor." },
    { 942, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the clamping nut breaks the IR beam otherwise the nut is not loaded." },
    { 943, "Value: Z Machine Coordinate (mm)\\n\\nThe Z position at which the clamping nut should not break the IR beam otherwise it is not properly threaded." },
    { 950, "Value: Enabled or Disabled\\n\\nEnables or disables the dust cover. If a dust cover is included with your magazine, be sure to properly configure the appropriate settings before enabling." },
};

#endif

// Hal settings API
// Restore default settings and write to non volatile storage (NVS).
static void atc_settings_restore (void)
{
    memset(&atc, 0, sizeof(atc_settings_t));
    atc.pocket_offset = 45.0f;
    atc.x_pocket_1 = 0.0f;
    atc.y_pocket_1 = 0.0f;
    atc.z_start = 23.0f;
    atc.z_retract = 13.0f;
    atc.z_engage = -10.0f;
    atc.z_traverse = -10.0f;
    atc.z_safe_clearance = -10.0f;
    atc.engage_feed_rate = 1800.0f;
    atc.load_rpm = 1200.0f;
    atc.unload_rpm = 1200.0f;
    atc.tool_setter_z_seek_start = -10.0f;
    atc.tool_setter_seek_feed_rate = DEFAULT_TOOLCHANGE_SEEK_RATE;
    atc.tool_setter_set_feed_rate = DEFAULT_TOOLCHANGE_FEED_RATE;
    atc.tool_setter_max_travel = DEFAULT_TOOLCHANGE_PROBING_DISTANCE;
    atc.tool_setter_seek_retreat = 2.0f;
    atc.tool_recognition_z_zone_1 = -10.0f;
    atc.tool_recognition_z_zone_2 = -10.0f;

    if(n_ports) {
        atc.tool_recognition_port = n_ports - 1;
    }

    hal.nvs.memcpy_to_nvs(nvs_address, (uint8_t *)&atc, sizeof(atc_settings_t), true);
}

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

    bool ok = true;

    if(n_ports)  {
        // Sanity check
        if(atc.tool_recognition_port >= n_ports)
            atc.tool_recognition_port = n_ports - 1;

        if(!(ok = ioport_claim(Port_Digital, Port_Input, &atc.tool_recognition_port, atc_tool_recognition_port_name)))
            atc.tool_recognition_port = 0xFE;
    }

    if(!ok)
        protocol_enqueue_foreground_task(report_warning, "RapidChange ATC plugin: configured port number(s) not available");
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

// HAL plugin API
// Reset claimed HAL entry points and restore previous tool if needed on soft restart.
// Called from EXEC_RESET and EXEC_STOP handlers (via HAL).
static void reset (void)
{
    RAPIDCHANGE_DEBUG_PRINT("Reset.");
    if(next_tool) { //TODO: move to gc_xxx() function?
        // Restore previous tool if reset is during change

        if(current_tool.tool_id != next_tool->tool_id) {
            memcpy(next_tool, &current_tool, sizeof(tool_data_t));
            system_add_rt_report(Report_Tool);
        }
        char tool_msg[20];
        sprintf(tool_msg, "Current tool: %lu", current_tool.tool_id);
        RAPIDCHANGE_DEBUG_PRINT(tool_msg);
        sprintf(tool_msg, "Next tool: %lu", next_tool->tool_id);
        RAPIDCHANGE_DEBUG_PRINT(tool_msg);

        gc_state.tool_pending = gc_state.tool->tool_id;
        next_tool = NULL;
    }

    // change_completed();
    driver_reset();
}

static void report_options (bool newopt)
{
    on_report_options(newopt);

    if(!newopt) {
        hal.stream.write("[PLUGIN: RapidChange ATC v0.01]" ASCII_EOL);
    }
}

// FluidNC port
static coord_data_t calculate_tool_pos (tool_id_t tool_id) {
    coord_data_t target = {0};
    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct
    target.x = atc.x_pocket_1;
    target.y = atc.y_pocket_1;

    int8_t multiplier = atc.direction ? -1 : 1;
    float tool_offset = (tool_id - 1) * atc.pocket_offset * multiplier;

    if(atc.alignment == X_AXIS)
        target.x = atc.x_pocket_1 + tool_offset;
    else if(atc.alignment == Y_AXIS)
        target.y = atc.y_pocket_1 + tool_offset;

    return target;
}

static coord_data_t get_manual_pos (void) {
    coord_data_t target = {0};
    memset(&target, 0, sizeof(coord_data_t)); // Zero plan_data struct

    target.x = atc.tool_setter_x;
    target.y = atc.tool_setter_y;

    return target;
}

static bool tool_has_pocket (tool_id_t tool_id) {
    return tool_id != 0 && tool_id <= atc.number_of_pockets;
}

static coord_data_t get_tool_pos (tool_id_t tool_id) {
    if(tool_has_pocket(tool_id)) {
        return calculate_tool_pos(tool_id);
    } else
        return get_manual_pos();
}

static bool rapid_to_tool_setter_xy() {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    target.x = atc.tool_setter_x;
    target.y = atc.tool_setter_y;
    if(!mc_line(target.values, &plan_data))
        return false;

    return protocol_buffer_synchronize();
}

static bool rapid_to_pocket_xy(tool_id_t tool_id) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    coord_data_t tool = get_tool_pos(tool_id);
    target.x = tool.x;
    target.y = tool.y;
    if(!mc_line(target.values, &plan_data))
        return false;

    return protocol_buffer_synchronize();
}

static bool rapid_to_z(float position) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;
    target.z = position;
    if(!mc_line(target.values, &plan_data))
        return false;

    return protocol_buffer_synchronize();
}

static bool linear_to_z(float position, float feed_rate) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    target.z = position;
    plan_data.feed_rate = feed_rate;
    if(!mc_line(target.values, &plan_data))
        return false;

    return protocol_buffer_synchronize();
}

static void spin_cw(float speed) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){ .on = On}, speed);
    hal.delay_ms(atc.spindle_ramp_time, NULL);
}

static void spin_ccw(float speed) {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){ .on = On, .ccw = On }, speed);
    hal.delay_ms(atc.spindle_ramp_time, NULL);
}

static void spin_stop() {
    plan_line_data_t plan_data;
    plan_data_init(&plan_data);
    plan_data.spindle.hal->set_state(plan_data.spindle.hal, (spindle_state_t){0}, 0.0f);
    hal.delay_ms(atc.spindle_ramp_time, NULL);
}

bool spindle_has_tool() {
    return hal.port.wait_on_input(Port_Digital, atc.tool_recognition_port, WaitMode_Immediate, 0.0f);
}

static void message_start() {
    char tool_msg[20];
    sprintf(tool_msg, "Current tool: %lu", current_tool.tool_id);
    RAPIDCHANGE_DEBUG_PRINT(tool_msg);
    if(next_tool) {
        sprintf(tool_msg, "Next tool: %lu", next_tool->tool_id);
        RAPIDCHANGE_DEBUG_PRINT(tool_msg);
    }
}

static void open_dust_cover(bool open) {
    if(!atc.dust_cover) {
        return;
    }

    // TODO(rcp1) Open dust cover

    // hal.delay_ms(500, NULL);

    return;
}

void record_program_state() {
    // Spindle off and coolant off
    RAPIDCHANGE_DEBUG_PRINT("Turning off spindle");
    spindle_all_off();
    RAPIDCHANGE_DEBUG_PRINT("Turning off coolant");
    hal.coolant.set_state((coolant_state_t){0});
    // Save current position.
    system_convert_array_steps_to_mpos(previous.values, sys.position);
    // Establish axis assignments.
    previous.z -= gc_get_offset(Z_AXIS);
    // Store current position as start
    memcpy(&target, &previous, sizeof(coord_data_t));
}

static bool restore_program_state (void) {
    plan_line_data_t plan_data;

    plan_data_init(&plan_data);
    plan_data.condition.rapid_motion = On;

    target.z = atc.z_safe_clearance;
    mc_line(target.values, &plan_data);

    if(!settings.flags.no_restore_position_after_M6) {
        memcpy(&target, &previous, sizeof(coord_data_t));
        target.z = atc.z_safe_clearance;
        mc_line(target.values, &plan_data);
    }

    if(protocol_buffer_synchronize()) {

        sync_position();

        coolant_sync(gc_state.modal.coolant);
        spindle_restore(plan_data.spindle.hal, gc_state.modal.spindle.state, gc_state.spindle.rpm);

        if(!settings.flags.no_restore_position_after_M6) {
            previous.z += gc_get_offset(Z_AXIS);
            mc_line(previous.values, &plan_data);
        }
    }

    // Already done after load tool
    // if(protocol_buffer_synchronize()) {
    //     sync_position();
    //     memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    // }

    return !ABORTED;
}

static void set_tool_change_state(void) {
    protocol_buffer_synchronize();
    sync_position();
}

static void pause (void) {
    system_set_exec_state_flag(EXEC_FEED_HOLD); // Use feed hold for program pause.
    protocol_execute_realtime(); // Execute suspend
}

static bool unload_tool(void) {
    if(!rapid_to_z(atc.z_safe_clearance))
        return false;

    // If we don't have a tool we're done
    if(current_tool.tool_id == 0) {
        return true;
    }

    // If the tool has a pocket, unload
    if(tool_has_pocket(current_tool.tool_id)) {

        // Perform first attempt
        if(!rapid_to_pocket_xy(current_tool.tool_id))
            return false;

        if(!rapid_to_z(atc.z_engage + atc.z_start))
            return false;
        spin_ccw(atc.unload_rpm);
        if(!linear_to_z(atc.z_engage, atc.engage_feed_rate))
            return false;

        // If we're using tool recognition, handle it
        if(atc.tool_recognition) {
            // TODO(rcp1) Add tool recognition handling
        // If we're not using tool recognition, go straight to traverse height for loading
        } else {
            if(!rapid_to_z(atc.z_traverse))
                return false;
            spin_stop();
        }

    // If the tool doesn't have a pocket, let's pause for manual removal
    } else {
        RAPIDCHANGE_DEBUG_PRINT("Current tool does not have an assigned pocket.");
        RAPIDCHANGE_DEBUG_PRINT("Please unload the tool manually and cycle start to continue.");
        pause();
    }

    // The tool has been removed, set current tool to 0, only set for completeness, not used anywhere
    current_tool.tool_id = 0;

    return true;
}

static bool load_tool(tool_id_t tool_id) {

    // If loading tool 0, we're done
    if(tool_id == 0) {
        return true;
    }

    // If selected tool has a pocket, perform automatic pick up
    if(tool_has_pocket(tool_id)) {
        if(!rapid_to_pocket_xy(tool_id))
            return false;
        if(!rapid_to_z(atc.z_engage + atc.z_start))
            return false;
        spin_cw(atc.load_rpm);
        if(!linear_to_z(atc.z_engage, atc.engage_feed_rate))
            return false;
        if(!rapid_to_z(atc.z_engage + atc.z_retract))
            return false;
        if(!linear_to_z(atc.z_engage, atc.engage_feed_rate))
            return false;

        // If we're using tool recognition, let's handle it
        if(atc.tool_recognition) {
            if (!rapid_to_z(atc.tool_recognition_z_zone_1))
                return false;
            spin_stop();

            // If we don't have a tool rise and pause for a manual load
            if (!spindle_has_tool()) {
                if(!rapid_to_z(atc.z_safe_clearance))
                    return false;
                RAPIDCHANGE_DEBUG_PRINT("FAILED TO LOAD THE SELECTED TOOL.");
                RAPIDCHANGE_DEBUG_PRINT("PLEASE LOAD THE TOOL MANUALLY AND CYCLE START TO CONTINUE.");
                pause();

            // Otherwise we have a tool and can perform the next check
            } else {
                if (rapid_to_z(atc.tool_recognition_z_zone_2))
                    return false;

                // If we show to have a tool here, we cross-threaded and need to manually load
                if (spindle_has_tool()) {
                    if(!rapid_to_z(atc.z_safe_clearance))
                        return false;
                    RAPIDCHANGE_DEBUG_PRINT("FAILED TO PROPERLY THREAD THE SELECTED TOOL.");
                    RAPIDCHANGE_DEBUG_PRINT("PLEASE RELOAD THE TOOL MANUALLY AND CYCLE START TO CONTINUE.");
                    pause();
                } // Otherwise all went well
            }
        }

        if(!rapid_to_z(atc.z_traverse))
            return false;
        spin_stop();

    // Otherwise, there is no pocket so let's rise and pause to load manually
    } else {
        if(!rapid_to_z(atc.z_safe_clearance))
            return false;
        RAPIDCHANGE_DEBUG_PRINT("Selected tool does not have an assigned pocket.");
        RAPIDCHANGE_DEBUG_PRINT("Please load the selected tool and press cycle start to continue.");
        pause();
    }

    // We've loaded our tool
    if(protocol_buffer_synchronize()) {
        sync_position();
        memcpy(&current_tool, next_tool, sizeof(tool_data_t));
    }

    return true;
}

static bool set_tool (void) {
    // If the tool setter is disabled or if we don't have a tool, rise up and be done
    if(!atc.tool_setter || current_tool.tool_id == 0) {
        if(!rapid_to_z(atc.z_safe_clearance))
            return false;
        return true;
    }

    RAPIDCHANGE_DEBUG_PRINT("Move to probe.");
    if(!rapid_to_z(atc.z_safe_clearance))
        return false;
    if(!rapid_to_tool_setter_xy())
        return false;
    if(!rapid_to_z(atc.tool_setter_z_seek_start))
        return false;

    RAPIDCHANGE_DEBUG_PRINT("Probe cycle.");
    // Probe cycle using GCode interface since tool change interface is private
    plan_line_data_t plan_data;
    gc_parser_flags_t flags = {0};

    plan_data_init(&plan_data);
    plan_data.feed_rate = atc.tool_setter_seek_feed_rate;
    target.z -= atc.tool_setter_max_travel;
    bool ok = true;
    // Locate probe
    if((ok = ok && mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found))
    {
        system_convert_array_steps_to_mpos(target.values, sys.probe_position);

        // Retract a bit and perform slow probe
        target.z += atc.tool_setter_seek_retreat;
        if((ok = mc_line(target.values, &plan_data))) {
            plan_data.feed_rate = atc.tool_setter_set_feed_rate;
            target.z -= (atc.tool_setter_seek_retreat + 2.0f);
            ok = mc_probe_cycle(target.values, &plan_data, flags) == GCProbe_Found;
        }
    }

    if(ok) {
        RAPIDCHANGE_DEBUG_PRINT("Set TLO.");
        if(!(sys.tlo_reference_set.mask & bit(Z_AXIS))) {
            sys.tlo_reference[Z_AXIS] = sys.probe_position[Z_AXIS];
            sys.tlo_reference_set.mask |= bit(Z_AXIS);
            system_add_rt_report(Report_TLOReference);
            grbl.report.feedback_message(Message_ReferenceTLOEstablished);
        } else
            gc_set_tool_offset(ToolLengthOffset_EnableDynamic, Z_AXIS,
                                sys.probe_position[Z_AXIS] - sys.tlo_reference[Z_AXIS]);

        char tlo_msg[20];
        float pos[3];
        system_convert_array_steps_to_mpos(pos, sys.tlo_reference);
        sprintf(tlo_msg, "Current TLO: %3.2f", pos[Z_AXIS]);
        RAPIDCHANGE_DEBUG_PRINT(tlo_msg);
    }

    RAPIDCHANGE_DEBUG_PRINT("End of probing.");
    if(ok)
      if(!rapid_to_z(atc.z_safe_clearance))
        return false;

    return ok;
}

// HAL tool change API
// Set next and/or current tool. Called by gcode.c on on a Tn or M61 command (via HAL).
static void tool_select (tool_data_t *tool, bool next)
{
    RAPIDCHANGE_DEBUG_PRINT("Tool select.");
    next_tool = tool;
    if(!next)
        memcpy(&current_tool, tool, sizeof(tool_data_t));
    char tool_msg[20];
    sprintf(tool_msg, "Current tool: %lu", current_tool.tool_id);
    RAPIDCHANGE_DEBUG_PRINT(tool_msg);
    sprintf(tool_msg, "Next tool: %lu", next_tool->tool_id);
    RAPIDCHANGE_DEBUG_PRINT(tool_msg);
}

// Start a tool change sequence. Called by gcode.c on a M6 command (via HAL).
static status_code_t tool_change (parser_state_t *parser_state)
{
    bool ok = true;
    RAPIDCHANGE_DEBUG_PRINT("Tool change.");
    if(next_tool == NULL) {
        RAPIDCHANGE_DEBUG_PRINT("Next tool is not available!");
        return Status_GCodeToolError;
    }

    if(current_tool.tool_id == next_tool->tool_id) {
        RAPIDCHANGE_DEBUG_PRINT("Current tool selected, tool change bypassed.");
        return Status_OK;
    }

    // Require homing
    uint8_t homed_req =  (X_AXIS_BIT|Y_AXIS_BIT|Z_AXIS_BIT);
    if((sys.homed.mask & homed_req) != homed_req) {
        RAPIDCHANGE_DEBUG_PRINT("Homing is required before tool change.");
        return Status_HomingRequired;
    }

    message_start();
    protocol_buffer_synchronize();

    RAPIDCHANGE_DEBUG_PRINT("Record program state.");
    record_program_state();

    RAPIDCHANGE_DEBUG_PRINT("Set tool change state.");
    set_tool_change_state();

    RAPIDCHANGE_DEBUG_PRINT("Open dust cover.");
    open_dust_cover(true);

    RAPIDCHANGE_DEBUG_PRINT("Unload tool.");
    ok = unload_tool();
    if(!ok)
        return Status_GCodeToolError;

    RAPIDCHANGE_DEBUG_PRINT("Load tool.");
    ok = load_tool(next_tool->tool_id);
    if(!ok)
        return Status_GCodeToolError;

    RAPIDCHANGE_DEBUG_PRINT("Set tool length.");
    ok = set_tool();
    if(!ok)
        return Status_GCodeToolError;

    RAPIDCHANGE_DEBUG_PRINT("Close dust cover.");
    open_dust_cover(false);

    RAPIDCHANGE_DEBUG_PRINT("Restore.");
    ok = restore_program_state();
    if(!ok)
        return Status_GCodeToolError;

    // change_completed();
    RAPIDCHANGE_DEBUG_PRINT("Finished tool change.");

    return Status_OK;
}

// Claim HAL tool change entry points and clear current tool offsets.
void atc_init (void)
{
    protocol_enqueue_foreground_task(report_info, "RapidChange ATC plugin trying to initialize!");
    hal.driver_cap.atc = On;

    atc.tool_recognition_port = 0xFF;
    bool ok;
    if(!ioport_can_claim_explicit()) {
        if((ok = hal.port.num_digital_in >= 1)) {
            hal.port.num_digital_in -= 1;
            atc.tool_recognition_port = hal.port.num_digital_in;
            if(hal.port.set_pin_description)
                hal.port.set_pin_description(Port_Digital, Port_Input, atc.tool_recognition_port, atc_tool_recognition_port_name);
        }
    } else if((ok = (n_ports = ioports_available(Port_Digital, Port_Input)) >= 1))
        strcpy(max_port, uitoa(n_ports - 1));
    if(!ok) {
        protocol_enqueue_foreground_task(report_warning, "RapidChange ATC plugin failed to initialize, unable to claim port for tool recognition!");
        return;
    }

    on_report_options = grbl.on_report_options;
    grbl.on_report_options = report_options;

    if(sys.tlo_reference_set.mask != 0) {
        sys.tlo_reference_set.mask = 0;
        system_add_rt_report(Report_TLOReference);
    }

    // gc_set_tool_offset(ToolLengthOffset_Cancel, 0, 0.0f);

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
