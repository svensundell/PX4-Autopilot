/****************************************************************************
 *
 *   Copyright (c) 2026 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/

#pragma once

#include "framework.h"

#include <uORB/topics/battery_status.h>

namespace failsafe_param_mapping
{

enum class LowBatteryAction : int32_t {
	Warning = 0,
	Return = 1,
	Land = 2,
	ReturnOrLand = 3
};

enum class offboard_loss_failsafe_mode : int32_t {
	Position_mode = 0,
	Altitude_mode = 1,
	Stabilized = 2,
	Return_mode = 3,
	Land_mode = 4,
	Hold_mode = 5,
	Terminate = 6,
	Disarm = 7,
};

enum class actuator_failure_failsafe_mode : int32_t {
	Warning_only = 0,
	Hold_mode = 1,
	Land_mode = 2,
	Return_mode = 3,
	Terminate = 4,
};

enum class geofence_violation_action : int32_t {
	None = 0,
	Warning = 1,
	Hold_mode = 2,
	Return_mode = 3,
	Terminate = 4,
	Land_mode = 5,
};

enum class gcs_connection_loss_failsafe_mode : int32_t {
	Disabled = 0,
	Hold_mode = 1,
	Return_mode = 2,
	Land_mode = 3,
	Terminate = 5,
	Disarm = 6,
	Hold_mode_no_failsafe = 7,
};

enum class command_after_quadchute : int32_t {
	Warning_only = -1,
	Return_mode = 0,
	Land_mode = 1,
	Hold_mode = 2,
};

enum class command_after_high_wind_failsafe : int32_t {
	None = 0,
	Warning = 1,
	Hold_mode = 2,
	Return_mode = 3,
	Terminate = 4,
	Land_mode = 5
};

enum class command_after_pos_low_failsafe : int32_t {
	None = 0,
	Warning = 1,
	Hold_mode = 2,
	Return_mode = 3,
	Terminate = 4,
	Land_mode = 5
};

enum class command_after_remaining_flight_time_low : int32_t {
	None = 0,
	Warning = 1,
	Return_mode = 3
};

enum class open_drone_id_failsafe_mode : int32_t {
	None = 0,
	Warning = 1,
	Error = 2,
	Return_mode = 3,
	Land_mode = 4,
	Terminate = 5,
};

enum class parachute_unhealthy_failsafe_mode : int32_t {
	Disabled = 0,
	Warning = 1,
	Return = 2,
	Land = 3,
};

enum class gps_redundancy_failsafe_mode : int32_t {
	Warning = 0,
	Return_mode = 1,
	Land_mode = 2,
	Terminate = 3,
};

FailsafeBase::ActionOptions fromNavDllOrRclActParam(int param_value);
FailsafeBase::ActionOptions fromGfActParam(int param_value);
FailsafeBase::ActionOptions fromActuatorFailureActParam(int param_value);
FailsafeBase::ActionOptions fromBatteryWarningActParam(int param_value, uint8_t battery_warning);
FailsafeBase::ActionOptions fromQuadchuteActParam(int param_value);
FailsafeBase::Action fromOffboardLossActParam(int param_value, uint8_t &user_intended_mode);
FailsafeBase::ActionOptions fromHighWindLimitActParam(int param_value);
FailsafeBase::ActionOptions fromPosLowActParam(int param_value);
FailsafeBase::ActionOptions fromGnssLossActParam(int param_value);
FailsafeBase::ActionOptions fromParachuteActParam(int param_value);
FailsafeBase::ActionOptions fromRemainingFlightTimeLowActParam(int param_value);
FailsafeBase::ActionOptions fromOdidFailActParam(int param_value);

} // namespace failsafe_param_mapping
