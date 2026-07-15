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

/**
 * @file failsafe_param_mapping.cpp
 *
 * Maps COM_* / NAV_* parameter enum values to failsafe ActionOptions.
 * Centralized here so Commander and the failsafe state machine share one table.
 */

#include "failsafe_param_mapping.hpp"

#include <uORB/topics/vehicle_status.h>

using FailsafeBase::Action;
using FailsafeBase::ActionOptions;
using FailsafeBase::ClearCondition;
using FailsafeBase::Cause;
using FailsafeBase::UserTakeoverAllowed;

namespace failsafe_param_mapping
{

ActionOptions fromNavDllOrRclActParam(int param_value)
{
	ActionOptions options{};

	switch (static_cast<gcs_connection_loss_failsafe_mode>(param_value)) {
	case gcs_connection_loss_failsafe_mode::Disabled:

	// No failsafe action: for NAV_RCL_ACT this is handled by Commander switching to Hold as a regular
	// mode change (see Commander::manualControlLossModeSwitch()).
	case gcs_connection_loss_failsafe_mode::Hold_mode_no_failsafe:
	default:
		options.action = Action::None;
		break;

	case gcs_connection_loss_failsafe_mode::Hold_mode:
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case gcs_connection_loss_failsafe_mode::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case gcs_connection_loss_failsafe_mode::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case gcs_connection_loss_failsafe_mode::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case gcs_connection_loss_failsafe_mode::Disarm: // Lockdown
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Disarm;
		break;
	}

	return options;
}

ActionOptions fromGfActParam(int param_value)
{
	ActionOptions options{};

	switch (static_cast<geofence_violation_action>(param_value)) {
	case geofence_violation_action::None:
		options.action = Action::None;
		break;

	case geofence_violation_action::Warning:
	default:
		options.action = Action::Warn;
		break;

	case geofence_violation_action::Hold_mode:
		options.allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly; // ensure the user can escape again
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case geofence_violation_action::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case geofence_violation_action::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		// Allow recovery after the pilot leaves the breached zone and selects a safe mode.
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case geofence_violation_action::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;
	}

	return options;
}

ActionOptions fromActuatorFailureActParam(int param_value)
{
	ActionOptions options{};

	switch (static_cast<actuator_failure_failsafe_mode>(param_value)) {
	case actuator_failure_failsafe_mode::Warning_only:
	default:
		options.action = Action::Warn;
		break;

	case actuator_failure_failsafe_mode::Hold_mode:
		options.action = Action::Hold;
		break;

	case actuator_failure_failsafe_mode::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case actuator_failure_failsafe_mode::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case actuator_failure_failsafe_mode::Terminate:
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;
	}

	return options;
}

ActionOptions fromBatteryWarningActParam(int param_value, uint8_t battery_warning)
{
	ActionOptions options{};

	switch (battery_warning) {
	case battery_status_s::WARNING_NONE:
	default:
		options.action = Action::None;
		break;

	case battery_status_s::WARNING_LOW:
		options.action = Action::Warn;
		options.cause = Cause::BatteryLow;
		break;

	case battery_status_s::WARNING_CRITICAL:
		options.action = Action::Warn;
		options.cause = Cause::BatteryCritical;

		switch (static_cast<LowBatteryAction>(param_value)) {
		case LowBatteryAction::Return:
		case LowBatteryAction::ReturnOrLand:
			options.action = Action::RTL;
			break;

		case LowBatteryAction::Land:
			options.action = Action::Land;
			break;

		case LowBatteryAction::Warning:
			options.action = Action::Warn;
			break;
		}

		break;

	case battery_status_s::WARNING_EMERGENCY:
		options.action = Action::Warn;
		options.cause = Cause::BatteryEmergency;

		switch (static_cast<LowBatteryAction>(param_value)) {
		case LowBatteryAction::Return:
			options.action = Action::RTL;
			break;

		case LowBatteryAction::ReturnOrLand:
		case LowBatteryAction::Land:
			options.action = Action::Land;
			break;

		case LowBatteryAction::Warning:
			options.action = Action::Warn;
			break;
		}

		break;
	}

	return options;
}

ActionOptions fromQuadchuteActParam(int param_value)
{
	ActionOptions options{};

	switch (static_cast<command_after_quadchute>(param_value)) {
	case command_after_quadchute::Warning_only:
	default:
		options.action = Action::Warn;
		break;

	case command_after_quadchute::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case command_after_quadchute::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case command_after_quadchute::Hold_mode:
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;
	}

	return options;
}

Action fromOffboardLossActParam(int param_value, uint8_t &user_intended_mode)
{
	Action action{Action::None};

	switch (static_cast<offboard_loss_failsafe_mode>(param_value)) {
	case offboard_loss_failsafe_mode::Position_mode:
	default:
		action = Action::FallbackPosCtrl;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_POSCTL;
		break;

	case offboard_loss_failsafe_mode::Altitude_mode:
		action = Action::FallbackAltCtrl;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_ALTCTL;
		break;

	case offboard_loss_failsafe_mode::Stabilized:
		action = Action::FallbackStab;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_STAB;
		break;

	case offboard_loss_failsafe_mode::Return_mode:
		action = Action::RTL;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_RTL;
		break;

	case offboard_loss_failsafe_mode::Land_mode:
		action = Action::Land;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_LAND;
		break;

	case offboard_loss_failsafe_mode::Hold_mode:
		action = Action::Hold;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_AUTO_LOITER;
		break;

	case offboard_loss_failsafe_mode::Terminate:
		action = Action::Terminate;
		user_intended_mode = vehicle_status_s::NAVIGATION_STATE_TERMINATION;
		break;

	case offboard_loss_failsafe_mode::Disarm:
		action = Action::Disarm;
		break;
	}

	return action;
}

ActionOptions fromHighWindLimitActParam(int param_value)
{
	ActionOptions options{};

	switch (static_cast<command_after_high_wind_failsafe>(param_value)) {
	case command_after_high_wind_failsafe::None:
		options.action = Action::None;
		break;

	case command_after_high_wind_failsafe::Warning:
	default:
		options.action = Action::Warn;
		break;

	case command_after_high_wind_failsafe::Hold_mode:
		options.allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly;
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case command_after_high_wind_failsafe::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case command_after_high_wind_failsafe::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case command_after_high_wind_failsafe::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;
	}

	return options;
}

ActionOptions fromPosLowActParam(int param_value)
{
	ActionOptions options{};
	options.allow_user_takeover = UserTakeoverAllowed::AlwaysModeSwitchOnly;

	switch (static_cast<command_after_pos_low_failsafe>(param_value)) {
	case command_after_pos_low_failsafe::None:
		options.action = Action::None;
		break;

	case command_after_pos_low_failsafe::Warning:
	default:
		options.action = Action::Warn;
		break;

	case command_after_pos_low_failsafe::Hold_mode:
		options.action = Action::Hold;
		options.clear_condition = ClearCondition::WhenConditionClears;
		break;

	case command_after_pos_low_failsafe::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::WhenConditionClears;
		break;

	case command_after_pos_low_failsafe::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case command_after_pos_low_failsafe::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::WhenConditionClears;
		break;
	}

	return options;
}

ActionOptions fromGnssLossActParam(int param_value)
{
	ActionOptions options{};

	switch (static_cast<gps_redundancy_failsafe_mode>(param_value)) {
	case gps_redundancy_failsafe_mode::Warning:
	default:
		options.action = Action::Warn;
		break;

	case gps_redundancy_failsafe_mode::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case gps_redundancy_failsafe_mode::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case gps_redundancy_failsafe_mode::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;
	}

	return options;
}

ActionOptions fromParachuteActParam(int param_value)
{
	ActionOptions options{};

	switch (static_cast<parachute_unhealthy_failsafe_mode>(param_value)) {
	case parachute_unhealthy_failsafe_mode::Disabled:
	default:
		options.action = Action::None;
		break;

	case parachute_unhealthy_failsafe_mode::Warning:
		options.action = Action::Warn;
		options.clear_condition = ClearCondition::WhenConditionClears;
		break;

	case parachute_unhealthy_failsafe_mode::Return:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case parachute_unhealthy_failsafe_mode::Land:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;
	}

	return options;
}

ActionOptions fromRemainingFlightTimeLowActParam(int param_value)
{
	ActionOptions options{};

	options.allow_user_takeover = UserTakeoverAllowed::Auto;
	options.cause = Cause::RemainingFlightTimeLow;

	switch (static_cast<command_after_remaining_flight_time_low>(param_value)) {
	case command_after_remaining_flight_time_low::None:
		options.action = Action::None;
		break;

	case command_after_remaining_flight_time_low::Warning:
		options.action = Action::Warn;
		break;

	case command_after_remaining_flight_time_low::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	default:
		options.action = Action::None;
		break;
	}

	return options;
}

ActionOptions fromOdidFailActParam(int param_value)
{
	ActionOptions options{};

	switch (static_cast<open_drone_id_failsafe_mode>(param_value)) {
	case open_drone_id_failsafe_mode::Return_mode:
		options.action = Action::RTL;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case open_drone_id_failsafe_mode::Land_mode:
		options.action = Action::Land;
		options.clear_condition = ClearCondition::OnModeChangeOrDisarm;
		break;

	case open_drone_id_failsafe_mode::Terminate:
		options.allow_user_takeover = UserTakeoverAllowed::Never;
		options.action = Action::Terminate;
		options.clear_condition = ClearCondition::Never;
		break;

	case open_drone_id_failsafe_mode::None:
	case open_drone_id_failsafe_mode::Warning:
	case open_drone_id_failsafe_mode::Error:
	default:
		options.action = Action::None;
		break;
	}

	return options;
}

} // namespace failsafe_param_mapping
