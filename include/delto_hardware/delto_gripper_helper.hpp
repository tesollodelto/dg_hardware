// delto_gripper_helper
//
// Release Notes:
//   0.4 - Run the limiter PI every cycle so the integral unwinds instead of
//         hard-resetting (removes the limit cycle at the threshold); cap
//         compensation at target magnitude minus a stall-torque floor
//         (replaces the MAX_TORQUE cap that stranded saturated-duty targets
//         above the limit, and stops post-overshoot current collapse); P term
//         acts on positive excess only; raise gains to 1/30000
//   0.3 - Fix S model error using non-S current limit; clamp compensation so
//         torque is only reduced in magnitude, never reversed; name gain
//         constants
//   0.2 - Add S model current limit and current control function
//   0.1 - Initial release

#include <vector>

namespace delto_gripper_helper {

// Units:
//   current  [mA]
//   torque   [Nm]
//   duty     [%]
// current_limit_flag, current_integral : internal state (caller-owned)

double GetLibraryVersion();

// current [mA] -> effort(torque) [Nm]
std::vector<double> ConvertEffort(const std::vector<double>& current);

std::vector<double> CurrentControl(int joint_count,
                                   const std::vector<int> &actual_current,    // [mA]
                                   const std::vector<double> &target_torque,  // [Nm]
                                   std::vector<int> &current_limit_flag,      // internal state
                                   std::vector<double> &current_integral);    // internal state

std::vector<double> CurrentControlSModel(int joint_count,
                                   const std::vector<int> &actual_current,    // [mA]
                                   const std::vector<double> &target_torque,  // [Nm]
                                   std::vector<int> &current_limit_flag,      // internal state
                                   std::vector<double> &current_integral);    // internal state

// target_torque [Nm] -> duty [%]
std::vector<double> ConvertDuty(int joint_count,
                                std::vector<double> target_torque);

}  // namespace delto_gripper_helper
