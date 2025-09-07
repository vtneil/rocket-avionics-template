#ifndef ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H
#define ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H

#include <cstdint>
#include <cstdlib>

// Number of IMU sensors
constexpr size_t RA_NUM_IMU = 1;

// Number of Altimeter sensors
constexpr size_t RA_NUM_ALTIMETER = 1;

// Number of GNSS sensors
constexpr size_t RA_NUM_GNSS = 1;

/* THREAD LOOP INTERVALS */

// IMU Reading
constexpr uint32_t RA_INTERVAL_IMU_READING = 10ul;  // ms

// Altimeter Reading
constexpr uint32_t RA_INTERVAL_ALTIMETER_READING = 100ul;  // ms

// GNSS Reading
constexpr uint32_t RA_INTERVAL_GNSS_READING = 500ul;  // ms

// FSM Evaluation
constexpr uint32_t RA_INTERVAL_FSM_EVAL = 5ul;  // ms

/* LAUNCH CONFIGURATION */

// Safeguard minimum time to motor burnout
constexpr uint32_t RA_TIME_TO_BURNOUT_MIN = 1 * 1000ul;  // ms

// Safeguard maximum time to motor burnout
constexpr uint32_t RA_TIME_TO_BURNOUT_MAX = 1 * 1000ul;  // ms

// Safeguard minimum time to apogee
constexpr uint32_t RA_TIME_TO_APOGEE_MIN = 1 * 1000ul;  // ms

// Safeguard maximum time to apogee
constexpr uint32_t RA_TIME_TO_APOGEE_MAX = 1 * 1000ul;  // ms

// Launch acceleration: acc. threshold (GT)
constexpr double RA_LAUNCH_ACC = 40.0;  // m/s^2

// Launch acceleration detection period
constexpr uint32_t RA_LAUNCH_TON = 150ul;  // ms

// Motor burnout detection: acc. threshold (LT)
constexpr double RA_BURNOUT_ACC = 40.0;  // m/s^2

// Motor burnout detection period
constexpr uint32_t RA_BURNOUT_TON = 500ul;  // ms

// Velocity at Apogee: vel. threshold (LT)
constexpr double RA_APOGEE_VEL = 10.0;  // m/s

// Velocity at Apogee detection period
constexpr uint32_t RA_APOGEE_TON = 500ul;  // ms

// Drogue Descent Theoretical Velocity
constexpr double RA_DROGUE_VEL = 15.0;  // m/s

// Main Deployment Event Altitude: altitude threshold (LT)
constexpr double RA_MAIN_ALT = 300.0;  // m

// Main Deployment Event Altitude detection period
constexpr uint32_t RA_MAIN_TON = 1000ul;  // ms

// Main Deployment Event Triggering Delay Compensation Multiplier
constexpr double RA_MAIN_COMPENSATION_MULT = 2.0;

// Main Deployment Event Triggering Delay Compensation Value
constexpr double RA_MAIN_ALT_COMPENSATED = RA_MAIN_ALT + RA_MAIN_COMPENSATION_MULT * RA_DROGUE_VEL * (static_cast<double>(RA_MAIN_TON) / 1000.);  // m

// Velocity at Landed State: vel. threshold (LT)
constexpr double RA_LANDED_VEL = 0.5;  // m/s

// Velocity at Landed State detection period
constexpr uint32_t RA_LANDED_TON = 5000ul;  // ms

namespace details::assertions {
  static_assert(RA_TIME_TO_BURNOUT_MAX >= RA_TIME_TO_BURNOUT_MIN, "Time to burnout is configured incorrectly!");
  static_assert(RA_TIME_TO_APOGEE_MAX >= RA_TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
}  // namespace details::assertions

#endif  //ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H
