#ifndef ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H
#define ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H

#include <cstdint>
#include <cstdlib>

// File Name
constexpr const char *RA_FILE_NAME = "MFC_LOGGER_";

// File Extension
constexpr const char *RA_FILE_EXT = "CSV";

// Number of IMU sensors
constexpr size_t RA_NUM_IMU = 1;

// Number of Altimeter sensors
constexpr size_t RA_NUM_ALTIMETER = 1;

// Number of GNSS sensors
constexpr size_t RA_NUM_GNSS = 1;

/* THREAD LOOP INTERVALS */

// IMU Reading
constexpr uint32_t RA_INTERVAL_IMU_READING = 5ul;  // ms

// Altimeter Reading
constexpr uint32_t RA_INTERVAL_ALTIMETER_READING = 100ul;  // ms

// GNSS Reading
constexpr uint32_t RA_INTERVAL_GNSS_READING = 500ul;  // ms

// FSM Evaluation
constexpr uint32_t RA_INTERVAL_FSM_EVAL = 5ul;  // ms

// FSM Evaluation interval maximum jitter tolerance
constexpr uint32_t RA_JITTER_TOLERANCE_FSM_EVAL = 1ul;  // ms

/* BOARD FEATURES */

// Start-up Countdown (for time-based arming)
constexpr uint32_t RA_STARTUP_COUNTDOWN = 5. * 1000ul * 60;  // minutes -> us

/* ACTUATOR SETTINGS */

constexpr int RA_SERVO_MIN = 500;                                // us PWM
constexpr int RA_SERVO_MAX = 2450;                               // us PWM
constexpr int RA_SERVO_CEN = (RA_SERVO_MIN + RA_SERVO_MAX) / 2;  // us PWM

constexpr float RA_SERVO_A_RELEASE = 0;    // deg
constexpr float RA_SERVO_A_LOCK    = 180;  // deg

/* SAMPLER SETTINGS */

// True to false ratio for comparator
constexpr double RA_TRUE_TO_FALSE_RATIO = 1.0;  // (#True / #False), 0.5 = 33.3% 1.0 = 50%, 2.0 = 66.7%

/* LAUNCH CONFIGURATION */

// Safeguard minimum time to motor burnout
constexpr uint32_t RA_TIME_TO_BURNOUT_MIN = 0.4 * 1000ul;  // ms

// Safeguard maximum time to motor burnout
constexpr uint32_t RA_TIME_TO_BURNOUT_MAX = 0.7 * 1000ul;  // ms

// Safeguard minimum time to apogee - drogue deployment
constexpr uint32_t RA_TIME_TO_APOGEE_MIN = 7 * 1000ul;  // ms

// Safeguard maximum time to apogee - drogue deployment
constexpr uint32_t RA_TIME_TO_APOGEE_MAX = 10 * 1000ul;  // ms

// Launch acceleration: acc. threshold (GT)
constexpr double RA_LAUNCH_ACC = 10.0;  // 9.81 m/s^2 (g)

// Launch acceleration detection period
constexpr uint32_t RA_LAUNCH_TON     = 200ul;  // ms
constexpr uint32_t RA_LAUNCH_SAMPLES = RA_LAUNCH_TON / RA_INTERVAL_FSM_EVAL;

// Motor burnout detection: acc. threshold (LT)
constexpr double RA_BURNOUT_ACC = 6.0;  // 9.81 m/s^2 (g)

// Motor burnout detection period
constexpr uint32_t RA_BURNOUT_TON     = 500ul;  // ms
constexpr uint32_t RA_BURNOUT_SAMPLES = RA_BURNOUT_TON / RA_INTERVAL_FSM_EVAL;

// Apogee altitude (nominal for safeguard calculation)
constexpr double RA_APOGEE_ALT = 450.0;  // m

// Velocity at Apogee: vel. threshold (LT)
constexpr double RA_APOGEE_VEL = 10.0;  // m/s

// Velocity at Apogee detection period
constexpr uint32_t RA_APOGEE_TON     = 500ul;  // ms
constexpr uint32_t RA_APOGEE_SAMPLES = RA_APOGEE_TON / RA_INTERVAL_FSM_EVAL;

// Drogue Descent Theoretical Velocity
constexpr double RA_DROGUE_VEL = 17.5;  // m/s

// Main Deployment Event Altitude: altitude threshold (LT)
constexpr double RA_MAIN_ALT_RAW = 150.0;  // m

// Safeguard minimum time to main deployment
constexpr uint32_t RA_TIME_TO_MAIN_NOM = ((RA_APOGEE_ALT - RA_MAIN_ALT_RAW) / RA_DROGUE_VEL) * 1000ul;  // ms

// Safeguard minimum time to main deployment
constexpr uint32_t RA_TIME_TO_MAIN_MIN = RA_TIME_TO_MAIN_NOM - 5000ul;  // ms

// Safeguard maximum time to main deployment
constexpr uint32_t RA_TIME_TO_MAIN_MAX = RA_TIME_TO_MAIN_NOM + 5000ul;  // ms

// Main Deployment Event Altitude detection period
constexpr uint32_t RA_MAIN_TON     = 500ul;  // ms
constexpr uint32_t RA_MAIN_SAMPLES = RA_MAIN_TON / RA_INTERVAL_FSM_EVAL;

// Main Deployment Event Triggering Delay Compensation Multiplier
constexpr double RA_MAIN_COMPENSATION_MULT = 2.0;

// Main Deployment Event Triggering Delay Compensation Value
constexpr double RA_MAIN_ALT_COMPENSATED = RA_MAIN_ALT_RAW + RA_MAIN_COMPENSATION_MULT * RA_DROGUE_VEL * (static_cast<double>(RA_MAIN_TON) / 1000.);  // m

// Velocity at Landed State: vel. threshold (LT)
constexpr double RA_LANDED_VEL = 0.5;  // m/s

// Velocity at Landed State detection period
constexpr uint32_t RA_LANDED_TON     = 5000ul;  // ms
constexpr uint32_t RA_LANDED_SAMPLES = RA_LANDED_TON / RA_INTERVAL_FSM_EVAL;

/* SD CARD LOGGER INTERVALS */

constexpr uint32_t RA_SDLOGGER_INTERVAL_IDLE     = 1000ul;  // 1 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_SLOW     = 200ul;   // 5 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_FAST     = 100ul;   // 10 Hz
constexpr uint32_t RA_SDLOGGER_INTERVAL_REALTIME = 50ul;    // 20 Hz

// Static assertions validate settings
namespace details::assertions {
  static_assert(RA_TIME_TO_BURNOUT_MAX >= RA_TIME_TO_BURNOUT_MIN, "Time to burnout is configured incorrectly!");
  static_assert(RA_TIME_TO_APOGEE_MAX >= RA_TIME_TO_APOGEE_MIN, "Time to apogee is configured incorrectly!");
}  // namespace details::assertions

#endif  //ROCKET_AVIONICS_TEMPLATE_USERCONFIG_H
