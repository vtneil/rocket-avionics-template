/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>          // Arduino Framework
#include <File_Utility.h>     // File Utility
#include <LibAvionics.h>      // Base Avionics Library and Utilities
#include "SystemFunctions.h"  // Function Declarations
#include "custom_kalman.h"    // Kalman Quick Table

#if __has_include("STM32FreeRTOS.h")
#  define USE_FREERTOS 1
#  include "hal_rtos.h"
#endif

#include <STM32SD.h>
#include <Servo.h>
/* END INCLUDE SYSTEM LIBRARIES */

/* BEGIN INCLUDE USER'S IMPLEMENTATIONS */
#include "UserConfig.h"   // User's Custom Configurations
#include "UserPins.h"     // User's Pins Mapping
#include "UserSensors.h"  // User's Hardware Implementations
#include "UserFSM.h"      // User's FSM States
/* END INCLUDE USER'S IMPLEMENTATIONS */

/* BEGIN INCLUDE MAIN */
#include "./main.h"
/* END INCLUDE MAIN */

/* BEGIN USER PRIVATE TYPEDEFS, INCLUDES AND MACROS */
/* END USER PRIVATE TYPEDEFS, INCLUDES AND MACROS */

/* BEGIN SENSOR INSTANCES */
SensorIMU *imu[RA_NUM_IMU] = {
  new IMU_ADXL372(SPI, USER_GPIO_ADXL372_NSS),  // IMU #1
};
SensorAltimeter *altimeter[RA_NUM_ALTIMETER] = {
  new Altimeter_BMP581(USER_GPIO_BMP581_NSS),  // Altimeter #1
};
SensorGNSS *gnss[RA_NUM_GNSS] = {
  nullptr,  // GNSS #1 (No GNSS)
};
/* END SENSOR INSTANCES */

/* BEGIN SENSOR STATUSES */
struct SensorsHealth {
  SensorStatus imu[RA_NUM_IMU]{};
  SensorStatus altimeter[RA_NUM_ALTIMETER]{};
  SensorStatus gnss[RA_NUM_GNSS]{};
} sensors_health;
/* END SENSOR STATUSES */

/* BEGIN PERSISTENT STATE */
// todo: EEPROM for persistent state
UserFSM  fsm;
double   acc;
uint32_t seq_no{};
/* END PERSISTENT STATE */

/* BEGIN DATA MEMORY */
struct DataMemory {
  SensorIMU::Data       imu[RA_NUM_IMU];
  SensorAltimeter::Data altimeter[RA_NUM_ALTIMETER];
  SensorGNSS::Data      gnss[RA_NUM_GNSS];
} data;
String sd_buf;
/* END DATA MEMORY */

/* BEGIN SD CARD */
FsUtil fs_sd;
/* END SD CARD */

/* BEGIN FILTERS */
FilterT filter_alt;
/* END FILTERS */

/* BEGIN ACTUATORS */
Servo servo_a;
int   pos_a = RA_SERVO_A_LOCK;
Servo servo_b;
int   pos_b = 90;
/* END ACTUATORS */

/* BEGIN USER PRIVATE VARIABLES */
hal::rtos::mutex_t mtx_sdio;
hal::rtos::mutex_t mtx_spi;
hal::rtos::mutex_t mtx_cdc;
/* END USER PRIVATE VARIABLES */

/* BEGIN USER PRIVATE FUNCTIONS */
uint32_t LoggerInterval() {
  switch (fsm.state()) {
    case UserState::STARTUP:
    case UserState::IDLE_SAFE:
      return RA_SDLOGGER_INTERVAL_IDLE;

    case UserState::ARMED:
    case UserState::PAD_PREOP:
      return RA_SDLOGGER_INTERVAL_SLOW;

    case UserState::POWERED:
    case UserState::COASTING:
      return RA_SDLOGGER_INTERVAL_REALTIME;

    case UserState::DROGUE_DEPLOY:
    case UserState::DROGUE_DESCEND:
    case UserState::MAIN_DEPLOY:
    case UserState::MAIN_DESCEND:
      return RA_SDLOGGER_INTERVAL_FAST;

    case UserState::LANDED:
    case UserState::RECOVERED_SAFE:
    default:
      return RA_SDLOGGER_INTERVAL_IDLE;
  }
}
/* END USER PRIVATE FUNCTIONS */

/* BEGIN USER SETUP */
void UserSetupGPIO() {
  pinMode(USER_GPIO_LED, OUTPUT);
}

void UserSetupActuator() {
  servo_a.attach(USER_GPIO_SERVO_A, RA_SERVO_MIN, RA_SERVO_MAX, RA_SERVO_MAX);
}

void UserSetupCDC() {
  Serial.begin();
}

void UserSetupSPI() {
  SPI.setMOSI(USER_GPIO_SPI1_MOSI);
  SPI.setMISO(USER_GPIO_SPI1_MISO);
  SPI.setSCLK(USER_GPIO_SPI1_SCK);
  SPI.begin();
}
/* END USER SETUP */

/* BEGIN USER THREADS */
void CB_ReadIMU(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_IMU_READING, [&]() -> void {
    mtx_spi.exec([&]() -> void {
      ReadIMU();
    });

    const double &ax = data.imu[0].acc_x;
    const double &ay = data.imu[0].acc_y;
    const double &az = data.imu[0].acc_z;

    // Total acceleration
    acc = std::sqrt(std::abs(ax * ax) + std::abs(ay * ay) + std::abs(az + az));
  });
}

void CB_ReadAltimeter(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_ALTIMETER_READING, [&]() -> void {
    mtx_spi.exec([&]() -> void {
      ReadAltimeter();
    });
    filter_alt.kf.predict().update(data.altimeter[0].altitude_m);
  });
}

void CB_EvalFSM(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_FSM_EVAL, [&]() -> void {
    EvalFSM();
  });
}

void CB_SDLogger(void *) {
  hal::rtos::interval_loop(LoggerInterval(), LoggerInterval, [&]() -> void {
    sd_buf = "";
    csv_stream_lf(sd_buf)
      << "MFC"
      << seq_no
      << millis()
      << state_string(fsm.state())
      << data.imu[0].acc_x
      << data.imu[0].acc_y
      << data.imu[0].acc_z
      << filter_alt.kf.state_vector()[1]
      << data.altimeter[0].altitude_m
      << data.altimeter[0].pressure_hpa
      << pos_a
      << ReadCPUTemp();
    mtx_sdio.exec([&]() -> void {
      fs_sd.file() << sd_buf;
    });
  });
}

void CB_SDSave(void *) {
  hal::rtos::interval_loop(1000ul, [&]() -> void {
    mtx_sdio.exec([&]() -> void {
      fs_sd.file().flush();
    });
  });
}

void CB_DebugLogger(void *) {
  hal::rtos::interval_loop(100ul, [&]() -> void {
    mtx_cdc.exec([&]() -> void {
      Serial.print(sd_buf);
    });
  });
}

void CB_RetainDeployment(void *) {
  hal::rtos::interval_loop(100ul, [&]() -> void {
    RetainDeployment();
  });
}

/* END USER THREADS */

void UserThreads() {
  hal::rtos::scheduler.create(CB_EvalFSM, {.stack_size = 8192, .priority = osPriorityRealtime});
  hal::rtos::scheduler.create(CB_ReadIMU, {.stack_size = 2048, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_ReadAltimeter, {.stack_size = 2048, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_RetainDeployment, {.stack_size = 1024, .priority = osPriorityHigh});
  hal::rtos::scheduler.create(CB_SDLogger, {.stack_size = 8192, .priority = osPriorityNormal});
  hal::rtos::scheduler.create(CB_SDSave, {.stack_size = 8192, .priority = osPriorityLow});
  hal::rtos::scheduler.create(CB_DebugLogger, {.stack_size = 8192, .priority = osPriorityBelowNormal});
}

void setup() {
  /* BEGIN GPIO AND INTERFACES SETUP */
  UserSetupGPIO();
  UserSetupActuator();
  UserSetupCDC();
  UserSetupUSART();
  UserSetupI2C();
  UserSetupSPI();
  /* END GPIO AND INTERFACES SETUP */

  /* BEGIN STORAGES SETUP */
  SD.setDx(USER_GPIO_SDIO_DAT0, USER_GPIO_SDIO_DAT1, USER_GPIO_SDIO_DAT2, USER_GPIO_SDIO_DAT3);
  SD.setCMD(USER_GPIO_SDIO_CMD);
  SD.setCK(USER_GPIO_SDIO_CK);
  SD.begin();
  fs_sd.find_file_name(RA_FILE_NAME, RA_FILE_EXT);
  fs_sd.open_one<FsMode::WRITE>();
  sd_buf.reserve(1024);
  /* END STORAGES SETUP */

  /* BEGIN SENSORS SETUP */
  // IMU
  for (size_t i = 0; i < RA_NUM_IMU; ++i) {
    if (!imu[i])
      sensors_health.imu[i] = SensorStatus::SENSOR_NO;
    else if (imu[i]->begin())
      sensors_health.imu[i] = SensorStatus::SENSOR_OK;
    else
      sensors_health.imu[i] = SensorStatus::SENSOR_ERR;
  }

  // Altimeter
  for (size_t i = 0; i < RA_NUM_ALTIMETER; ++i) {
    if (!altimeter[i])
      sensors_health.altimeter[i] = SensorStatus::SENSOR_NO;
    else if (altimeter[i]->begin())
      sensors_health.altimeter[i] = SensorStatus::SENSOR_OK;
    else
      sensors_health.altimeter[i] = SensorStatus::SENSOR_ERR;
  }

  // GNSS
  for (size_t i = 0; i < RA_NUM_GNSS; ++i) {
    if (!gnss[i])
      sensors_health.gnss[i] = SensorStatus::SENSOR_NO;
    else if (gnss[i]->begin())
      sensors_health.gnss[i] = SensorStatus::SENSOR_OK;
    else
      sensors_health.gnss[i] = SensorStatus::SENSOR_ERR;
  }
  /* END SENSORS SETUP */

  /* BEGIN SYSTEM/KERNEL SETUP */
  hal::rtos::scheduler.initialize();
  UserThreads();
  hal::rtos::scheduler.start();
  /* END SYSTEM/KERNEL SETUP */
}

void EvalFSM() {
  static uint32_t                       state_millis_start   = 0;
  static uint32_t                       state_millis_elapsed = 0;
  static xcore::sampler_t<2048, double> sampler;

  switch (fsm.state()) {
    case UserState::STARTUP: {
      // <--- Next: always transfer --->
      digitalWrite(USER_GPIO_LED, 1);
      fsm.transfer(UserState::IDLE_SAFE);
      break;
    }

    case UserState::IDLE_SAFE: {
      // <--- Next: always transfer (should wait for uplink) --->
      fsm.transfer(UserState::ARMED);
      break;
    }

    case UserState::ARMED: {
      // <--- Next: always transfer (should wait for uplink) --->
      fsm.transfer(UserState::PAD_PREOP);
      break;
    }

    case UserState::PAD_PREOP: {
      // !!!! Next: DETECT launch !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_LAUNCH_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_LAUNCH_ACC, /*recount*/ false);
      }

      sampler.add_sample(acc);

      if (sampler.is_sampled() &&
          sampler.over_by_under<double>() > RA_TRUE_TO_FALSE_RATIO)
        fsm.transfer(UserState::POWERED);
      break;
    }

    case UserState::POWERED: {
      // !!!! Next: DETECT motor burnout !!!!
      if (fsm.on_enter()) {  // Run once
        digitalWrite(USER_GPIO_LED, 0);
        sampler.reset();
        sampler.set_capacity(RA_BURNOUT_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_BURNOUT_ACC, /*recount*/ false);
        state_millis_start = millis();
      }

      sampler.add_sample(acc);
      state_millis_elapsed = millis() - state_millis_start;

      if (state_millis_elapsed >= RA_TIME_TO_BURNOUT_MAX ||
          (state_millis_elapsed >= RA_TIME_TO_BURNOUT_MIN &&
           sampler.is_sampled() &&
           sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO))
        fsm.transfer(UserState::COASTING);
      break;
    }

    case UserState::COASTING: {
      // !!!! Next: DETECT apogee !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_APOGEE_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_APOGEE_VEL, /*recount*/ false);
        state_millis_start = millis();
      }

      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(vel);
      state_millis_elapsed = millis() - state_millis_start;

      if (state_millis_elapsed >= RA_TIME_TO_APOGEE_MAX ||
          (state_millis_elapsed >= RA_TIME_TO_APOGEE_MIN &&
           sampler.is_sampled() &&
           sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO))
        fsm.transfer(UserState::DROGUE_DEPLOY);
      break;
    }

    case UserState::DROGUE_DEPLOY: {
      // <--- Next: activate pyro/servo and always transfer --->
      ActivateDeployment(/*index*/ 0);
      fsm.transfer(UserState::DROGUE_DESCEND);
      break;
    }

    case UserState::DROGUE_DESCEND: {
      // !!!! Next: DETECT main deployment altitude !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_MAIN_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_MAIN_ALT_COMPENSATED, /*recount*/ false);
      }

      const double alt = filter_alt.kf.state_vector()[0];
      sampler.add_sample(alt);

      if (sampler.is_sampled() &&
          sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO)
        fsm.transfer(UserState::MAIN_DEPLOY);
      break;
    }

    case UserState::MAIN_DEPLOY: {
      // <--- Next: activate pyro/servo and always transfer --->
      // ActivateDeployment(/*index*/ 1);
      fsm.transfer(UserState::MAIN_DESCEND);
      break;
    }

    case UserState::MAIN_DESCEND: {
      // !!!! Next: DETECT landing !!!!
      if (fsm.on_enter()) {  // Run once
        sampler.reset();
        sampler.set_capacity(RA_LANDED_SAMPLES, /*recount*/ false);
        sampler.set_threshold(RA_LANDED_VEL, /*recount*/ false);
      }

      const double vel = filter_alt.kf.state_vector()[1];
      sampler.add_sample(std::abs(vel));

      if (sampler.is_sampled() &&
          sampler.under_by_over<double>() > RA_TRUE_TO_FALSE_RATIO)
        fsm.transfer(UserState::LANDED);
      break;
    }

    case UserState::LANDED: {
      digitalWrite(USER_GPIO_LED, 1);
      break;
    }

    case UserState::RECOVERED_SAFE: {
      break;
    }
  }
}

void ReadIMU() {
  for (size_t i = 0; i < RA_NUM_IMU; ++i) {
    if (sensors_health.imu[i] != SensorStatus::SENSOR_OK ||
        !imu[i]->read())
      continue;
    data.imu[i].acc_x = imu[i]->acc_x();
    data.imu[i].acc_y = imu[i]->acc_y();
    data.imu[i].acc_z = imu[i]->acc_z();
    data.imu[i].gyr_x = imu[i]->gyr_x();
    data.imu[i].gyr_y = imu[i]->gyr_y();
    data.imu[i].gyr_z = imu[i]->gyr_z();
  }
}

void ReadAltimeter() {
  for (size_t i = 0; i < RA_NUM_ALTIMETER; ++i) {
    if (sensors_health.altimeter[i] != SensorStatus::SENSOR_OK ||
        !altimeter[i]->read())
      continue;
    data.altimeter[i].pressure_hpa = altimeter[i]->pressure_hpa();
    data.altimeter[i].altitude_m   = altimeter[i]->altitude_m();
  }
}

void ReadGNSS() {
  for (size_t i = 0; i < RA_NUM_GNSS; ++i) {
    if (sensors_health.gnss[i] != SensorStatus::SENSOR_OK ||
        !gnss[i]->read())
      continue;
    data.gnss[i].timestamp_epoch = gnss[i]->timestamp_epoch();
    data.gnss[i].siv             = gnss[i]->siv();
    data.gnss[i].latitude        = gnss[i]->latitude();
    data.gnss[i].longitude       = gnss[i]->longitude();
    data.gnss[i].altitude_msl    = gnss[i]->altitude_msl();
  }
}

void ActivateDeployment(const size_t index) {
  switch (index) {
    case 0: {  // Drogue/First Deployment
      pos_a = RA_SERVO_A_RELEASE;
      servo_a.write(pos_a);
      break;
    }

    case 1: {  // Main/Second Deployment
      break;
    }

    default:
      break;
  }
}

void RetainDeployment() {
  servo_a.write(pos_a);
}
