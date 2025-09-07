/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>          // Arduino Framework
#include <LibAvionics.h>      // Base Avionics Library and Utilities
#include "SystemFunctions.h"  // Function Declarations

#if __has_include("STM32FreeRTOS.h")
#  define USE_FREERTOS 1
#  include "hal_rtos.h"
#endif
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
UserFSM fsm;
double  acc;
/* END PERSISTENT STATE */

/* BEGIN DATA MEMORY */
struct DataMemory {
  SensorIMU::Data       imu[RA_NUM_IMU];
  SensorAltimeter::Data altimeter[RA_NUM_ALTIMETER];
  SensorGNSS::Data      gnss[RA_NUM_GNSS];
} data;
/* END DATA MEMORY */

/* BEGIN USER PRIVATE VARIABLES */
hal::rtos::mutex_t mtx_spi;
hal::rtos::mutex_t mtx_cdc;
/* END USER PRIVATE VARIABLES */

/* BEGIN USER SETUP */
void UserSetupGPIO() {
  pinMode(USER_GPIO_LED, OUTPUT);
}

void UserSetupCDC() {
  Serial.begin();
}

void UserSetupSPI() {
  SPI.setMOSI(PA7);
  SPI.setMISO(PA6);
  SPI.setSCLK(PA5);
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
  });
}

void CB_EvalFSM(void *) {
  hal::rtos::interval_loop(RA_INTERVAL_FSM_EVAL, [&]() -> void {
    EvalFSM();
  });
}

/* END USER THREADS */

void UserThreads() {
  hal::rtos::scheduler.create(CB_ReadIMU, {.stack_size = 2048, .priority = osPriorityRealtime});
  hal::rtos::scheduler.create(CB_ReadAltimeter, {.stack_size = 2048, .priority = osPriorityHigh});
}

void setup() {
  /* BEGIN GPIO AND INTERFACES SETUP */
  UserSetupGPIO();
  UserSetupCDC();
  UserSetupUSART();
  UserSetupI2C();
  UserSetupSPI();
  /* END GPIO AND INTERFACES SETUP */

  /* BEGIN STORAGES SETUP */
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
  static bool                          state_succeeded = false;
  static uint32_t                      state_millis    = 0;
  static xcore::sampler_t<128, double> sampler;

  switch (fsm.state()) {
    case UserState::STARTUP: {
      // <--- Next: always transfer --->
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
      if (fsm.is_transferred()) {  // Run once
        sampler.set_capacity(1, /*recount*/ false);
        sampler.set_threshold(1, /*recount*/ false);
        sampler.reset();
      }
      break;
    }

    case UserState::POWERED: {
      break;
    }

    case UserState::COASTING: {
      break;
    }

    case UserState::DROGUE_DEPLOY: {
      break;
    }

    case UserState::DROGUE_DESCEND: {
      break;
    }

    case UserState::MAIN_DEPLOY: {
      break;
    }

    case UserState::MAIN_DESCEND: {
      break;
    }

    case UserState::LANDED: {
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
