#ifndef ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H
#define ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H

#include <LibAvionics.h>
#include <SPI.h>
#include <ADXL372.h>
#include <SparkFun_BMP581_Arduino_Library.h>

class IMU_ADXL372 final : public SensorIMU {
protected:
  ADXL372class acc;
  float        axf{}, ayf{}, azf{};
  double       ax{}, ay{}, az{};

public:
  IMU_ADXL372(SPIClass &spi, const int cs) : SensorIMU(), acc(spi, cs) {
  }

  bool begin() override {
    acc.begin();
    acc.setOperatingMode(FULL_BANDWIDTH);
    acc.setOdr(ODR_6400Hz);
    acc.setBandwidth(BW_1600Hz);
    acc.setFilterSettling(FSP_16ms);
    acc.disableLowPassFilter(true);
    acc.disableHighPassFilter(true);
    return true;
  }

  bool read() override {
    acc.readAcceleration(axf, ayf, azf);
    ax = axf, ay = ayf, az = azf;
    ax *= 9.81, ay *= 9.81, az *= 9.81;
    return true;
  }

  double acc_x() override {
    return ax;
  }

  double acc_y() override {
    return ay;
  }

  double acc_z() override {
    return az;
  }

  double gyr_x() override {
    return 0.;
  }

  double gyr_y() override {
    return 0.;
  }

  double gyr_z() override {
    return 0.;
  }
};

class Altimeter_BMP581 final : public SensorAltimeter {
protected:
  BMP581                    bmp;
  bmp5_osr_odr_press_config bmp_osr =
    {
      .osr_t    = BMP5_OVERSAMPLING_8X,    // T Oversampling
      .osr_p    = BMP5_OVERSAMPLING_128X,  // P Oversampling
      .press_en = 0,                       // UNUSED
      .odr      = 0                        // UNUSED
    };
  bmp5_sensor_data data{};
  uint8_t          cs;

public:
  explicit Altimeter_BMP581(const uint8_t cs) : SensorAltimeter(), cs(cs) {
  }

  bool begin() override {
    return bmp.beginSPI(PC1, 10'000'000) == BMP5_OK &&
           bmp.setOSRMultipliers(&bmp_osr) == BMP5_OK &&
           bmp.setODRFrequency(BMP5_ODR_10_HZ) == BMP5_OK;
  }

  bool read() override {
    return bmp.getSensorData(&data) == BMP5_OK;
  }

  double pressure_hpa() override {
    return data.pressure * 0.01;  // Pa -> hPa
  }
};

class GNSS1 final : public SensorGNSS {
public:
  GNSS1() : SensorGNSS() {
  }

  bool begin() override {
    return false;
  }

  bool read() override {
    return false;
  }

  uint32_t timestamp_epoch() override {
    return 0ul;
  }

  uint8_t siv() override {
    return 0;
  }

  double latitude() override {
    return 0.;
  }

  double longitude() override {
    return 0.;
  }

  double altitude_msl() override {
    return 0.;
  }
};

#endif  //ROCKET_AVIONICS_TEMPLATE_USERSENSORS_H
