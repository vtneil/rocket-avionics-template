#ifndef ROCKET_AVIONICS_TEMPLATE_USERPINS_H
#define ROCKET_AVIONICS_TEMPLATE_USERPINS_H

#include <Arduino.h>

constexpr uint32_t USER_GPIO_LED     = PC6;
constexpr uint32_t USER_GPIO_SERVO_A = PB6;
constexpr uint32_t USER_GPIO_SERVO_B = PB5;

constexpr uint32_t USER_GPIO_SPI1_SCK  = PA5;
constexpr uint32_t USER_GPIO_SPI1_MISO = PA6;
constexpr uint32_t USER_GPIO_SPI1_MOSI = PA7;

constexpr uint32_t USER_GPIO_SDIO_CMD  = PD2;
constexpr uint32_t USER_GPIO_SDIO_CK   = PC12;
constexpr uint32_t USER_GPIO_SDIO_DAT0 = PB13;
constexpr uint32_t USER_GPIO_SDIO_DAT1 = PC9;
constexpr uint32_t USER_GPIO_SDIO_DAT2 = PC10;
constexpr uint32_t USER_GPIO_SDIO_DAT3 = PC11;

constexpr uint32_t USER_GPIO_BMP581_INT = PC0;
constexpr uint32_t USER_GPIO_BMP581_NSS = PC1;

constexpr uint32_t USER_GPIO_ADXL372_INT1 = PB2;
constexpr uint32_t USER_GPIO_ADXL372_INT2 = PB1;
constexpr uint32_t USER_GPIO_ADXL372_NSS  = PB0;

#endif  //ROCKET_AVIONICS_TEMPLATE_USERPINS_H
