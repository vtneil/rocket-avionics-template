#ifndef ROCKET_AVIONICS_TEMPLATE_SYSTEMFUNCTIONS_H
#define ROCKET_AVIONICS_TEMPLATE_SYSTEMFUNCTIONS_H

#include <Arduino.h>

void ReadIMU();

void ReadAltimeter();

void ReadGNSS();

void ActivateDeployment(size_t index);

#endif  //ROCKET_AVIONICS_TEMPLATE_SYSTEMFUNCTIONS_H
