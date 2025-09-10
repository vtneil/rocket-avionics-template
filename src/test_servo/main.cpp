/* BEGIN INCLUDE SYSTEM LIBRARIES */
#include <Arduino.h>  // Arduino Framework
                      // #include <STM32Servo.h>
                      /* END INCLUDE SYSTEM LIBRARIES */

// STM32ServoList servos(TIMER_SERVO);

/* END USER THREADS */
void tick() {
  digitalWrite(PC6, !digitalRead(PC6));
}

HardwareTimer *htim;

void setup() {
  pinMode(PC6, OUTPUT);
  digitalWrite(PC6, HIGH);

  htim = new HardwareTimer(TIM7);
  htim->setPrescaleFactor(htim->getTimerClkFreq() / 1'000'000ul);
  htim->setOverflow(2, HERTZ_FORMAT);
  htim->attachInterrupt(tick);
  htim->resume();
}

void loop() {
}