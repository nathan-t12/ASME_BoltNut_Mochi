#include "Hardware_Layer.h"

Motor::Motor(float Kp, float Ki, float Kd, uint8_t pwmPin, uint8_t dirPin1, uint8_t dirPin2, uint8_t encoderPin)
    : pwmPin(pwmPin), dirPin1(dirPin1), dirPin2(dirPin2), encoderPin(encoderPin), speedPID(Kp, Ki, Kd) {
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(encoderPin, INPUT_PULLUP);
}



volatile bool timerFlag = false;

void initTimer1_20ms() {
    noInterrupts();
    TCCR1A = 0;
    TCCR1B = 0;
    TCNT1 = 0;
    OCR1A = 39999;            // 20ms @ 16MHz, Prescaler 8
    TCCR1B |= (1 << WGM12);   // CTC Mode
    TCCR1B |= (1 << CS11);    // Prescaler 8
    TIMSK1 |= (1 << OCIE1A);  // Enable Interrupt
    interrupts();
}

ISR(TIMER1_COMPA_vect) {
    timerFlag = true;
}


// ISR(TIMER1_COMPA_vect) {
//     timerFlag = true;
//     static bool state = false;
//     state = !state;
//     digitalWrite(13, state); 
// }