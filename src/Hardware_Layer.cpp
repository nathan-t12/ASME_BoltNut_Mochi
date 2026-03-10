#include "Hardware_Layer.h"
#include "config.h"
#include <Arduino.h>

Motor::Motor(float Kp, float Ki, float Kd, uint8_t pwmPin, uint8_t dirPin1, uint8_t dirPin2, uint8_t encoderPin1, uint8_t encoderPin2)
    : pwmPin(pwmPin), dirPin1(dirPin1), dirPin2(dirPin2), encoderPin1(encoderPin1), encoderPin2(encoderPin2), incrementPID(Kp, Ki, Kd), encoderFilter(FilterConfig::ALPHA_FIXED) {
    
}

void Motor::begin(){
    pinMode(pwmPin, OUTPUT);
    pinMode(dirPin1, OUTPUT);
    pinMode(dirPin2, OUTPUT);
    pinMode(encoderPin1, INPUT_PULLUP);
    pinMode(encoderPin2, INPUT_PULLUP);
}



void Motor::setSpeed(int16_t speed) {
    if (speed > 0) {
        digitalWrite(dirPin1, HIGH);
        digitalWrite(dirPin2, LOW);
    } else if (speed < 0) {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, HIGH);
        speed = -speed; // 取絕對值
    } else {
        digitalWrite(dirPin1, LOW);
        digitalWrite(dirPin2, LOW);
    }
    analogWrite(pwmPin, speed);
}

void Motor::updateEncoder(int16_t target) {
    noInterrupts();
    int16_t count =encoderCount;
    encoderCount=0;
    interrupts();
    currentSpeed = encoderFilter.update(count);
    lastPWM = incrementPID.compute(target, currentSpeed);
    setSpeed(lastPWM);
}

void Motor::encoderISR() {
    if (digitalRead(encoderPin2) == LOW) {
        encoderCount--;
    } else {
        encoderCount++;
    }
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