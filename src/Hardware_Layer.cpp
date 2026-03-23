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
    int16_t commanded = speed;

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
    lastPWM = commanded;
}

void Motor::updateEncoder(int16_t target) {
    noInterrupts();
    int16_t count = encoderCount;
    encoderCount = 0;
    interrupts();

    // Ignore impossible spikes caused by electrical noise.
    if (abs(count) > 2000) {
        count = 0;
    }

    currentSpeed = encoderFilter.update(count);
    lastPWM = incrementPID.compute(target, currentSpeed);
    setSpeed(lastPWM);
}

void Motor::updateFeedbackOnly() {
    noInterrupts();
    int16_t count = encoderCount;
    encoderCount = 0;
    interrupts();

    if (abs(count) > 2000) {
        count = 0;
    }

    currentSpeed = encoderFilter.update(count);
}

void Motor::stopAndResetControl() {
    noInterrupts();
    encoderCount = 0;
    interrupts();

    incrementPID.reset();
    encoderFilter.reset();
    currentSpeed = 0;
    lastPWM = 0;
    setSpeed(0);
}

void Motor::encoderISR() {
    if (digitalRead(encoderPin2) == LOW) {
        encoderCount--;
    } else {
        encoderCount++;
    }
}

volatile bool timerFlag = false;

void initControlTimer20ms() {
    noInterrupts();
    // Use Timer2 (8-bit) to avoid Servo library conflicts on 16-bit timers.
    // 16MHz / 1024 = 15625Hz, OCR2A=155 -> ~9.984ms per interrupt.
    // Accumulate 2 ticks to get ~19.968ms control period.
    TCCR2A = 0;
    TCCR2B = 0;
    TCNT2 = 0;
    OCR2A = 155;
    TCCR2A |= (1 << WGM21);                       // CTC Mode
    TCCR2B |= (1 << CS22) | (1 << CS21) | (1 << CS20); // Prescaler 1024
    TIMSK2 |= (1 << OCIE2A);                      // Enable Compare Match A interrupt
    interrupts();
}

ISR(TIMER2_COMPA_vect) {
    static uint8_t tickDiv = 0;
    tickDiv++;
    if (tickDiv >= 2) {
        tickDiv = 0;
        timerFlag = true;
    }
}


// ISR(TIMER1_COMPA_vect) {
//     timerFlag = true;
//     static bool state = false;
//     state = !state;
//     digitalWrite(13, state); 
// }