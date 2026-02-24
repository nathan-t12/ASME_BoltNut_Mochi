#include "Hardware_Layer.h"

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