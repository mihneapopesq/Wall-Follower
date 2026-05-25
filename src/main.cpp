#ifndef F_CPU
#define F_CPU 16000000UL
#endif
#include <avr/io.h>
#include <util/delay.h>

#define BASE_SPEED 200
#define WALL_THRESH 15
#define FRONT_THRESH 15

float Kp = 5.0, Kd = 35.0; 
float oldError = 0;
int offset = 5;

float oldL = 0, oldR = 0, oldF = 0;
bool first_turn = false;
bool rightWallFollow = false, leftWallFollow = false;

void init_hardware() {
    DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
    DDRB |= (1 << PB2) | (1 << PB3);
    DDRD |= (1 << PD2); DDRD &= ~(1 << PD3); // Stânga
    DDRB |= (1 << PB0); DDRB &= ~(1 << PB1); // Mijloc
    DDRB |= (1 << PB4); DDRB &= ~(1 << PB5); // Dreapta

    TCCR1A |= (1 << COM1B1) | (1 << WGM10);
    TCCR1B |= (1 << WGM12) | (1 << CS11); 
    
    TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << CS21); 
}

void set_motors(int speedL, int speedR) {
    if (speedL >= 0) { PORTD |= (1 << PD4); PORTD &= ~(1 << PD5); }
    else { PORTD &= ~(1 << PD4); PORTD |= (1 << PD5); speedL = -speedL; }
    
    if (speedR >= 0) { PORTD |= (1 << PD6); PORTD &= ~(1 << PD7); }
    else { PORTD &= ~(1 << PD6); PORTD |= (1 << PD7); speedR = -speedR; }

    OCR1B = (speedL > 255) ? 255 : speedL;
    OCR2A = (speedR > 255) ? 255 : speedR;
}

uint16_t read_distance(volatile uint8_t *port, volatile uint8_t *pin_reg, uint8_t trig, uint8_t echo) {
    *port &= ~(1 << trig); _delay_us(2);
    *port |= (1 << trig);  _delay_us(10);
    *port &= ~(1 << trig);

    uint32_t timeout = 600000;
    while (!(*pin_reg & (1 << echo))) { if (--timeout == 0) return 999; }
    
    uint32_t duration = 0;
    while (*pin_reg & (1 << echo)) { duration++; _delay_us(1); if (duration > 20000) return 999; }
    
    return duration * 0.01715; 
}

int main() {
    init_hardware();

    while (1) {
        float f = read_distance(&PORTB, &PINB, PB0, PB1);
        float r = read_distance(&PORTB, &PINB, PB4, PB5);
        float l = read_distance(&PORTD, &PIND, PD2, PD3);

        if (f == 999) f = 50; 
        if (r == 999) r = 50; 
        if (l == 999) l = 50;

        float front = (f + oldF) / 2.0; oldF = front;
        float right = (r + oldR) / 2.0; oldR = right;
        float left = (l + oldL) / 2.0;  oldL = left;

        bool frontwall = (front < FRONT_THRESH);
        bool rightwall = (right < WALL_THRESH);
        bool leftwall = (left < WALL_THRESH);

        if (leftwall && !rightwall && frontwall && !first_turn) {
            first_turn = true;
            rightWallFollow = true;
        }
        if (!leftwall && rightwall && frontwall && !first_turn) {
            first_turn = true;
            leftWallFollow = true;
        }

        if (frontwall) {
            if (right > left) {
                set_motors(255, -255);
            } else {
                set_motors(-255, 255);
            }
            _delay_ms(350);
            oldError = 0; 
        } else {
            float error = left - right;
            if (rightWallFollow) error -= offset;
            else if (leftWallFollow) error += offset;

            float deriv = error - oldError;
            oldError = error;
            
            int totalError = (Kp * error) + (Kd * deriv);
            int RMS = BASE_SPEED + totalError;
            int LMS = BASE_SPEED - totalError;

            if (RMS > 255) { RMS = 255; }
            if (RMS < 0)   { RMS = 0; }
            if (LMS > 255) { LMS = 255; }
            if (LMS < 0)   { LMS = 0; }

            set_motors(LMS, RMS);
        }

        _delay_ms(50);
    }
}