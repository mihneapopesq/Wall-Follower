#define F_CPU 16000000UL
#include <avr/io.h>
#include <util/delay.h>

// Setari PID si navigare
#define DIST_REF 10      // Distanta dorita fata de perete (cm)
#define BASE_SPEED 150   // Viteza de baza (0-255)
#define Kp 3.0           // Constanta proportionala
#define Kd 1.5           // Constanta derivativa

int last_error = 0;

void init_hardware() {
    // Pini Directie Motoare (PD4, PD5, PD6, PD7) ca OUTPUT
    DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
    
    // Pini PWM: ENA(PB2/D10), ENB(PB3/D11) ca OUTPUT
    DDRB |= (1 << PB2) | (1 << PB3);
    
    // Pini Senzori (Trig=OUT, Echo=IN)
    DDRD |= (1 << PD2); DDRD &= ~(1 << PD3); // Stanga: Trig D2, Echo D3
    DDRB |= (1 << PB0); DDRB &= ~(1 << PB1); // Mijloc: Trig D8, Echo D9
    DDRB |= (1 << PB4); DDRB &= ~(1 << PB5); // Dreapta: Trig D12, Echo D13

    // Configurare Timer1 (PWM 8-bit pe PB2 - ENA)
    TCCR1A |= (1 << COM1B1) | (1 << WGM10);
    TCCR1B |= (1 << WGM12) | (1 << CS11); // Prescaler 8
    
    // Configurare Timer2 (Fast PWM pe PB3 - ENB)
    TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << CS21); // Prescaler 8
}

void set_motors(int speedL, int speedR) {
    // Directie Stanga (IN1=PD4, IN2=PD5)
    if (speedL >= 0) { PORTD |= (1 << PD4); PORTD &= ~(1 << PD5); }
    else { PORTD &= ~(1 << PD4); PORTD |= (1 << PD5); speedL = -speedL; }
    
    // Directie Dreapta (IN3=PD6, IN4=PD7)
    if (speedR >= 0) { PORTD |= (1 << PD6); PORTD &= ~(1 << PD7); }
    else { PORTD &= ~(1 << PD6); PORTD |= (1 << PD7); speedR = -speedR; }

    // Aplicare PWM cu limitare 0-255
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
    
    return duration * 0.01715; // Conversie in cm
}

int main() {
    init_hardware();
    //DIS = 20 pentru spatiu de franare, dar folosimcommit forward
    int DIS = 20; 

    while (1) {
        uint16_t dFront = read_distance(&PORTB, &PINB, PB0, PB1);
        
        if (dFront < DIS) {
            // S-a oprit la zid.
            set_motors(0, 0);
            _delay_ms(200); // Pauza scurta sa se stabilizeze senzorii
            
            uint16_t dRight = read_distance(&PORTB, &PINB, PB4, PB5);
            uint16_t dLeft  = read_distance(&PORTD, &PIND, PD2, PD3);

            if (dRight > dLeft) {
                // Dreapta e mai libera -> Intoarcere ~90 grade dreapta
                set_motors(255, -255); 
                _delay_ms(350); 
            } else {
                // Stanga e mai libera -> Intoarcere ~90 grade stanga
                set_motors(-255, 255); 
                _delay_ms(350); 
            }

            // Faza obligatorie de mers inainte "commit"
            // Aceasta forteaza robotul sa iasa din zona de conflict
            // Mergem inainte un pic (de exemplu 200ms) 
            set_motors(255, 255); 
            _delay_ms(200);
            
            // last_error = 0;        // PID logic reset is important
            // set_motors(0, 0); // Faza de cooldown - oprim motoarele
            // _delay_ms(100);

            // last_error = 0;        // Reset PID after maneuver
            continue;              // Sarim peste restul logicii in acest ciclu
        } 
        else {
            // Nu are obstacol in fata -> inainte
            set_motors(255, 255);
        }
        
        _delay_ms(50);
    }
}