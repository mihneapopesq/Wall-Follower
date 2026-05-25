#ifndef F_CPU
#define F_CPU 16000000UL // Frecventa procesorului
#endif
#include <avr/io.h>
#include <util/delay.h>

#define BASE_SPEED 200     // Viteza de baza a robotului
#define WALL_THRESH 15     // Pragul pentru detectarea peretilor laterali (cm)
#define FRONT_THRESH 15    // Pragul pentru detectarea obstacolului frontal (cm)

// Constante pentru algoritmul de control PID
float Kp = 5.0, Kd = 35.0; 
float oldError = 0;
int offset = 5; // Ajuta la ajustarea distantei fata de peretele urmarit

// Variabile pentru medierea citirilor de la senzori
float oldL = 0, oldR = 0, oldF = 0;

// Variabile de stare pentru deciziile de navigare
bool first_turn = false;
bool rightWallFollow = false, leftWallFollow = false;

void init_hardware() {
    // Configurare pini pentru directia motoarelor ca OUTPUT
    DDRD |= (1 << PD4) | (1 << PD5) | (1 << PD6) | (1 << PD7);
    // Configurare pini PWM (viteza) ca OUTPUT
    DDRB |= (1 << PB2) | (1 << PB3);
    
    // Configurare pini senzori ultrasonici (Trig = OUT, Echo = IN)
    DDRD |= (1 << PD2); DDRD &= ~(1 << PD3); // Senzor Stanga
    DDRB |= (1 << PB0); DDRB &= ~(1 << PB1); // Senzor Mijloc
    DDRB |= (1 << PB4); DDRB &= ~(1 << PB5); // Senzor Dreapta

    // Setari Timer1 pentru generare PWM (motor stanga)
    TCCR1A |= (1 << COM1B1) | (1 << WGM10);
    TCCR1B |= (1 << WGM12) | (1 << CS11); 
    
    // Setari Timer2 pentru generare PWM (motor dreapta)
    TCCR2A |= (1 << COM2A1) | (1 << WGM21) | (1 << WGM20);
    TCCR2B |= (1 << CS21); 
}

void set_motors(int speedL, int speedR) {
    // Setare directie motor stanga (inainte sau inapoi)
    if (speedL >= 0) { PORTD |= (1 << PD4); PORTD &= ~(1 << PD5); }
    else { PORTD &= ~(1 << PD4); PORTD |= (1 << PD5); speedL = -speedL; }
    
    // Setare directie motor dreapta
    if (speedR >= 0) { PORTD |= (1 << PD6); PORTD &= ~(1 << PD7); }
    else { PORTD &= ~(1 << PD6); PORTD |= (1 << PD7); speedR = -speedR; }

    // Aplicare valori PWM cu limitare maxima la 255
    OCR1B = (speedL > 255) ? 255 : speedL;
    OCR2A = (speedR > 255) ? 255 : speedR;
}

uint16_t read_distance(volatile uint8_t *port, volatile uint8_t *pin_reg, uint8_t trig, uint8_t echo) {
    // Generare puls scurt pe pinul Trigger
    *port &= ~(1 << trig); _delay_us(2);
    *port |= (1 << trig);  _delay_us(10);
    *port &= ~(1 << trig);

    // Asteptare incepere raspuns Echo cu protectie de blocare (timeout)
    uint32_t timeout = 600000;
    while (!(*pin_reg & (1 << echo))) { if (--timeout == 0) return 999; }
    
    // Masurare durata semnal Echo
    uint32_t duration = 0;
    while (*pin_reg & (1 << echo)) { duration++; _delay_us(1); if (duration > 20000) return 999; }
    
    // Conversie timp in centimetri
    return duration * 0.01715; 
}

int main() {
    init_hardware();

    while (1) {
        // 1. Citire distante
        float f = read_distance(&PORTB, &PINB, PB0, PB1);
        float r = read_distance(&PORTB, &PINB, PB4, PB5);
        float l = read_distance(&PORTD, &PIND, PD2, PD3);

        // Corectare valori eronate cand senzorul nu vede nimic (timeout)
        if (f == 999) f = 50; 
        if (r == 999) r = 50; 
        if (l == 999) l = 50;

        // 2. Mediere cu citirea anterioara pentru a elimina erorile bruste de citire
        float front = (f + oldF) / 2.0; oldF = front;
        float right = (r + oldR) / 2.0; oldR = right;
        float left = (l + oldL) / 2.0;  oldL = left;

        // 3. Verificare prezenta pereti pe baza pragurilor setate
        bool frontwall = (front < FRONT_THRESH);
        bool rightwall = (right < WALL_THRESH);
        bool leftwall = (left < WALL_THRESH);

        // 4. Setare logica de urmarire la prima intalnire a unui colt/perete
        if (leftwall && !rightwall && frontwall && !first_turn) {
            first_turn = true;
            rightWallFollow = true; // Alege sa urmareasca peretele din dreapta
        }
        if (!leftwall && rightwall && frontwall && !first_turn) {
            first_turn = true;
            leftWallFollow = true;  // Alege sa urmareasca peretele din stanga
        }

        // 5. Luarea deciziilor de miscare
        if (frontwall) {
            // Daca are perete in fata, se intoarce catre partea cu mai mult spatiu liber
            if (right > left) {
                set_motors(255, -255); // Intoarcere la dreapta pe loc
            } else {
                set_motors(-255, 255); // Intoarcere la stanga pe loc
            }
            _delay_ms(350);  // Timpul necesar pentru rotatie
            oldError = 0;    // Resetare eroare PID dupa viraj
        } else {
            // Nu are obstacol frontal -> merge inainte si se centreaza intre pereti
            
            float error = left - right; // Calculam deviatia

            // Ajustam eroarea in functie de peretele pe care ne-am decis sa il urmarim
            if (rightWallFollow) error -= offset;
            else if (leftWallFollow) error += offset;

            // Calculare componenta derivativa a PID-ului
            float deriv = error - oldError;
            oldError = error;
            
            // Calculare eroare totala PID
            int totalError = (Kp * error) + (Kd * deriv);
            
            // Aplicare corectii la viteza de baza a rotilor (RMS = dreapta, LMS = stanga)
            int RMS = BASE_SPEED + totalError;
            int LMS = BASE_SPEED - totalError;

            // Limitare de siguranta a vitezelor in intervalul valid pentru PWM [0, 255]
            if (RMS > 255) { RMS = 255; }
            if (RMS < 0)   { RMS = 0; }
            if (LMS > 255) { LMS = 255; }
            if (LMS < 0)   { LMS = 0; }

            // Trimitere comenzi finale catre motoare
            set_motors(LMS, RMS);
        }

        // Scurta pauza inainte de urmatoarea citire (esantionare)
        _delay_ms(50);
    }
}