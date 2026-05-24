# Proiect: Robot Autonom (Maze Solver)

## 1. Stadiul actual al implementării software
Proiectul se află în stadiu final, cu un software complet funcțional. Implementarea curentă utilizează un algoritm de tip "Stop & Turn" adaptat pentru navigarea în labirinturi strâmte. Robotul se deplasează autonom, detectează obstacolele frontale, scanează mediul lateral și ia decizii de schimbare a direcției bazate pe spațiul disponibil, executând manevre de evitare și reorientare.

## 2. Motivarea alegerii bibliotecilor
Pentru acest proiect s-a optat pentru programarea de tip "bare-metal", minimizând dependențele externe:
* `<avr/io.h>`: Utilizată pentru accesul direct și lucrul la nivel de regiștri (PORTB, PORTD, Timere). Oferă o eficiență superioară și un control mult mai precis asupra hardware-ului comparativ cu framework-urile de nivel înalt (ex: Arduino core).
* `<util/delay.h>`: Necesară pentru generarea triggerelor precise de microsecunde cerute de senzorii ultrasonici și pentru temporizarea manevrelor (rotații de 90 de grade, faza de "commit").

## 3. Elementul de noutate
Spre deosebire de un robot clasic de tip "wall-follower" care depinde exclusiv de un algoritm PID (care poate oscila puternic în medii înguste din cauza limitărilor de putere sau frecare), acest proiect implementează o **logică de evadare decizională cu "commit forward"**. Atunci când detectează o înfundătură, robotul nu se rotește la întâmplare, ci compară distanțele stânga-dreapta. Mai mult, după rotație, execută un pas forțat înainte (orb) de 200ms pentru a părăsi zona de conflict, prevenind buclele recursive de întoarcere pe loc.

## 4. Utilizarea funcționalităților din laborator
* **Timere și Semnale PWM:** Timer1 și Timer2 au fost configurate pentru a genera semnale PWM pe pinii PB2 (ENA) și PB3 (ENB). Acestea controlează puntea H (L298N) pentru ajustarea puterii motoarelor.
* **GPIO (General Purpose Input/Output):** Configurați specific pentru controlul direcției motoarelor (pinii PD4-PD7) și pentru comunicarea cu cei 3 senzori ultrasonici HC-SR04 (pini setați dinamic ca Output pentru Trigger și Input pentru Echo).
* **Întârzieri și Polling:** Citirea pinilor digitali în buclă (polling) pentru a măsura durata semnalului de întoarcere de la senzori.

## 5. Scheletul proiectului, interacțiunea și validarea
**Schelet și Interacțiune:**
1.  `init_hardware()`: Setează direcția pinilor și configurează registrele Timer1/Timer2 pentru PWM hardware.
2.  `read_distance()`: Funcție de citire a senzorilor. Trimite un impuls de 10µs și măsoară timpul până la recepția ecoului. Include un sistem de timeout de siguranță.
3.  `set_motors()`: Abstracție care mapează vitezele dorite (-255 la 255) direct în comenzi de direcție pentru puntea H și valori în registrele OCR1B/OCR2A.
4.  `main()`: Bucla infinită de control. Interoghează senzorul frontal. Dacă traseul e liber, comandă mers înainte. Dacă detectează obstacol, oprește motoarele, interoghează senzorii laterali, decide direcția, apelează `set_motors` pentru rotație, urmat de un pas de commit.

**Validare:** Sistemul a fost testat inițial cu un algoritm PD (Proporțional-Derivativ), care a fost invalidat fizic de fluctuațiile de tensiune pe suprafețe cu frecare (covor). Validarea finală s-a făcut prin teste empirice (trial and error) care au dus la simplificarea logicii către modelul actual, dovedit a fi stabil pe constrângerile hardware date.

## 6. Calibrarea elementelor de senzoristică
Senzorii HC-SR04 necesită calibrare la nivel de software pentru a funcționa corect:
* **Conversia timpului în distanță:** S-a utilizat constanta `0.01715` aplicată duratei pulsului. (Viteza sunetului fiind ~343 m/s, distanța dus-întors înseamnă împărțirea la 2 a valorii teoretice).
* **Timeout-ul (Siguranța):** Senzorii sunt susceptibili la a bloca microcontrolerul într-o buclă `while` dacă unda nu se întoarce (spațiu prea deschis sau unghi ascuțit defavorabil). S-a calibrat o limită superioară (`if (duration > 20000)`) care forțează returnarea valorii `999` (simulând o distanță sigură infinită).
* **Pragul de reacție (`DIS = 20`):** Valoarea a fost calibrată empiric ținând cont de inerția motoarelor și frecarea pardoselii, oferind distanța de frânare optimă înainte de impact.

## 7. Optimizări realizate (Cum, de ce, unde)
* **Optimizare logică (Unde: `main`, De ce: prevenirea oscilațiilor):** S-a renunțat la implementarea inițială PID cu ajustări continui care provocau mișcări în zig-zag și derapaje. Am înlocuit cu decizii discrete la intersecții ("Stop & Turn"), optimizând fluiditatea navigării în spații extrem de limitate.
* **Optimizare hardware-software (Unde: `set_motors`, De ce: lipsă de cuplu):** Vitezele intermediare PWM s-au dovedit ineficiente din cauza căderilor de tensiune interne ale driverului L298N. Optimizarea a constat în trecerea la regim binar de putere (0 sau 255) pentru a garanta depășirea forței de frecare la pornirea de pe loc.
* **Optimizare de zgomot (Unde: `main`, De ce: citiri eronate):** S-au introdus delay-uri scurte de stabilizare (ex: `_delay_ms(200)` la oprire) înainte de citirea senzorilor laterali, prevenind deciziile bazate pe citiri luate în timpul mișcării.
