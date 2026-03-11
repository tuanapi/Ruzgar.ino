#include <avr/io.h>
#include <avr/interrupt.h>

// =========================================================
// PIN TABLOSU (kılavuz sayfa 8)
//   Sol Fuchs rakip  → A3 (PC3)  dijital, pull-up, 0=VAR
//   Sağ Fuchs rakip  → A2 (PC2)  dijital, pull-up, 0=VAR
//   Sol QTR zemin    → A5 (PC5)  analog,  düşük=beyaz
//   Sağ QTR zemin    → A4 (PC4)  analog,  düşük=beyaz
//   QTR_THRESHOLD    → 700 (kalibrasyon gerekli!)
// =========================================================

#define QTR_THRESHOLD 700

volatile int     global_solQTR  = 1023;
volatile int     global_sagQTR  = 1023;
volatile uint8_t adc_channel    = 5;
volatile bool    adc_dummy_flag = true;

void printPadded(int val) {
    if (val < 1000) Serial.print(' ');
    if (val < 100)  Serial.print(' ');
    if (val < 10)   Serial.print(' ');
    Serial.print(val);
}

void setup() {
    DDRC  &= ~((1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5));
    PORTC |=  ((1 << PC2) | (1 << PC3)); // pull-up sadece Fuchs pinlerine

    ADMUX  = (1 << REFS0) | 5;
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADSC);
    sei();

    Serial.begin(115200);
    Serial.println();
    Serial.println(F("╔══════════════════════════════════════╗"));
    Serial.println(F("║   SENJU DEBUG - Sensor Test Araci   ║"));
    Serial.println(F("╠══════════════════════════════════════╣"));
    Serial.println(F("║  QTR  : dusuk  = beyaz zemin        ║"));
    Serial.println(F("║         yuksek = siyah zemin        ║"));
    Serial.println(F("║  Fuchs: 0 = nesne VAR (LOW)         ║"));
    Serial.println(F("║         1 = nesne YOK (pull-up)     ║"));
    Serial.println(F("║  Threshold: 700 (kalibrasyon lazim) ║"));
    Serial.println(F("╚══════════════════════════════════════╝"));
    Serial.println();
}

void loop() {
    int solQTR, sagQTR;
    cli();
    solQTR = global_solQTR;
    sagQTR = global_sagQTR;
    sei();

    uint8_t pinc     = PINC;
    uint8_t solFuchs = (pinc >> PC3) & 1;  // 0=VAR, 1=yok
    uint8_t sagFuchs = (pinc >> PC2) & 1;

    bool solBeyaz = (solQTR  < QTR_THRESHOLD);
    bool sagBeyaz = (sagQTR  < QTR_THRESHOLD);
    bool solRakip = (solFuchs == 0);
    bool sagRakip = (sagFuchs == 0);

    Serial.println(F("┌─────────────────────────────────────┐"));

    Serial.print(F("│  Sol QTR  (A5) : "));
    printPadded(solQTR);
    Serial.print(F("  ->  "));
    if (solBeyaz) Serial.print(F("BEYAZ ")); else Serial.print(F("siyah "));
    Serial.println(F("         │"));

    Serial.print(F("│  Sag QTR  (A4) : "));
    printPadded(sagQTR);
    Serial.print(F("  ->  "));
    if (sagBeyaz) Serial.print(F("BEYAZ ")); else Serial.print(F("siyah "));
    Serial.println(F("         │"));

    Serial.println(F("├─────────────────────────────────────┤"));

    Serial.print(F("│  Sol Fuchs(A3) :    "));
    Serial.print(solFuchs);
    Serial.print(F("  ->  "));
    if (solRakip) Serial.print(F("RAKIP VAR ")); else Serial.print(F("rakip yok "));
    Serial.println(F("      │"));

    Serial.print(F("│  Sag Fuchs(A2) :    "));
    Serial.print(sagFuchs);
    Serial.print(F("  ->  "));
    if (sagRakip) Serial.print(F("RAKIP VAR ")); else Serial.print(F("rakip yok "));
    Serial.println(F("      │"));

    Serial.println(F("└─────────────────────────────────────┘"));
    Serial.println();

    delay(500);
}

ISR(ADC_vect) {
    if (adc_dummy_flag) {
        adc_dummy_flag = false;
        ADCSRA |= (1 << ADSC);
    } else {
        int adc_val = ADC;
        if (adc_channel == 5) {
            global_solQTR = adc_val;
            adc_channel   = 4;
        } else {
            global_sagQTR = adc_val;
            adc_channel   = 5;
        }
        ADMUX = (ADMUX & 0xF0) | adc_channel;
        adc_dummy_flag = true;
        ADCSRA |= (1 << ADSC);
    }
}
