// Beta test v0.1
#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>
#define STATE_WAITING   0
#define STATE_SEARCHING 1
#define STATE_ATTACK    2
#define STATE_ESCAPE    3
#define ESCAPE_BACK       0
#define ESCAPE_RIGHT_BACK 1
#define ESCAPE_LEFT_BACK  2
#define QTR_THRESHOLD 700
#define MAX_SPEED 102
#define SEARCH_SPEED 60
#define IMPACT_SPEED 170
#define MAX_CRUSH_SPEED 230
#define CRUSH_STEP_DELAY 300
#define CRUSH_STEP_INCREMENT 2
volatile uint8_t currentState = STATE_WAITING;
uint8_t escapeStrategy = 0; 
volatile unsigned long escapeStartTime = 0;
uint8_t lastOpponentDir = 0; 
unsigned long attackStartTime = 0;
int currentCrushSpeed = IMPACT_SPEED;
bool isCrushing = false;
volatile int global_solQTR = 1023;
volatile int global_sagQTR = 1023;
volatile uint8_t adc_channel = 5; 
volatile bool adc_dummy_flag = true; 
void initHardware();
void setMotorLeft(int16_t speed);
void setMotorRight(int16_t speed);
#define BUZZER_ON  (PORTB |=  (1 << PB5))
#define BUZZER_OFF (PORTB &= ~(1 << PB5))
void setup() {
    initHardware();
    BUZZER_ON;
    delay(100);
    BUZZER_OFF;
    delay(100);
    BUZZER_ON;
    delay(100);
    BUZZER_OFF;
    wdt_enable(WDTO_120MS); 
}
void loop() {
    wdt_reset(); 
    int solQTR, sagQTR;
    cli(); 
    solQTR = global_solQTR;
    sagQTR = global_sagQTR;
    sei(); 
    if (currentState != STATE_WAITING) {
        if (currentState != STATE_ESCAPE) {
            if (solQTR < QTR_THRESHOLD || sagQTR < QTR_THRESHOLD) {
                currentState = STATE_ESCAPE;
                escapeStartTime = millis();
                isCrushing = false; 
                if (solQTR < QTR_THRESHOLD && sagQTR < QTR_THRESHOLD) {
                    escapeStrategy = ESCAPE_BACK; 
                } else if (solQTR < QTR_THRESHOLD) {
                    escapeStrategy = ESCAPE_RIGHT_BACK; 
                } else {
                    escapeStrategy = ESCAPE_LEFT_BACK;  
                }
            }
        }
    }
    switch (currentState) {
        case STATE_WAITING:
            break;
        case STATE_SEARCHING: {
            bool leftOpp  = ((PINC & (1 << PC3)) == 0); 
            bool rightOpp = ((PINC & (1 << PC2)) == 0);
            if (leftOpp || rightOpp) {
                currentState = STATE_ATTACK;
            } else {
                if (lastOpponentDir == 0) { 
                    setMotorLeft(-SEARCH_SPEED);
                    setMotorRight(SEARCH_SPEED);
                } else {                    
                    setMotorLeft(SEARCH_SPEED);
                    setMotorRight(-SEARCH_SPEED);
                }
            }
            break;
        }
        case STATE_ATTACK: {
            bool leftOpp  = ((PINC & (1 << PC3)) == 0);
            bool rightOpp = ((PINC & (1 << PC2)) == 0);
            if (leftOpp && rightOpp) {
                if (!isCrushing) {
                    isCrushing = true;
                    attackStartTime = millis();
                    currentCrushSpeed = IMPACT_SPEED; 
                } else {
                    unsigned long crushDuration = millis() - attackStartTime;
                    if (crushDuration >= CRUSH_STEP_DELAY) {
                        currentCrushSpeed += CRUSH_STEP_INCREMENT;
                        if (currentCrushSpeed > MAX_CRUSH_SPEED) {
                            currentCrushSpeed = MAX_CRUSH_SPEED;
                        }
                        attackStartTime = millis(); 
                    }
                }
                setMotorLeft(currentCrushSpeed);
                setMotorRight(currentCrushSpeed);
            } else if (leftOpp) {
                isCrushing = false;
                setMotorLeft(0); 
                setMotorRight(MAX_SPEED);
                lastOpponentDir = 0; 
            } else if (rightOpp) {
                isCrushing = false;
                setMotorLeft(MAX_SPEED);
                setMotorRight(0); 
                lastOpponentDir = 1; 
            } else {
                isCrushing = false;
                currentState = STATE_SEARCHING;
            }
            break;
        }
        case STATE_ESCAPE: {
            unsigned long elapsed = millis() - escapeStartTime;
            bool sensorsClear = (solQTR >= QTR_THRESHOLD && sagQTR >= QTR_THRESHOLD);
            if (escapeStrategy == ESCAPE_BACK) {
                if (elapsed < 200) {
                    setMotorLeft(-MAX_SPEED);
                    setMotorRight(-MAX_SPEED);
                } else if (elapsed < 350) {
                    setMotorLeft(MAX_SPEED);
                    setMotorRight(-MAX_SPEED);
                } else if (sensorsClear || elapsed > 1000) {
                    currentState = STATE_SEARCHING;
                }
            } 
            else if (escapeStrategy == ESCAPE_RIGHT_BACK) {
                if (elapsed < 250) {
                    setMotorLeft(-MAX_SPEED);
                    setMotorRight(0); 
                } else if (sensorsClear || elapsed > 1000) {
                    currentState = STATE_SEARCHING;
                }
            }
            else  {
                if (elapsed < 250) {
                    setMotorLeft(0); 
                    setMotorRight(-MAX_SPEED);
                } else if (sensorsClear || elapsed > 1000) {
                    currentState = STATE_SEARCHING;
                }
            }
            break;
        }
    }
}
void initHardware() {
    DDRC  &= ~((1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5)); 
    PORTC |=  ((1 << PC2) | (1 << PC3)); 
    DDRD  |= (1 << PD3); 
    DDRB  |= (1 << PB3) | (1 << PB1) | (1 << PB2); 
    DDRB  |= (1 << PB5); 
    DDRD  &= ~(1 << PD2);
    PORTD |=  (1 << PD2);
    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10); 
    TCCR2A = (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22); 
    EICRA |= (1 << ISC00); 
    EIMSK |= (1 << INT0);  
    ADMUX = (1 << REFS0) | (5); 
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADSC);
    sei();
}
void setMotorLeft(int16_t speed) {
    cli(); if (currentState == STATE_WAITING) speed = 0; sei();
    if (speed > 0) {
        if (speed > 255) speed = 255;
        TCCR2A = (TCCR2A | (1 << COM2A1)) & ~(1 << COM2B1);
        PORTD &= ~(1 << PD3);
        OCR2A = speed;
    } else if (speed < 0) {
        speed = -speed;
        if (speed > 255) speed = 255;
        TCCR2A = (TCCR2A | (1 << COM2B1)) & ~(1 << COM2A1);
        PORTB &= ~(1 << PB3);
        OCR2B = speed;
    } else {
        TCCR2A &= ~((1 << COM2A1) | (1 << COM2B1));
        PORTD &= ~(1 << PD3);
        PORTB &= ~(1 << PB3);
    }
}
void setMotorRight(int16_t speed) {
    cli(); if (currentState == STATE_WAITING) speed = 0; sei();
    if (speed > 0) {
        if (speed > 255) speed = 255;
        TCCR1A = (TCCR1A | (1 << COM1A1)) & ~(1 << COM1B1);
        PORTB &= ~(1 << PB2);
        OCR1A = speed;
    } else if (speed < 0) {
        speed = -speed;
        if (speed > 255) speed = 255;
        TCCR1A = (TCCR1A | (1 << COM1B1)) & ~(1 << COM1A1);
        PORTB &= ~(1 << PB1);
        OCR1B = speed;
    } else {
        TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
        PORTB &= ~((1 << PB1) | (1 << PB2));
    }
}
ISR(INT0_vect) {
    if ((PIND & (1 << PD2)) == 0) { 
        currentState = STATE_SEARCHING;
    } else {
        currentState = STATE_WAITING;
        setMotorLeft(0);
        setMotorRight(0);
    }
}
ISR(ADC_vect) {
    if (adc_dummy_flag) {
        adc_dummy_flag = false;
        ADCSRA |= (1 << ADSC); 
    } else {
        int adc_val = ADC; 
        if (adc_channel == 5) {
            global_solQTR = adc_val; 
            adc_channel = 4;         
        } else {
            global_sagQTR = adc_val; 
            adc_channel = 5;         
        }
        ADMUX = (ADMUX & 0xF0) | adc_channel;
        adc_dummy_flag = true;
        ADCSRA |= (1 << ADSC);
    }
}
