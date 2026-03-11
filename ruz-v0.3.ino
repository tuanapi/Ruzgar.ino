#include <avr/io.h>
#include <avr/interrupt.h>
#include <avr/wdt.h>

// BARE METAL MAGIC: Bootloader'dan (Optiboot) önce çalışıp MCUSR registerını
// kurtaran .init3 bloğu. WDT reset sonrası otonom canlanmayı sağlar.
// volatile: .init3 yazar, setup() okur. LTO açıksa derleyici bu yazımı
// "görünmez" sayabilir — volatile garanti sağlar, sıfır maliyet.
volatile uint8_t mcusr_mirror __attribute__ ((section (".noinit")));
// WDT RESET GÜVENLİĞİ: Robot maç bitmişken (STATE_WAITING) WDT reset atarsa
// direkt STATE_SEARCHING'e geçmemesi için önceki state .noinit'te saklanır.
// .noinit: WDT/Brown-out reset'te RAM temizlenmez, değer korunur.
// Power-on reset'te rastgele ama bu zaten WDRF=0 → else dalı → sorun yok.
volatile uint8_t savedState __attribute__ ((section (".noinit")));
void get_mcusr(void) __attribute__((naked)) __attribute__((section(".init3")));
void get_mcusr(void) {
    mcusr_mirror = MCUSR;
    MCUSR = 0;
    wdt_disable();
}

#define STATE_WAITING   0
#define STATE_SEARCHING 1
#define STATE_ATTACK    2
#define STATE_ESCAPE    3
#define ESCAPE_BACK       0
#define ESCAPE_RIGHT_BACK 1
#define ESCAPE_LEFT_BACK  2
#define QTR_THRESHOLD 700
#define MAX_SPEED 102
#define SEARCH_SPEED 80        // Arama hızı — düşük hızda motorlar stall yapabilir
#define SEARCH_ARC_INNER 45    // Arc fazında iç tekerlek — tam durma yerine yavaş ileri
#define MAX_CRUSH_SPEED 230  // Saldırı hızı — her iki sensör aktifken direkt bu hız

volatile uint8_t  currentState    = STATE_WAITING;
uint8_t           escapeStrategy  = 0;
// FIX 3: ISR-shared multi-byte variables must be volatile so the compiler never
//         caches them in registers across loop iterations.
volatile unsigned long escapeStartTime  = 0;
volatile unsigned long escapeTurnStart  = 0; // Sensörler temizlenince set edilir, turn fazı buna göre ölçülür
// FIX 12: Written by INT0_vect START branch (lastOpponentDir = preScanDir),
//          read by loop() in ATTACK/ESCAPE/SEARCHING. Without volatile the
//          compiler may cache it in a register and loop() never sees the ISR's
//          write → angled crush fires in wrong direction after pre-scan START.
volatile uint8_t  lastOpponentDir = 0; // 0=sol, 1=sağ

// PRE-SCAN: Sinyal bekleme sırasında rakibin konumunu kaydeder.
// loop() yazar, INT0_vect START branch okur → volatile zorunlu.
volatile bool    preScanFound = false;
volatile uint8_t preScanDir   = 0; // 0=sol, 1=sağ, (her ikisi görünürse önceki yön)
uint8_t           currentCrushSpeed    = MAX_CRUSH_SPEED;
// FIX 3: isCrushing is written by INT0 ISR → must be volatile.
volatile bool     isCrushing      = false;
volatile unsigned long oppLostTime      = 0;
volatile unsigned long searchStartTime  = 0; // Hibrit arama zamanlayıcısı (spin -> arc)

// ADAPTİF PİVOT: Rakip yana geçtiği anı kaydeder.
// Süreye göre dönüş hızı artar — yeni kaydıysa nazik, kaçıyorsa agresif.
// Sadece loop() yazar, ISR yazmaz → volatile gerekmez.
volatile unsigned long sideStartTime = 0; // FIX 11: written by INT0_vect → must be volatile

#define OPP_LOST_DELAY   50    // Rakip kayboldu demek için gereken MS
#define SEARCH_SPIN_PHASE 300  // İlk 300ms saf dönüş (kör nokta taraması)
#define SEARCH_TIMEOUT   2000  // 2 saniye hedefsiz kalınca tarama yönünü çevir

// Adaptif pivot kademeleri (sideStartTime'dan itibaren geçen süreye göre):
//   0-200ms  : Küçük sapma, nazik düzelt  -> her iki tekerlek ileri, dış daha hızlı
//   200-500ms: Kaçmaya başlıyor           -> dış hızlı, iç yavaş
//   500ms+   : Aktif kaçış, tam agresif   -> dış tam, iç geri
// NOT: İç tekerlek 0-200ms arasında pozitif — robot ileri giderken döner,
//      bu sayede overshoot olmaz ve rakip üzerine kapanır.
#define PIVOT_OUTER_SLOW  100
#define PIVOT_OUTER_MID   150
#define PIVOT_OUTER_FAST  180
#define PIVOT_INNER_SLOW  50
#define PIVOT_INNER_MID   20
#define PIVOT_INNER_FAST -60

// ANGLED CRUSH: Her iki sensör aktifken lastOpponentDir'e göre
// iç tekerleğe %12.5 daha az hız verilir (>> 3 = /8).
// Rakip köşeye sıkışmışsa açılı baskıyla dışarı kaydırılır.
#define ANGLE_SHIFT 3

// PING-PONG ADC (Otonom Kesme) Değişkenleri
volatile int     global_solQTR   = 1023;
volatile int     global_sagQTR   = 1023;
volatile uint8_t adc_channel     = 5; // A5 sol QTR sensörüyle başla
volatile bool    adc_dummy_flag  = true;

// FIX 4: stopMotors() must be able to reset the static currentDir variables
//         inside setMotorLeft/Right so that the first PWM pulse after a
//         stop→start cycle correctly applies the 5µs shoot-through deadtime.
//         Exposing them as file-scope variables is the cleanest fix without
//         restructuring the motor API.
// FIX 7: Written by stopMotors() which is called from INT0 ISR, read in loop().
//         Without volatile the compiler may cache these in registers and loop()
//         would never see the ISR's reset to 0, causing missing deadtime on
//         the first PWM pulse after a stop→start cycle.
volatile int8_t motorLeftDir  = 0;
volatile int8_t motorRightDir = 0;

void initHardware();
void setMotorLeft(int16_t speed);
void setMotorRight(int16_t speed);
void stopMotors(); // ISR-güvenli motor durdurma (sei() çağırmaz)
// Escape/Attack sonrası STATE_SEARCHING'e geçiş — 6 yerde tekrar eden
// aynı blok buraya taşındı. cli/sei ile atomik state değişimi.
void transitionToSearching() {
    setMotorLeft(0);
    setMotorRight(0);
    unsigned long now = millis();
    cli();
    if (currentState != STATE_WAITING) {
        isCrushing      = false;
        searchStartTime = now;
        currentState    = STATE_SEARCHING;
    }
    sei();
}
#define BUZZER_ON  (PORTB |=  (1 << PB5))
#define BUZZER_OFF (PORTB &= ~(1 << PB5))

void setup() {
    uint8_t resetCause = mcusr_mirror;
    mcusr_mirror = 0;
    wdt_disable();

    initHardware();

    if (resetCause & (1 << WDRF)) {
        // Watchdog reset — robot maç ortasında çöktü.
        // Ama maç bitmişse (STATE_WAITING) tekrar başlatma — hakem şaşırmasın.
        BUZZER_ON;
        delay(50);
        BUZZER_OFF;
        // FIX 6: Reset all stale combat state so we don't re-enter ATTACK or
        //         ESCAPE with garbage timestamps or a stuck isCrushing flag.
        isCrushing        = false;
        oppLostTime       = 0;
        escapeStartTime   = 0;
        escapeTurnStart   = 0;
        currentCrushSpeed = MAX_CRUSH_SPEED;
        sideStartTime     = 0;
        preScanFound      = false;
        // FIX 10: AVR RAM is NOT cleared on WDT reset. motorLeftDir/motorRightDir
        //         survive from the crash moment. A stale non-zero value causes the
        //         first setMotor call to skip the 5us shoot-through deadtime.
        motorLeftDir      = 0;
        motorRightDir     = 0;
        // savedState kontrolü: maç bitmişken (STATE_WAITING) WDT reset attıysa
        // direkt savaşa dönme. Maç devam ediyorken çöktüyse arama başlat.
        if (savedState == STATE_WAITING) {
            currentState = STATE_WAITING;  // Maç bitti, bekle
        } else {
            currentState    = STATE_SEARCHING;  // Maç devam ediyordu, savaşa dön
            searchStartTime = millis();
        }
    } else {
        // Normal açılış
        BUZZER_ON;
        delay(100);
        BUZZER_OFF;
        delay(100);
        BUZZER_ON;
        delay(100);
        BUZZER_OFF;
    }

    wdt_enable(WDTO_250MS);
    // INT0 en son açılıyor: boot sırası ve buzzer bittikten sonra.
    // Salon IR gürültüsü artık setup() sırasında robotu başlatamaz.
    EIMSK |= (1 << INT0);
}

void loop() {
    wdt_reset();
    savedState = currentState; // WDT reset sonrası hangi state'teydiğimizi sakla

    // FIX 1: Declare solQTR / sagQTR as local variables.
    //         They were missing in v2 which prevented compilation.
    int solQTR, sagQTR;
    cli();
    solQTR = global_solQTR;
    sagQTR = global_sagQTR;
    sei();

    if (currentState != STATE_WAITING) {

        if (currentState != STATE_ESCAPE) {
            if (solQTR < QTR_THRESHOLD || sagQTR < QTR_THRESHOLD) {
                // FIX 2 (partial): Snapshot millis() BEFORE cli() so we never
                //                   call an interrupt-dependent function while
                //                   interrupts are disabled.
                unsigned long now = millis();
                cli();
                if (currentState != STATE_WAITING) {
                    currentState    = STATE_ESCAPE;
                    escapeStartTime = now;
                    escapeTurnStart = 0;   // Sensörler temizlenince set edilecek
                    isCrushing      = false;
                    oppLostTime     = 0;
                    sideStartTime   = 0;

                    if (solQTR < QTR_THRESHOLD && sagQTR < QTR_THRESHOLD) {
                        escapeStrategy = ESCAPE_BACK;
                    } else if (solQTR < QTR_THRESHOLD) {
                        escapeStrategy = ESCAPE_RIGHT_BACK;
                    } else {
                        escapeStrategy = ESCAPE_LEFT_BACK;
                    }
                }
                sei();
            }
        }
    }

    switch (currentState) {
        case STATE_WAITING: {
            // PRE-SCAN: Sinyal gelene kadar rakibi izle ve yönünü kaydet.
            // Motor çalışmıyor, sadece sensör okuyoruz.
            bool leftOpp  = ((PINC & (1 << PC3)) == 0);
            bool rightOpp = ((PINC & (1 << PC2)) == 0);
            if (leftOpp || rightOpp) {
                // preScanDir ÖNCE yazılıyor, sonra preScanFound = true.
                // Tersi olsaydı: INT0 preScanFound=true görüp preScanDir'i
                // henüz güncellenmemiş okuyabilirdi → yanlış yön.
                if (leftOpp && rightOpp) {
                    preScanDir = lastOpponentDir;
                } else if (leftOpp) {
                    preScanDir = 0;
                } else {
                    preScanDir = 1;
                }
                preScanFound = true; // Dir'den SONRA — INT0 tutarlı çift okur
            } else {
                preScanFound = false; // kaybolursa temizle — anlık veri kullan
            }
            break;
        }

        case STATE_SEARCHING: {
            bool leftOpp  = ((PINC & (1 << PC3)) == 0);
            bool rightOpp = ((PINC & (1 << PC2)) == 0);
            if (leftOpp || rightOpp) {
                oppLostTime   = 0;
                sideStartTime = 0; // Taze ATTACK — pivot sıfırdan başlasin
                cli();
                if (currentState != STATE_WAITING) currentState = STATE_ATTACK;
                sei();
            } else {
                unsigned long searchElapsed = millis() - searchStartTime;

                // FIX 5: After a timeout reset, searchElapsed was stale so the
                //         SEARCH_SPIN_PHASE check always fell into arc mode,
                //         permanently skipping the 300ms spin phase.
                //         Fix: reset the local variable after resetting the timer.
                if (searchElapsed >= SEARCH_TIMEOUT) {
                    lastOpponentDir ^= 1;
                    searchStartTime  = millis();
                    searchElapsed    = 0; // ← reset local so spin phase fires correctly
                }

                if (searchElapsed < SEARCH_SPIN_PHASE) {
                    // FAZ 1: Hızlı yerinde dönüş — arkasındaki rakibi bul
                    if (lastOpponentDir == 0) {
                        setMotorLeft(-SEARCH_SPEED);
                        setMotorRight(SEARCH_SPEED);
                    } else {
                        setMotorLeft(SEARCH_SPEED);
                        setMotorRight(-SEARCH_SPEED);
                    }
                } else {
                    // FAZ 2: Ark arama — hem dön hem mesafe kat
                    // SEARCH_SPEED/3 yerine SEARCH_ARC_INNER: düşük hızda motor stall yapar
                    if (lastOpponentDir == 0) {
                        setMotorLeft(SEARCH_ARC_INNER);
                        setMotorRight(SEARCH_SPEED);
                    } else {
                        setMotorLeft(SEARCH_SPEED);
                        setMotorRight(SEARCH_ARC_INNER);
                    }
                }
            }
            break;
        }

        case STATE_ATTACK: {
            bool leftOpp  = ((PINC & (1 << PC3)) == 0);
            bool rightOpp = ((PINC & (1 << PC2)) == 0);
            if (leftOpp && rightOpp) {
                oppLostTime = 0;
                // sideStartTime'i sıfırlama — rakip yana kayarsa pivot agresifliği korunsun.
                // Sadece rakip tamamen kaybolunca (oppLostTime branch) sıfırlanır.
                if (!isCrushing) {
                    // KOMBİNE STRATEJİ — ilk temas: direkt MAX_CRUSH_SPEED.
                    // Rakip köşede bekliyorsa dönmeye fırsat bulamadan tam güçle vur.
                    isCrushing        = true;
                    currentCrushSpeed = MAX_CRUSH_SPEED;
                }
                // ANGLED CRUSH: lastOpponentDir'e göre iç tekerleğe %12.5 daha az hız.
                // Rakip tam önümüzde bile olsa son bilinen yönünden hafif açıyla bas,
                // köşeye sıkışmış robotu kaydırarak dışarı çıkar.
                {
                    int16_t outerSpd = currentCrushSpeed;
                    int16_t innerSpd = currentCrushSpeed - (currentCrushSpeed >> ANGLE_SHIFT);
                    if (lastOpponentDir == 0) {
                        // Son bilinen yön sol → sola doğru açı
                        setMotorLeft(innerSpd);
                        setMotorRight(outerSpd);
                    } else {
                        // Son bilinen yön sağ → sağa doğru açı
                        setMotorLeft(outerSpd);
                        setMotorRight(innerSpd);
                    }
                }
            } else if (leftOpp) {
                // ADAPTİF PİVOT — rakip solda
                oppLostTime = 0;
                isCrushing  = false;
                lastOpponentDir = 0;
                if (sideStartTime == 0) sideStartTime = millis();
                {
                    unsigned long sideDur = millis() - sideStartTime;
                    int outerSpd, innerSpd;
                    if (sideDur < 200) {
                        outerSpd = PIVOT_OUTER_SLOW; innerSpd = PIVOT_INNER_SLOW;
                    } else if (sideDur < 500) {
                        outerSpd = PIVOT_OUTER_MID;  innerSpd = PIVOT_INNER_MID;
                    } else {
                        outerSpd = PIVOT_OUTER_FAST; innerSpd = PIVOT_INNER_FAST;
                    }
                    // Rakip solda: sağ tekerlek dış (iter), sol tekerlek iç (frenleri/geri)
                    setMotorLeft(innerSpd);
                    setMotorRight(outerSpd);
                }
            } else if (rightOpp) {
                // ADAPTİF PİVOT — rakip sağda
                oppLostTime = 0;
                isCrushing  = false;
                lastOpponentDir = 1;
                if (sideStartTime == 0) sideStartTime = millis();
                {
                    unsigned long sideDur = millis() - sideStartTime;
                    int outerSpd, innerSpd;
                    if (sideDur < 200) {
                        outerSpd = PIVOT_OUTER_SLOW; innerSpd = PIVOT_INNER_SLOW;
                    } else if (sideDur < 500) {
                        outerSpd = PIVOT_OUTER_MID;  innerSpd = PIVOT_INNER_MID;
                    } else {
                        outerSpd = PIVOT_OUTER_FAST; innerSpd = PIVOT_INNER_FAST;
                    }
                    // Rakip sağda: sol tekerlek dış (iter), sağ tekerlek iç (frenleri/geri)
                    setMotorLeft(outerSpd);
                    setMotorRight(innerSpd);
                }
            } else {
                // Fiziksel histerezis: rakip 50ms kaybolmadıkça momentum koru.
                if (oppLostTime == 0) {
                    oppLostTime = millis();
                } else if (millis() - oppLostTime >= OPP_LOST_DELAY) {
                    isCrushing    = false;
                    oppLostTime   = 0;
                    sideStartTime = 0; // Rakip tamamen kayboldu, pivot sıfırla
                    transitionToSearching();
                }
            }
            break;
        }

        case STATE_ESCAPE: {
            unsigned long elapsed      = millis() - escapeStartTime;
            bool          sensorsClear = (solQTR >= QTR_THRESHOLD && sagQTR >= QTR_THRESHOLD);

            // Sensörler ilk temizlendiği an kaydet — turn fazı buna göre ölçülür.
            // elapsed < 350 kullanmak yanlıştı: geri gitme 350ms'den uzun sürerse turn atlanıyordu.
            if (sensorsClear && escapeTurnStart == 0) {
                escapeTurnStart = millis();
            }
            bool inTurnPhase = (escapeTurnStart != 0) && (millis() - escapeTurnStart < 350);

            // Turn fazında rakip sensörü kontrolü.
            // Sensörler temizlendi (border'dan uzaklaştık) → saldırmak güvenli.
            // Rakip görünürse: lastOpponentDir güncelle, direkt ATTACK'e geç.
            // break ile escape strateji bloğuna düşmeyi engelle —
            // yoksa escapeTurnStart != 0 dalı hemen SEARCHING'e ezer.
            if (inTurnPhase) {
                bool leftO  = ((PINC & (1 << PC3)) == 0);
                bool rightO = ((PINC & (1 << PC2)) == 0);
                if (leftO || rightO) {
                    if      (leftO && !rightO) lastOpponentDir = 0;
                    else if (rightO && !leftO) lastOpponentDir = 1;
                    oppLostTime   = 0;
                    sideStartTime = 0;
                    cli();
                    if (currentState != STATE_WAITING) currentState = STATE_ATTACK;
                    sei();
                    break;
                }
            }

            if (escapeStrategy == ESCAPE_BACK) {
                if (!sensorsClear && elapsed < 1000) {
                    // Sensörler hala beyaz — geri gitmeye devam
                    setMotorLeft(-MAX_SPEED);
                    setMotorRight(-MAX_SPEED);
                } else if (inTurnPhase) {
                    if (lastOpponentDir == 0) {
                        setMotorLeft(-MAX_SPEED);
                        setMotorRight(MAX_SPEED);
                    } else {
                        setMotorLeft(MAX_SPEED);
                        setMotorRight(-MAX_SPEED);
                    }
                } else if (escapeTurnStart != 0) {
                    // Turn bitti — aramaya geç
                    transitionToSearching();
                } else {
                    // 1000ms timeout, sensörler hala beyaz — yine de arama
                    transitionToSearching();
                }
            } else if (escapeStrategy == ESCAPE_RIGHT_BACK) {
                if (!sensorsClear && elapsed < 1000) {
                    setMotorLeft(-MAX_SPEED);
                    setMotorRight(0);
                } else if (inTurnPhase) {
                    setMotorLeft(MAX_SPEED);
                    setMotorRight(MAX_SPEED / 2);
                } else if (escapeTurnStart != 0) {
                    transitionToSearching();
                } else {
                    transitionToSearching();
                }
            } else { // ESCAPE_LEFT_BACK
                if (!sensorsClear && elapsed < 1000) {
                    setMotorLeft(0);
                    setMotorRight(-MAX_SPEED);
                } else if (inTurnPhase) {
                    setMotorLeft(MAX_SPEED / 2);
                    setMotorRight(MAX_SPEED);
                } else if (escapeTurnStart != 0) {
                    transitionToSearching();
                } else {
                    transitionToSearching();
                }
            }
            break;
        }
    }
}

void initHardware() {
    DDRC  &= ~((1 << PC2) | (1 << PC3) | (1 << PC4) | (1 << PC5));
    PORTC |=  ((1 << PC2) | (1 << PC3)); // Pull-Up sadece rakip sensörleri için (A2, A3)
    DDRD  |= (1 << PD3);
    DDRB  |= (1 << PB3) | (1 << PB1) | (1 << PB2);
    DDRB  |= (1 << PB5);
    DDRD  &= ~(1 << PD2);
    PORTD |=  (1 << PD2);
    TCCR1A = (1 << WGM10);
    TCCR1B = (1 << WGM12) | (1 << CS11) | (1 << CS10);
    TCCR2A = (1 << WGM21) | (1 << WGM20);
    TCCR2B = (1 << CS22);
    // Sadece düşen kenar (falling edge) tetikler — ISC01=1, ISC00=0.
    // Start modülü butona basınca PD2'yi LOW'a çeker → düşen kenar.
    // Herhangi kenar (ISC00=1) olsaydı: açılışta yüzen pin + gürültü
    // sahte INT0 tetiklerdi ve robot kumandasız başlardı.
    // INT0 burada AKTİF EDİLMİYOR — setup() boot sırası bitmeden
    // INT0 açık olursa salon IR gürültüsü / başka robot kümandası
    // sahte START tetikler. INT0 setup() sonunda açılır.
    EICRA |= (1 << ISC01);
    EICRA &= ~(1 << ISC00);
    // EIMSK |= (1 << INT0) — setup() sonuna taşındı
    ADMUX  = (1 << REFS0) | (5);
    ADCSRA = (1 << ADEN) | (1 << ADIE) | (1 << ADPS2) | (1 << ADPS1) | (1 << ADPS0);
    ADCSRA |= (1 << ADSC);
    sei();
}

void setMotorLeft(int16_t speed) {
    if (speed > 0) {
        if (speed > 255) speed = 255;
        if (motorLeftDir != 1) {
            TCCR2A &= ~((1 << COM2A1) | (1 << COM2B1));
            delayMicroseconds(5);
            PORTD &= ~(1 << PD3);
            motorLeftDir = 1;
        }
        OCR2A = speed;
        TCCR2A |= (1 << COM2A1);
    } else if (speed < 0) {
        speed = -speed;
        if (speed > 255) speed = 255;
        if (motorLeftDir != -1) {
            TCCR2A &= ~((1 << COM2A1) | (1 << COM2B1));
            delayMicroseconds(5);
            PORTB &= ~(1 << PB3);
            motorLeftDir = -1;
        }
        OCR2B = speed;
        TCCR2A |= (1 << COM2B1);
    } else {
        if (motorLeftDir != 0) {
            TCCR2A &= ~((1 << COM2A1) | (1 << COM2B1));
            delayMicroseconds(5);
            PORTD &= ~(1 << PD3);
            PORTB &= ~(1 << PB3);
            motorLeftDir = 0;
        }
    }
    // TOCTOU çift kontrol: ISR stop bastıysa üzerine yazmış olabiliriz.
    if (currentState == STATE_WAITING) {
        TCCR2A &= ~((1 << COM2A1) | (1 << COM2B1));
        PORTD &= ~(1 << PD3);
        PORTB &= ~(1 << PB3);
        motorLeftDir = 0; // FIX 4: sync dir state so next start gets deadtime
    }
}

void setMotorRight(int16_t speed) {
    if (speed > 0) {
        if (speed > 255) speed = 255;
        if (motorRightDir != 1) {
            TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
            delayMicroseconds(5);
            PORTB &= ~(1 << PB2);
            motorRightDir = 1;
        }
        OCR1A = speed;
        TCCR1A |= (1 << COM1A1);
    } else if (speed < 0) {
        speed = -speed;
        if (speed > 255) speed = 255;
        if (motorRightDir != -1) {
            TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
            delayMicroseconds(5);
            PORTB &= ~(1 << PB1);
            motorRightDir = -1;
        }
        OCR1B = speed;
        TCCR1A |= (1 << COM1B1);
    } else {
        if (motorRightDir != 0) {
            TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
            delayMicroseconds(5);
            PORTB &= ~((1 << PB1) | (1 << PB2));
            motorRightDir = 0;
        }
    }
    if (currentState == STATE_WAITING) {
        TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
        PORTB &= ~((1 << PB1) | (1 << PB2));
        motorRightDir = 0; // FIX 4: sync dir state so next start gets deadtime
    }
}

// ISR-güvenli acil durdurma. sei() çağırmaz.
void stopMotors() {
    TCCR2A &= ~((1 << COM2A1) | (1 << COM2B1));
    PORTD &= ~(1 << PD3);
    PORTB &= ~(1 << PB3);
    // FIX 4: Reset direction state so the first setMotor call after restart
    //         correctly applies the 5µs deadtime before re-energising the H-bridge.
    motorLeftDir = 0;

    TCCR1A &= ~((1 << COM1A1) | (1 << COM1B1));
    PORTB &= ~((1 << PB1) | (1 << PB2));
    motorRightDir = 0; // FIX 4
}

ISR(INT0_vect) {
    // RC5 protokolü: buton basılı tutulursa 114ms aralıklarla repeat frame üretir.
    // Her frame = yeni falling edge = toggle → buton 200ms basılırsa start+stop gelir.
    // 150ms debounce: ilk press geçer, 114ms repeat yutulur, farklı buton 150ms sonra geçer.
    static unsigned long lastToggleTime = 0;
    unsigned long now = millis();
    if ((now - lastToggleTime) < 150) return;
    lastToggleTime = now;

    // Toggle mantığı: WAITING → SEARCHING/ATTACK, diğer her state → WAITING.
    if (currentState == STATE_WAITING) {
        // START — maç başlıyor
        // 'now' zaten dış scope'ta tanımlı (debounce için kullanılan aynı değer).
        // İkinci millis() çağrısı gereksiz ve değişkeni gölgeler — kaldırıldı.
        searchStartTime = now;
        isCrushing      = false;
        oppLostTime     = 0;
        sideStartTime   = 0;
        if (preScanFound) {
            // Pre-scan verisi var: rakibin yeri biliniyor, direkt saldır.
            lastOpponentDir = preScanDir;
            currentState    = STATE_ATTACK;
        } else {
            // Rakip görünmüyor, normal arama başlat.
            currentState    = STATE_SEARCHING;
        }
    } else {
        // STOP — maç bitiyor
        currentState = STATE_WAITING;
        preScanFound = false;
        stopMotors();
    }
}

// =========================================================================
// PING-PONG ADC ISR — MUX GHOSTING KORUMALARI
// =========================================================================
ISR(ADC_vect) {
    if (adc_dummy_flag) {
        // Dummy okuma bitti, S/H kapasitörü oturdu. Asıl okumayı başlat.
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
