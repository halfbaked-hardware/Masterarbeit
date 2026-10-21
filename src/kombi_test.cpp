// Kombi-Test: alle Module gleichzeitig über den TCA9548A
// ESP32: GPIO21 -> SDA, GPIO22 -> SCL am Multiplexer, Module an beliebigen Kanälen.
//
// Erkennung pro Kanal (automatisch, egal wo gesteckt):
//   Prox-Modul   = VCNL4010 (0x13) + MCP23017, Ampel-LEDs grün GPA0, gelb GPA1, rot GPA2
//   Haptik-Modul = DRV2605L (0x5A, erkannt an der Chip-ID) + MCP23017, Taster an GPA0
//   Touch-Modul  = MPR121 (0x5A-0x5D) + OLED + MCP23017, Elektroden 0 bis 3
//   Akkumodul    = OLED (ohne MPR121) und/oder DAC MCP4725 (0x60-0x63) + MCP23017,
//                  LED-Bar (MY9221) an GPB4 (DCKI) / GPB5 (DI)
//
// Stromsparend, damit kein Brownout auftritt:
//   - Module werden nacheinander mit Pause eingerichtet (kein gleichzeitiger Einschaltstrom)
//   - OLED-Kontrast niedrig, OLEDs zeichnen nur bei Änderungen neu
//   - LED-Bar gedimmt, Vibration schwach und kurz, VCNL4010-IR-LED mit 50 mA statt 200 mA
//   - LED-Bar wird nur bei Änderung neu beschrieben
//
// Ablauf:
//   Akku:   simulierter Ladestand fährt langsam hoch und runter; LED-Bar, OLED und
//           DAC-Ausgangsspannung (an OUT messbar) zeigen denselben Ladestand
//   Haptik: Taster gedrückt = Vibration, zusätzlich alle 5 s ein kurzer Puls
//   Touch:  OLED zeigt, welche Elektrode berührt wird
//   Prox:   Ampel grün/gelb/rot je nach Nähe
// Alle 1 s wird geprüft, ob Module umgesteckt oder neu gestartet wurden.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_MCP23X17.h>
#include <Adafruit_DRV2605.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_MPR121.h>
#include <Adafruit_VCNL4010.h>

#define PIN_SDA 21
#define PIN_SCL 22

// ---------- Einstellungen ----------

#define OLED_KONTRAST 0x10        // 0x00 bis 0xFF; niedrig = wenig Strom

// Haptik
#define MOTOR_IST_LRA false       // true = LRA, false = ERM (Rundmotor mit Unwucht)
#define STAERKE 0x30              // 0x00 bis 0x7F
#define PULS_MS 150
#define PULS_ABSTAND_MS 5000
#define HAPTIK_TASTER 0           // GPA0

// Akku
#define BAR_LEDS 10
#define BAR_HELLIGKEIT 0x10       // 0x00 bis 0xFF
#define BAR_DCKI 12               // GPB4
#define BAR_DI 13                 // GPB5
#define DAC_VCC_MV 3300
#define LADESTAND_MS 1000         // Zeit pro 10-%-Schritt

// Touch
#define ELEKTRODEN 4

// Prox
#define LED_GRUEN 0               // GPA0
#define LED_GELB 1                // GPA1
#define LED_ROT 2                 // GPA2
#define LED_AKTIV_HIGH true
#define VCNL_STROM 5              // IR-LED-Strom in 10 mA (5 = 50 mA, Standard wäre 200 mA)
#define SCHWELLE_GELB 300         // Abstand über dem Ruhewert (bei 50 mA kleiner als bei 200 mA)
#define SCHWELLE_ROT 2000
#define PROX_MS 200

#define PRUEF_MS 1000
#define EINRICHT_PAUSE_MS 300     // Pause zwischen den Modulen beim Einrichten

// ---------- Module ----------

enum Typ { KEINS, PROX, HAPTIK, TOUCH, AKKU, UNBEKANNT };
const char *TYP_NAME[] = {"-", "Prox", "Haptik", "Touch", "Akku", "unbekannt"};

struct Modul {
  bool da = false;
  bool bereit = false;
  int8_t kanal = -1;
  uint8_t mcp = 0;        // Adresse MCP23017 (0 = keiner)
};

Modul prox, haptik, touch, akku;
Typ kanalTyp[8];

uint8_t tcaAdresse = 0;
int8_t aktiverKanal = -2;

Adafruit_MCP23X17 mcpProx, mcpHaptik, mcpTouch, mcpAkku;
Adafruit_DRV2605 drv;
Adafruit_MPR121 mpr;
Adafruit_VCNL4010 vcnl;
Adafruit_SSD1306 oledAkku(128, 64, &Wire, -1);
Adafruit_SSD1306 oledTouch(128, 64, &Wire, -1);

uint8_t akkuOled = 0, akkuDac = 0;     // Adressen auf dem Akkumodul (0 = fehlt)
uint8_t touchOled = 0, touchMpr = 0;   // Adressen auf dem Touch-Modul

// ---------- I2C-Hilfen ----------

bool antwortet(uint8_t adresse) {
  Wire.beginTransmission(adresse);
  return Wire.endTransmission() == 0;
}

int regLesen(uint8_t adresse, uint8_t reg) {
  Wire.beginTransmission(adresse);
  Wire.write(reg);
  if (Wire.endTransmission(false) != 0) return -1;
  if (Wire.requestFrom(adresse, (uint8_t)1) != 1) return -1;
  return Wire.read();
}

void regSchreiben(uint8_t adresse, uint8_t reg, uint8_t wert) {
  Wire.beginTransmission(adresse);
  Wire.write(reg);
  Wire.write(wert);
  Wire.endTransmission();
}

void kanal(int8_t k) {
  if (k == aktiverKanal) return;
  Wire.beginTransmission(tcaAdresse);
  Wire.write(k < 0 ? 0 : 1 << k);
  Wire.endTransmission();
  aktiverKanal = k;
}

uint8_t ersteAdresse(uint8_t von, uint8_t bis) {
  for (uint8_t a = von; a <= bis; a++) {
    if (antwortet(a)) return a;
  }
  return 0;
}

// DRV2605(L): Bits 7..5 im Statusregister 0x00 = Chip-ID (3, 4, 6 oder 7).
// Beim MPR121 steht dort der Touch-Status (nach dem Start 0).
bool istDrv2605(uint8_t adresse) {
  int s = regLesen(adresse, 0x00);
  if (s < 0) return false;
  uint8_t id = s >> 5;
  return id == 3 || id == 4 || id == 6 || id == 7;
}

// ---------- Erkennung ----------

Typ kanalErkennen() {
  if (antwortet(0x13)) return PROX;
  if (antwortet(0x5A) && istDrv2605(0x5A)) return HAPTIK;
  if (ersteAdresse(0x5A, 0x5D)) return TOUCH;
  if (ersteAdresse(0x3C, 0x3D) || ersteAdresse(0x60, 0x63)) return AKKU;
  if (ersteAdresse(0x20, 0x27)) return UNBEKANNT;
  return KEINS;
}

void zuordnen(Modul &m, int8_t k, Typ typ) {
  if (m.da) {
    Serial.printf("  Kanal %d: weiteres %s-Modul wird ignoriert (nur eins pro Typ)\n", k, TYP_NAME[typ]);
    return;
  }
  m.da = true;
  m.kanal = k;
  m.mcp = ersteAdresse(0x20, 0x27);
}

void alleSuchen() {
  prox = Modul(); haptik = Modul(); touch = Modul(); akku = Modul();
  akkuOled = akkuDac = touchOled = touchMpr = 0;
  aktiverKanal = -2;

  Serial.printf("\nMultiplexer bei 0x%02X, Kanäle:\n", tcaAdresse);
  for (int8_t k = 0; k < 8; k++) {
    kanal(k);
    Serial.printf("  Kanal %d:", k);
    for (uint8_t a = 1; a < 127; a++) {
      if (a >= 0x70 && a <= 0x77) continue;
      if (antwortet(a)) Serial.printf(" 0x%02X", a);
    }
    kanalTyp[k] = kanalErkennen();
    Serial.printf("  -> %s\n", kanalTyp[k] == KEINS ? "leer" : TYP_NAME[kanalTyp[k]]);

    switch (kanalTyp[k]) {
      case PROX:   zuordnen(prox, k, PROX); break;
      case HAPTIK: zuordnen(haptik, k, HAPTIK); break;
      case TOUCH:
        zuordnen(touch, k, TOUCH);
        if (touch.kanal == k) {
          touchMpr = ersteAdresse(0x5A, 0x5D);
          touchOled = ersteAdresse(0x3C, 0x3D);
        }
        break;
      case AKKU:
        zuordnen(akku, k, AKKU);
        if (akku.kanal == k) {
          akkuOled = ersteAdresse(0x3C, 0x3D);
          akkuDac = ersteAdresse(0x60, 0x63);
        }
        break;
      default: break;
    }
  }
  kanal(-1);
}

void pruefzeile(const char *name, uint8_t adresse, const char *erwartet) {
  if (adresse) Serial.printf("    [OK]     %-9s 0x%02X\n", name, adresse);
  else         Serial.printf("    [FEHLT]  %-9s (erwartet %s)\n", name, erwartet);
}

// ---------- Akku: LED-Bar (MY9221 über MCP23017) ----------

const uint8_t BIT_DCKI = 1 << (BAR_DCKI - 8);
const uint8_t BIT_DI = 1 << (BAR_DI - 8);
uint8_t barGpiob = 0;

void barPin(uint8_t bit, bool pegel) {
  if (pegel) barGpiob |= bit; else barGpiob &= ~bit;
  regSchreiben(akku.mcp, 0x13, barGpiob);       // GPIOB
}

void barSenden(uint16_t daten) {
  for (uint8_t i = 0; i < 16; i++) {
    barPin(BIT_DI, daten & 0x8000);
    barPin(BIT_DCKI, !(barGpiob & BIT_DCKI));
    daten <<= 1;
  }
}

void barSetzen(uint8_t anzahl) {
  if (!akku.mcp) return;
  kanal(akku.kanal);
  barSenden(0x0000);
  for (uint8_t i = 0; i < 12; i++) {
    barSenden(i < anzahl && i < BAR_LEDS ? BAR_HELLIGKEIT : 0x0000);
  }
  barPin(BIT_DI, false);                        // Latch
  delayMicroseconds(300);
  for (uint8_t i = 0; i < 4; i++) {
    barPin(BIT_DI, true);
    barPin(BIT_DI, false);
  }
}

// ---------- Akku: DAC MCP4725 ----------

void dacSetzen(uint16_t mv) {
  if (!akkuDac) return;
  kanal(akku.kanal);
  uint16_t wert = min((uint32_t)4095, (uint32_t)mv * 4095 / DAC_VCC_MV);
  Wire.beginTransmission(akkuDac);
  Wire.write((wert >> 8) & 0x0F);               // Fast Mode, kein EEPROM
  Wire.write(wert & 0xFF);
  Wire.endTransmission();
}

// ---------- OLED ----------

bool oledStarten(Adafruit_SSD1306 &oled, uint8_t adresse) {
  if (!adresse || !oled.begin(SSD1306_SWITCHCAPVCC, adresse)) return false;
  oled.ssd1306_command(SSD1306_SETCONTRAST);
  oled.ssd1306_command(OLED_KONTRAST);
  oled.clearDisplay();
  oled.setTextColor(SSD1306_WHITE);
  oled.display();
  return true;
}

void akkuAnzeigen(uint8_t prozent, uint16_t mv) {
  if (!akkuOled) return;
  kanal(akku.kanal);
  oledAkku.clearDisplay();
  oledAkku.setTextSize(1);
  oledAkku.setCursor(0, 0);
  oledAkku.printf("Akku  K%d", akku.kanal);
  oledAkku.setTextSize(2);
  oledAkku.setCursor(0, 16);
  oledAkku.printf("%3u %%", prozent);
  oledAkku.setTextSize(1);
  oledAkku.setCursor(0, 40);
  if (akkuDac) oledAkku.printf("U an OUT: %u,%02u V", mv / 1000, (mv % 1000) / 10);
  else         oledAkku.print("DAC fehlt");
  // Übersicht aller Module in einer Zeile
  oledAkku.setCursor(0, 54);
  oledAkku.printf("H%c T%c P%c", haptik.bereit ? '+' : '-', touch.bereit ? '+' : '-',
                  prox.bereit ? '+' : '-');
  oledAkku.display();
}

void touchAnzeigen(uint16_t status) {
  if (!touchOled) return;
  kanal(touch.kanal);
  oledTouch.clearDisplay();
  oledTouch.setTextSize(1);
  oledTouch.setCursor(0, 0);
  oledTouch.printf("Touch  K%d", touch.kanal);
  const uint8_t breite = 128 / ELEKTRODEN;
  for (uint8_t i = 0; i < ELEKTRODEN; i++) {
    uint8_t x = i * breite + 2;
    bool an = status & (1 << i);
    if (an) oledTouch.fillRect(x, 20, breite - 4, 24, SSD1306_WHITE);
    else    oledTouch.drawRect(x, 20, breite - 4, 24, SSD1306_WHITE);
    oledTouch.setTextColor(an ? SSD1306_BLACK : SSD1306_WHITE);
    oledTouch.setTextSize(2);
    oledTouch.setCursor(x + (breite - 4) / 2 - 5, 25);
    oledTouch.print(i);
  }
  oledTouch.setTextColor(SSD1306_WHITE);
  oledTouch.display();
}

// ---------- Prox: Ampel ----------

void ampel(uint8_t stufe) {          // 0 = aus, 1 = grün, 2 = gelb, 3 = rot
  if (!prox.mcp) return;
  kanal(prox.kanal);
  uint8_t gpioa = 0;
  if (stufe == 1) gpioa = 1 << LED_GRUEN;
  if (stufe == 2) gpioa = 1 << LED_GELB;
  if (stufe == 3) gpioa = 1 << LED_ROT;
  if (!LED_AKTIV_HIGH) gpioa = ~gpioa & 0x07;
  regSchreiben(prox.mcp, 0x14, gpioa);          // OLATA, nur die drei LED-Pins sind Ausgänge
}

// ---------- Einrichten ----------

uint16_t proxRuhewert = 0;

void haptikEinrichten() {
  if (!haptik.da) return;
  kanal(haptik.kanal);
  Serial.printf("  Haptik an Kanal %d:\n", haptik.kanal);
  pruefzeile("DRV2605L", 0x5A, "0x5A");
  pruefzeile("MCP23017", haptik.mcp, "0x20-0x27");
  if (haptik.mcp && mcpHaptik.begin_I2C(haptik.mcp, &Wire)) {
    mcpHaptik.pinMode(HAPTIK_TASTER, INPUT_PULLUP);
  } else {
    haptik.mcp = 0;
  }
  if (!drv.begin(&Wire)) { Serial.println("    DRV2605L startet nicht"); return; }
  if (MOTOR_IST_LRA) drv.useLRA(); else drv.useERM();
  drv.setMode(DRV2605_MODE_REALTIME);
  drv.setRealtimeValue(0);
  haptik.bereit = true;
}

void touchEinrichten() {
  if (!touch.da) return;
  kanal(touch.kanal);
  Serial.printf("  Touch an Kanal %d:\n", touch.kanal);
  pruefzeile("MPR121", touchMpr, "0x5A-0x5D");
  pruefzeile("OLED", touchOled, "0x3C/0x3D");
  pruefzeile("MCP23017", touch.mcp, "0x20-0x27");
  if (!oledStarten(oledTouch, touchOled)) touchOled = 0;
  if (!mpr.begin(touchMpr, &Wire, MPR121_TOUCH_THRESHOLD_DEFAULT,
                 MPR121_RELEASE_THRESHOLD_DEFAULT, true)) {
    Serial.println("    MPR121 startet nicht");
    return;
  }
  mpr.writeRegister(MPR121_ECR, 0x00);          // nur die genutzten Elektroden einschalten
  mpr.writeRegister(MPR121_ECR, 0x80 | ELEKTRODEN);
  touch.bereit = true;
  touchAnzeigen(0);
}

void akkuEinrichten() {
  if (!akku.da) return;
  kanal(akku.kanal);
  Serial.printf("  Akku an Kanal %d:\n", akku.kanal);
  pruefzeile("MCP23017", akku.mcp, "0x20-0x27");
  pruefzeile("OLED", akkuOled, "0x3C/0x3D");
  pruefzeile("DAC", akkuDac, "0x60-0x63");
  if (akku.mcp && mcpAkku.begin_I2C(akku.mcp, &Wire)) {
    for (uint8_t pin = 0; pin < 16; pin++) mcpAkku.pinMode(pin, INPUT_PULLUP);
    mcpAkku.pinMode(BAR_DCKI, OUTPUT);
    mcpAkku.pinMode(BAR_DI, OUTPUT);
    barGpiob = 0;
    barPin(BIT_DCKI | BIT_DI, false);
    barSetzen(0);
  } else {
    akku.mcp = 0;
  }
  if (!oledStarten(oledAkku, akkuOled)) akkuOled = 0;
  dacSetzen(0);
  akku.bereit = true;
}

void proxEinrichten() {
  if (!prox.da) return;
  kanal(prox.kanal);
  Serial.printf("  Prox an Kanal %d:\n", prox.kanal);
  pruefzeile("VCNL4010", 0x13, "0x13");
  pruefzeile("MCP23017", prox.mcp, "0x20-0x27");
  if (prox.mcp && mcpProx.begin_I2C(prox.mcp, &Wire)) {
    mcpProx.pinMode(LED_GRUEN, OUTPUT);
    mcpProx.pinMode(LED_GELB, OUTPUT);
    mcpProx.pinMode(LED_ROT, OUTPUT);
    ampel(0);
  } else {
    prox.mcp = 0;
  }
  if (!vcnl.begin(0x13, &Wire)) { Serial.println("    VCNL4010 startet nicht"); return; }
  vcnl.setLEDcurrent(VCNL_STROM);
  uint32_t summe = 0;
  for (uint8_t i = 0; i < 8; i++) summe += vcnl.readProximity();
  proxRuhewert = summe / 8;
  Serial.printf("    Ruhewert %u (beim Start nichts vor den Sensor halten)\n", proxRuhewert);
  prox.bereit = true;
}

void alleEinrichten() {
  alleSuchen();
  Serial.println("Einrichten (nacheinander, um Einschaltströme zu verteilen):");
  // Reihenfolge: stromsparende Module zuerst
  touchEinrichten();  delay(EINRICHT_PAUSE_MS);
  proxEinrichten();   delay(EINRICHT_PAUSE_MS);
  akkuEinrichten();   delay(EINRICHT_PAUSE_MS);
  haptikEinrichten();
  Serial.printf("Bereit: Haptik %s, Touch %s, Akku %s, Prox %s\n\n",
                haptik.bereit ? "ja" : "nein", touch.bereit ? "ja" : "nein",
                akku.bereit ? "ja" : "nein", prox.bereit ? "ja" : "nein");
}

// ---------- Umstecken / Neustart erkennen ----------

bool belegungGeaendert() {
  for (int8_t k = 0; k < 8; k++) {
    kanal(k);
    if (kanalErkennen() != kanalTyp[k]) return true;
  }
  return false;
}

// Ein Modul, das kurz ohne Strom war, steht wieder auf Werkseinstellung
bool modulNeuGestartet() {
  if (haptik.bereit) {
    kanal(haptik.kanal);
    if (regLesen(0x5A, DRV2605_REG_MODE) != DRV2605_MODE_REALTIME) return true;
  }
  if (touch.bereit) {
    kanal(touch.kanal);
    if (regLesen(touchMpr, MPR121_ECR) == 0) return true;
  }
  if (akku.bereit && akku.mcp) {
    kanal(akku.kanal);
    int iodirb = regLesen(akku.mcp, 0x01);
    if (iodirb < 0 || (iodirb & (BIT_DCKI | BIT_DI))) return true;
  }
  if (prox.bereit && prox.mcp) {
    kanal(prox.kanal);
    int iodira = regLesen(prox.mcp, 0x00);
    if (iodira < 0 || (iodira & 0x07)) return true;
  }
  return false;
}

// ---------- Laufender Betrieb ----------

void haptikSchritt() {
  if (!haptik.bereit) return;
  static uint32_t pulsStart = 0, letzterPuls = 0;
  static bool vibriert = false;

  kanal(haptik.kanal);
  bool taster = haptik.mcp && mcpHaptik.digitalRead(HAPTIK_TASTER) == LOW;
  bool puls = millis() - letzterPuls >= PULS_ABSTAND_MS;
  if (puls) { letzterPuls = millis(); pulsStart = millis(); }
  bool soll = taster || millis() - pulsStart < PULS_MS;

  if (soll != vibriert) {
    drv.setRealtimeValue(soll ? STAERKE : 0);
    vibriert = soll;
    if (taster && soll) Serial.println("Haptik: Taster gedrückt -> Vibration");
  }
}

void touchSchritt() {
  if (!touch.bereit) return;
  static uint16_t letzter = 0;
  kanal(touch.kanal);
  uint16_t status = mpr.touched() & ((1 << ELEKTRODEN) - 1);
  if (status == letzter) return;
  for (uint8_t i = 0; i < ELEKTRODEN; i++) {
    uint16_t bit = 1 << i;
    if ((status & bit) && !(letzter & bit)) Serial.printf("Touch: Pin %u berührt\n", i);
  }
  touchAnzeigen(status);
  letzter = status;
}

void akkuSchritt() {
  if (!akku.bereit) return;
  static uint32_t letzterSchritt = 0;
  static int8_t prozent = 0, richtung = 10;
  if (millis() - letzterSchritt < LADESTAND_MS) return;
  letzterSchritt = millis();

  prozent += richtung;
  if (prozent >= 100) { prozent = 100; richtung = -10; }
  if (prozent <= 0)   { prozent = 0;   richtung = 10; }

  uint16_t mv = (uint32_t)prozent * DAC_VCC_MV / 100;   // Maßstab: 0-100 % = 0-3,3 V
  barSetzen(prozent / 10);
  dacSetzen(mv);
  akkuAnzeigen(prozent, mv);
}

void proxSchritt() {
  if (!prox.bereit) return;
  static uint32_t letzteMessung = 0;
  static uint8_t letzteStufe = 0;
  if (millis() - letzteMessung < PROX_MS) return;
  letzteMessung = millis();

  kanal(prox.kanal);
  int32_t abstand = (int32_t)vcnl.readProximity() - proxRuhewert;
  uint8_t stufe = abstand >= SCHWELLE_ROT ? 3 : abstand >= SCHWELLE_GELB ? 2 : 1;
  if (stufe != letzteStufe) {
    ampel(stufe);
    Serial.printf("Prox: %s (+%ld)\n", stufe == 3 ? "ROT" : stufe == 2 ? "GELB" : "gruen",
                  abstand > 0 ? (long)abstand : 0L);
    letzteStufe = stufe;
  }
}

// ---------- Setup / Loop ----------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Kombi-Test alle Module ===");

  pinMode(PIN_SDA, INPUT_PULLDOWN);
  pinMode(PIN_SCL, INPUT_PULLDOWN);
  delay(5);
  Serial.printf("Bus-Leitungen: SDA=%s  SCL=%s (HIGH = Pull-ups vorhanden)\n",
                digitalRead(PIN_SDA) ? "HIGH" : "LOW", digitalRead(PIN_SCL) ? "HIGH" : "LOW");

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);

  while (!(tcaAdresse = ersteAdresse(0x70, 0x77))) {
    Serial.println("Kein Multiplexer gefunden (0x70 bis 0x77), neuer Versuch in 2 s ...");
    delay(2000);
  }
  alleEinrichten();
}

void loop() {
  static uint32_t letztePruefung = 0;
  if (millis() - letztePruefung >= PRUEF_MS) {
    letztePruefung = millis();
    if (belegungGeaendert() || modulNeuGestartet()) {
      Serial.println("\nModule umgesteckt oder neu gestartet -> neu einrichten");
      delay(200);
      alleEinrichten();
    }
  }

  haptikSchritt();
  touchSchritt();
  akkuSchritt();
  proxSchritt();
  delay(10);
}
