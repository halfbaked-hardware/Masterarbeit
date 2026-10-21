// Hauptprogramm: vier Lernstationen am TCA9548A
// ESP32: GPIO21 -> SDA, GPIO22 -> SCL am Multiplexer, Module an beliebigen Kanälen.
// Jedes Modul ist eine eigene Station und wird automatisch erkannt, egal wo es steckt:
//
//   Akku   (OLED und/oder DAC ohne MPR121, MCP23017)
//          Simulierter, verkleinerter Handy-Akku mit Innenwiderstand.
//          Lasttaster GPA0: Last aus -> klein -> groß; rote LEDs GPA3 (klein) / GPA2 (groß)
//          Ladetaster GPA1: Ladegerät an/ab. Laden mit CC/CV: bis ~80 % konstanter Strom
//          (schnell), danach konstante Spannung, der Strom sinkt (langsam).
//          LED-Bar (MY9221 an GPB4/GPB5): Ladestand, oberste LED blinkt beim Laden.
//          OLED: Ladestand, Spannung, Strom, Innenwiderstand.
//          DAC (MCP4725): Klemmenspannung an den Messbuchsen, direkt in Volt.
//   Haptik (DRV2605L 0x5A + MCP23017, Taster GPA0)
//          kurz = Benachrichtigung, zweimal = Nachricht, lang = Anruf
//   Touch  (MPR121 + OLED + MCP23017, Elektroden 0 bis 3)
//          OLED zeigt pro Elektrode einen Balken für die Änderung des Messwerts und
//          eine Linie für die Schwelle "berührt"; gefüllter Balken = berührt
//   Prox   (VCNL4010 0x13 + MCP23017, LEDs grün GPA0, gelb GPA1, rot GPA2)
//          Ampel je nach Nähe
//
// Stromsparend gegen Brownout: Module nacheinander einrichten, OLEDs mit niedrigem
// Kontrast und nur bei Änderung neu zeichnen, LED-Bar gedimmt, Vibration schwach,
// VCNL4010-IR-LED mit 50 mA.
// Umstecken: Alle 0,5 s wird jeder Kanal geprüft. Ein Modul gilt erst als gesteckt bzw.
// gezogen, wenn es zweimal hintereinander gleich erkannt wird (Pogo-Kontakte federn).
// Dann wird nur dieses eine Modul neu eingerichtet, die anderen laufen ungestört weiter.
// Der simulierte Ladestand bleibt erhalten. Ein hängender I2C-Bus wird freigetaktet.
// Läuft ohne PC: Hängt das Programm länger als 5 s, startet der Watchdog den ESP32 neu.

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
#define PRUEF_MS 500              // Umstecken prüfen
#define WIEDERHOL_MS 3000         // fehlgeschlagenes Einrichten erneut versuchen
#define EINRICHT_PAUSE_MS 300     // Pause zwischen den Modulen beim Einrichten
#define AUFFRISCH_MS 1500         // alle 1,5 s wird reihum ein Modul aufgefrischt

// Akku: Pins am MCP23017
#define AKKU_TASTER_LAST 0        // GPA0
#define AKKU_TASTER_LADEN 1       // GPA1
#define AKKU_LED_GROSS 2          // GPA2 (LED 2)
#define AKKU_LED_KLEIN 3          // GPA3 (LED 1)
#define BAR_DCKI 12               // GPB4
#define BAR_DI 13                 // GPB5
#define BAR_LEDS 10
#define BAR_HELLIGKEIT 0x10       // 0x00 bis 0xFF

// Akku: Modell (verkleinerter Akku, Werte direkt an der Messbuchse messbar)
#define RI 0.20f                  // Innenwiderstand in Ohm
#define LADESTROM 1.00f           // Konstantstrom in der CC-Phase in A
#define LADESPANNUNG 3.30f        // Konstantspannung in der CV-Phase in V
#define LADEENDE_STROM 0.05f      // unterhalb ist der Akku voll
#define LAST_KLEIN 0.30f          // A
#define LAST_GROSS 1.00f          // A
#define KAPAZITAET_AS 87.5f       // Ladung in As; ergibt ~2 min für einmal Vollladen
#define DAC_VCC_MV 3300           // Versorgung des DAC = größte Ausgangsspannung

// Haptik
#define MOTOR_IST_LRA false       // true = LRA, false = ERM (Rundmotor mit Unwucht)
#define STAERKE 0x30              // 0x00 bis 0x7F
#define HAPTIK_TASTER 0           // GPA0
#define LANG_MS 600               // ab hier gilt ein Druck als lang
#define DOPPEL_MS 300             // Zeitfenster für den zweiten Druck

// Touch
#define ELEKTRODEN 4
#define TOUCH_SKALA 60            // Messwertänderung für einen vollen Balken
#define TOUCH_MS 100              // Bildwiederholung

// Prox
#define LED_GRUEN 0               // GPA0
#define LED_GELB 1                // GPA1
#define LED_ROT 2                 // GPA2
#define VCNL_STROM 5              // IR-LED-Strom in 10 mA (Standard wäre 200 mA)
#define SCHWELLE_GELB 300         // Abstand über dem Ruhewert
#define SCHWELLE_ROT 2000
#define PROX_MS 200

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
Typ kanalTyp[8];          // bestätigte Belegung
Typ kandidat[8];          // zuletzt gesehene, noch nicht bestätigte Belegung
bool busOk = true;

uint8_t tcaAdresse = 0;
int8_t aktiverKanal = -2;

Adafruit_MCP23X17 mcpProx, mcpHaptik, mcpAkku;
Adafruit_DRV2605 drv;
Adafruit_MPR121 mpr;
Adafruit_VCNL4010 vcnl;
Adafruit_SSD1306 oledAkku(128, 64, &Wire, -1);
Adafruit_SSD1306 oledTouch(128, 64, &Wire, -1);

uint8_t akkuOled = 0, akkuDac = 0;
uint8_t touchOled = 0, touchMpr = 0;

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
  // Schlägt das Umschalten fehl, beim nächsten Mal sicher neu schreiben
  aktiverKanal = Wire.endTransmission() == 0 ? k : -2;
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

// Taster mit Entprellung: liefert true genau beim Drücken
struct Taster {
  bool gedrueckt = false;
  uint32_t seit = 0;
  bool neuGedrueckt(bool jetzt) {
    if (jetzt == gedrueckt || millis() - seit < 30) return false;
    gedrueckt = jetzt;
    seit = millis();
    return jetzt;
  }
};

// ---------- Erkennung ----------

Typ kanalErkennen() {
  if (antwortet(0x13)) return PROX;
  if (antwortet(0x5A) && istDrv2605(0x5A)) return HAPTIK;
  if (ersteAdresse(0x5A, 0x5D)) return TOUCH;
  if (ersteAdresse(0x3C, 0x3D) || ersteAdresse(0x60, 0x63)) return AKKU;
  if (ersteAdresse(0x20, 0x27)) return UNBEKANNT;
  return KEINS;
}

Modul *modulVon(Typ typ) {
  switch (typ) {
    case PROX:   return &prox;
    case HAPTIK: return &haptik;
    case TOUCH:  return &touch;
    case AKKU:   return &akku;
    default:     return nullptr;
  }
}

void adressenAusgeben(int8_t k) {
  kanal(k);
  Serial.printf("  Kanal %d:", k);
  for (uint8_t a = 1; a < 127; a++) {
    if (a >= 0x70 && a <= 0x77) continue;
    if (antwortet(a)) Serial.printf(" 0x%02X", a);
  }
  Serial.println();
}

void pruefzeile(const char *name, uint8_t adresse, const char *erwartet) {
  if (adresse) Serial.printf("    [OK]     %-9s 0x%02X\n", name, adresse);
  else         Serial.printf("    [FEHLT]  %-9s (erwartet %s)\n", name, erwartet);
}

// leeren = false beim Auffrischen: das alte Bild bleibt stehen, bis neu gezeichnet wird
bool oledStarten(Adafruit_SSD1306 &oled, uint8_t adresse, bool leeren = true) {
  if (!adresse || !oled.begin(SSD1306_SWITCHCAPVCC, adresse)) return false;
  oled.ssd1306_command(SSD1306_SETCONTRAST);
  oled.ssd1306_command(OLED_KONTRAST);
  oled.cp437(true);                             // für das Ω-Zeichen
  oled.setTextColor(SSD1306_WHITE);
  if (leeren) {
    oled.clearDisplay();
    oled.display();
  }
  return true;
}

// =====================================================================
// Station Akku
// =====================================================================

enum Last { LAST_AUS, LAST_KL, LAST_GR };
const char *LAST_NAME[] = {"aus", "klein", "gross"};

struct AkkuZustand {
  float soc = 50.0f;      // Ladestand in %
  bool laden = false;     // Ladegerät angesteckt
  bool voll = false;
  bool aus = false;       // Handy hat sich abgeschaltet (0 %)
  Last last = LAST_AUS;
  float strom = 0;        // A, positiv = Laden
  float spannung = 0;     // Klemmenspannung in V
  bool cv = false;        // Konstantspannungs-Phase
} ak;

Taster tasterLast, tasterLaden;

// Ruhespannung abhängig vom Ladestand (Kennlinie eines verkleinerten Li-Akkus)
float ruhespannung(float soc) {
  static const float S[] = {0, 5, 10, 20, 40, 60, 80, 90, 95, 100};
  static const float U[] = {2.70, 2.85, 2.92, 2.97, 3.01, 3.05, 3.10, 3.20, 3.25, 3.29};
  if (soc <= 0) return U[0];
  for (uint8_t i = 1; i < 10; i++) {
    if (soc <= S[i]) return U[i - 1] + (U[i] - U[i - 1]) * (soc - S[i - 1]) / (S[i] - S[i - 1]);
  }
  return U[9];
}

// LED-Bar (MY9221 über den MCP23017)
const uint8_t BIT_DCKI = 1 << (BAR_DCKI - 8);
const uint8_t BIT_DI = 1 << (BAR_DI - 8);
uint8_t barGpiob = 0;
int16_t barStand = -1;

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

void barSetzen(uint16_t maske) {
  if (!akku.mcp || maske == barStand) return;   // nur bei Änderung schreiben
  barStand = maske;
  kanal(akku.kanal);
  Wire.setClock(400000);                        // ~400 Schreibzugriffe, so dauert es nur ~35 ms
  barSenden(0x0000);
  for (uint8_t i = 0; i < 12; i++) {
    barSenden(maske & (1 << i) ? BAR_HELLIGKEIT : 0x0000);
  }
  barPin(BIT_DI, false);                        // Latch
  delayMicroseconds(300);
  for (uint8_t i = 0; i < 4; i++) {
    barPin(BIT_DI, true);
    barPin(BIT_DI, false);
  }
  Wire.setClock(100000);
}

void dacSetzen(float volt) {
  if (!akkuDac) return;
  kanal(akku.kanal);
  uint16_t wert = constrain((int32_t)(volt * 1000 * 4095 / DAC_VCC_MV), 0, 4095);
  Wire.beginTransmission(akkuDac);
  Wire.write((wert >> 8) & 0x0F);               // Fast Mode, kein EEPROM
  Wire.write(wert & 0xFF);
  Wire.endTransmission();
}

void lastLeds() {
  if (!akku.mcp) return;
  kanal(akku.kanal);
  mcpAkku.digitalWrite(AKKU_LED_KLEIN, ak.last != LAST_AUS);
  mcpAkku.digitalWrite(AKKU_LED_GROSS, ak.last == LAST_GR);
}

void akkuAnzeigen() {
  if (!akkuOled) return;
  kanal(akku.kanal);
  oledAkku.clearDisplay();

  if (ak.aus) {
    oledAkku.setTextSize(2);
    oledAkku.setCursor(10, 14);
    oledAkku.print("Handy aus");
    oledAkku.setTextSize(1);
    oledAkku.setCursor(31, 40);
    oledAkku.print("bitte laden");
    oledAkku.display();
    return;
  }

  oledAkku.setTextSize(1);
  oledAkku.setCursor(0, 0);
  if (ak.laden)    oledAkku.printf("LADEN  %s", ak.voll ? "voll" : ak.cv ? "CV" : "CC");
  else             oledAkku.printf("Last: %s", LAST_NAME[ak.last]);

  oledAkku.setTextSize(2);
  oledAkku.setCursor(0, 12);
  oledAkku.printf("%3d %%", (int)(ak.soc + 0.5f));

  // Werte in Hundertsteln, damit Komma und Rundung stimmen (z. B. 2,999 -> 3,00)
  int u = (int)(ak.spannung * 100 + 0.5f);
  int i = (int)(fabsf(ak.strom) * 100 + 0.5f);
  int r = (int)(RI * 100 + 0.5f);
  oledAkku.setTextSize(1);
  oledAkku.setCursor(0, 34);
  oledAkku.printf("U  = %d,%02d V", u / 100, u % 100);
  oledAkku.setCursor(0, 44);
  oledAkku.printf("I  = %d,%02d A", i / 100, i % 100);
  oledAkku.setCursor(0, 54);
  oledAkku.printf("Ri = %d,%02d ", r / 100, r % 100);
  oledAkku.write(234);                          // Ω im CP437-Zeichensatz
  oledAkku.display();
}

void akkuEinrichten() {
  if (!akku.da) return;
  akku.bereit = false;                         // erst nach erfolgreichem Einrichten wieder true
  kanal(akku.kanal);
  Serial.printf("  Akku an Kanal %d:\n", akku.kanal);
  pruefzeile("MCP23017", akku.mcp, "0x20-0x27");
  pruefzeile("OLED", akkuOled, "0x3C/0x3D");
  pruefzeile("DAC", akkuDac, "0x60-0x63");
  if (akku.mcp && mcpAkku.begin_I2C(akku.mcp, &Wire)) {
    mcpAkku.pinMode(AKKU_TASTER_LAST, INPUT_PULLUP);
    mcpAkku.pinMode(AKKU_TASTER_LADEN, INPUT_PULLUP);
    mcpAkku.pinMode(AKKU_LED_KLEIN, OUTPUT);
    mcpAkku.pinMode(AKKU_LED_GROSS, OUTPUT);
    mcpAkku.pinMode(BAR_DCKI, OUTPUT);
    mcpAkku.pinMode(BAR_DI, OUTPUT);
    barGpiob = 0;
    barPin(BIT_DCKI | BIT_DI, false);
    barStand = -1;
    lastLeds();
  } else {
    akku.mcp = 0;
  }
  if (!oledStarten(oledAkku, akkuOled)) akkuOled = 0;
  akku.bereit = true;
}

void akkuSchritt() {
  if (!akku.bereit) return;

  // Taster
  if (akku.mcp) {
    kanal(akku.kanal);
    uint16_t pins = mcpAkku.readGPIOA();
    if (tasterLaden.neuGedrueckt(!(pins & (1 << AKKU_TASTER_LADEN)))) {
      ak.laden = !ak.laden;
      ak.voll = false;
      if (ak.laden) { ak.last = LAST_AUS; ak.aus = false; }
      Serial.printf("Akku: Ladegeraet %s\n", ak.laden ? "an" : "ab");
    }
    if (tasterLast.neuGedrueckt(!(pins & (1 << AKKU_TASTER_LAST))) && !ak.aus) {
      ak.last = (Last)((ak.last + 1) % 3);
      if (ak.last != LAST_AUS) ak.laden = false;   // Handy wird benutzt, Ladegerät ab
      Serial.printf("Akku: Last %s\n", LAST_NAME[ak.last]);
    }
  }

  // Simulation
  static uint32_t letzteRechnung = millis();
  float dt = (millis() - letzteRechnung) / 1000.0f;
  letzteRechnung = millis();

  float u0 = ruhespannung(ak.soc);
  ak.cv = false;
  if (ak.laden && !ak.voll) {
    ak.strom = LADESTROM;                       // CC: konstanter Strom
    if (u0 + ak.strom * RI >= LADESPANNUNG) {   // CV: Spannung begrenzt, Strom sinkt
      ak.cv = true;
      ak.strom = max(0.0f, (LADESPANNUNG - u0) / RI);
    }
    if (ak.cv && ak.strom <= LADEENDE_STROM) {
      ak.voll = true;
      ak.soc = 100;
      ak.strom = 0;
    }
  } else if (ak.laden) {
    ak.strom = 0;                               // voll, Ladegerät steckt noch
  } else {
    ak.strom = ak.last == LAST_GR ? -LAST_GROSS : ak.last == LAST_KL ? -LAST_KLEIN : 0;
  }

  ak.soc = constrain(ak.soc + ak.strom * dt / KAPAZITAET_AS * 100, 0.0f, 100.0f);
  if (ak.soc <= 0 && !ak.laden && ak.strom < 0) {
    ak.aus = true;                              // Handy schaltet ab
    ak.last = LAST_AUS;
    ak.strom = 0;
    Serial.println("Akku: leer, Handy aus");
  }
  ak.spannung = u0 + ak.strom * RI;             // Klemmenspannung: U = U0 + I*Ri (I < 0 beim Entladen)

  // Ausgaben, höchstens alle 250 ms
  static uint32_t letzteAusgabe = 0;
  if (millis() - letzteAusgabe < 250) return;
  letzteAusgabe = millis();

  uint8_t stufen = ak.soc <= 0 ? 0 : (uint8_t)ceilf(ak.soc / 10);
  if (ak.laden && stufen == 0) stufen = 1;          // leer am Ladegerät: erste LED blinkt
  uint16_t maske = (1 << stufen) - 1;
  if (ak.laden && !ak.voll && (millis() / 500) % 2) {
    maske &= ~(1 << (stufen > 0 ? stufen - 1 : 0));   // oberste LED blinkt beim Laden
  }
  barSetzen(maske);
  lastLeds();
  dacSetzen(ak.spannung);                       // auch "Handy aus": der Akku hat noch Spannung
  akkuAnzeigen();
}

bool akkuNeuGestartet() {
  if (!akku.bereit || !akku.mcp) return false;
  kanal(akku.kanal);
  int iodirb = regLesen(akku.mcp, 0x01);
  return iodirb < 0 || (iodirb & (BIT_DCKI | BIT_DI));
}

// =====================================================================
// Station Haptik
// =====================================================================

// Muster: abwechselnd Dauer an, Dauer aus (ms), 0 = Ende
const uint16_t MUSTER_KURZ[]   = {150, 0};
const uint16_t MUSTER_DOPPEL[] = {120, 120, 120, 0};
const uint16_t MUSTER_LANG[]   = {400, 200, 400, 200, 400, 0};

Taster tasterHaptik;
const uint16_t *muster = nullptr;
uint8_t musterSchritt = 0;
uint32_t musterSeit = 0;
uint32_t druckStart = 0, letztesLoslassen = 0;
bool wartetAufZweiten = false, langAusgeloest = false;

void vibration(bool an) {
  kanal(haptik.kanal);
  drv.setRealtimeValue(an ? STAERKE : 0);
}

void musterStarten(const uint16_t *m, const char *name) {
  muster = m;
  musterSchritt = 0;
  musterSeit = millis();
  vibration(true);
  Serial.printf("Haptik: %s\n", name);
}

void haptikEinrichten() {
  if (!haptik.da) return;
  haptik.bereit = false;                         // erst nach erfolgreichem Einrichten wieder true
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
  muster = nullptr;
  haptik.bereit = true;
}

void haptikSchritt() {
  if (!haptik.bereit) return;

  // Laufendes Muster weiterschalten
  if (muster && millis() - musterSeit >= muster[musterSchritt]) {
    musterSchritt++;
    musterSeit = millis();
    if (muster[musterSchritt] == 0) {
      muster = nullptr;
      vibration(false);
    } else {
      vibration(musterSchritt % 2 == 0);        // gerade Schritte = an
    }
  }

  if (!haptik.mcp) return;
  kanal(haptik.kanal);
  bool jetzt = mcpHaptik.digitalRead(HAPTIK_TASTER) == LOW;
  bool vorher = tasterHaptik.gedrueckt;

  if (tasterHaptik.neuGedrueckt(jetzt)) {       // gedrückt
    druckStart = millis();
    langAusgeloest = false;
    if (wartetAufZweiten && millis() - letztesLoslassen < DOPPEL_MS) {
      wartetAufZweiten = false;
      musterStarten(MUSTER_DOPPEL, "zweimal gedrueckt -> Nachricht");
      langAusgeloest = true;                    // dieser Druck ist verbraucht
    }
  } else if (vorher && !tasterHaptik.gedrueckt) {   // losgelassen
    letztesLoslassen = millis();
    if (!langAusgeloest) wartetAufZweiten = true;
  }

  if (tasterHaptik.gedrueckt && !langAusgeloest && millis() - druckStart >= LANG_MS) {
    langAusgeloest = true;
    wartetAufZweiten = false;
    musterStarten(MUSTER_LANG, "lang gedrueckt -> Anruf");
  }
  if (wartetAufZweiten && millis() - letztesLoslassen >= DOPPEL_MS) {
    wartetAufZweiten = false;
    musterStarten(MUSTER_KURZ, "kurz gedrueckt -> Benachrichtigung");
  }
}

bool haptikNeuGestartet() {
  if (!haptik.bereit) return false;
  kanal(haptik.kanal);
  return regLesen(0x5A, DRV2605_REG_MODE) != DRV2605_MODE_REALTIME;
}

// =====================================================================
// Station Touch
// =====================================================================

void touchEinrichten() {
  if (!touch.da) return;
  touch.bereit = false;                         // erst nach erfolgreichem Einrichten wieder true
  kanal(touch.kanal);
  Serial.printf("  Touch an Kanal %d:\n", touch.kanal);
  pruefzeile("MPR121", touchMpr, "0x5A-0x5D");
  pruefzeile("OLED", touchOled, "0x3C/0x3D");
  pruefzeile("MCP23017", touch.mcp, "0x20-0x27");
  if (!oledStarten(oledTouch, touchOled)) touchOled = 0;
  // Autoconfig: Ladestrom passend zu den Elektroden (ohne liegen die Messwerte nahe 0)
  if (!mpr.begin(touchMpr, &Wire, MPR121_TOUCH_THRESHOLD_DEFAULT,
                 MPR121_RELEASE_THRESHOLD_DEFAULT, true)) {
    Serial.println("    MPR121 startet nicht");
    return;
  }
  mpr.writeRegister(MPR121_ECR, 0x00);          // nur die genutzten Elektroden einschalten
  mpr.writeRegister(MPR121_ECR, 0x80 | ELEKTRODEN);
  touch.bereit = true;
}

bool touchNeuZeichnen = true;

void touchSchritt() {
  if (!touch.bereit) return;
  static uint32_t letzteAnzeige = 0;
  static uint8_t letzteHoehe[ELEKTRODEN];
  static uint16_t letzterStatus = 0xFFFF;
  if (touchNeuZeichnen) { letzterStatus = 0xFFFF; touchNeuZeichnen = false; }
  if (millis() - letzteAnzeige < TOUCH_MS) return;
  letzteAnzeige = millis();

  kanal(touch.kanal);
  uint16_t status = mpr.touched() & ((1 << ELEKTRODEN) - 1);

  // Balkenhöhe = wie weit der Messwert unter den Ruhewert fällt
  const uint8_t oben = 10, hoehe = 54;
  uint8_t h[ELEKTRODEN];
  bool geaendert = status != letzterStatus;
  for (uint8_t i = 0; i < ELEKTRODEN; i++) {
    int16_t diff = (int16_t)mpr.baselineData(i) - (int16_t)mpr.filteredData(i);
    h[i] = constrain((int32_t)diff * hoehe / TOUCH_SKALA, 0, hoehe);
    if (abs(h[i] - letzteHoehe[i]) > 1) geaendert = true;
  }
  if (status != letzterStatus) {
    for (uint8_t i = 0; i < ELEKTRODEN; i++) {
      if ((status & (1 << i)) && !(letzterStatus & (1 << i))) Serial.printf("Touch: Pin %u beruehrt\n", i);
    }
  }
  letzterStatus = status;
  if (!geaendert || !touchOled) return;
  memcpy(letzteHoehe, h, sizeof(h));

  oledTouch.clearDisplay();
  const uint8_t breite = 128 / ELEKTRODEN;
  for (uint8_t i = 0; i < ELEKTRODEN; i++) {
    uint8_t x = i * breite + 4;
    oledTouch.setTextSize(1);
    oledTouch.setCursor(x + breite / 2 - 7, 0);
    oledTouch.print(i);
    if (h[i] > 0) {
      if (status & (1 << i)) oledTouch.fillRect(x, oben + hoehe - h[i], breite - 8, h[i], SSD1306_WHITE);
      else                   oledTouch.drawRect(x, oben + hoehe - h[i], breite - 8, h[i], SSD1306_WHITE);
    }
  }
  // Schwelle "berührt" als gestrichelte Linie
  uint8_t ys = oben + hoehe - MPR121_TOUCH_THRESHOLD_DEFAULT * hoehe / TOUCH_SKALA;
  for (uint8_t x = 0; x < 128; x += 4) oledTouch.drawFastHLine(x, ys, 2, SSD1306_WHITE);
  oledTouch.display();
}

bool touchNeuGestartet() {
  if (!touch.bereit) return false;
  kanal(touch.kanal);
  return regLesen(touchMpr, MPR121_ECR) == 0;
}

// =====================================================================
// Station Prox
// =====================================================================

uint16_t proxRuhewert = 0;

void ampel(uint8_t stufe) {          // 0 = aus, 1 = grün, 2 = gelb, 3 = rot
  if (!prox.mcp) return;
  kanal(prox.kanal);
  uint8_t gpioa = stufe == 1 ? 1 << LED_GRUEN : stufe == 2 ? 1 << LED_GELB : stufe == 3 ? 1 << LED_ROT : 0;
  regSchreiben(prox.mcp, 0x14, gpioa);          // OLATA
}

void proxEinrichten() {
  if (!prox.da) return;
  prox.bereit = false;                         // erst nach erfolgreichem Einrichten wieder true
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
    letzteStufe = stufe;
  }
}

bool proxNeuGestartet() {
  if (!prox.bereit || !prox.mcp) return false;
  kanal(prox.kanal);
  int iodira = regLesen(prox.mcp, 0x00);
  return iodira < 0 || (iodira & 0x07);
}

// =====================================================================
// Auffrischen
// =====================================================================
// Bei einem kurzen Spannungseinbruch können OLED, LED-Bar oder VCNL4010 neu starten,
// ohne dass es auffällt: das OLED bleibt dann dunkel, die LED-Bar leer, und der
// VCNL4010 springt auf 200 mA IR-Strom zurück. Deshalb wird reihum je ein Modul
// neu beschrieben (nie alle gleichzeitig, damit keine Stromspitze entsteht).

void auffrischen() {
  static uint8_t reihum = 0;
  reihum = (reihum + 1) % 3;

  if (reihum == 0 && akku.bereit) {
    kanal(akku.kanal);
    if (akkuOled && oledStarten(oledAkku, akkuOled, false)) akkuAnzeigen();
    barStand = -1;                                  // LED-Bar beim nächsten Schritt neu senden
  }
  if (reihum == 1 && touch.bereit && touchOled) {
    kanal(touch.kanal);
    oledStarten(oledTouch, touchOled, false);
    touchNeuZeichnen = true;                        // wird im nächsten Touch-Schritt neu gezeichnet
  }
  if (reihum == 2 && prox.bereit) {
    kanal(prox.kanal);
    if (vcnl.getLEDcurrent() != VCNL_STROM) {
      vcnl.setLEDcurrent(VCNL_STROM);
      Serial.println("Prox: IR-Strom war zurueckgesetzt -> wieder 50 mA");
    }
  }
}

// =====================================================================
// Ablauf
// =====================================================================

void modulEinrichten(Typ typ) {
  switch (typ) {
    case PROX:   proxEinrichten(); break;
    case HAPTIK: haptikEinrichten(); break;
    case TOUCH:  touchEinrichten(); break;
    case AKKU:   akkuEinrichten(); break;
    default: break;
  }
}

// Modul an Kanal k übernehmen und einrichten
void modulAnmelden(Typ typ, int8_t k) {
  Modul *m = modulVon(typ);
  if (!m) {
    if (typ == UNBEKANNT) Serial.printf("Kanal %d: unbekanntes Modul (nur MCP23017)\n", k);
    return;
  }
  if (m->da && m->kanal != k) {
    Serial.printf("Kanal %d: weiteres %s-Modul wird ignoriert (nur eins pro Typ)\n", k, TYP_NAME[typ]);
    return;
  }
  kanal(k);
  *m = Modul();
  m->da = true;
  m->kanal = k;
  m->mcp = ersteAdresse(0x20, 0x27);
  if (typ == TOUCH) {
    touchMpr = ersteAdresse(0x5A, 0x5D);
    touchOled = ersteAdresse(0x3C, 0x3D);
  }
  if (typ == AKKU) {
    akkuOled = ersteAdresse(0x3C, 0x3D);
    akkuDac = ersteAdresse(0x60, 0x63);
  }
  modulEinrichten(typ);
  Serial.printf("Kanal %d: %s %s\n", k, TYP_NAME[typ], m->bereit ? "bereit" : "NICHT bereit, neuer Versuch folgt");
}

// Modul an Kanal k wurde gezogen
void modulAbmelden(Typ typ, int8_t k) {
  Modul *m = modulVon(typ);
  if (!m || !m->da || m->kanal != k) return;
  *m = Modul();
  Serial.printf("Kanal %d: %s gezogen\n", k, TYP_NAME[typ]);
  // Ein zweites, bisher ignoriertes Modul gleichen Typs darf übernehmen
  for (int8_t j = 0; j < 8; j++) {
    if (j != k && kanalTyp[j] == typ) kanalTyp[j] = kandidat[j] = KEINS;
  }
}

// Hängt ein Modul SDA auf LOW (z. B. mitten in einer Übertragung gezogen),
// wird der Bus mit bis zu 9 Takten und einem STOP wieder freigegeben.
void busRetten() {
  Wire.end();
  pinMode(PIN_SDA, INPUT_PULLUP);
  pinMode(PIN_SCL, OUTPUT_OPEN_DRAIN);
  digitalWrite(PIN_SCL, HIGH);
  for (uint8_t i = 0; i < 9 && digitalRead(PIN_SDA) == LOW; i++) {
    digitalWrite(PIN_SCL, LOW);  delayMicroseconds(5);
    digitalWrite(PIN_SCL, HIGH); delayMicroseconds(5);
  }
  pinMode(PIN_SDA, OUTPUT_OPEN_DRAIN);          // STOP: SDA steigt, während SCL HIGH ist
  digitalWrite(PIN_SDA, LOW);  delayMicroseconds(5);
  digitalWrite(PIN_SDA, HIGH); delayMicroseconds(5);
  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);
  aktiverKanal = -2;
}

// Ein Durchgang: Bus prüfen, Belegung prüfen, neu gestartete Module neu einrichten
void pruefen() {
  aktiverKanal = -2;                            // Kanal sicher neu setzen (falls der TCA neu gestartet ist)

  if (!antwortet(tcaAdresse)) {
    busRetten();
    busOk = antwortet(tcaAdresse);
    Serial.println(busOk ? "I2C-Bus hing -> freigegeben" : "Multiplexer antwortet nicht");
    if (!busOk) return;
  }
  busOk = true;

  // Belegung: höchstens eine Änderung pro Durchgang, damit Einschaltströme verteilt bleiben
  for (int8_t k = 0; k < 8; k++) {
    kanal(k);
    Typ t = kanalErkennen();
    if (t == kanalTyp[k]) { kandidat[k] = t; continue; }
    if (t != kandidat[k]) { kandidat[k] = t; continue; }   // erst beim zweiten Mal gleich: übernehmen

    Typ alt = kanalTyp[k];
    kanalTyp[k] = t;
    modulAbmelden(alt, k);
    if (t != KEINS) modulAnmelden(t, k);
    return;
  }

  // Neu gestartete Module (kurzer Spannungseinbruch) einzeln neu einrichten
  if (akkuNeuGestartet())   { Serial.println("Akku neu gestartet");   akkuEinrichten();   return; }
  if (haptikNeuGestartet()) { Serial.println("Haptik neu gestartet"); haptikEinrichten(); return; }
  if (touchNeuGestartet())  { Serial.println("Touch neu gestartet");  touchEinrichten();  return; }
  if (proxNeuGestartet())   { Serial.println("Prox neu gestartet");   proxEinrichten();   return; }

  // Gesteckt, aber Einrichten fehlgeschlagen: nach einer Weile erneut versuchen
  static uint32_t letzterVersuch = 0;
  if (millis() - letzterVersuch >= WIEDERHOL_MS) {
    letzterVersuch = millis();
    Typ typen[] = {AKKU, HAPTIK, TOUCH, PROX};
    for (Typ typ : typen) {
      Modul *m = modulVon(typ);
      if (m->da && !m->bereit) {
        Serial.printf("%s: neuer Versuch\n", TYP_NAME[typ]);
        modulEinrichten(typ);
        return;
      }
    }
  }
}

// Start: alle Kanäle einmal ausführlich anzeigen und die Module nacheinander einrichten
void startEinrichten() {
  Serial.printf("\nMultiplexer bei 0x%02X, Kanäle:\n", tcaAdresse);
  for (int8_t k = 0; k < 8; k++) {
    adressenAusgeben(k);
    kanal(k);
    kanalTyp[k] = kandidat[k] = kanalErkennen();
  }
  Serial.println("Einrichten (nacheinander, um Einschaltströme zu verteilen):");
  for (int8_t k = 0; k < 8; k++) {
    if (kanalTyp[k] == KEINS) continue;
    modulAnmelden(kanalTyp[k], k);
    delay(EINRICHT_PAUSE_MS);
  }
  Serial.printf("Bereit: Akku %s, Haptik %s, Touch %s, Prox %s\n\n",
                akku.bereit ? "ja" : "nein", haptik.bereit ? "ja" : "nein",
                touch.bereit ? "ja" : "nein", prox.bereit ? "ja" : "nein");
}

const char *neustartGrund() {
  switch (esp_reset_reason()) {
    case ESP_RST_POWERON:  return "Einschalten";
    case ESP_RST_EXT:      return "Reset-Taster";
    case ESP_RST_SW:       return "Software";
    case ESP_RST_PANIC:    return "Absturz";
    case ESP_RST_INT_WDT:
    case ESP_RST_TASK_WDT:
    case ESP_RST_WDT:      return "Watchdog (Programm hing)";
    case ESP_RST_BROWNOUT: return "Brownout (Spannung zu niedrig)";
    default:               return "unbekannt";
  }
}

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Lernstationen Akku / Haptik / Touch / Prox ===");
  Serial.printf("Neustart durch: %s\n", neustartGrund());

  Wire.begin(PIN_SDA, PIN_SCL);
  Wire.setClock(100000);
  Wire.setTimeOut(50);                // ein hängendes Modul blockiert höchstens 50 ms

  while (!(tcaAdresse = ersteAdresse(0x70, 0x77))) {
    Serial.println("Kein Multiplexer gefunden (0x70 bis 0x77), neuer Versuch in 2 s ...");
    delay(2000);
  }
  startEinrichten();
  enableLoopWDT();                    // Watchdog: loop() muss mindestens alle 5 s durchlaufen
}

void loop() {
  static uint32_t letztePruefung = 0;
  if (millis() - letztePruefung >= PRUEF_MS) {
    letztePruefung = millis();
    pruefen();
  }
  if (!busOk) { delay(5); return; }   // ohne Multiplexer nichts ansprechen

  static uint32_t letztesAuffrischen = 0;
  if (millis() - letztesAuffrischen >= AUFFRISCH_MS) {
    letztesAuffrischen = millis();
    auffrischen();
  }

  akkuSchritt();
  haptikSchritt();
  touchSchritt();
  proxSchritt();
  delay(5);
}
