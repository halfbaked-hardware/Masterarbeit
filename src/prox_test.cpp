// Prox-Modul-Test: VCNL4010 + MCP23017 mit LED-Ampel
// ESP32: GPIO21 -> SDA, GPIO22 -> SCL. Das Modul darf direkt am ESP32 oder
// an einem Kanal des TCA9548A hängen; beides wird automatisch erkannt.
// Auf dem Modul: VCNL4010 (fest 0x13), MCP23017 (0x20 bis 0x27),
// LEDs am MCP23017: grün GPA0, gelb GPA1, rot GPA2.
// Ampel: grün = nichts in der Nähe, gelb = etwas nähert sich, rot = nah.
// Beim Start werden die LEDs einzeln durchgeschaltet und der Ruhewert gemessen
// (dabei nichts vor den Sensor halten). Alle 1 s wird geprüft, ob das Modul noch da ist.

#include <Arduino.h>
#include <Wire.h>
#include <Adafruit_VCNL4010.h>
#include <Adafruit_MCP23X17.h>

#define PIN_SDA 21
#define PIN_SCL 22

#define VCNL_ADRESSE 0x13
#define VCNL_REG_ID 0x81      // Product-ID/Revision, beim VCNL4010 0x21

#define LED_GRUEN 0           // GPA0
#define LED_GELB 1            // GPA1
#define LED_ROT 2             // GPA2
#define LED_AKTIV_HIGH true   // false, falls die LEDs gegen 3,3 V geschaltet sind (leuchten bei LOW)

// Ampel-Schwellen: Abstand des Näherungswerts über dem Ruhewert
#define SCHWELLE_GELB 1000
#define SCHWELLE_ROT 8000

#define MESS_MS 200
#define PRUEF_MS 1000

Adafruit_VCNL4010 vcnl;
Adafruit_MCP23X17 mcp;

uint8_t tcaAdresse = 0;       // 0 = kein Multiplexer
int8_t kanalProx = -1;        // -1 = direkt am Bus
uint8_t mcpAdresse = 0;
bool vcnlDa = false, bereit = false;
uint16_t ruhewert = 0;

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

void kanal(int8_t k) {
  if (!tcaAdresse) return;
  Wire.beginTransmission(tcaAdresse);
  Wire.write(k < 0 ? 0 : 1 << k);
  Wire.endTransmission();
}

uint8_t ersteAdresse(uint8_t von, uint8_t bis) {
  for (uint8_t a = von; a <= bis; a++) {
    if (antwortet(a)) return a;
  }
  return 0;
}

// ---------- Prüfungen ----------

void leitungenPruefen() {
  pinMode(PIN_SDA, INPUT_PULLDOWN);
  pinMode(PIN_SCL, INPUT_PULLDOWN);
  delay(5);
  Serial.printf("Bus-Leitungen: SDA=%s  SCL=%s (HIGH = Pull-ups vorhanden)\n",
                digitalRead(PIN_SDA) ? "HIGH" : "LOW",
                digitalRead(PIN_SCL) ? "HIGH" : "LOW");
}

void adressenAusgeben(const char *wo) {
  Serial.printf("  %s:", wo);
  uint8_t n = 0;
  for (uint8_t a = 1; a < 127; a++) {
    if (tcaAdresse && a >= 0x70 && a <= 0x77) continue;
    if (antwortet(a)) { Serial.printf(" 0x%02X", a); n++; }
  }
  Serial.println(n ? "" : " -");
}

void modulSuchen() {
  kanalProx = -1;
  mcpAdresse = 0;
  vcnlDa = false;

  tcaAdresse = ersteAdresse(0x70, 0x77);
  Serial.println("\nGefundene I2C-Adressen:");
  if (tcaAdresse) {
    Serial.printf("  Multiplexer TCA9548A bei 0x%02X\n", tcaAdresse);
    for (int8_t k = 0; k < 8; k++) {
      kanal(k);
      char wo[12];
      snprintf(wo, sizeof(wo), "Kanal %d", k);
      adressenAusgeben(wo);
      if (kanalProx < 0 && antwortet(VCNL_ADRESSE)) kanalProx = k;
    }
    if (kanalProx < 0) {
      Serial.println("Prox-Modul: kein VCNL4010 (0x13) an einem Kanal gefunden.");
      return;
    }
    kanal(kanalProx);
  } else {
    Serial.println("  Kein Multiplexer, Modul direkt am Bus");
    adressenAusgeben("Bus");
  }

  vcnlDa = antwortet(VCNL_ADRESSE);
  mcpAdresse = ersteAdresse(0x20, 0x27);

  if (tcaAdresse) Serial.printf("Prox-Modul an Kanal %d:\n", kanalProx);
  else            Serial.println("Prox-Modul:");
  if (vcnlDa) {
    int id = regLesen(VCNL_ADRESSE, VCNL_REG_ID);
    Serial.printf("  [OK]     VCNL4010  0x%02X  Product-ID 0x%02X %s\n", VCNL_ADRESSE, id,
                  (id & 0xF0) == 0x20 ? "" : "(unerwartet, erwartet 0x2x)");
  } else {
    Serial.printf("  [FEHLT]  VCNL4010  (erwartet 0x%02X)\n", VCNL_ADRESSE);
  }
  if (mcpAdresse) Serial.printf("  [OK]     MCP23017  0x%02X\n", mcpAdresse);
  else            Serial.println("  [FEHLT]  MCP23017  (erwartet 0x20-0x27), LEDs werden übersprungen");
}

// ---------- LEDs ----------

void led(uint8_t pin, bool an) {
  if (!mcpAdresse) return;
  mcp.digitalWrite(pin, an == LED_AKTIV_HIGH ? HIGH : LOW);
}

void ampel(bool gruen, bool gelb, bool rot) {
  kanal(kanalProx);
  led(LED_GRUEN, gruen);
  led(LED_GELB, gelb);
  led(LED_ROT, rot);
}

void ledTest() {
  Serial.println("LED-Test: grün, gelb, rot je 0,5 s ...");
  ampel(true, false, false); delay(500);
  ampel(false, true, false); delay(500);
  ampel(false, false, true); delay(500);
  ampel(false, false, false);
}

// ---------- Einrichten ----------

void modulEinrichten() {
  bereit = false;
  modulSuchen();

  if (mcpAdresse) {
    if (mcp.begin_I2C(mcpAdresse, &Wire)) {
      mcp.pinMode(LED_GRUEN, OUTPUT);
      mcp.pinMode(LED_GELB, OUTPUT);
      mcp.pinMode(LED_ROT, OUTPUT);
      ledTest();
    } else {
      mcpAdresse = 0;
      Serial.println("MCP23017 antwortet, lässt sich aber nicht starten.");
    }
  }

  if (!vcnlDa) {
    Serial.println("VCNL4010 fehlt, neuer Versuch in 1 s ...");
    return;
  }
  kanal(kanalProx);
  if (!vcnl.begin(VCNL_ADRESSE, &Wire)) {
    Serial.println("VCNL4010 antwortet, lässt sich aber nicht starten.");
    return;
  }

  // Ruhewert: Mittel aus 10 Messungen ohne Gegenstand vor dem Sensor
  uint32_t summe = 0;
  for (uint8_t i = 0; i < 10; i++) summe += vcnl.readProximity();
  ruhewert = summe / 10;
  Serial.printf("VCNL4010 gestartet, Ruhewert %u. Gelb ab +%u, rot ab +%u.\n",
                ruhewert, SCHWELLE_GELB, SCHWELLE_ROT);
  Serial.println("Hand langsam über den Sensor bewegen ...\n");
  bereit = true;
}

bool modulNochDa() {
  if (tcaAdresse && !antwortet(tcaAdresse)) return false;
  kanal(kanalProx);
  return antwortet(VCNL_ADRESSE);
}

// ---------- Setup / Loop ----------

void setup() {
  Serial.begin(115200);
  delay(500);
  Serial.println("\n=== Prox-Modul-Test VCNL4010 ===");

  leitungenPruefen();
  Wire.begin(PIN_SDA, PIN_SCL);
  modulEinrichten();
}

void loop() {
  static uint32_t letztePruefung = 0;
  if (millis() - letztePruefung >= PRUEF_MS) {
    letztePruefung = millis();
    if (!bereit || !modulNochDa()) {
      if (bereit) Serial.println("\nProx-Modul weg -> neu suchen");
      delay(200);                     // Kontakte nach dem Umstecken zur Ruhe kommen lassen
      modulEinrichten();
    }
  }
  if (!bereit) return;

  static uint32_t letzteMessung = 0;
  if (millis() - letzteMessung < MESS_MS) return;
  letzteMessung = millis();

  kanal(kanalProx);
  uint16_t naehe = vcnl.readProximity();
  uint16_t licht = vcnl.readAmbient();
  int32_t abstand = (int32_t)naehe - ruhewert;

  const char *stufe;
  if (abstand >= SCHWELLE_ROT)       { ampel(false, false, true); stufe = "ROT  "; }
  else if (abstand >= SCHWELLE_GELB) { ampel(false, true, false); stufe = "GELB "; }
  else                               { ampel(true, false, false); stufe = "gruen"; }

  // Balken für die Nähe: grob logarithmisch, damit auch kleine Änderungen sichtbar sind
  uint8_t laenge = 0;
  for (int32_t w = abstand; w > 200 && laenge < 30; w = w * 3 / 4) laenge++;
  char balken[31];
  memset(balken, '#', laenge);
  balken[laenge] = '\0';

  Serial.printf("Naehe: %5u (+%5ld)  Licht: %5u  %s |%-30s|\n",
                naehe, abstand > 0 ? (long)abstand : 0L, licht, stufe, balken);
}
